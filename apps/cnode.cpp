/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_jpeg.cpp - minimal libcamera jpeg capture app.
 */

#include <stdexcept>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <array>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <nlohmann/json.hpp>
#include <cbor.h>

#include "mqtt/client.h"

#include "core/rpicam_app.hpp"
#include "core/still_options.hpp"

#include "image/image.hpp"

#include "post_processing_stages/object_detect.hpp"


using json = nlohmann::json;


template< uint8_t End, uint8_t Start >
struct BitField
{
    static constexpr uint8_t pos = Start;
    static constexpr uint8_t mask = ((1u << (End - Start + 1)) - 1) << Start;
};


struct RegField
{
    uint8_t pos{};
    uint8_t mask{};

    template< uint8_t End, uint8_t Start >
    RegField( BitField<End, Start> bf ): pos(bf.pos), mask(bf.mask) {}
};


template< typename T >
struct REG
{
    T val_{};

    uint8_t VALUE(){ return val_; };

    REG & RESET(){ val_ = 0; return * this; }

    REG & SET(RegField rf, uint8_t value)
    {
        val_ &= ~ rf.mask;
        val_ |= (value << rf.pos) & rf.mask;

        return *this;
    }

    REG & SET(RegField rf)
    {
        SET(rf, 1);

        return *this;
    }

    REG & SET(T value)
    {
        val_ = value;

        return *this;
    }

    T GET(RegField const & rf){ return (val_ & rf.mask) >> rf.pos; }
};


struct sensor_data_t
{
    float temperature;
    float voltage;
    float current;
    uint16_t als_data;
};


class I2C_Device
{
protected:
    int handler_ {};

public:
    I2C_Device(std::string const & i2c_bus, int const i2c_address)
    {
        handler_ = open(i2c_bus.c_str(), O_RDWR);
        if( handler_ < 0 )
        {
            throw std::runtime_error("Failed to open I2C bus!");
        }

        if( ioctl(handler_, I2C_SLAVE, i2c_address) < 0 )
        {
            close(handler_);
            throw std::runtime_error("Failed to acquire I2C bus access!");
        }
    }

    ~I2C_Device()
    {
        close(handler_);
    }

    void writeN(uint8_t addr, uint8_t * buffer, uint8_t n)
    {
        if( write(handler_, buffer, n) != n )
        {
            throw std::runtime_error("I2C Bus Operation Failed!");
        }
    }

    void write8(uint8_t addr, uint8_t val)
    {
        constexpr unsigned N = 2;

        uint8_t buffer[N] = { addr, val };

        if( write(handler_, buffer, N) != N )
        {
            throw std::runtime_error("I2C Bus Operation Failed!");
        }
    }

    void write8(uint8_t val)
    {
        constexpr unsigned N = 1;

        uint8_t buffer[N] = { val };

        if( write(handler_, buffer, N) != N )
        {
            throw std::runtime_error("I2C Bus Operation Failed!");
        }
    }

    uint8_t read8(uint8_t addr)
    {
        if( write(handler_, & addr, 1) != 1 )
        {
            throw std::runtime_error("I2C Bus Operation Failed!");
        }

        constexpr unsigned N = 1;

        uint8_t buffer[N];

        if( read(handler_, buffer, N) != N )
        {
            throw std::runtime_error("I2C Bus Operation Failed!");
        }

        return buffer[0];
    }

    uint16_t read16(uint8_t addr)
    {
        if( write(handler_, & addr, 1) != 1 )
        {
            throw std::runtime_error("I2C Bus Operation Failed!");
        }

        constexpr unsigned N = 2;

        uint8_t buffer[N];

        if( read(handler_, buffer, N) != N )
        {
            throw std::runtime_error("I2C Bus Operation Failed!");
        }

        return (buffer[1] << 8) | buffer[0];
    }
};


namespace TSL2591
{
struct REG_COMMAND: public REG<uint8_t>
{
    static BitField< 7, 7 > CMD_FIELD;
    static BitField< 6, 5 > TRANSACTION_FIELD;
    static BitField< 4, 0 > ADDR_SF_FIELD;

    static constexpr uint8_t TRANSACTION_NORMAL = 0b01;
    static constexpr uint8_t TRANSACTION_SPECIAL = 0b11;

    static constexpr uint8_t SF_CLEAR_INTERRUPTS = 0b00111;
};

struct REG_CONTROL: public REG<uint8_t>
{
    static constexpr uint8_t ADDR = 0x01;

    static BitField< 5, 4 > AGAIN_FIELD;
    static BitField< 2, 0 > ATIME_FIELD;

    static constexpr uint8_t GAIN_LOW = 0b00;
    static constexpr uint8_t GAIN_MEDIUM = 0b01;
    static constexpr uint8_t GAIN_HIGH = 0b10;
    static constexpr uint8_t GAIN_MAXIMUM = 0b11;

    static constexpr uint8_t TIME_100MS = 0b000;
    static constexpr uint8_t TIME_200MS = 0b001;
    static constexpr uint8_t TIME_300MS = 0b010;
    static constexpr uint8_t TIME_400MS = 0b011;
    static constexpr uint8_t TIME_500MS = 0b100;
    static constexpr uint8_t TIME_600MS = 0b101;
};

struct REG_ENABLE: public REG<uint8_t>
{
    static constexpr uint8_t ADDR = 0x00;

    static BitField< 0, 0 > PON;
    static BitField< 1, 1 > AEN;
    static BitField< 4, 4 > AIEN;
    static BitField< 6, 6 > SAI;
    static BitField< 7, 7 > NPIEN;
};

struct REG_NPAIHTL: public REG<uint8_t>
{
    static constexpr uint8_t ADDR = 0x0A;
};

struct REG_NPAIHTH: public REG<uint8_t>
{
    static constexpr uint8_t ADDR = 0x0B;
};

struct REG_C0DATAL: public REG<uint8_t>
{
    static constexpr uint8_t ADDR = 0x14;
};

struct REG_C0DATAH: public REG<uint8_t>
{
    static constexpr uint8_t ADDR = 0x15;
};

struct REG_C1DATAL: public REG<uint8_t>
{
    static constexpr uint8_t ADDR = 0x16;
};

struct REG_C1DATAH: public REG<uint8_t>
{
    static constexpr uint8_t ADDR = 0x17;
};

class TSL2591_Sensor: public I2C_Device
{
public:

    TSL2591_Sensor(std::string const & i2c_bus, int const i2c_address = 0x29)
            :I2C_Device(i2c_bus, i2c_address)
    {}

    enum class CHANNEL{ CH0, CH1 };

    uint16_t read_als(CHANNEL channel)
    {
        using namespace TSL2591;

        REG<uint8_t> reg;
        reg
            .SET(REG_COMMAND::CMD_FIELD)
            .SET(REG_COMMAND::TRANSACTION_FIELD, REG_COMMAND::TRANSACTION_NORMAL);

        switch( channel )
        {
        case CHANNEL::CH0:
            reg.SET(REG_COMMAND::ADDR_SF_FIELD, REG_C0DATAL::ADDR);
            break;
        case CHANNEL::CH1:
            reg.SET(REG_COMMAND::ADDR_SF_FIELD, REG_C1DATAL::ADDR);
            break;
        }

        return read16(reg.VALUE());
    }

    void setup()
    {
        using namespace TSL2591;

        REG<uint8_t> reg_addr;
        REG<uint8_t> reg_val;

        reg_addr
            .SET(REG_COMMAND::CMD_FIELD)
            .SET(REG_COMMAND::TRANSACTION_FIELD, REG_COMMAND::TRANSACTION_NORMAL);

        reg_addr.SET(REG_COMMAND::ADDR_SF_FIELD, REG_ENABLE::ADDR);
        auto status = read8(reg_addr.VALUE());

        // If value of Enable register is already set to the desired value,
        // the sensor has alreay been configured
        if( (REG_ENABLE::PON.mask & status) && (REG_ENABLE::AEN.mask & status) )
        {
            return;
        }

        reg_val
            .RESET()
            .SET(REG_ENABLE::PON)
            .SET(REG_ENABLE::AEN);
        reg_addr.SET(REG_COMMAND::ADDR_SF_FIELD, REG_ENABLE::ADDR);
        write8(reg_addr.VALUE(), reg_val.VALUE());

        reg_val
            .RESET()
            .SET(REG_CONTROL::AGAIN_FIELD, REG_CONTROL::GAIN_HIGH)
            .SET(REG_CONTROL::ATIME_FIELD, REG_CONTROL::TIME_500MS);
        reg_addr.SET(REG_COMMAND::ADDR_SF_FIELD, REG_CONTROL::ADDR);
        write8(reg_addr.VALUE(), reg_val.VALUE());
    }

    void enable_interrupt(uint16_t threshold)
    {
        using namespace TSL2591;

        REG<uint8_t> reg_val;
        REG<uint8_t> reg_addr;

        reg_addr
            .SET(REG_COMMAND::CMD_FIELD)
            .SET(REG_COMMAND::TRANSACTION_FIELD, REG_COMMAND::TRANSACTION_SPECIAL)
            .SET(REG_COMMAND::ADDR_SF_FIELD, REG_COMMAND::SF_CLEAR_INTERRUPTS);
        write8(reg_addr.VALUE());

        reg_addr
            .RESET()
            .SET(REG_COMMAND::CMD_FIELD)
            .SET(REG_COMMAND::TRANSACTION_FIELD, REG_COMMAND::TRANSACTION_NORMAL);

        reg_val
            .SET(static_cast<uint8_t>(threshold >> 8));
        reg_addr
            .SET(REG_COMMAND::ADDR_SF_FIELD, REG_NPAIHTH::ADDR);
        write8(reg_addr.VALUE(), reg_val.VALUE());

        reg_val
            .SET(static_cast<uint8_t>(threshold));
        reg_addr
            .SET(REG_COMMAND::ADDR_SF_FIELD, REG_NPAIHTL::ADDR);
        write8(reg_addr.VALUE(), reg_val.VALUE());

        reg_val
            .RESET()
            .SET(REG_ENABLE::PON)
            .SET(REG_ENABLE::AEN)
            .SET(REG_ENABLE::NPIEN);
        reg_addr
            .SET(REG_COMMAND::ADDR_SF_FIELD, REG_ENABLE::ADDR);
        write8(reg_addr.VALUE(), reg_val.VALUE());
    }

    void disable_interrupt()
    {
        using namespace TSL2591;

        REG<uint8_t> reg_val;
        REG<uint8_t> reg_addr;

        reg_val
            .RESET()
            .SET(REG_ENABLE::PON)
            .SET(REG_ENABLE::AEN);
        reg_addr
            .RESET()
            .SET(REG_COMMAND::CMD_FIELD)
            .SET(REG_COMMAND::TRANSACTION_FIELD, REG_COMMAND::TRANSACTION_NORMAL)
            .SET(REG_COMMAND::ADDR_SF_FIELD, REG_ENABLE::ADDR);
        write8(reg_addr.VALUE(), reg_val.VALUE());
    }
};

}


struct CNodeConfig
{
private:
    std::string factory_sid;
    std::string factory_mqtt_server;
    std::string factory_mqtt_client_id;
    std::string factory_mqtt_password;
    std::string factory_config_topic;
    std::string factory_data_topic;

    void substitute(std::string & target)
    {
        static std::string const id_tmpl { "{{ID}}" };
        auto pos = target.find(id_tmpl);
        if( pos != std::string::npos )
        {
            target.replace(pos, id_tmpl.size(), factory_sid);
        }
    }

public:
    bool stay_awake{ false };
    uint32_t wakeup_interval{ 30 };  // minutes
    uint32_t fallback_wakeup_interval{ 60 };
    uint16_t person_wakeup_interval{ 2 };
    uint16_t light_threshold{ 100 };
    std::string ca_certificate_file { "/usr/local/share/ca-certificates/ca.crt" };
    std::string config_topic;
    std::string data_topic;

    CNodeConfig()
    {
        load_fuses();

        substitute(factory_config_topic);
        substitute(factory_data_topic);
        substitute(factory_mqtt_client_id);
    }

    bool operator==( CNodeConfig const & other ) const
    {
        return stay_awake == other.stay_awake
                && wakeup_interval == other.wakeup_interval
                && fallback_wakeup_interval == other.fallback_wakeup_interval
                && person_wakeup_interval == other.person_wakeup_interval
                && light_threshold == other.light_threshold
                && config_topic == other.config_topic
                && data_topic == other.data_topic;
    }

    bool operator!=( CNodeConfig const & other) const = default;

    std::string get_mqtt_client_id(){ return factory_mqtt_client_id; }
    std::string get_mqtt_server(){ return factory_mqtt_server; }
    std::string get_mqtt_password(){ return factory_mqtt_password; }

    void load_fuses()
    {
        std::string path{ "/etc/cnode/cnode.conf" };
        std::ifstream file(path);

        if( ! file )
        {
            std::cerr << "Failed to open fuses file: " << path << std::endl;
            return;
        }

        std::string line;
        int ix {};
        while( std::getline(file, line) )
        {
            if( line.starts_with("#") ){ continue; }

            switch( ix )
            {
            case 0: factory_sid = line; break;
            case 1: factory_mqtt_server = line; break;
            case 2: factory_mqtt_password = line; break;
            case 3: factory_mqtt_client_id = line; break;
            case 4: factory_config_topic = line; break;
            case 5: factory_data_topic = line; break;
            }

            ++ix;
        }
    }

    void load()
    {
        init();  // Load factory settings

        std::string path{ "cnode-config.json" };
        std::ifstream file(path);
        if( ! file )
        {
            std::cerr << "Failed to open config file: " << path << std::endl;
            return;
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        file.close();

        auto config = nlohmann::json::parse(buffer.str());

        init(config);  // Override factory settings
    }

    void store()
    {
        nlohmann::json config;

        std::string path{ "cnode-config.json" };

        config["stay_awake"] = stay_awake;
        config["wakeup_interval"] = wakeup_interval;
        config["fallback_wakeup_interval"] = fallback_wakeup_interval;
        config["person_wakeup_interval"] = person_wakeup_interval;
        config["light_threshold"] = light_threshold;
        config["config_topic"] = config_topic;
        config["data_topic"] = data_topic;

        auto data = config.dump(4);

        std::ofstream ofs(path);
        if( ofs.is_open() )
        {
            ofs << data;
            ofs.close();
        }
        else{
            std::cerr << "Failed to open config file: " << path << std::endl;
        }
    }

    void init()
    {
        data_topic = factory_data_topic;
        config_topic = factory_config_topic;
    }

    void init(json const & config)
    {
        stay_awake = config.value("stay_awake", stay_awake);
        wakeup_interval = config.value("wakeup_interval", wakeup_interval);
        fallback_wakeup_interval = config.value("fallback_wakeup_interval", fallback_wakeup_interval);
        person_wakeup_interval = config.value("person_wakeup_interval", person_wakeup_interval);
        light_threshold = config.value("light_threshold", light_threshold);
        data_topic = config.value("data_topic", data_topic);
        config_topic = config.value("config_topic", config_topic);
    }

    void update(json const & config)
    {
        auto curr = * this;
        init(config);

        if( curr != * this ){ store(); }
    }

    void print()
    {
        std::cout << "ca_certificate_file: " << ca_certificate_file << std::endl;
        std::cout << "data_topic: " << data_topic << std::endl;
        std::cout << "config_topic: " << config_topic << std::endl;
        std::cout << "wakeup_interval: " << wakeup_interval << std::endl;
        std::cout << "fallback_wakeup_interval: " << fallback_wakeup_interval << std::endl;
        std::cout << "person_wakeup_interval: " << person_wakeup_interval << std::endl;
        std::cout << "light_threshold: " << light_threshold << std::endl;
        std::cout << "stay_awake: " << stay_awake << std::endl;
    }

} G_config;


class MQTTAgent
{
    class Callback;
    mqtt::client_ptr  client_ptr_;
public:
    MQTTAgent(std::string const & server, std::string const & client_id)
    {
        // Create MQTT Client
        client_ptr_ = std::make_shared<mqtt::client>(
            server, client_id, mqtt::create_options(MQTTVERSION_5)
        );
    }

    void connect()
    {
        mqtt::connect_options conn_opts;

        conn_opts.set_clean_session(false);
        conn_opts.set_mqtt_version(MQTTVERSION_5);

        conn_opts.set_user_name("");
        conn_opts.set_password(G_config.get_mqtt_password());

        auto server = client_ptr_->get_server_uri();

        if(server.starts_with("ssl") || server.starts_with("tls")){
            mqtt::ssl_options sslopts;
            sslopts.set_verify(false);  // FIXME:
            sslopts.set_enable_server_cert_auth(true);
            sslopts.set_trust_store(G_config.ca_certificate_file);
            conn_opts.set_ssl(sslopts);
        }
        auto response = client_ptr_->connect(conn_opts);
    }

    void send(std::string const & topic, std::string const & payload)
    {
        try {
            client_ptr_->publish(topic, payload.c_str(), payload.size());
        }
        catch (const mqtt::exception& exc) {
            std::cerr << exc << std::endl;
            throw std::runtime_error("Unable to send message to MQTT server");
        }
    }

    void subscribe()
    {
        client_ptr_->subscribe(G_config.config_topic, 1);
    }

    std::string receive_config()
    {
        mqtt::const_message_ptr msg;
        if( client_ptr_->try_consume_message_for(& msg, std::chrono::seconds(10)) )
        {
            return msg.get()->get_payload_str();
        }

        return "";
    }
};


using namespace std::placeholders;
using libcamera::Stream;

class RPiCamJpegApp : public RPiCamApp {
public:
    RPiCamJpegApp() : RPiCamApp(std::make_unique<StillOptions>()) {}

    StillOptions *GetOptions() const { return static_cast<StillOptions *>(RPiCamApp::GetOptions()); }
};



bool capture_frame(RPiCamJpegApp & app, std::basic_ostringstream<char> & buff)
{
    StillOptions const *options = app.GetOptions();
    app.OpenCamera();
    app.ConfigureStill();
    app.StartCamera();
    auto start_time = std::chrono::high_resolution_clock::now();
    bool detected{ false };

    while (true) {
        RPiCamApp::Msg msg = app.Wait();
        if (msg.type == RPiCamApp::MsgType::Timeout) {
            LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
            app.StopCamera();
            app.StartCamera();
            continue;
        }

        if (msg.type == RPiCamApp::MsgType::Quit) {
            break;
        }
        else if (msg.type != RPiCamApp::MsgType::RequestComplete) {
            throw std::runtime_error("unrecognised message!");
        }
        else if (app.StillStream()) {
            app.StopCamera();
            LOG(1, "Still capture image received");
            CompletedRequestPtr & completed_request = std::get<CompletedRequestPtr>(msg.payload);

            std::vector<Detection> detections;
            if(completed_request->post_process_metadata.Get(
                "object_detect.results", detections) == 0){
                    detected = (std::find_if(
                        detections.begin(),
                        detections.end(),
                        [options](const Detection &d){
                            return d.name.find(options->Get().object) != std::string::npos;
                        }
                    ) != detections.end());
            }

            if( ! detected )
            {
                Stream *stream = app.StillStream();
                StreamInfo info = app.GetStreamInfo(stream);
                BufferReadSync r(&app, completed_request->buffers[stream]);
                const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
                jpeg_write(mem, info, completed_request->metadata, buff, app.CameraModel(), options);
            }
            break;
        }
    }

    return detected;
}


std::string get_iso_datetime(std::chrono::time_point<std::chrono::system_clock> tp)
{
    std::time_t now_time = std::chrono::system_clock::to_time_t(tp);

    std::stringstream ss;
    ss << std::put_time(std::gmtime(&now_time), "%Y-%m-%dT%H:%M:%SZ");

    return ss.str();
}


float read_temperature(int file)
{
    unsigned char reg = 50;
    if( write(file, & reg, 1) != 1 )
    {
        return -255;
    }

    unsigned char value[2];

    if( read(file, value, 2) != 2 )
    {
        return -255;
    }

    return (float)((int16_t)((value[0] << 8) | value[1]) >> 7) / 2;
}


float read_voltage(int file)
{
    unsigned char reg;
    unsigned char value;
    float result;

    reg = 1;
    if( write(file, & reg, 1) != 1 )
    {
        return -255;
    }
    if( read(file, & value, 1) != 1 )
    {
        return -255;
    }
    result = value;

    reg = 2;
    if( write(file, & reg, 1) != 1 )
    {
        return -255;
    }
    if( read(file, & value, 1) != 1 )
    {
        return -255;
    }

    return result + (float)value / 100;
}


float read_current(int file)
{
    unsigned char reg;
    unsigned char value;
    float result;

    reg = 5;
    if( write(file, & reg, 1) != 1 )
    {
        return -255;
    }
    if( read(file, & value, 1) != 1 )
    {
        return -255;
    }
    result = value;

    reg = 6;
    if( write(file, & reg, 1) != 1 )
    {
        return -255;
    }
    if( read(file, & value, 1) != 1 )
    {
        return -255;
    }

    return result + (float)value / 100;
}


sensor_data_t read_sensors()
{
    char const * i2c_device = "/dev/i2c-1";
    int const sensor_address = 0x08;

    int file = open(i2c_device, O_RDWR);
    if( file < 0 )
    {
        std::cerr << "Failed to open I2C bus" << std::endl;
        return { -255, -255, -255 };
    }

    if( ioctl(file, I2C_SLAVE, sensor_address) < 0 )
    {
        std::cerr << "Failed to acquire bus access or talk to slave" << std::endl;
        close(file);
        return { -255, -255, -255 };
    }

    sensor_data_t sd = {
        read_temperature(file),
        read_voltage(file),
        read_current(file)
    };

    close(file);

    TSL2591::TSL2591_Sensor light_sensor{ std::string("/dev/i2c-1"), 0x29 };
    light_sensor.setup();
    sd.als_data = light_sensor.read_als(TSL2591::TSL2591_Sensor::CHANNEL::CH0);

    return sd;
}


inline uint8_t to_bcd(uint8_t val)
{
    return static_cast<uint8_t>((val / 10 << 4) | (val % 10));
}


std::chrono::time_point<std::chrono::system_clock> set_next_wakeup(unsigned wakeup_interval)
{
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto local = * std::localtime(& time_t_now);

    I2C_Device witty{ "/dev/i2c-1", 0x08 };

    std::array<uint8_t, 7> buffer_n{
        to_bcd(local.tm_sec),
        to_bcd(local.tm_min),
        to_bcd(local.tm_hour),
        to_bcd(local.tm_mday),
        to_bcd(local.tm_wday),
        to_bcd(local.tm_mon + 1),
        to_bcd(local.tm_year % 100)
    };

    // Witty does not support multi-register writes, so we need to stop RTC
    // to keep time consistency
    witty.write8(54, 0x40);  // Set STOP bit
    for( int i = 0; i < buffer_n.size(); ++i )
    {
        witty.write8(58 + i, buffer_n[i]);
    }
    witty.write8(54, 0x00);  // Clear STOP bit

    auto next = now + std::chrono::minutes(wakeup_interval);
    auto time_t_next = std::chrono::system_clock::to_time_t(next);
    auto local_next = * std::localtime(& time_t_next);

    std::array<uint8_t, 4> buffer_m{
        to_bcd(local_next.tm_sec),
        to_bcd(local_next.tm_min),
        to_bcd(local_next.tm_hour),
        to_bcd(local_next.tm_mday),
    };

    for( int i = 0; i < buffer_m.size(); ++i ) { witty.write8(27 + i, buffer_m[i]); }

    // Clear Alarms
    witty.write8(55, witty.read8(55) & 0xbf);
    witty.write8(39, 0);
    witty.write8(40, 0);

    // Pulse interval in seconds, when Raspberry Pi is off.
    witty.write8(18, 240);

    return next;
}


void shutdown()
{
    if( ! G_config.stay_awake )
    {
        system("sudo /usr/bin/systemctl poweroff -i"); // --force
    }
}


int main(int argc, char *argv[])
{
    G_config.load();
    G_config.print();

    // Set fallback wakeup in case something goes wrong
    // std::chrono::time_point<std::chrono::system_clock>
    auto next_wakeup = set_next_wakeup(G_config.fallback_wakeup_interval);

    try {
        RPiCamJpegApp app;
        StillOptions *options = app.GetOptions();
        if (options->Parse(argc, argv)) {
            if (options->Get().verbose >= 2) { options->Get().Print(); }

            MQTTAgent mqtt_agent{ G_config.get_mqtt_server(), G_config.get_mqtt_client_id() };

            mqtt_agent.connect();
            mqtt_agent.subscribe();

            try{
                std::string config_str = mqtt_agent.receive_config();
                nlohmann::json config = nlohmann::json::parse(config_str);

                G_config.update(config);
            }
            catch( std::exception const & e )
            {
                std::cerr << "Can not parse new config: " << e.what() << std::endl;
            }

            G_config.print();

            sensor_data_t sd { read_sensors() };

            TSL2591::TSL2591_Sensor light_sensor{"/dev/i2c-1"};

            std::basic_ostringstream<char> buff{ std::ios::binary };

            bool person_detected { false };

            if( sd.als_data < G_config.light_threshold )  // No light
            {
                light_sensor.enable_interrupt(G_config.light_threshold);
            }
            else{
                light_sensor.disable_interrupt();
                person_detected = capture_frame(app, buff);
                if( person_detected )
                {
                    next_wakeup = set_next_wakeup(G_config.person_wakeup_interval);
                }
                else{
                    next_wakeup = set_next_wakeup(G_config.wakeup_interval);
                }
            }

            // Create keys (text strings)
            constexpr uint8_t N{ 9 };
            cbor_item_t* key_cnode_id = cbor_build_string("cnode_id");
            cbor_item_t* key_created = cbor_build_string("created");
            cbor_item_t* key_frame = cbor_build_string("frame");
            cbor_item_t* key_temperature = cbor_build_string("temperature");
            cbor_item_t* key_battery_voltage = cbor_build_string("battery_voltage");
            cbor_item_t* key_current = cbor_build_string("current");
            cbor_item_t* key_light = cbor_build_string("ALS_DATA");
            cbor_item_t* key_person_detected = cbor_build_string("person_detected");
            cbor_item_t* key_next_wakeup = cbor_build_string("next_wakeup");

            // Create values

            cbor_item_t* val_cnode_id = cbor_build_string("1");
            cbor_item_t* val_created = cbor_build_string(
                get_iso_datetime(std::chrono::system_clock::now()).c_str()
            );
            // Binary data as byte string
            auto frame = buff.str();
            cbor_item_t* val_binary = cbor_build_bytestring(
                reinterpret_cast<cbor_data>(frame.c_str()),
                frame.length()
            );
            // Temperature
            cbor_item_t* val_temperature = cbor_build_float4(sd.temperature);
            // Battery
            cbor_item_t* val_battery_voltage = cbor_build_float4(sd.voltage);
            // Current
            cbor_item_t* val_current = cbor_build_float4(sd.current);
            // Light
            cbor_item_t* val_light = cbor_build_uint16(sd.als_data);
            // Person detected
            cbor_item_t* val_person_detected = cbor_build_bool(person_detected);
            // Next wakeup
            cbor_item_t* val_next_wakeup = cbor_build_string(get_iso_datetime(next_wakeup).c_str());

            cbor_item_t* map = cbor_new_definite_map(N);
            cbor_map_add(map, cbor_pair{ key_cnode_id, val_cnode_id });
            cbor_map_add(map, cbor_pair{ key_created, val_created });
            cbor_map_add(map, cbor_pair{ key_frame, val_binary });
            cbor_map_add(map, cbor_pair{ key_temperature, val_temperature });
            cbor_map_add(map, cbor_pair{ key_battery_voltage, val_battery_voltage });
            cbor_map_add(map, cbor_pair{ key_current, val_current });
            cbor_map_add(map, cbor_pair{ key_light, val_light });
            cbor_map_add(map, cbor_pair{ key_person_detected, val_person_detected });
            cbor_map_add(map, cbor_pair{ key_next_wakeup, val_next_wakeup });

            // Serialize the map
            unsigned char *buffer = nullptr;
            size_t buffer_size = 0;
            size_t serialized_length = cbor_serialize_alloc(map, &buffer, &buffer_size);

            if (serialized_length == 0) {
                std::cerr << "Serialization failed" << std::endl;
                return 1;
            }

            auto payload = std::string(
                reinterpret_cast<char *>(buffer),
                reinterpret_cast<char *>(buffer + buffer_size)
            );

            mqtt_agent.send(G_config.data_topic, payload);

            free(buffer);
            cbor_decref(&map);
        }
    }
    catch (std::exception const &e) {
        LOG_ERROR("ERROR: *** " << e.what() << " ***");
        return -1;
    }

    shutdown();

    return 0;
}
