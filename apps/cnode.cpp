/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_jpeg.cpp - minimal libcamera jpeg capture app.
 */

#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <cbor.h>
#include "mqtt/async_client.h"

#include "core/rpicam_app.hpp"
#include "core/still_options.hpp"

#include "image/image.hpp"

#include "post_processing_stages/object_detect.hpp"


struct sensor_data_t
{
    float temperature;
    float voltage;
    float current;
};


class callback : public virtual mqtt::callback {
public:
    void connection_lost(const std::string &cause) override
    {
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
            std::cout << "\tcause: " << cause << std::endl;
    }

    void delivery_complete(mqtt::delivery_token_ptr tok) override
    {
        std::cout << "\tDelivery complete for token: " << (tok ? tok->get_message_id() : -1) << std::endl;
    }
};

using namespace std::placeholders;
using libcamera::Stream;

class RPiCamJpegApp : public RPiCamApp {
public:
    RPiCamJpegApp() : RPiCamApp(std::make_unique<StillOptions>()) {}

    StillOptions *GetOptions() const { return static_cast<StillOptions *>(RPiCamApp::GetOptions()); }
};



std::basic_ostringstream<char> capture_frame(RPiCamJpegApp &app)
{
    StillOptions const *options = app.GetOptions();
    app.OpenCamera();
    app.ConfigureStill();
    app.StartCamera();
    auto start_time = std::chrono::high_resolution_clock::now();

    std::basic_ostringstream<char> buff {std::ios::binary};

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
            CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

            std::vector<Detection> detections;
            bool detected {false};
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

            std::cout << detected << std::endl;

            Stream *stream = app.StillStream();
            StreamInfo info = app.GetStreamInfo(stream);
            BufferReadSync r(&app, completed_request->buffers[stream]);
            const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
            jpeg_write(mem, info, completed_request->metadata, buff, app.CameraModel(), options);
            break;
        }
    }
    return buff;
}


void send_frame(RPiCamJpegApp &app, std::string &buff)
{
    using namespace std;

    const int  QOS = 1;

    StillOptions const * options = app.GetOptions();
/*
    // Note that we don't actually need to open the trust or key stores.
    // We just need a quick, portable way to check that they exist.
    {
        ifstream tstore(TRUST_STORE);
        if (!tstore) {
            cerr << "The trust store file does not exist: " << TRUST_STORE << endl;
            cerr << "  Get a copy from \"paho.mqtt.c/test/ssl/test-root-ca.crt\"" << endl;
            return 1;
        }

        ifstream kstore(KEY_STORE);
        if (!kstore) {
            cerr << "The key store file does not exist: " << KEY_STORE << endl;
            cerr << "  Get a copy from \"paho.mqtt.c/test/ssl/client.pem\"" << endl;
            return 1;
        }
    }
*/
    cout << "Initializing for server '" << options->Get().mqtt_host << "'..." << endl;
    mqtt::async_client client(options->Get().mqtt_host, options->Get().mqtt_client_id);

    callback cb;
    client.set_callback(cb);
/*
    // Build the connect options, including SSL and a LWT message.
    auto sslopts = mqtt::ssl_options_builder()
                       .trust_store(TRUST_STORE)
                       .key_store(KEY_STORE)
                       .error_handler([](const std::string &msg) { std::cerr << "SSL Error: " << msg << std::endl; })
                       .finalize();
*/
    auto connopts = mqtt::connect_options_builder()
                        .user_name("testuser")
                        .password("testpassword")
                        //.ssl(std::move(sslopts))
                        .finalize();

    cout << "  ...OK" << endl;

    try {
        // Connect using SSL/TLS
        cout << "\nConnecting..." << endl;
        mqtt::token_ptr conntok = client.connect(connopts);
        cout << "Waiting for the connection..." << endl;
        conntok->wait();
        cout << "  ...OK" << endl;

        // Send a message
        cout << "\nSending message to topic " << options->Get().mqtt_topic  << endl;
        auto msg = mqtt::make_message(options->Get().mqtt_topic, buff, QOS, false);
        client.publish(msg)->wait_for(20000);
        cout << "  ...OK" << endl;

        // Disconnect

        cout << "\nDisconnecting..." << endl;
        client.disconnect()->wait();
        cout << "  ...OK" << endl;
    }
    catch (const mqtt::exception &exc) {
        cerr << exc.what() << endl;
        return;
    }

    return;
}


std::string get_iso_datetime()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

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

    return sd;
}


int main(int argc, char *argv[])
{
    try {
        RPiCamJpegApp app;
        StillOptions *options = app.GetOptions();
        if (options->Parse(argc, argv)) {
            if (options->Get().verbose >= 2) { options->Get().Print(); }
            if (options->Get().output.empty()) {
                throw std::runtime_error("output file name required");
            }

            std::basic_ostringstream<char> buff = capture_frame(app);

            // Create keys (text strings)
            cbor_item_t* key_cnode_id = cbor_build_string("cnode_id");
            cbor_item_t* key_created = cbor_build_string("created");
            cbor_item_t* key_frame = cbor_build_string("frame");
            cbor_item_t* key_temperature = cbor_build_string("temperature");
            cbor_item_t* key_battery_voltage = cbor_build_string("battery_voltage");
            cbor_item_t* key_current = cbor_build_string("current");

            // Create values
            cbor_item_t* val_cnode_id = cbor_build_string("1");
            auto timestamp = get_iso_datetime();
            cbor_item_t* val_created = cbor_build_string(timestamp.c_str());
            // Binary data as byte string
            auto frame = buff.str();
            cbor_item_t* val_binary = cbor_build_bytestring(
                reinterpret_cast<cbor_data>(frame.c_str()),
                frame.length()
            );

            sensor_data_t sd { read_sensors() };
            // Temperature
            cbor_item_t* val_temperature = cbor_build_float4(sd.temperature);
            // Battery
            cbor_item_t* val_battery_voltage = cbor_build_float4(sd.voltage);
            // Current
            cbor_item_t* val_current = cbor_build_float4(sd.current);

            cbor_item_t* map = cbor_new_definite_map(6);

            cbor_map_add(map, (struct cbor_pair){ .key = key_cnode_id, .value = val_cnode_id });
            cbor_map_add(map, (struct cbor_pair){ .key = key_created, .value = val_created });
            cbor_map_add(map, (struct cbor_pair){ .key = key_frame, .value = val_binary });
            cbor_map_add(map, (struct cbor_pair){ .key = key_temperature, .value = val_temperature });
            cbor_map_add(map, (struct cbor_pair){ .key = key_battery_voltage, .value = val_battery_voltage });
            cbor_map_add(map, (struct cbor_pair){ .key = key_current, .value = val_current });

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

            send_frame(app, payload);

            free(buffer);
            cbor_decref(&map);
        }
    }
    catch (std::exception const &e) {
        LOG_ERROR("ERROR: *** " << e.what() << " ***");
        return -1;
    }
    return 0;
}
