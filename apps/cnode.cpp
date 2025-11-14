/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_jpeg.cpp - minimal libcamera jpeg capture app.
 */

#include <chrono>
#include <fstream>
#include <sstream>

#include "mqtt/async_client.h"

#include "core/rpicam_app.hpp"
#include "core/still_options.hpp"

#include "image/image.hpp"

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
    //app.ConfigureViewfinder();
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

            Stream *stream = app.StillStream();
            StreamInfo info = app.GetStreamInfo(stream);
            CompletedRequestPtr &payload = std::get<CompletedRequestPtr>(msg.payload);
            BufferReadSync r(&app, payload->buffers[stream]);
            const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
            jpeg_write(mem, info, payload->metadata, buff, app.CameraModel(), options);
            break;
        }
    }
    return buff;
}


void send_frame(RPiCamJpegApp & app, std::basic_ostringstream<char> & buff)
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
        auto msg = mqtt::make_message(options->Get().mqtt_topic, buff.str(), QOS, false);
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
            send_frame(app, buff);

            //options->Get().output
        }
    }
    catch (std::exception const &e) {
        LOG_ERROR("ERROR: *** " << e.what() << " ***");
        return -1;
    }
    return 0;
}
