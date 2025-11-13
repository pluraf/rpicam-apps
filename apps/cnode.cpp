/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_jpeg.cpp - minimal libcamera jpeg capture app.
 */

#include <chrono>

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

// The main event loop for the application.

void capture_frame(RPiCamJpegApp &app)
{
    StillOptions const *options = app.GetOptions();
    app.OpenCamera();
    app.ConfigureViewfinder();
    app.StartCamera();
    auto start_time = std::chrono::high_resolution_clock::now();

    while (true) {
        RPiCamApp::Msg msg = app.Wait();
        if (msg.type == RPiCamApp::MsgType::Timeout) {
            LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
            app.StopCamera();
            app.StartCamera();
            continue;
        }

        if (msg.type == RPiCamApp::MsgType::Quit) {
            return;
        }
        else if (msg.type != RPiCamApp::MsgType::RequestComplete) {
            throw std::runtime_error("unrecognised message!");
        }
        else {
            app.StopCamera();
            LOG(1, "Still capture image received");

            Stream *stream = app.StillStream();
            StreamInfo info = app.GetStreamInfo(stream);
            CompletedRequestPtr &payload = std::get<CompletedRequestPtr>(msg.payload);
            BufferReadSync r(&app, payload->buffers[stream]);
            const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
            jpeg_save(mem, info, payload->metadata, options->Get().output, app.CameraModel(), options);
            return;
        }
    }
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

            capture_frame(app);

            options->Get().output
        }
    }
    catch (std::exception const &e) {
        LOG_ERROR("ERROR: *** " << e.what() << " ***");
        return -1;
    }
    return 0;
}


const std::string LWT_TOPIC				{ "events/disconnect" };
const std::string LWT_PAYLOAD			{ "Last will and testament." };

const int  QOS = 1;
const auto TIMEOUT = std::chrono::seconds(10);


void send_frame(
    std::string const & broker_url,
    std::string const & client_id,
    std::string const & frame_file_path)
{
    using namespace std;
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
    cout << "Initializing for server '" << broker_url << "'..." << endl;
    mqtt::async_client client(broker_url, client_id);

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
    auto willmsg = mqtt::message(LWT_TOPIC, LWT_PAYLOAD, QOS, true);

    auto connopts = mqtt::connect_options_builder()
                        .user_name("testuser")
                        .password("testpassword")
                        .will(std::move(willmsg))
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

        cout << "\nSending message..." << endl;
        auto msg = mqtt::make_message("hello", "Hello secure C++ world!", QOS, false);
        client.publish(msg)->wait_for(TIMEOUT);
        cout << "  ...OK" << endl;

        // Disconnect

        cout << "\nDisconnecting..." << endl;
        client.disconnect()->wait();
        cout << "  ...OK" << endl;
    }
    catch (const mqtt::exception &exc) {
        cerr << exc.what() << endl;
        return 1;
    }

    return 0;
}
