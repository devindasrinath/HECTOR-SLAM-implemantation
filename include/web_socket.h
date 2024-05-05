#include <iostream>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <rapidjson/document.h>

typedef websocketpp::client<websocketpp::config::asio_client> client;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

class ROSBridgeClient {
public:
    ROSBridgeClient() {
        // Initialize the WebSocket client
        m_client.init_asio();
        
        // Set up the connection handler
        m_client.set_open_handler(bind(&ROSBridgeClient::on_open, this, _1));
        
        // Set up the message handler
        m_client.set_message_handler(bind(&ROSBridgeClient::on_message, this, _1, _2));
    }

    void connect(std::string uri) {
        websocketpp::lib::error_code ec;
        client::connection_ptr con = m_client.get_connection(uri, ec);
        if (ec) {
            std::cout << "Error: " << ec.message() << std::endl;
            return;
        }
        m_client.connect(con);
        m_client.run();
    }

private:
    client m_client;

    void on_open(websocketpp::connection_hdl hdl) {
        std::cout << "Connected to ROSBridge server" << std::endl;
        // Subscribe to the desired ROS topic
        std::string subscribe_msg = "{\"op\":\"subscribe\",\"topic\":\"/your_topic\"}";
        m_client.send(hdl, subscribe_msg, websocketpp::frame::opcode::text);
    }

    void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg) {
        std::string payload = msg->get_payload();
        std::cout << "Received message: " << payload << std::endl;

        // Parse the JSON message
        rapidjson::Document doc;
        doc.Parse(payload.c_str());

        // Check if parsing succeeded and if the document is an object
        if (!doc.IsObject()) {
            std::cout << "Failed to parse JSON" << std::endl;
            return;
        }

        // Extract specific fields from the JSON document
        if (doc.HasMember("field1") && doc["field1"].IsString()) {
            std::string field1 = doc["field1"].GetString();
            std::cout << "Field1: " << field1 << std::endl;
        }

        if (doc.HasMember("field2") && doc["field2"].IsInt()) {
            int field2 = doc["field2"].GetInt();
            std::cout << "Field2: " << field2 << std::endl;
        }

        // Extract more fields as needed
    }
};

int main() {
    ROSBridgeClient client;
    client.connect("ws://localhost:9090"); // Replace with your ROSBridge server URI
    return 0;
}
