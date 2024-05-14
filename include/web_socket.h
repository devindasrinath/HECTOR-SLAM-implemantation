#include <iostream>
#include <array>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <rapidjson/document.h>

typedef websocketpp::client<websocketpp::config::asio_client> client;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

#define NUM_DATA 505
#define DATA_RECEIVED 1

class ROSBridgeClient {
public:
    std::atomic<bool> message_received;
    ROSBridgeClient() {
        // Initialize the WebSocket client
        m_client.init_asio();
        
        // Set up the connection handler
        m_client.set_open_handler(bind(&ROSBridgeClient::on_open, this, _1));
        
        // Set up the message handler
        m_client.set_message_handler(bind(&ROSBridgeClient::on_message, this, _1, _2));
        message_received=false;
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
    const std::array<double, NUM_DATA>& getRanges() const {
        return ranges;
    }

    const std::array<double, NUM_DATA>& getAngles() const {
        return angles;
    }

    void reset_status(){
        message_received=false;
    }

    bool get_status(){
        return message_received;
    }

private:
; // Example size of angles array

    client m_client;
    double angle_min;
    double angle_max;
    double angle_increment;
    std::array<double, NUM_DATA> ranges;
    std::array<double, NUM_DATA> angles;

    void on_open(websocketpp::connection_hdl hdl) {
        std::cout << "Connected to ROSBridge server" << std::endl;
        // Subscribe to the desired ROS topic
        std::string subscribe_msg = "{\"op\":\"subscribe\",\"topic\":\"/scan\"}";
        m_client.send(hdl, subscribe_msg, websocketpp::frame::opcode::text);
    }

    void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg) {
        std::string payload = msg->get_payload();

        // Parse the JSON message
        rapidjson::Document doc;
        doc.Parse(payload.c_str());

        // Check if parsing succeeded and if the document is an object
        if (!doc.IsObject()) {
            std::cout << "Failed to parse JSON" << std::endl;
            return;
        }

        // Extract the "msg" field from the JSON document
        if (doc.HasMember("msg") && doc["msg"].IsObject()) {
            const rapidjson::Value& msg_obj = doc["msg"];

            // Extract angle_min, angle_max, and angle_increment fields
            if (msg_obj.HasMember("angle_min") && msg_obj["angle_min"].IsDouble() &&
                msg_obj.HasMember("angle_max") && msg_obj["angle_max"].IsDouble() &&
                msg_obj.HasMember("angle_increment") && msg_obj["angle_increment"].IsDouble()) {
                angle_min = msg_obj["angle_min"].GetDouble();
                angle_max = msg_obj["angle_max"].GetDouble();
                angle_increment = msg_obj["angle_increment"].GetDouble();
                //std::cout << "Angle Min: " << angle_min << ", Angle Max: " << angle_max << ", Angle Increment: " << angle_increment << std::endl;
            }

            // Extract the "ranges" field
            if (msg_obj.HasMember("ranges") && msg_obj["ranges"].IsArray()) {
                const rapidjson::Value& ranges_array = msg_obj["ranges"];
                if (ranges_array.Size() != NUM_DATA) {
                    //std::cout << "Received ranges size does not match expected size. : "<< ranges_array.Size() << std::endl;
                    return;
                }
                // auto start1 = std::chrono::high_resolution_clock::now();  
                for (size_t i = 0; i < NUM_DATA; i++) {
                    if (ranges_array[i].IsDouble()) {
                        ranges[i] = ranges_array[i].GetDouble()*40;
                        //std::cout << "Range " << i << ": " << ranges[i] << std::endl;
                    }
                }
                // auto start2 = std::chrono::high_resolution_clock::now();
                // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(start2 - start1).count();
                // std::cout << "Execution time : " << duration << " ms" << std::endl;
                // Generate the array of angles
                generateAngles();
            }
        }
        message_received = true;
        //std::cout<<"message_received : "<<message_received<<std::endl;
    }

    void generateAngles() {
        angles.fill(0.0); // Clear the previous data
        double current_angle = angle_min;
        for (size_t i = 0; i < NUM_DATA; i++) {
            angles[i] = current_angle;
            //std::cout << "angle " << i << ": " << current_angle << std::endl;
            current_angle += angle_increment;
        }
    }
};
