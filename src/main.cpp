#include <iostream>
#include <thread>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include "Vanttec_CANLib/Utils/CANDeserialization.h"
#include "Vanttec_CANLib/Utils/CANSerialization.h"
#include "Vanttec_CANLib_Linux/CANHandler.h"

std::vector<float> lastArray{0,0,0,0,0,0,0,0};
std::stringstream debugLogBuffer;

void motorCallback(vanttec::CANHandler &handler, const std_msgs::Float32MultiArrayConstPtr& msg){
    if(msg->data.size() != 8)
        ROS_WARN("Invalid motor array size!");

    for(int i = 0; i < msg->data.size(); i++){
        if(msg->data[i] == lastArray[i]) continue;
        ROS_INFO("Sending motor message");
        vanttec::CANMessage canMsg;
        uint8_t id = 0x15 + i;
        vanttec::packFloat(canMsg, id, msg->data[i]);
        handler.write(canMsg);
    }

    lastArray = msg->data;
}

void handleDebugMsg(can_frame frame){
    if (frame.can_dlc - 1 <= 0)
        return;
    if(debugLogBuffer.str().size() > 10000){
        ROS_ERROR("Debug log has not been flushed with \n!, Clearing buffer...");
        debugLogBuffer.str(std::string()); //Clear buffer
    }
    for(int i = 1; i < frame.can_dlc; i++){
        char newChar = frame.data[i];
        if(newChar == '\n'){
            //Flush current buffer
            ROS_INFO("%s", debugLogBuffer.str().c_str());
            debugLogBuffer.str(std::string()); //Clear buffer
        } else {
            debugLogBuffer << newChar;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vanttec_can_bus_comms_node");
    ros::NodeHandle n;

    vanttec::CANHandler handler("can0");

    auto can_bus_ros_parser = [&](uint8_t id, can_frame frame){
        ROS_INFO("Recieved CAN MSG from id %d", id);
    };

    //handler.register_parser(can_bus_ros_parser);

    handler.register_parser(0x14, handleDebugMsg);

    // auto can_bus_parser = [&](uint8_t id, can_frame frame){
    //     if(id == 0x14) handleDebugMsg(frame);
    // };

    // //handler.register_parser(can_bus_parser);

    std::atomic<bool> runThreads{true};

    std::thread update_read_thread([&]{
        while(runThreads.load() && ros::ok()) handler.update_read();
    });

    std::thread update_write_thread([&]{
        while(runThreads.load() && ros::ok()) handler.update_write();
    });

    std::thread hb_write_thread([&]{
        ros::Rate rate(1); //1Hz
        while(runThreads.load() && ros::ok()){
            vanttec::CANMessage msg;
            msg.len = can_pack_byte(0x1E, 0xFF, msg.data, msg.len);
            handler.write(msg);
            rate.sleep();
        }
    });

    auto motorSub = n.subscribe<std_msgs::Float32MultiArray>("motors", 10, [&](const std_msgs::Float32MultiArrayConstPtr &msg){
        motorCallback(handler, msg);
    });

    auto hbPub = n.advertise<std_msgs::Int16>("stm32_heartbeat", 100);

    handler.register_parser(0x01, [hbPub](can_frame frame){
        std_msgs::Int16 msg;
        auto data = can_parse_short(frame.data, frame.can_dlc);
        msg.data = data;
        hbPub.publish(msg);
    });

    ros::spin();
    runThreads.store(true);
    update_read_thread.join();
    update_write_thread.join();
    hb_write_thread.join();

    return 0;
}