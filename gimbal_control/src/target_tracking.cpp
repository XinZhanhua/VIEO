#include <ros/ros.h>
#include <iostream>
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/message_filter.h"
#include "tf2_msgs/TFMessage.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <serial/serial.h> 

long tf_times;
void tfdata_Callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) // Change the message type
{
    // 如果detections中没有内容，直接return出去不再进行串口发送
    if (msg->detections.empty()) {
        return;
    }

    // 解算/tf数据
    tf2::Quaternion quat;
    // 将四元数形式的姿态数据存放到一个float数组中
    for (const auto& detection : msg->detections)
    {
        quat.setX(detection.pose.pose.pose.orientation.x);
        quat.setY(detection.pose.pose.pose.orientation.y);
        quat.setZ(detection.pose.pose.pose.orientation.z);
        quat.setW(detection.pose.pose.pose.orientation.w);
    }
    
    // 将姿态数据的XYZW分别乘以10000并保留到整数
    int x = quat.x() * 10000;
    int y = quat.y() * 10000;
    int z = quat.z() * 10000;
    int w = quat.w() * 10000;

    // 分为高八位和低八位
    int x_high = (x >> 8) & 0xFF;
    int x_low = x & 0xFF;
    int y_high = (y >> 8) & 0xFF;
    int y_low = y & 0xFF;
    int z_high = (z >> 8) & 0xFF;
    int z_low = z & 0xFF;
    int w_high = (w >> 8) & 0xFF;
    int w_low = w & 0xFF;

    // 加上帧头
    int frame_header1 = 0xAA;
    int frame_header2 = 0xAF;
    int frame_header3 = 0x06;
    int frame_header4 = 0x14;
    int frame_header5 = 0x08;

    // 计算和校验
    int checksum = (frame_header1 + frame_header2 + frame_header3 + frame_header4 + frame_header5 + x_high + x_low + y_high + y_low + z_high + z_low + w_high + w_low) & 0xFF;

    // 发送数据到/dev/ttyUSB0串口
    serial::Serial serial_port;
    serial_port.setPort("/dev/ttyUSB0");
    serial_port.setBaudrate(921600);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);
    serial_port.setTimeout(timeout);
    try 
    {
        serial_port.open();
    } 
    catch (const std::exception& e) 
    {
        ROS_ERROR("Failed to open serial port: %s", e.what());
        return;
    }
    if(tf_times == 0)
        ROS_INFO("Serial port opened successfully");

    // 发送数据
    uint8_t data[] = {frame_header1, frame_header2, frame_header3, frame_header4, frame_header5, x_high, x_low, y_high, y_low, z_high, z_low, w_high, w_low, checksum};
    serial_port.write(data, sizeof(data));
    // ROS_INFO("Quaternion: %d %d %d %d %d %d %d %d", data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12]);

    // 关闭串口
    serial_port.close();
    tf_times++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_control");
    ros::NodeHandle nh;

    serial::Serial serial_port;
    serial_port.setPort("/dev/ttyUSB0");
    serial_port.setBaudrate(921600);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);
    serial_port.setTimeout(timeout);
    try 
    {
        serial_port.open();
    } 
    catch (const std::exception& e) 
    {
        ROS_ERROR("Failed to open serial port: %s", e.what());
    }
    uint8_t data[] = {0xAA, 0xAF, 0x05, 0x11, 0x02, 0x00, 0x00, 0x71};
    if(serial_port.isOpen())
    {
        serial_port.write(data, sizeof(data));
        serial_port.close();
        ROS_INFO("Set position to zero");
    }
    
    

    // 创建一个订阅器，订阅/topic为/tf的消息
    ros::Subscriber sub = nh.subscribe("/tag_detections", 10, tfdata_Callback);
    
    setlocale(LC_ALL, "");
    ROS_INFO("启动电机位置闭环控制节点");

    ros::spin();

    return 0;
}
