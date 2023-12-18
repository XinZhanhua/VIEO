#include "calibrate_gimbal.h"
#include "parameters.h"

#include <stdio.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <tf/transform_listener.h>
#include "tf2/LinearMath/Quaternion.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h> 
#include <cv_bridge/cv_bridge.h>
#include "apriltag_ros/AprilTagDetectionArray.h"

gimbal_calibrator calibrator;

std::condition_variable con;
double current_time = -1;
std::queue<std_msgs::Header> header_buf;

double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

void encoder_callback(const std_msgs::Header::ConstPtr& msg)
{
    std::string frame_id = msg->frame_id;
    calibrator.encoder_data = std::stoi(frame_id);
}

void image_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    //检查是否为首帧
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }

    // detect unstable camera stream
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        //pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();

    //完成ROS的sensor_msgs::Image向cv::Mat的格式转换
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat image = ptr->image;
}

void tf_callback(const tf::tfMessage::ConstPtr& msg)
{
    for (const auto& transform : msg->transforms)
    {
        if (transform.header.frame_id == "/usb_cam")
        {
            // translation represents position
            Eigen::Vector3d translation(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
            // rotation represents quaternion pose
            Eigen::Quaterniond rotation(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
            // process the translation and rotation data
        }
    }
}

void tfdata_Callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    if (msg->detections.empty()) 
    {
        return;
    }

    tf2::Quaternion quat;// 解算/tf数据
    for (const auto& detection : msg->detections)
    {
        quat.setX(detection.pose.pose.pose.orientation.x);
        quat.setY(detection.pose.pose.pose.orientation.y);
        quat.setZ(detection.pose.pose.pose.orientation.z);
        quat.setW(detection.pose.pose.pose.orientation.w);
    }
}

void process()
{
    while (true)
    {
    }
}

void registerPub(ros::NodeHandle &n)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_calibrator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    calibrator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    //ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_encoder = n.subscribe(ENCODER_TOPIC, 2000, encoder_callback);
    //ros::Subscriber sub_image = n.subscribe<sensor_msgs::Image>(IMAGE_TOPIC, 2000, image_callback); // Added <sensor_msgs::Image> template argument
    //ros::Subscriber sub_tf = n.subscribe("/tf", 2000, tf_callback);
    ros::Subscriber sub = n.subscribe("/tag_detections", 10, tfdata_Callback);

    std::thread measurement_process{process};
    ros::spin();

    return 0;
}







