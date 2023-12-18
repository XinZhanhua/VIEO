#include <ros/ros.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>

class PoseRecorder
{
public:
  PoseRecorder(std::string _output_file) :output_file(_output_file)
  {
    // 初始化ROS节点
    ros::NodeHandle nh("~");
    std::ofstream file(output_file);
    file.close();

    // 订阅位姿话题
    pose_sub_ = nh.subscribe("/slam_out_pose", 10, &PoseRecorder::poseCallback, this);
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    // 打开CSV文件，以app模式写入数据
    std::ofstream file(output_file, std::ios::app);

    // 获取位姿数据 将位姿数据写入CSV文件
    file.setf(std::ios::fixed, std::ios::floatfield);
    file.precision(0);
    long long timestamp = pose_msg->header.stamp.toNSec();
    file << timestamp;
    file.precision(5);
    double x = pose_msg->pose.position.x;
    double y = pose_msg->pose.position.y;
    file << "," << x << "," << y << "," << std::endl;
  
    // 关闭文件
    file.close();
  }

private:
  std::string output_file;
  ros::Subscriber pose_sub_;
};

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "pose_recorder");

  // 获取参数
  std::string output_file;
  if (argc > 1) {
    output_file = argv[1];
  } else {
    ROS_ERROR("No output file specified");
    return 1;
  }

  // 创建PoseRecorder对象
  PoseRecorder pose_recorder(output_file);

  // 进入ROS循环
  ros::spin();

  return 0;
}