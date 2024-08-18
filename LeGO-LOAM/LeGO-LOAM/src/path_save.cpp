#include "utility.h"

void path_save(nav_msgs::Odometry odomAftMapped )
{
 
	    //保存轨迹，path_save是文件目录,txt文件提前建好,/home/xxx/xxx.txt,
   			std::ofstream pose1("/home/huarsuma/result/result_backup/ground_truth.csv", std::ios::app);
			pose1.setf(std::ios::fixed, std::ios::floatfield);
            pose1.precision(0);
            pose1 << odomAftMapped.header.stamp.toSec() * 1e9 << ",";
            pose1.precision(5);
			// pose1.precision(9);
	
			// static double timeStart = odomAftMapped.header.stamp.toSec();
			// auto T1 =ros::Time().fromSec(timeStart) ;
			pose1  << odomAftMapped.pose.pose.position.x << ","
              << odomAftMapped.pose.pose.position.y << ","
              << odomAftMapped.pose.pose.position.z << ","
              << odomAftMapped.pose.pose.orientation.x << ","
              << odomAftMapped.pose.pose.orientation.y << ","
              << odomAftMapped.pose.pose.orientation.z << ","
              << odomAftMapped.pose.pose.orientation.w << "," << std::endl;
			pose1.close();
            
}
 
int main(int argc, char **argv){
    ros::init(argc, argv, "path_save");
    ros::NodeHandle nh;
    std::ofstream pose1("/home/huarsuma/result/result_backup/ground_truth.csv", std::ios::trunc);
    ros::Subscriber save_path = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init",     100, path_save);	    //保存轨迹，a_loam直接订阅话题/aft_mapped_to_init。
 
    ros::spin();
     }