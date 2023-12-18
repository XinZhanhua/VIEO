#include "parameters.h"

namespace FileSystemHelper = std::filesystem;

std::string IMU_TOPIC;
std::string ENCODER_TOPIC;
std::string IMAGE_TOPIC;
std::string CALIB_RESULT_PATH;
double ROW, COL;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["encoder_topic"] >> ENCODER_TOPIC;
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    
    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    CALIB_RESULT_PATH = OUTPUT_PATH + "/calib_result.csv";
    FileSystemHelper::create_directory(OUTPUT_PATH);// create folder if not exists
    std::ofstream fout(CALIB_RESULT_PATH, std::ios::out);
    fout.close();

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    fsSettings.release();
}
