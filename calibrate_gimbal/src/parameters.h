#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <filesystem>

const int WINDOW_SIZE = 10;

extern std::string IMU_TOPIC;
extern std::string ENCODER_TOPIC;
extern std::string IMAGE_TOPIC;
extern std::string CALIB_RESULT_PATH;
extern double ROW, COL;

void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};