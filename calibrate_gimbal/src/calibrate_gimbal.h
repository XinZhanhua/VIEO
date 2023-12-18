#pragma once

#include "parameters.h"

#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>



class gimbal_calibrator
{
  public:
    gimbal_calibrator();
    void setParameter();
    
    int encoder_data;
};
