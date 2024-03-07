#pragma once
#include <ros/assert.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../utility/utility.h"
#include "../parameters.h"

#include <ceres/ceres.h>

class EncoderFactor : public ceres::SizedCostFunction<12, 7, 7>
{
  public:
    EncoderFactor() = delete;
    EncoderFactor(double _angle_i, double _angle_j, const Eigen::Vector3d &_tic_i, const Eigen::Vector3d &_tic_j, const Eigen::Matrix3d &_ric_i, const Eigen::Matrix3d &_ric_j) 
    : angle_i(_angle_i), angle_j(_angle_j), tic_i(_tic_i), tic_j(_tic_j), ric_i(_ric_i), ric_j(_ric_j)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Vector3d tie_i(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond qie_i(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d tie_j(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond qie_j(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Map<Eigen::Matrix<double, 12, 1>> residual(residuals);
        // double derta_theta = angle_j - angle_i;
        // double k = 1 / sqrt(1 + derta_theta * derta_theta / 4);
        // Eigen::Vector3d xyz = 1/2 * derta_theta * k * axis[0];
        // Eigen::Quaterniond derta_q(k, xyz.x(), xyz.y(), xyz.z());
        // residual.block<3, 1>(0, 0) = tic_j - tic_i + Utility::Rodrigues(axis[0], angle_i) * derta_theta * Utility::skewSymmetric(TEC[0]) * axis[0];
        // residual.block<3, 1>(3, 0) = 2 * (qic_j * qic_i.inverse() * derta_q.inverse()).vec();
        Eigen::Vector3d p = {1, 1, 1};
        Eigen::Matrix3d rie_i = qie_i.toRotationMatrix();
        Eigen::Matrix3d rie_j = qie_j.toRotationMatrix();
        residual.block<3, 1>(0, 0) = tic_i - (tie_i + Utility::Rodrigues(axis[0], angle_i) * TEC[0]);
        residual.block<3, 1>(3, 0) = tic_j - (tie_j + Utility::Rodrigues(axis[0], angle_j) * TEC[0]);
        residual.block<3, 1>(6, 0) = ric_i * p - (Utility::Rodrigues(axis[0], angle_i) * rie_i) * p;
        residual.block<3, 1>(9, 0) = ric_j * p - (Utility::Rodrigues(axis[0], angle_j) * rie_j) * p;
        if (jacobians)
        {

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 12, 7, Eigen::RowMajor>> jacobian_ex_pose_i(jacobians[0]);
                jacobian_ex_pose_i.setZero();

                jacobian_ex_pose_i.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
                //jacobian_ex_pose_i.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
                jacobian_ex_pose_i.block<3, 3>(6, 3) = Utility::Rodrigues(axis[0], angle_i) * rie_i * Utility::skewSymmetric(p);
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 12, 7, Eigen::RowMajor>> jacobian_ex_pose_j(jacobians[1]);
                jacobian_ex_pose_j.setZero();

                jacobian_ex_pose_j.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
                //jacobian_ex_pose_i.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
                jacobian_ex_pose_j.block<3, 3>(9, 3) = Utility::Rodrigues(axis[0], angle_j) * rie_j * Utility::skewSymmetric(p);
            }

            // if (jacobians[1])
            // {
            //     Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_ex_pose_j(jacobians[1]);
            //     jacobian_ex_pose_j.setZero();

            //     jacobian_ex_pose_j.block<3, 3>(0, 3) = -(Utility::Qleft(derta_q * qic_i) * Utility::Qright(qic_j)).bottomRightCorner<3, 3>();
            //     jacobian_ex_pose_j.block<3, 3>(3, 3) = (Utility::Qleft(derta_q * qic_i) * Utility::Qright(qic_j)).bottomRightCorner<3, 3>();
            // }
        }
    }
    double angle_i, angle_j;
    Eigen::Vector3d tic_i, tic_j;
    Eigen::Matrix3d ric_i, ric_j;
};
//， const Eigen::Vector3d &_tic_i， const Eigen::Vector3d &__tic_j     , tic_i(_tic_i), tic_j(_tic_j)
