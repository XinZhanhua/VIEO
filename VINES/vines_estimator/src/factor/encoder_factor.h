#pragma once
#include <ros/assert.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../utility/utility.h"
#include "../parameters.h"

#include <ceres/ceres.h>

class EncoderFactor : public ceres::SizedCostFunction<3, 7, 7>
{
  public:
    EncoderFactor() = delete;
    EncoderFactor(double _angle_i, double _angle_j) 
    : angle_i(_angle_i), angle_j(_angle_j)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        //printf("helloworld \r\n");
        Eigen::Vector3d tic_i(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond qic_i(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d tic_j(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond qic_j(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Map<Eigen::Vector3d> residual(residuals);
        // double derta_theta = angle_j - angle_i;
        // double k = 1 / sqrt(1 + derta_theta * derta_theta / 4);
        // Eigen::Vector3d xyz = 1/2 * derta_theta * k * axis[0];
        // Eigen::Quaterniond derta_q(k, xyz.x(), xyz.y(), xyz.z());
        // residual.block<3, 1>(0, 0) = tic_j - tic_i + Utility::Rodrigues(axis[0], angle_i) * derta_theta * Utility::skewSymmetric(TEC[0]) * axis[0];
        // residual.block<3, 1>(3, 0) = 2 * (qic_j * qic_i.inverse() * derta_q.inverse()).vec();
        // Eigen::Vector3d p = {1, 1, 1};
        Eigen::Matrix3d ric_i = qic_i.toRotationMatrix();
        Eigen::Matrix3d ric_j = qic_j.toRotationMatrix();
        // Eigen::Matrix3d ric_i = Utility::Rodrigues(axis[0], angle_i) * rie_i;
        // Eigen::Matrix3d ric_j = Utility::Rodrigues(axis[0], angle_j) * rie_j;

        double derta_angle;
        if (fabs(angle_j - angle_i) < CV_PI)
            derta_angle = angle_j - angle_i;
        else if(angle_j > angle_i)
            derta_angle = angle_j - CV_2PI - angle_i;
        else
            derta_angle = angle_j + CV_2PI - angle_i;

        Eigen::Vector3d p = {1, 1, 1};
        double k = 1;
        residual = (ric_j*ric_i.transpose() - Utility::Rodrigues(axis[0], derta_angle))*p; // tr(R)=1+cos(2*theta)
        // residual.block<3, 1>(0, 0) = tic_i - (tie_i + Utility::Rodrigues(axis[0], angle_i) * TEC[0]);
        // residual.block<3, 1>(3, 0) = tic_j - (tie_j + Utility::Rodrigues(axis[0], angle_j) * TEC[0]);
        // residual.block<3, 1>(6, 0) = ric_i * p - (Utility::Rodrigues(axis[0], angle_i) * rie_i) * p;
        // residual.block<3, 1>(9, 0) = ric_j * p - (Utility::Rodrigues(axis[0], angle_j) * rie_j) * p;
        if (jacobians)
        {
            // if (jacobians[0])
            // {
            //     Eigen::Map<Eigen::Matrix<double, 12, 7, Eigen::RowMajor>> jacobian_ex_pose_i(jacobians[0]);
            //     jacobian_ex_pose_i.setZero();

            //     jacobian_ex_pose_i.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
            //     //jacobian_ex_pose_i.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
            //     jacobian_ex_pose_i.block<3, 3>(6, 3) = Utility::Rodrigues(axis[0], angle_i) * rie_i * Utility::skewSymmetric(p);
            // }
            // if (jacobians[1])
            // {
            //     Eigen::Map<Eigen::Matrix<double, 12, 7, Eigen::RowMajor>> jacobian_ex_pose_j(jacobians[1]);
            //     jacobian_ex_pose_j.setZero();

            //     jacobian_ex_pose_j.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
            //     //jacobian_ex_pose_i.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
            //     jacobian_ex_pose_j.block<3, 3>(9, 3) = Utility::Rodrigues(axis[0], angle_j) * rie_j * Utility::skewSymmetric(p);
            // }
                        
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_ex_pose_i(jacobians[0]);
                //jacobian_ex_pose_i.setZero();
                Eigen::Matrix<double, 3, 6> jaco_i;

                jaco_i.leftCols<3>() = k * Eigen::Matrix3d::Identity();
                jaco_i.rightCols<3>() = k * ric_j * Utility::skewSymmetric(ric_i.transpose() * p);

                jacobian_ex_pose_i.leftCols<6>() = jaco_i;
                jacobian_ex_pose_i.rightCols<1>().setZero();
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_ex_pose_j(jacobians[1]);
                //jacobian_ex_pose_j.setZero();
                Eigen::Matrix<double, 3, 6> jaco_j;

                jaco_j.leftCols<3>() = -k * Eigen::Matrix3d::Identity();
                jaco_j.rightCols<3>() = -k * ric_j * Utility::skewSymmetric(ric_i.transpose() * p);

                jacobian_ex_pose_j.leftCols<6>() = jaco_j;
                jacobian_ex_pose_j.rightCols<1>().setZero();
            }

            // if (jacobians[1])
            // {
            //     Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_ex_pose_j(jacobians[1]);
            //     jacobian_ex_pose_j.setZero();

            //     jacobian_ex_pose_j.block<3, 3>(0, 3) = -(Utility::Qleft(derta_q * qic_i) * Utility::Qright(qic_j)).bottomRightCorner<3, 3>();
            //     jacobian_ex_pose_j.block<3, 3>(3, 3) = (Utility::Qleft(derta_q * qic_i) * Utility::Qright(qic_j)).bottomRightCorner<3, 3>();
            // }
        }
        //printf("parameters[0][0]:%.3f \r\n", parameters[0][0]);
    }
    double angle_i, angle_j;
};
//， const Eigen::Vector3d &_tic_i， const Eigen::Vector3d &__tic_j     , tic_i(_tic_i), tic_j(_tic_j)

class EncoderCostFunctor {
public:
  EncoderCostFunctor(double angle_): angle(angle_) {}
  // 形参: 输入参数块, 残差
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> tic_(x);
    Eigen::Matrix<T, 3, 1> tic = tic_;
    Eigen::Map<const Eigen::Matrix<T, 4, 1>> qic_coeffs(x + 3); // 假设 x 的前三个元素用于其他目的  
    Eigen::Quaternion<T> qic(qic_coeffs);  
    Eigen::Matrix<T, 3, 3> ric = qic.matrix();
    // Eigen::Quaterniond qic(x[6], x[3], x[4], x[5]);
    // Eigen::Matrix3d ric = qic.toRotationMatrix();
    double k = 1;
    // residual[0] = angle - k * acos(((ric * RIE[0].transpose()).trace() - T(1.0)) * T(0.5)) - (1 - k) * acos((((tic - TIE[0]).normalized() * (TEC[0].normalized().transpose())).trace() - T(1.0)) * T(0.5));
    Eigen::Matrix<T, 3, 1> rvec;
    Eigen::Matrix<T, 3, 3> RIC_T = ric * RIE[0].transpose();
    ceres::RotationMatrixToAngleAxis(RIC_T.data(), rvec.data());
    //double length = (double)ceres::sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);
    cout << rvec << endl;
    // cout << "angle: " << angle << "        rvec" << sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]) << endl;
    residual[0] = T(-1000.0)*(angle - (T)ceres::sqrt(rvec(0)*rvec(0) + rvec(1)*rvec(1) + rvec(2)*rvec(2)));
    return true;
  }

 private:
  double angle;
};
