#pragma once

#include "../common_lib.hpp"
#include "sophus/so3.hpp"

class IMUIntegrator
{
public:
   IMUIntegrator()
   {
      Reset();
      noise.setZero();
      noise.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyr_n * gyr_n;
      noise.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * acc_n * acc_n;
      noise.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * gyr_w * gyr_w;
      noise.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * acc_w * acc_w;
   }

  /** \brief constructor of IMUIntegrator
     * \param[in] vIMU: IMU messages need to be integrated
     */
   IMUIntegrator(std::vector<sensor_msgs::ImuConstPtr> vIMU) : vimuMsg(std::move(vIMU))
   {
      Reset();
      noise.setZero();
      noise.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyr_n * gyr_n;
      noise.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * acc_n * acc_n;
      noise.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * gyr_w * gyr_w;
      noise.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * acc_w * acc_w;
   }

   void Reset()
   {
      dq.setIdentity();
      dp.setZero();
      dv.setZero();
      dtime = 0;
      covariance.setZero();
      jacobian.setIdentity();
      linearized_bg.setZero();
      linearized_ba.setZero();
   }

  /** \brief get delta quaternion after IMU integration
     */
  const Eigen::Quaterniond &GetDeltaQ() const { return dq; }

  /** \brief get delta displacement after IMU integration
     */
  const Eigen::Vector3d &GetDeltaP() const { return dp; }

  /** \brief get delta velocity after IMU integration
     */
  const Eigen::Vector3d &GetDeltaV() const { return dv; }

  /** \brief get time span after IMU integration
      */
  const double &GetDeltaTime() const { return dtime; }

  /** \brief get linearized bias gyr
			*/
  const Eigen::Vector3d &GetBiasGyr() const { return linearized_bg; }

  /** \brief get linearized bias acc
	   */
  const Eigen::Vector3d &GetBiasAcc() const { return linearized_ba; }

  /** \brief get covariance matrix after IMU integration
     */
  const Eigen::Matrix<double, 15, 15> &GetCovariance() { return covariance; }

  /** \brief get jacobian matrix after IMU integration
      */
  const Eigen::Matrix<double, 15, 15> &GetJacobian() const { return jacobian; }

  /** \brief get average acceleration of IMU messages for initialization
     */
   Eigen::Vector3d GetAverageAcc()
   {
      int i = 0;
      Eigen::Vector3d sum_acc(0, 0, 0);
      for (auto &imu : vimuMsg)
      {
         Eigen::Vector3d acc;
         acc << imu->linear_acceleration.x * gnorm,
            imu->linear_acceleration.y * gnorm,
            imu->linear_acceleration.z * gnorm;
         sum_acc += acc;
         i++;
         if (i > 30)
            break;
      }
      return sum_acc / i;
   }

  /** \brief push IMU message to the IMU buffer vimuMsg
     * \param[in] imu: the IMU message need to be pushed
     */
   void PushIMUMsg(const sensor_msgs::ImuConstPtr &imu)
   {
      vimuMsg.push_back(imu);
   }
   void PushIMUMsg(const std::vector<sensor_msgs::ImuConstPtr> &vimu)
   {
      vimuMsg.insert(vimuMsg.end(), vimu.begin(), vimu.end());
   }
  const std::vector<sensor_msgs::ImuConstPtr> &GetIMUMsg() const { return vimuMsg; }

  /** \brief only integrate gyro information of each IMU message stored in vimuMsg
     * \param[in] lastTime: the left time boundary of vimuMsg
     */
   void GyroIntegration(double lastTime)
   {
      double current_time = lastTime;
      for (auto &imu : vimuMsg)
      {
         Eigen::Vector3d gyr;
         gyr << imu->angular_velocity.x,
         imu->angular_velocity.y,
         imu->angular_velocity.z;
         double dt = imu->header.stamp.toSec() - current_time;
         ROS_ASSERT(dt >= 0);
         Eigen::Matrix3d dR = Sophus::SO3d::exp(gyr * dt).matrix();
         Eigen::Quaterniond qr(dq * dR);
         if (qr.w() < 0)
            qr.coeffs() *= -1;
         dq = qr.normalized();
         current_time = imu->header.stamp.toSec();
      }
   }     

  /** \brief pre-integration of IMU messages stored in vimuMsg
     */
  void PreIntegration(double lastTime, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba)
{
  Reset();
  linearized_bg = bg;
  linearized_ba = ba;
  double current_time = lastTime;
  for (auto &imu : vimuMsg)
  {
    Eigen::Vector3d gyr;
    gyr << imu->angular_velocity.x,
        imu->angular_velocity.y,
        imu->angular_velocity.z;
    Eigen::Vector3d acc;
    acc << imu->linear_acceleration.x * gnorm,
        imu->linear_acceleration.y * gnorm,
        imu->linear_acceleration.z * gnorm;
    double dt = imu->header.stamp.toSec() - current_time;
    if (dt <= 0)
      ROS_WARN("dt <= 0");
    gyr -= bg;
    acc -= ba;
    double dt2 = dt * dt;
    Eigen::Vector3d gyr_dt = gyr * dt;
    Eigen::Matrix3d dR = Sophus::SO3d::exp(gyr_dt).matrix();
    Eigen::Matrix3d Jr = Eigen::Matrix3d::Identity();
    double gyr_dt_norm = gyr_dt.norm();
    if (gyr_dt_norm > 0.00001)
    {
      Eigen::Vector3d k = gyr_dt.normalized();
      Eigen::Matrix3d K = Sophus::SO3d::hat(k);
      Jr = Eigen::Matrix3d::Identity() - (1 - cos(gyr_dt_norm)) / gyr_dt_norm * K + (1 - sin(gyr_dt_norm) / gyr_dt_norm) * K * K;
    }
    Eigen::Matrix<double, 15, 15> A = Eigen::Matrix<double, 15, 15>::Identity();
    A.block<3, 3>(0, 3) = -0.5 * dq.matrix() * Sophus::SO3d::hat(acc) * dt2;
    A.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
    A.block<3, 3>(0, 12) = -0.5 * dq.matrix() * dt2;
    A.block<3, 3>(3, 3) = dR.transpose();
    A.block<3, 3>(3, 9) = -Jr * dt;
    A.block<3, 3>(6, 3) = -dq.matrix() * Sophus::SO3d::hat(acc) * dt;
    A.block<3, 3>(6, 12) = -dq.matrix() * dt;
    Eigen::Matrix<double, 15, 12> B = Eigen::Matrix<double, 15, 12>::Zero();
    B.block<3, 3>(0, 3) = 0.5 * dq.matrix() * dt2;
    B.block<3, 3>(3, 0) = Jr * dt;
    B.block<3, 3>(6, 3) = dq.matrix() * dt;
    B.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity() * dt;
    B.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * dt;
    jacobian = A * jacobian;
    covariance = A * covariance * A.transpose() + B * noise * B.transpose();

    dp += dv * dt + 0.5 * dq.matrix() * acc * dt2;
    dv += dq.matrix() * acc * dt;
    Eigen::Matrix3d m3dR = dq.matrix() * dR;
    Eigen::Quaterniond qtmp(m3dR);
    if (qtmp.w() < 0)
      qtmp.coeffs() *= -1;
    dq = qtmp.normalized();
    dtime += dt;
    current_time = imu->header.stamp.toSec();
  }
}

  /** \brief normal integration of IMU messages stored in vimuMsg
     */
  void Integration() {}

public:
  const double acc_n = 0.08;
  const double gyr_n = 0.004;
  const double acc_w = 2.0e-4;
  const double gyr_w = 2.0e-5;
  constexpr static const double lidar_m = 1.5e-3;
  constexpr static const double gnorm = 1.0; //9.805;

  enum JacobianOrder
  {
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BG = 9,
    O_BA = 12
  };

private:
  std::vector<sensor_msgs::ImuConstPtr> vimuMsg;
  Eigen::Quaterniond dq;
  Eigen::Vector3d dp;
  Eigen::Vector3d dv;
  Eigen::Vector3d linearized_bg;
  Eigen::Vector3d linearized_ba;
  Eigen::Matrix<double, 15, 15> covariance;
  Eigen::Matrix<double, 15, 15> jacobian;
  Eigen::Matrix<double, 12, 12> noise;
  double dtime;
};

