#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"

#define MAX_INI_COUNT (20)

/**
 * @brief 判断点的时间是否先后颠倒
 *
 */
const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); };

class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;       // imu参数输出文件
  V3D cov_acc;             // 加速度测量协方差
  V3D cov_gyr;             // 角速度测量协方差
  V3D cov_acc_scale;       // ？？？
  V3D cov_gyr_scale;       // ？？？
  V3D cov_bias_gyr;        // 角速度bias协方差
  V3D cov_bias_acc;        // 加速度bias协方差
  double first_lidar_time; // 当前帧第一个点的时间

private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;        // 当前帧点云未去畸变
  sensor_msgs::ImuConstPtr last_imu_;     // 上一帧imu数据
  deque<sensor_msgs::ImuConstPtr> v_imu_; // imu队列
  vector<Pose6D> IMUpose;                 // imu位姿信息(时间,加速度,角速度,速度,位置,旋转矩阵)
  vector<M3D> v_rot_pcl_;                 // ？？？
  M3D Lidar_R_wrt_IMU;                    // lidar到imu的旋转外参
  V3D Lidar_T_wrt_IMU;                    // lidar到imu的平移外参
  V3D mean_acc;                           // 加速度均值,用于计算方差
  V3D mean_gyr;                           // 角速度均值,用于计算方差
  V3D angvel_last;                        // 上一帧角速度
  V3D acc_s_last;                         // 上一帧加速度
  double start_timestamp_;                // 开始的时间戳
  double last_lidar_end_time_;            // 上一帧结束时间
  int init_iter_num = 1;                  // 初始化时,imu迭代的次数
  bool b_first_frame_ = true;             // 判断是否是第一帧
  bool imu_need_init_ = true;             // 判断是否需要初始化imu
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;                          // 初始化imu迭代的次数
  Q = process_noise_cov();                    // 调用use-ikfom.hpp里面的process_noise_cov完成噪声协方差的初始化
  cov_acc = V3D(0.1, 0.1, 0.1);               // 加速度测量协方差初始化
  cov_gyr = V3D(0.1, 0.1, 0.1);               // 角速度测量协方差初始化
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001); // 角速度bias协方差初始化
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001); // 加速度bias协方差初始化
  mean_acc = V3D(0, 0, -1.0);                 // 加速度平均值初始化
  mean_gyr = V3D(0, 0, 0);                    // 角速度平均值初始化
  angvel_last = Zero3d;                       // 上一帧角速度初始化
  Lidar_T_wrt_IMU = Zero3d;                   // lidar到imu的位置外参初始化
  Lidar_R_wrt_IMU = Eye3d;                    // lidar到imu的旋转外参初始化
  last_imu_.reset(new sensor_msgs::Imu());    // 上一帧imu初始化
}

/**
 * @brief 析构函数
 *
 */
ImuProcess::~ImuProcess() {}

/**
 * @brief 重置参数
 *
 */
void ImuProcess::Reset()
{
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;
  start_timestamp_ = -1;
  init_iter_num = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

/**
 * @brief 传递4*4矩阵进外参,R和T
 *
 */
void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
  Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
  Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}

/**
 * @brief 传递3*1向量进外参,只有T
 *
 */
void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity(); // 因为没有旋转,设置为单位矩阵
}

/**
 * @brief 传递3*3矩阵和3*1向量进外参,R和T
 *
 */
void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

/**
 * @brief 传入陀螺仪角速度协方差
 *
 */
void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

/**
 * @brief 传入加速度计加速度协方差
 *
 */
void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

/**
 * @brief 传入陀螺仪角速度bias协方差
 *
 */
void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

/**
 * @brief 传入加速度计加速度bias协方差
 *
 */
void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

/**
 * @brief imu初始化
 *
 */
void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  // 1.初始化重力、角速度bias、加速度、角速度协方差
  // 2.将加速度测量值标准化为单位重力
  // 这里应该是静止初始化

  V3D cur_acc, cur_gyr; // 当前加速度,角速度

  // 对于第一帧IMU数据处理
  if (b_first_frame_)
  {
    Reset(); // 重置参数
    N = 1;   // 迭代次数置为1
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration; // 从common_lib.h中拿到imu初始时刻的加速度
    const auto &gyr_acc = meas.imu.front()->angular_velocity;    // 从common_lib.h中拿到imu初始时刻的角速度
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;                 // 加速度测量作为初始化均值,记录最早加速度为加速度均值
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;                 // 角速度测量作为初始化均值,记录最早角速度为角速度均值
    first_lidar_time = meas.lidar_beg_time;                      // 记录lidar起始时间作为初始时间,即当前imu帧对应的lidar时间
  }

  // 读取所有imu信息,计算方差
  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration; // 读加速度
    const auto &gyr_acc = imu->angular_velocity;    // 读角速度
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;     // 记录当前加速度
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;     // 记录当前角速度

    // 根据当前帧和均值差作为均值的更新
    mean_acc += (cur_acc - mean_acc) / N; // 更新加速度均值
    mean_gyr += (cur_gyr - mean_gyr) / N; // 更新角速度均值

    // 更新加速度协方差
    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    // 更新角速度协方差
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    N++;
  }

  state_ikfom init_state = kf_state.get_x();                  // 在esekfom.hpp获得x_的状态
  init_state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2); // 从common_lib.h中拿到重力,并与加速度测量均值的单位重力求出SO2的旋转矩阵类型的重力加速度,- acc * (g / acc),负值,根据加速度均值,重力模长做归一化,重力记为S2流形

  // state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg = mean_gyr;                  // 角速度测量作为陀螺仪偏差
  init_state.offset_T_L_I = Lidar_T_wrt_IMU; // 将lidar和imu外参位移量传入
  init_state.offset_R_L_I = Lidar_R_wrt_IMU; // 将lidar和imu外参旋转量传入
  kf_state.change_x(init_state);             // 将初始化状态传入esekfom.hpp中的x_

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P(); // 在esekfom.hpp获得P_的协方差矩阵
  init_P.setIdentity();                                                        // 将协方差矩阵置为单位阵
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;                        // 将协方差矩阵的位置和旋转的协方差置为0.00001
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;                    // 将协方差矩阵的速度和位姿的协方差置为0.00001
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;                   // 将协方差矩阵的重力和姿态的协方差置为0.0001
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;                    // 将协方差矩阵的陀螺仪偏差和姿态的协方差置为0.001
  init_P(21, 21) = init_P(22, 22) = 0.00001;                                   // 将协方差矩阵的lidar和imu外参位移量的协方差置为0.00001
  kf_state.change_P(init_P);                                                   // 将初始化协方差矩阵传入esekfom.hpp中的P_
  last_imu_ = meas.imu.back();                                                 // 将最后一帧的imu数据传入last_imu_中,暂时没用到
}

/**
 * @brief imu更新,正向传播,反向传播,点云去畸变
 *
 */
void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  // 将最后一帧尾部的imu添加到当前帧头部的imu
  auto v_imu = meas.imu;                                            // 拿到当前的imu数据
  v_imu.push_front(last_imu_);                                      // 将上一帧尾部的imu添加到当前帧头部的imu
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec(); // 拿到当前帧头部的imu的时间
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();  // 拿到当前帧尾部的imu的时间
  const double &pcl_beg_time = meas.lidar_beg_time;                 // 当前帧开始的时间
  const double &pcl_end_time = meas.lidar_end_time;                 // 当前帧结束的时间

  // 根据点云中每个点的时间戳对点云进行重排序
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list); //点按时间从小到大排序(从旧到新)

  // 初始化IMU
  state_ikfom imu_state = kf_state.get_x(); // 获取上一次KF估计的后验状态作为本次IMU预测的初始状态
  IMUpose.clear();                          // 清空IMUpose

  // 将初始状态加入IMUpose中,包含有时间间隔,上一帧加速度,上一帧角速度,上一帧速度,上一帧位置,上一帧旋转矩阵
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  // 每个IMU数据的正向传播对应的参数
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu; // angvel_avr为平均角速度,acc_avr为平均加速度,acc_imu为imu加速度,vel_imu为imu速度,pos_imu为imu位置
  M3D R_imu;                                          // imu旋转矩阵

  double dt = 0; // 时间间隔

  input_ikfom in; // eskf传入参数,输入状态量:记录加速度和角速度

  // 遍历本次估计的所有IMU测量(除了最后一帧)并且进行积分,离散中值法,正向传播
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);     // 拿到当前帧的imu数据
    auto &&tail = *(it_imu + 1); // 拿到下一帧的imu数据

    // 判断时间先后顺序 不符合直接continue
    if (tail->header.stamp.toSec() < last_lidar_end_time_)
      continue;

    // 两个IMU数据的加速度和角速度均值,j与j+1的均值做中值积分
    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // 根据初始化得到的加速度均值,将加速度归算到重力尺度下,通过重力数值对加速度进行一下微调
    acc_avr = acc_avr * G_m_s2 / mean_acc.norm();

    // 如果IMU开始时刻早于上次雷达最晚时刻(因为将上次最后一个IMU插入到此次开头了,所以会出现一次这种情况)
    if (head->header.stamp.toSec() < last_lidar_end_time_) // 前一IMU时间小于上一lidar结束时间
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_; // 记录后一IMU时间与上一lidar结束时间之差
    }
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec(); // 相邻imu时刻时间差
    }

    // 值传入
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    // 配置协方差矩阵
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

    // IMU正向传播,每次传播的时间间隔为dt
    kf_state.predict(dt, Q, in); // 根据输入数据向前传播(两imu数据间隔时间差,协方差矩阵,输入数据)

    imu_state = kf_state.get_x();                          // 保存每个IMU预测的状态
    angvel_last = angvel_avr - imu_state.bg;               // 计算出来的角速度与预测的角速度的差值(前一时刻加速度)
    acc_s_last = imu_state.rot * (acc_avr - imu_state.ba); // 计算出来的加速度与预测的加速度的差值,并转到IMU坐标系下(前一时刻角速度)
    for (int i = 0; i < 3; i++)
    {
      acc_s_last[i] += imu_state.grav[i]; // 加上重力得到world系的加速度
    }

    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time; // 后一个IMU时刻与lidar起始时刻时间差
    // 保存IMU预测过程的状态,时间、前一时刻角速度、前一时刻加速度、v、p、r、
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  // 计算最后一帧IMU测量的情况,判断雷达结束时间是否晚于IMU,最后一个IMU时刻可能早于雷达末尾,也可能晚于雷达末尾
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time); // lidar终点与imu终点时间差
  kf_state.predict(dt, Q, in);               // in此时为最后一个imu数据,传播到lidar终点时刻与imu终点时刻较大者

  imu_state = kf_state.get_x();        // lidar终点时刻与imu终点时刻较大者,imu位姿(更新IMU状态,以便于下一帧使用)
  last_imu_ = meas.imu.back();         // 记录最后一个imu数据为上一imu数据,用于下一次imu正向传播的开头
  last_lidar_end_time_ = pcl_end_time; // 记录点云结束时间为上一帧结束时间,以便于下一帧使用

  // 在处理完所有的IMU预测后,基于IMU预测对lidar点云去畸变(反向传播)
  auto it_pcl = pcl_out.points.end() - 1;                                 // 点云从后向前遍历,此前点云已经按照时间从小到大排序
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) // imu pose从后向前遍历
  {
    auto head = it_kp - 1; // 前一时刻imu位姿
    auto tail = it_kp;     // 后一时刻imu位姿
    // 前一时刻imu的p、v、r
    R_imu << MAT_FROM_ARRAY(head->rot);      // 拿到前一时刻的IMU旋转矩阵
    vel_imu << VEC_FROM_ARRAY(head->vel);    // 拿到前一时刻的IMU速度
    pos_imu << VEC_FROM_ARRAY(head->pos);    // 拿到前一时刻的IMU位置
    acc_imu << VEC_FROM_ARRAY(tail->acc);    // 拿到后一时刻的IMU加速度
    angvel_avr << VEC_FROM_ARRAY(tail->gyr); // 拿到后一时刻的IMU角速度

    // 点云时间需要迟于前一IMU时刻,因为是在两个IMU时刻之间去畸变,此时默认雷达的时间戳在后一个IMU时刻之前,时间除以1000单位化为s
    for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) // head->offset_time为前一时刻imu的时间
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time; // 点到IMU开始时刻的时间间隔

      /*
       * 变换到"结束"帧,仅使用旋转;注意:补偿方向与帧的移动方向相反,所以如果我们想补偿时间戳i到帧e的一个点
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei),其中T_ei在全局框架中表示
       */
      M3D R_i(R_imu * Exp(angvel_avr, dt)); // 点所在时刻的旋转

      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z); // lidar系下索引i点的坐标

      // imu_state:lidar终点时刻与imu终点时刻较大者,imu位姿;T_ei:从imu终点位置指向索引i时刻imu位置的平移向量,world系, 即T_i - T_e
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos); // 从点所在的world位置-末尾时刻的world位置

      // .conjugate()取旋转矩阵的转置;e是end时刻;
      // P_compensate是点在end时刻在雷达系的坐标,记为L^P_e;
      // imu_state.offset_R_L_I是从lidar系到imu(body)系的旋转矩阵,记为I^R_L;
      // imu_state.offset_T_L_I是imu(body)系下lidar系原点的位置,记为I^t_L;
      // 去畸变补偿的公式倒推,将右侧矩阵乘过来并加上右侧平移,左边变为I^R_L * L^P_e + I^t_L= I^P_e,也就是end时刻点在imu(body)系的坐标
      // 右边剩下imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei)
      // imu_state.rot.conjugate()是结束时刻imu(body)系到world系的旋转矩阵的转置,也就是(W^R_i_e)^T
      // T_ei展开是pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos,也就是点所在时刻IMU在world系下的位置 - end时刻IMU在world系下的位置 W^t_I-W^t_I_e
      // 现在等式两边变为 I^P_e =  (W^R_i_e)^T * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + W^t_I - W^t_I_e)
      // (W^R_i_e) * I^P_e + W^t_I_e = (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + W^t_I
      // world系也无所谓时刻了,因为只有一个world系,两边变为:W^P = R_i * I^P+ W^t_I->W^P = W^P

      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() *
                                                                   (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
                                                               imu_state.offset_T_L_I); // not accurate!

      // 保存去畸变的点坐标
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin())
        break;
    }
  }
}

/**
 * @brief IMU的主程序
 *
 */
void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1, t2;
  t1 = omp_get_wtime();

  if (meas.imu.empty())
  {
    return;
  }; // 拿到的当前帧的imu测量为空,则直接返回
  ROS_ASSERT(meas.lidar != nullptr);

  // IMU初始化,初始化完成后才做去畸变
  if (imu_need_init_)
  {
    // 第一个激光雷达帧
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;

    last_imu_ = meas.imu.back(); // 记录最后一个imu数据

    state_ikfom imu_state = kf_state.get_x(); // 获取状态量
    if (init_iter_num > MAX_INI_COUNT)        // 初始化imu数据量大于阈值
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2); // 在上面IMU_init()基础上乘上缩放系数
      imu_need_init_ = false;

      cov_acc = cov_acc_scale; // ??
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
    }

    return;
  }

  // 正向传播,反向传播,去畸变
  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  // cout<<"[ IMU Process ]: Time: "<<t2 - t1<<endl;
}
