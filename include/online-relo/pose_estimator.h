#pragma once

#include "../common_lib.h"
#include "featureExtract.h"
#include "Map_Manager.h"
#include "ceresfunc.h"
#include <ceres/ceres.h>
#include "../mutexDeque.h"
#include "../tool_color_printf.h"
#include "../tictoc.hpp"

struct pcdmap
{
  std::vector<pcl::PointCloud<PointType>::Ptr> corner_keyframes_;
  std::vector<pcl::PointCloud<PointType>::Ptr> surf_keyframes_;
  pcl::PointCloud<PointType>::Ptr globalMapCloud_;
  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D_;
  pcl::PointCloud<PointType>::Ptr globalCornerMapCloud_;
  pcl::PointCloud<PointType>::Ptr globalSurfMapCloud_;

  pcdmap()
  {
    globalMapCloud_.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses3D_.reset(new pcl::PointCloud<PointType>());
    globalCornerMapCloud_.reset(new pcl::PointCloud<PointType>());
    globalSurfMapCloud_.reset(new pcl::PointCloud<PointType>());
  }
};

enum InitializedFlag
{
  NonInitialized,
  Initializing,
  Initialized,
  MayLost
};

/** \brief point to line feature */
struct FeatureLine
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pointOri;
  Eigen::Vector3d lineP1;
  Eigen::Vector3d lineP2;
  double error;
  bool valid;
  FeatureLine(Eigen::Vector3d po, Eigen::Vector3d p1, Eigen::Vector3d p2)
      : pointOri(std::move(po)), lineP1(std::move(p1)), lineP2(std::move(p2))
  {
    valid = false;
    error = 0;
  }
  double ComputeError(const Eigen::Matrix4d &pose)
  {
    Eigen::Vector3d P_to_Map = pose.topLeftCorner(3, 3) * pointOri + pose.topRightCorner(3, 1);
    double l12 = std::sqrt((lineP1(0) - lineP2(0)) * (lineP1(0) - lineP2(0)) + (lineP1(1) - lineP2(1)) * (lineP1(1) - lineP2(1)) + (lineP1(2) - lineP2(2)) * (lineP1(2) - lineP2(2)));
    double a012 = std::sqrt(
        ((P_to_Map(0) - lineP1(0)) * (P_to_Map(1) - lineP2(1)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(1) - lineP1(1))) * ((P_to_Map(0) - lineP1(0)) * (P_to_Map(1) - lineP2(1)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(1) - lineP1(1))) + ((P_to_Map(0) - lineP1(0)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(2) - lineP1(2))) * ((P_to_Map(0) - lineP1(0)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(2) - lineP1(2))) + ((P_to_Map(1) - lineP1(1)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(1) - lineP2(1)) * (P_to_Map(2) - lineP1(2))) * ((P_to_Map(1) - lineP1(1)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(1) - lineP2(1)) * (P_to_Map(2) - lineP1(2))));
    error = a012 / l12;
  }
};

/** \brief point to plan feature */
struct FeaturePlanVec
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pointOri;
  Eigen::Vector3d pointProj;
  Eigen::Matrix3d sqrt_info;
  double error;
  bool valid;
  FeaturePlanVec(const Eigen::Vector3d &po, const Eigen::Vector3d &p_proj, Eigen::Matrix3d sqrt_info_)
      : pointOri(po), pointProj(p_proj), sqrt_info(sqrt_info_)
  {
    valid = false;
    error = 0;
  }
  double ComputeError(const Eigen::Matrix4d &pose)
  {
    Eigen::Vector3d P_to_Map = pose.topLeftCorner(3, 3) * pointOri + pose.topRightCorner(3, 1);
    error = (P_to_Map - pointProj).norm();
  }
};

struct LidarFrame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    pcl::PointCloud<PointType>::Ptr laserCloud;
    pcl::PointCloud<PointType>::Ptr corner;
    pcl::PointCloud<PointType>::Ptr surf;
    IMUIntegrator imuIntegrator;
    Eigen::Vector3d P;
    Eigen::Vector3d V;
    Eigen::Quaterniond Q;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
    double timeStamp;

    LidarFrame()
    {
      corner.reset(new pcl::PointCloud<PointType>());
      surf.reset(new pcl::PointCloud<PointType>());
      laserCloud.reset(new pcl::PointCloud<PointType>());
      P.setZero();
      V.setZero();
      Q.setIdentity();
      bg.setZero();
      ba.setZero();
      timeStamp = 0;
    }
};

class pose_estimator
{
public:
    // FIXME: https://blog.csdn.net/derteanoo/article/details/83303760
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::NodeHandle nh_;
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_initial_pose_;

  ros::Publisher pub_corner_map;
  ros::Publisher pub_surf_;
  ros::Publisher pubMappedPoints_;
  ros::Publisher pubLaserOdometryPath_;

  tf::StampedTransform transform_;
  tf::TransformBroadcaster broadcaster_; //  publish laser to map tf

  nav_msgs::Path laserOdoPath;

  FeatureExtract FE;

  pcdmap map;
  std::string filename;
  std::string pointCloudTopic;
  std::string imu_topic;
  int IMU_Mode = 0;
  bool use_lio = false;
  double corner_leaf_;
  double surf_leaf_;

  pcl::KdTreeFLANN<PointType>::Ptr kdtree_keyposes_3d_;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_map;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_map;

  pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_localmap;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_localmap;

  pcl::PointCloud<PointType>::Ptr surround_surf;
  pcl::PointCloud<PointType>::Ptr surround_corner;

  pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

  pcl::VoxelGrid<PointType> ds_corner_;
  pcl::VoxelGrid<PointType> ds_surf_;

  MutexDeque<sensor_msgs::PointCloud2ConstPtr> _lidarMsgQueue;
  MutexDeque<sensor_msgs::ImuConstPtr> _imuMsgQueue;
  InitializedFlag initializedFlag;

  PointTypePose initpose;

  Eigen::Vector3d GravityVector;

  int pushCount = 0;
  double startTime = 0;
  int WINDOWSIZE;  // slide window
  bool LidarIMUInited = false;
  boost::shared_ptr<std::list<LidarFrame>> lidarFrameList;
  std::string rootDir;

  MAP_MANAGER *map_manager;
  static const int SLIDEWINDOWSIZE = 2;
  double para_PR[SLIDEWINDOWSIZE][6];
  double para_VBias[SLIDEWINDOWSIZE][9];
  MarginalizationInfo *last_marginalization_info = nullptr;
  std::vector<double *> last_marginalization_parameter_blocks;

  Eigen::Matrix4d exTlb = Eigen::Matrix4d::Identity();
  double thres_dist = 1.0;
  double plan_weight_tan = 0.0;
  Eigen::Matrix3d delta_Rl = Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_tl = Eigen::Vector3d::Zero();
  Eigen::Matrix4d transformLastMapped = Eigen::Matrix4d::Identity();

  static const int localMapWindowSize = 30;
  int localMapID = 0;
  pcl::PointCloud<PointType>::Ptr localCornerMap[localMapWindowSize];
  pcl::PointCloud<PointType>::Ptr localSurfMap[localMapWindowSize];
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromLocal;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromLocal;
  
  pose_estimator();

  ~pose_estimator(){}
  
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg);

  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);

  void ExtractFeature(LidarFrame &kf);

  void initialPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

  void pubOdometry(Eigen::Matrix4d pose, double &time);

  void pubOdometry(LidarFrame &frame);

  void vector2double(const std::list<LidarFrame> &tempFrameList);

  void double2vector(std::list<LidarFrame> &tempFrameList);

  void Estimate(std::list<LidarFrame> &frameList, const Eigen::Vector3d &gravity);

  void run();

  bool TryMAPInitialization();

  void RemoveLidarDistortion(pcl::PointCloud<PointType>::Ptr &laserCloud,
                             const Eigen::Matrix3d &dRlc, const Eigen::Vector3d &dtlc);

  bool fetchImuMsgs(double startTime, double endTime, std::vector<sensor_msgs::ImuConstPtr> &vimuMsg);

  void processPointToLine(std::vector<ceres::CostFunction *> &edges,
                          std::vector<FeatureLine> &vLineFeatures,
                          const pcl::PointCloud<PointType>::Ptr &laserCloudCorner,
                          const pcl::PointCloud<PointType>::Ptr &laserCloudCornerGlobal,
                          const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeGlobal,
                          const pcl::PointCloud<PointType>::Ptr &laserCloudCornerLocal,
                          const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeLocal,
                          const Eigen::Matrix4d &exTlb,
                          const Eigen::Matrix4d &m4d);

  void processPointToPlanVec(std::vector<ceres::CostFunction *> &edges,
                             std::vector<FeaturePlanVec> &vPlanFeatures,
                             const pcl::PointCloud<PointType>::Ptr &laserCloudSurf,
                             const pcl::PointCloud<PointType>::Ptr &laserCloudSurfGlobal,
                             const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeGlobal,
                             const pcl::PointCloud<PointType>::Ptr &laserCloudSurfLocal,
                             const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeLocal,
                             const Eigen::Matrix4d &exTlb,
                             const Eigen::Matrix4d &m4d);

  void MapIncrementLocal(LidarFrame &kframe);

  bool ICPScanMatchGlobal(std::list<LidarFrame> &kframeList);

  Eigen::Matrix4d toMatrix(PointTypePose &p);

  void saveTrajectoryTUMformat(std::fstream &fout, std::string &stamp, Eigen::Vector3d &xyz, Eigen::Quaterniond &xyzw);

  void saveTrajectoryTUMformat(std::fstream &fout, std::string &stamp, double x, double y, double z, double qx, double qy, double qz, double qw);

  pcl::PointCloud<PointType>::Ptr TransformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);

  bool extractSurroundKeyFrames(const PointType &p);

  bool loadmap();

};



