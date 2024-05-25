// online relocalization writted by @Yixin Fang

#pragma once

#include "../common_lib.h"
#include "../multi-session/Incremental_mapping.hpp"
#include "../FRICP-toolkit/registeration.h"
#include "../tool_color_printf.h"
#include "../tictoc.hpp"

class pose_estimator{
public:
    ros::NodeHandle nh;

    std::string priorDir;
    std::string cloudTopic;
    std::string poseTopic;
    std::string cloudTopic_repub;
    std::string poseTopic_repub;
    float searchDis;
    int searchNum; // >=2
    float trustDis;
    int regMode;
    
    // subscribe from fast-lio2
    ros::Subscriber subCloud;
    ros::Subscriber subPose;
    ros::Publisher pubCloud;
    ros::Publisher pubPose;

    // online relocalization
    ros::Subscriber subExternalPose;
    ros::Publisher pubPriorMap;
    ros::Publisher pubPriorPath;
    ros::Publisher pubInitCloud;
    ros::Publisher pubReloCloud;
    ros::Publisher pubNearCloud;
    ros::Publisher pubMeasurementEdge;
    ros::Publisher pubPath;

    pcl::PointCloud<PointType>::Ptr priorMap;
    pcl::PointCloud<PointType>::Ptr priorPath;
    pcl::PointCloud<PointType>::Ptr reloCloud;
    pcl::PointCloud<PointType>::Ptr initCloud;
    pcl::PointCloud<PointType>::Ptr initCloud_;
    pcl::PointCloud<PointType>::Ptr nearCloud;

    PointTypePose externalPose;
    PointTypePose initPose;
    std::vector<double> extrinT_;
    std::vector<double> extrinR_;
    Eigen::Vector3d extrinT;
    Eigen::Matrix3d extrinR;
    PointTypePose pose_zero;
    PointTypePose pose_ext;

    std::vector<int> idxVec;
    std::vector<float> disVec;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses;

    std::vector<int> idxVec_copy;
    std::vector<float> disVec_copy;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses_copy;

    pcl::VoxelGrid<PointType> downSizeFilterPub;
    
    int idx = 0;
    std::deque<pcl::PointCloud<PointType>::Ptr> cloudBuffer;
    std::deque<PointTypePose> poseBuffer_6D;
    std::deque<PointType> poseBuffer_3D;
    ofstream fout_relo;
    


    double ld_time;
    nav_msgs::Path path;                    
    nav_msgs::Odometry odomAftMapped;
    geometry_msgs::PoseStamped msg_body_pose; 
    std::deque<PointTypePose> reloPoseBuffer;

    std::vector<MultiSession::Session> sessions;
    std::vector<Registeration> reg;
    std::pair<int, float> detectResult;

    bool buffer_flg = true;
    bool global_flg = false;
    bool external_flg = false;
    bool sc_flg = false;

    float height;

    int cout_count = 0;
    int cout_count_ = 0;

    int sc_new = 1;
    int sc_old = -1;

    pose_estimator();
    ~pose_estimator() {}
    void allocateMemory();

    void cloudCBK(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void poseCBK(const nav_msgs::Odometry::ConstPtr& msg);
    void externalCBK(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void run();
    void publishThread();

    bool easyToRelo(const PointType& pose3d);
    bool globalRelo();

    void publish_odometry(const ros::Publisher &pubOdomAftMapped);  // tf
    void publish_path(const ros::Publisher& pubPath); // path
};

