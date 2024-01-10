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
    ros::Publisher pubMeasurementEdge;
    ros::Publisher pubPath;

    pcl::PointCloud<PointType>::Ptr priorMap;
    pcl::PointCloud<PointType>::Ptr priorPath;
    pcl::PointCloud<PointType>::Ptr reloCloud;
    pcl::PointCloud<PointType>::Ptr initCloud;

    PointTypePose externalPose;
    PointTypePose initPose;

    std::vector<int> idxVec;
    std::vector<float> disVec;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses;
    
    int idx = 0;
    std::deque<pcl::PointCloud<PointType>::Ptr> cloudBuffer;
    std::deque<PointTypePose> poseBuffer_6D;
    std::deque<PointType> poseBuffer_3D;

    nav_msgs::Path path;                    
    nav_msgs::Odometry odomAftMapped;
    geometry_msgs::PoseStamped msg_body_pose; 
    std::deque<PointTypePose> reloPoseBuffer;

    MultiSession::Session *priorKnown;
    Registeration *reg;

    bool buffer_flg = true;
    bool global_flg = true;
    bool external_flg = false;

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

