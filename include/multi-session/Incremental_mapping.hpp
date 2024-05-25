// multi-session writted by @Yixin Fang, refering LT-Mapper

#pragma once

#include "../common_lib.h"
#include "BetweenFactorWithAnchoring.h"
#include "sc-relo/Scancontext.h"

namespace MultiSession{

// session
class Session{
  public:
    int index_;

    std::string name_;
    std::string session_dir_path_;

    bool is_base_session_;

    SessionNodes nodes_;
    SessionEdges edges_;

    int anchor_node_idx_;

    Trajectory cloudKeyPoses6D;  // parsing
    Trajectory originPoses6D;
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;

    pcl::PointCloud<PointType>::Ptr globalMap;

    std::vector<KeyFrame> cloudKeyFrames;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    ScanContext::SCManager scManager;

    Session(){}
    Session(int _idx, std::string _name, std::string _session_dir, bool _is_base_session); 

    void allocateMemory(){
      cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
      originPoses6D.reset(new pcl::PointCloud<PointTypePose>());
      cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
      globalMap.reset(new pcl::PointCloud<PointType>());
    }

    void loadSessionGraph();
    void loadSessionScanContextDescriptors();
    void loadSessionKeyframePointclouds();
    void loadGlobalMap();

    void initKeyPoses(void);
    void updateKeyPoses(const gtsam::ISAM2 * isam, const gtsam::Pose3& _anchor_transform);

    void loopFindNearKeyframesCentralCoord(KeyFrame& nearKeyframes, const int& key, const int& searchNum);
    void loopFindNearKeyframesLocalCoord(KeyFrame& nearKeyframes, const int& key, const int& searchNum);
};

using Sessions = std::map<int, Session>;
using SessionsDict = std::map<std::string, Session>;

class IncreMapping
{
  public:
    ros::NodeHandle  nh;

    ros::Publisher pubCentralTrajectory = nh.advertise<sensor_msgs::PointCloud2>("/central_trajectory", 5);
    ros::Publisher pubRegisteredTrajectory = nh.advertise<sensor_msgs::PointCloud2>("/registered_trajectory", 5);  
    ros::Publisher pubCentralGlobalMap = nh.advertise<sensor_msgs::PointCloud2>("/central_map", 10);
    ros::Publisher pubReloCloud = nh.advertise<sensor_msgs::PointCloud2>("/relo_cloud", 10);        
    ros::Publisher pubSCLoop = nh.advertise<visualization_msgs::MarkerArray>("sc_loop", 10);
    ros::Publisher pubRSLoop = nh.advertise<visualization_msgs::MarkerArray>("rs_loop", 10);

    std::string sessions_dir_;
    std::string central_sess_name_;
    std::string query_sess_name_;

    std::string save_directory_;

    const int numberOfCores = 8;
    
    bool is_display_debug_msgs_ = true;

    ros::Time publishTimeStamp = ros::Time::now();

    float loopFitnessScoreThreshold = 0.2;
    static inline int num_sessions = 1; // session index should start from 1

    Sessions sessions_;
    SessionsDict sessions_dict_;

    pcl::PointCloud<PointType>::Ptr traj_central;  // publish trajectory
    pcl::PointCloud<PointType>::Ptr traj_regis;

    std::vector<std::pair<int, KeyFrame>> reloKeyFrames;

    pcl::PointCloud<PointType>::Ptr centralMap_;
    pcl::PointCloud<PointType>::Ptr regisMap_;

    float pubSize = 1.0;
    pcl::VoxelGrid<PointType> downSizeFilterPub;

    const int target_sess_idx = 1; // means the centralt session. recommend to use 1 for it (because for the stable indexing for anchor node index) 
    const int source_sess_idx = 2; // TODO: generalize this function, change this sess_idx pair for arbitary pair (e.g., [2,7], [1,5])

    std::vector<std::pair<int, int>> SCLoopIdxPairs_;
    std::vector<std::pair<int, int>> RSLoopIdxPairs_;

    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    const gtsam::Pose3 poseOrigin;

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise;
    gtsam::noiseModel::Diagonal::shared_ptr loopNoise;
    gtsam::noiseModel::Diagonal::shared_ptr largeNoise;
    gtsam::noiseModel::Base::shared_ptr robustNoise;

    std::mutex mtx;

    IncreMapping(std::string sessions_dir, std::string central_sess_name, std::string query_sess_name, std::string save_directory) : poseOrigin(gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0))) {
      sessions_dir_ = sessions_dir;
      central_sess_name_ = central_sess_name;
      query_sess_name_ = query_sess_name;
      save_directory_ = save_directory;

      fsmkdir(std::string(sessions_dir + save_directory));  // aft instance of multi-session
      
      centralMap_.reset(new pcl::PointCloud<PointType>());
      regisMap_.reset(new pcl::PointCloud<PointType>());
      traj_central.reset(new pcl::PointCloud<PointType>());
      traj_regis.reset(new pcl::PointCloud<PointType>());
      downSizeFilterPub.setLeafSize(pubSize, pubSize, pubSize);
      loadCentralMap();

      initOptimizer();
      initNoiseConstants();

      loadAllSessions();
      addAllSessionsToGraph();
    }
    ~IncreMapping(){}

    void run( int iteration );

    void getReloKeyFrames();

    void initNoiseConstants();
    void initOptimizer();

    void loadAllSessions();
    void loadCentralMap();

    void addAllSessionsToGraph();
    void initTrajectoryByAnchoring(const Session& _sess);
    void addSessionToCentralGraph(const Session& _sess);

    void detectInterSessionSCloops();
    void detectInterSessionRSloops();
    void addSCloops();
    std::vector<std::pair<int, int>> equisampleElements(const std::vector<std::pair<int, int>>& _input_pair, float _gap, int _num);

    double calcInformationGainBtnTwoNodes(const int loop_idx_target_session, const int loop_idx_source_session);

    void findNearestRSLoopsTargetNodeIdx();
    bool addRSloops();
    
    void addFactorsWrtInfoGain();

    void optimizeMultisesseionGraph(bool _toOpt, int iteration);
    void updateSessionsPoses();

    std::experimental::optional<gtsam::Pose3> doICPVirtualRelative(Session& target_sess, Session& source_sess, 
                        const int& loop_idx_target_session, const int& loop_idx_source_session);
    std::experimental::optional<gtsam::Pose3> doICPGlobalRelative(Session& target_sess, Session& source_sess, 
                        const int& loop_idx_target_session, const int& loop_idx_source_session);

    // saver 
    gtsam::Pose3 getPoseOfIsamUsingKey(gtsam::Key _key);
    void writeAllSessionsTrajectories(std::string _postfix);

    // visualization
    void visualizeLoopClosure();
    void publish();
    
};

}