#pragma once

#include "utility.h"
#include "BetweenFactorWithAnchoring.h"
#include "sc-relo/Scancontext.h"

#include <experimental/optional>

// TODO: rviz interaction

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

    std::vector<KeyFrame> cloudKeyFrames;
    pcl::VoxelGrid<PointType> downSizeFilterICP;

    ScanContext::SCManager scManager;

    Session(){}
    Session(int _idx, std::string _name, std::string _session_dir, bool _is_base_session);
    void allocateMemory(){
      cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
      originPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    }

    void loadSessionGraph();
    void loadSessionScanContextDescriptors();
    void loadSessionKeyframePointclouds();

    void initKeyPoses(void);
    void updateKeyPoses(const gtsam::ISAM2 * isam, const gtsam::Pose3& _anchor_transform);

    void loopFindNearKeyframesCentralCoord(KeyFrame& nearKeyframes, const int& key, const int& searchNum);
    void loopFindNearKeyframesLocalCoord(KeyFrame& nearKeyframes, const int& key, const int& searchNum);
};

using Sessions = std::map<int, Session>;
using SessionsDict = std::map<std::string, Session>;

using namespace LTslamParam;

class IncreMapping
{
  public:
    ros::NodeHandle  nh;

    std::string sessions_dir_;
    std::string central_sess_name_;
    std::string query_sess_name_;

    std::string save_directory_;

    const int numberOfCores = 8;
    
    bool is_display_debug_msgs_;

    int kNumSCLoopsUpperBound;
    int kNumRSLoopsUpperBound;

    float loopFitnessScoreThreshold;
    static inline int num_sessions = 1; // session index should start from 1

    Sessions sessions_;
    SessionsDict sessions_dict_;

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

    IncreMapping() : poseOrigin(gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0))) {}
    ~IncreMapping(){}

    void run( void );

    void initNoiseConstants();
    void initOptimizer();

    void loadAllSessions();

    friend int genGlobalNodeIdx(const int&, const int&);
    friend int genAnchorNodeIdx(const int&);

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

    void optimizeMultisesseionGraph(bool _toOpt);
    void updateSessionsPoses();

    std::experimental::optional<gtsam::Pose3> doICPVirtualRelative(Session& target_sess, Session& source_sess, 
                        const int& loop_idx_target_session, const int& loop_idx_source_session);
    std::experimental::optional<gtsam::Pose3> doICPGlobalRelative(Session& target_sess, Session& source_sess, 
                        const int& loop_idx_target_session, const int& loop_idx_source_session);

    // saver 
    gtsam::Pose3 getPoseOfIsamUsingKey(gtsam::Key _key);
    void writeAllSessionsTrajectories(std::string _postfix);
    
};

}