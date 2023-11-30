#pragma once

#include "utility.h"
#include "BetweenFactorWithAnchoring.h"
#include "sc-relo/Scancontext.h"

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

    std::vector<keyFrame> cloudKeyFrames;
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

    void loopFindNearKeyframesCentralCoord(keyFrame& nearKeyframes, const int& key, const int& searchNum);
    void loopFindNearKeyframesLocalCoord(keyFrame& nearKeyframes, const int& key, const int& searchNum);
};

class IncreMapping
{
  public:
    int session_num_;

    IncreMapping(){}
    // IncreMapping(const std::string& centralPath, const std::string& regisPath);
    ~IncreMapping(){}

    void loadPoseGraph();
    void reloByPoseGraph();
    void localMapUpdate(); 
    
};

using multiSession = std::map<int, Session>;
using multiSessionInfo = std::map<std::string, Session>;

}