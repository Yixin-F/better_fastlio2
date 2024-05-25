// Scan Context accepted by IROS, but it has been rewritten by @Yixin Fang

#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "../nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

#include "../tictoc.hpp"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using PointType = pcl::PointXYZINormal; // using xyzinormal for fast-lio. but a user can exchange the original bin encoding function (i.e., max hegiht) to max intensity (for detail, refer 20 ICRA Intensity Scan Context)
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;


namespace ScanContext
{

void coreImportTest ( void );


// sc param-independent helper functions 
float xy2theta( const float & _x, const float & _y );
MatrixXd circshift( MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( MatrixXd _eigmat );


class SCManager
{
public: 
    SCManager( ) = default; // reserving data space (of std::vector) could be considered. but the descriptor is lightweight so don't care.

    Eigen::MatrixXd makeScancontext( pcl::PointCloud<PointType> & _scan_down );
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    int fastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 ); 
    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "d" (eq 5) in the original paper (IROS 18)
    std::pair<double, int> distanceBtnScanContext ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "D" (eq 6) in the original paper (IROS 18)

    // User-side API
    void makeAndSaveScancontextAndKeys( pcl::PointCloud<PointType> & _scan_down );
    std::pair<int, float> detectClosestKeyframeID(int num_exclude_recent, const std::vector<float> &curr_key, Eigen::MatrixXd &curr_desc);
    std::pair<int, float> detectLoopClosureID( int num_exclude_recent = 50 ); // int: nearest node index, float: relative yaw  

    void saveCurrentSCD(const std::string &fileName, int num_digits = 6, const std::string &delimiter = " ");
    void loadPriorSCD(const std::string &path, int num_digits, int num_keyframe);
    std::pair<int, float> relocalize(pcl::PointCloud<PointType> &_scan_down);

    // User-side API for multi-session
    void saveScancontextAndKeys( Eigen::MatrixXd _scd );
    std::pair<int, float> detectLoopClosureIDBetweenSession ( std::vector<float>& curr_key,  Eigen::MatrixXd& curr_desc);

    const Eigen::MatrixXd& getConstRefRecentSCD(void);

public:
    // hyper parameters ()
    double LIDAR_HEIGHT = 1.5; // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.

    const int    PC_NUM_RING = 20; // 20 in the original paper (IROS 18)
    const int    PC_NUM_SECTOR = 60; // 60 in the original paper (IROS 18)
    const double PC_MAX_RADIUS = 80.0; // 80 meter max in the original paper (IROS 18)
    const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

    // tree
    const int    NUM_EXCLUDE_RECENT = 30; // simply just keyframe gap (related with loopClosureFrequency in yaml), but node position distance-based exclusion is ok. 
    const int    NUM_CANDIDATES_FROM_TREE = 3; // 10 is enough. (refer the IROS 18 paper)

    // loop thres
    const double SEARCH_RATIO = 0.2; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    // const double SC_DIST_THRES = 0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
    double SC_DIST_THRES = 0.3; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

    // config 
    const int    TREE_MAKING_PERIOD_ = 50; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / if you want to find a very recent revisits use small value of it (it is enough fast ~ 5-50ms wrt N.).
    int          tree_making_period_conter = 0;

    // data 
    std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::shared_ptr<InvKeyTree> polarcontext_tree_;

    bool is_tree_batch_made = false;
    std::unique_ptr<InvKeyTree> polarcontext_tree_batch_;

}; // SCManager

} // namespace ScanContext
