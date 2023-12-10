#include "Incremental_mapping.hpp"

bool fileNameSort(std::string name1_, std::string name2_){  // filesort by name
    std::string::size_type iPos1 = name1_.find_last_of('/') + 1;
	std::string filename1 = name1_.substr(iPos1, name1_.length() - iPos1);
	std::string name1 = filename1.substr(0, filename1.rfind("."));

    std::string::size_type iPos2 = name2_.find_last_of('/') + 1;
    std::string filename2 = name2_.substr(iPos2, name2_.length() - iPos2);
	std::string name2 = filename2.substr(0, filename2.rfind(".")); 

    return std::stoi(name1) < std::stoi(name2);
}

bool pairIntAndStringSort(const std::pair<int, std::string>& pair_1, const std::pair<int, std::string>& pair_2){
    return pair_1.first < pair_2.first;
}

// initialize Session
MultiSession::Session::Session(int _idx, std::string _name, std::string _session_dir_path, bool _is_base_session)
    : index_(_idx), name_(_name), session_dir_path_(_session_dir_path), is_base_session_(_is_base_session){

    allocateMemory();
    
    loadSessionGraph();

    loadSessionKeyframePointclouds();
    loadSessionScanContextDescriptors();
    
    const float kICPFilterSize = 0.2; // TODO move to yaml 
    downSizeFilterICP.setLeafSize(kICPFilterSize, kICPFilterSize, kICPFilterSize);
} // ctor

// read poses in graph
void MultiSession::Session::initKeyPoses(void){
    for(auto & _node_info: nodes_){
        PointTypePose thisPose6D;

        int node_idx = _node_info.first;
        Node node = _node_info.second; 
        gtsam::Pose3 pose = node.initial;

        thisPose6D.x = pose.translation().x();
        thisPose6D.y = pose.translation().x();
        thisPose6D.z = pose.translation().x();
        thisPose6D.intensity = node_idx; // TODO
        thisPose6D.roll  = pose.rotation().roll();
        thisPose6D.pitch = pose.rotation().pitch();
        thisPose6D.yaw   = pose.rotation().yaw();
        thisPose6D.time = 0.0; // TODO: no-use

        cloudKeyPoses6D->push_back(thisPose6D);    
    }

    PointTypePose thisPose6D;
    thisPose6D.x = 0.0;
    thisPose6D.y = 0.0;
    thisPose6D.z = 0.0;
    thisPose6D.intensity = 0.0;
    thisPose6D.roll = 0.0;
    thisPose6D.pitch = 0.0;
    thisPose6D.yaw = 0.0;
    thisPose6D.time = 0.0;
    originPoses6D->push_back(thisPose6D);
} 

// isam2 update
void MultiSession::Session::updateKeyPoses(const gtsam::ISAM2 * _isam, const gtsam::Pose3& _anchor_transform){
    gtsam::Values isamCurrentEstimate = _isam->calculateEstimate();

    int numPoses = cloudKeyFrames.size();
    for (int node_idx_in_sess = 0; node_idx_in_sess < numPoses; ++node_idx_in_sess){
        int node_idx_in_global = genGlobalNodeIdx(index_, node_idx_in_sess);
        std::cout << "update the session " << index_ << "'s node: " << node_idx_in_sess << " (global idx: " << node_idx_in_global << ")" << std::endl;

        gtsam::Pose3 pose_self_coord = isamCurrentEstimate.at<gtsam::Pose3>(node_idx_in_global);
        gtsam::Pose3 pose_central_coord = _anchor_transform * pose_self_coord;

        cloudKeyPoses6D->points[node_idx_in_sess].x = pose_central_coord.translation().x();
        cloudKeyPoses6D->points[node_idx_in_sess].y = pose_central_coord.translation().y();
        cloudKeyPoses6D->points[node_idx_in_sess].z = pose_central_coord.translation().z();
        cloudKeyPoses6D->points[node_idx_in_sess].roll  = pose_central_coord.rotation().roll();
        cloudKeyPoses6D->points[node_idx_in_sess].pitch = pose_central_coord.rotation().pitch();
        cloudKeyPoses6D->points[node_idx_in_sess].yaw   = pose_central_coord.rotation().yaw();
    }
} 

void MultiSession::Session::loopFindNearKeyframesCentralCoord(KeyFrame& nearKeyframes, const int& key, const int& searchNum){
    // extract near keyframes
    nearKeyframes.all_cloud->clear();
    int cloudSize = cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i){
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        *nearKeyframes.all_cloud += *transformPointCloud(cloudKeyFrames[keyNear].all_cloud, &cloudKeyPoses6D->points[keyNear]);
    }

    if (nearKeyframes.all_cloud->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes.all_cloud);
    downSizeFilterICP.filter(*cloud_temp);
    nearKeyframes.all_cloud->clear(); // redundant?
    *nearKeyframes.all_cloud = *cloud_temp;
} 

void MultiSession::Session::loopFindNearKeyframesLocalCoord(KeyFrame& nearKeyframes, const int& key, const int& searchNum){
    // extract near keyframes
    nearKeyframes.all_cloud->clear();
    int cloudSize = cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i){
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        *nearKeyframes.all_cloud += *transformPointCloud(cloudKeyFrames[keyNear].all_cloud, &originPoses6D->points[0]);
    }

    if (nearKeyframes.all_cloud->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes.all_cloud);
    downSizeFilterICP.filter(*cloud_temp);
    nearKeyframes.all_cloud->clear(); // redundant?
    *nearKeyframes.all_cloud = *cloud_temp;
} 

// load pointcloud
void MultiSession::Session::loadSessionKeyframePointclouds(){
    std::string pcd_dir = session_dir_path_ + "/PCDs/";

    // parse names (un-sorted)
    std::vector<std::pair<int, std::string>> pcd_names;
    for(auto& _pcd : fs::directory_iterator(pcd_dir)) {
        std::string pcd_name = _pcd.path().filename();
        std::stringstream pcd_name_stream {pcd_name};
        std::string pcd_idx_str; 
        getline(pcd_name_stream, pcd_idx_str, ',');
        int pcd_idx = std::stoi(pcd_idx_str);
        std::string pcd_name_filepath = _pcd.path();

        pcd_names.emplace_back(std::make_pair(pcd_idx, pcd_name_filepath));
    }    

    // filesort
    std::sort(pcd_names.begin(), pcd_names.end(), pairIntAndStringSort);   // easy 

    // load PCDs
    int num_pcd_loaded = 0;
    for (auto const& _pcd_name: pcd_names){
        std::cout << " load " << _pcd_name.second << std::endl;
        pcl::PointCloud<PointType>::Ptr thisCloudFrame(new pcl::PointCloud<PointType>());   
        // pcl::io::loadPCDFile<PointType> (_pcd_name.second, *thisCloudFrame);  // TODO: cannot load PointType
        pcl::PointCloud<pcl::PointXYZI>::Ptr thisCloudFrame_(new pcl::PointCloud<pcl::PointXYZI>());   
        pcl::io::loadPCDFile<pcl::PointXYZI> (_pcd_name.second, *thisCloudFrame_);
        // std::cout << 1 << std::endl;
        // thisCloudFrame = convertPointXYZI(thisCloudFrame_);
        // std::cout << 1 << std::endl;
        for(int i = 0; i < thisCloudFrame_->points.size(); i++){
            PointType pt;
            pt.x = thisCloudFrame_->points[i].x;
            pt.y = thisCloudFrame_->points[i].y;
            pt.z = thisCloudFrame_->points[i].z;
            thisCloudFrame->points.emplace_back(pt);
        }
        // std::cout << 1 << std::endl;
        KeyFrame thisKeyFrame;
        thisKeyFrame.all_cloud = thisCloudFrame;
        cloudKeyFrames.push_back(thisKeyFrame);

        num_pcd_loaded++;
        if(num_pcd_loaded > nodes_.size()) {
            std::cout << "error in the num of pcds" << std::endl;
            break;
        }
    }
    std::cout << "PCDs are loaded (" << name_ << ")" << std::endl;
}

// load sc
void MultiSession::Session::loadSessionScanContextDescriptors(){
    std::string scd_dir = session_dir_path_ + "/SCDs/";

    // parse names (un-sorted)
    std::vector<std::pair<int, std::string>> scd_names;
    for(auto& _scd : fs::directory_iterator(scd_dir)){
        std::string scd_name = _scd.path().filename();

        std::stringstream scd_name_stream {scd_name};
        std::string scd_idx_str; 
        getline(scd_name_stream, scd_idx_str, '.');
        int scd_idx = std::stoi(scd_idx_str);
        std::string scd_name_filepath = _scd.path();

        scd_names.emplace_back(std::make_pair(scd_idx, scd_name_filepath));
    }    

    // filesort
    std::sort(scd_names.begin(), scd_names.end(), pairIntAndStringSort);   // easy 
    
    // load SCDs
    int num_scd_loaded = 0;
    for (auto const& _scd_name: scd_names){
        std::cout << "load a SCD: " << _scd_name.second << endl;
        Eigen::MatrixXd scd = readSCD(_scd_name.second);
        cloudKeyFrames[num_scd_loaded].scv_od = scd;  // TODO: how to use as scv-od
        scManager.saveScancontextAndKeys(scd);

        num_scd_loaded++;
        if(num_scd_loaded > nodes_.size()) {
            std::cout << "error in the num of scds" << std::endl;
            break;
        }
    }
    std::cout << "SCDs are loaded (" << name_ << ")" << std::endl;
}

// load pose-graph
void MultiSession::Session::loadSessionGraph() 
{
    std::string posefile_path = session_dir_path_ + "/singlesession_posegraph.g2o";

    std::ifstream posefile_handle (posefile_path);
    std::string strOneLine;
    while (getline(posefile_handle, strOneLine)) 
    {
        G2oLineInfo line_info = splitG2oFileLine(strOneLine);

        // save variables (nodes)
        if( isTwoStringSame(line_info.type, G2oLineInfo::kVertexTypeName) ) {
            Node this_node { line_info.curr_idx, gtsam::Pose3( 
                gtsam::Rot3(gtsam::Quaternion(line_info.quat[3], line_info.quat[0], line_info.quat[1], line_info.quat[2])), // xyzw to wxyz
                gtsam::Point3(line_info.trans[0], line_info.trans[1], line_info.trans[2])) }; 
            nodes_.insert(std::pair<int, Node>(line_info.curr_idx, this_node)); 
        }
 
        // save edges 
        if( isTwoStringSame(line_info.type, G2oLineInfo::kEdgeTypeName) ) {
            Edge this_edge { line_info.prev_idx, line_info.curr_idx, gtsam::Pose3( 
                gtsam::Rot3(gtsam::Quaternion(line_info.quat[3], line_info.quat[0], line_info.quat[1], line_info.quat[2])), // xyzw to wxyz
                gtsam::Point3(line_info.trans[0], line_info.trans[1], line_info.trans[2])) }; 
            edges_.insert(std::pair<int, Edge>(line_info.prev_idx, this_edge)); 
        }
    }
    
    initKeyPoses();

    // 
    ROS_INFO_STREAM("\033[1;32m Session loaded: " << posefile_path << "\033[0m");
    ROS_INFO_STREAM("\033[1;32m - num nodes: " << nodes_.size() << "\033[0m");
}


// IncreMapping

gtsam::Pose3 MultiSession::IncreMapping::getPoseOfIsamUsingKey (const gtsam::Key _key) {
    const gtsam::Value& pose_value = isam->calculateEstimate(_key);
    auto p_pose_value = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&pose_value);
    gtsam::Pose3 pose = gtsam::Pose3{p_pose_value->value()};
    return pose;
}


void MultiSession::IncreMapping::writeAllSessionsTrajectories(std::string _postfix = ""){
    // parse
    std::map<int, gtsam::Pose3> parsed_anchor_transforms;
    std::map<int, std::vector<gtsam::Pose3>> parsed_poses;

    isamCurrentEstimate = isam->calculateEstimate();
    for(const auto& key_value: isamCurrentEstimate) {

        int curr_node_idx = int(key_value.key); // typedef std::uint64_t Key

        std::vector<int> parsed_digits;
        collect_digits(parsed_digits, curr_node_idx);
        int session_idx = parsed_digits.at(0);
        int anchor_node_idx = genAnchorNodeIdx(session_idx);

        auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&key_value.value);
        if (!p) continue;
        gtsam::Pose3 curr_node_pose = gtsam::Pose3{p->value()};

        if( curr_node_idx == anchor_node_idx ) { 
            // anchor node 
            parsed_anchor_transforms[session_idx] = curr_node_pose;
        } else { 
            // general nodes
            parsed_poses[session_idx].push_back(curr_node_pose);
        }
    }

    std::map<int, std::string> session_names;
    for(auto& _sess_pair: sessions_)
    {
        auto& _sess = _sess_pair.second;
        session_names[_sess.index_] = _sess.name_;
    }

    // write
    for(auto& _session_info: parsed_poses) {
        int session_idx = _session_info.first;

    	std::string filename_local = sessions_dir_ + save_directory_ + session_names[session_idx] + "_local_" + _postfix + ".txt";
    	std::string filename_central = sessions_dir_ +  save_directory_ + session_names[session_idx] + "_central_" + _postfix + ".txt";
        cout << filename_central << endl;

        std::fstream stream_local(filename_local.c_str(), std::fstream::out);
        std::fstream stream_central(filename_central.c_str(), std::fstream::out);

        gtsam::Pose3 anchor_transform = parsed_anchor_transforms[session_idx];
        for(auto& _pose: _session_info.second) {
            writePose3ToStream(stream_local, _pose);

            gtsam::Pose3 pose_central = anchor_transform * _pose; // se3 compose (oplus) 
            writePose3ToStream(stream_central, pose_central);
        }
    }
}

void MultiSession::IncreMapping::run( int iteration ){
    
    std::cout << "----------  current estimate -----------" << std::endl;
    optimizeMultisesseionGraph(true, iteration); // optimize the graph with existing edges 
    writeAllSessionsTrajectories(std::string("bfr_intersession_loops"));

    std::cout << "----------  sc estimate -----------" << std::endl;
    detectInterSessionSCloops(); // detectInterSessionRSloops was internally done while sc detection 
    addSCloops();
    optimizeMultisesseionGraph(true, iteration); // optimize the graph with existing edges + SC loop edges

    std::cout << "----------  rs estimate -----------" << std::endl;
    bool toOpt = addRSloops(); // using the optimized estimates (rough alignment using SC)
    optimizeMultisesseionGraph(toOpt, iteration); // optimize the graph with existing edges + SC loop edges + RS loop edges

    writeAllSessionsTrajectories(std::string("aft_intersession_loops"));

    std::string aftPose1 = sessions_dir_ + save_directory_ + "aft_tansformation1.pcd";
    pcl::io::savePCDFileASCII(aftPose1, *sessions_.at(target_sess_idx).cloudKeyPoses6D);

    std::string aftPose2 = sessions_dir_ + save_directory_ + "aft_tansformation2.pcd";
    pcl::io::savePCDFileASCII(aftPose2, *sessions_.at(source_sess_idx).cloudKeyPoses6D);

    getReloKeyFrames();  // get relo clouds

    std::string aftMap2 = sessions_dir_ + save_directory_ + "aft_map2.pcd";
    downSizeFilterPub.setInputCloud(regisMap_);
    downSizeFilterPub.filter(*regisMap_);
    pcl::io::savePCDFileASCII(aftMap2, *regisMap_);

    std::cout << "save all optimization files" << std::endl;
}

void MultiSession::IncreMapping::initNoiseConstants(){
    // Variances Vector6 order means
    // : rad*rad, rad*rad, rad*rad, meter*meter, meter*meter, meter*meter
    {
        gtsam::Vector Vector6(6);
        Vector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
        priorNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    }
    {
        gtsam::Vector Vector6(6);
        Vector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
        odomNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    }
    {
        gtsam::Vector Vector6(6);
        Vector6 << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3;
        loopNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    }
    {
        gtsam::Vector Vector6(6);
        Vector6 << M_PI*M_PI, M_PI*M_PI, M_PI*M_PI, 1e8, 1e8, 1e8;
        // Vector6 << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3;
        largeNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    }

    float robustNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); 
    robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
    robustNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
    ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)
}

void MultiSession::IncreMapping::initOptimizer(){
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1; // TODO: study later
    isam = new gtsam::ISAM2(parameters);
}


void MultiSession::IncreMapping::updateSessionsPoses(){
    for(auto& _sess_pair: sessions_)
    {
        auto& _sess = _sess_pair.second;
        gtsam::Pose3 anchor_transform = isamCurrentEstimate.at<gtsam::Pose3>(genAnchorNodeIdx(_sess.index_));
        // cout << anchor_transform << endl;
        _sess.updateKeyPoses(isam, anchor_transform);
    }
} // updateSessionsPoses


void MultiSession::IncreMapping::optimizeMultisesseionGraph(bool _toOpt, int iteration){
    if(!_toOpt)
        return;

    isam->update(gtSAMgraph, initialEstimate);
    for(int i = 0; i < iteration; i++){
        isam->update();
    }
    isamCurrentEstimate = isam->calculateEstimate(); // must be followed by update 

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    updateSessionsPoses(); 

    if(is_display_debug_msgs_) {
        std::cout << "***** variable values after optimization" << iteration << " *****" << std::endl;
        // std::cout << std::endl;
        // isamCurrentEstimate.print("Current estimate: ");
        // std::cout << std::endl;
        // std::ofstream os("/home/user/Documents/catkin2021/catkin_ltmapper/catkin_ltmapper_dev/src/ltmapper/data/3d/kaist/PoseGraphExample.dot");
        // gtSAMgraph.saveGraph(os, isamCurrentEstimate);
    }

} // optimizeMultisesseionGraph


std::experimental::optional<gtsam::Pose3> MultiSession::IncreMapping::doICPVirtualRelative( // for SC loop
    Session& target_sess, Session& source_sess, 
    const int& loop_idx_target_session, const int& loop_idx_source_session){

    // parse pointclouds
    mtx.lock();
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    KeyFrame cureKeyframe;
    KeyFrame targetKeyframe;
    cureKeyframe.all_cloud = cureKeyframeCloud;
    targetKeyframe.all_cloud = targetKeyframeCloud;

    int base_key = 0; // its okay. (using the origin for sc loops' co-base) 
    int historyKeyframeSearchNum = 2; // TODO move to yaml 

    source_sess.loopFindNearKeyframesLocalCoord(cureKeyframe, loop_idx_source_session, 0);
    target_sess.loopFindNearKeyframesLocalCoord(targetKeyframe, loop_idx_target_session, historyKeyframeSearchNum); 
    mtx.unlock(); // unlock after loopFindNearKeyframesWithRespectTo because many new in the loopFindNearKeyframesWithRespectTo

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(30); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(10);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframe.all_cloud);
    icp.setInputTarget(targetKeyframe.all_cloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
 
    // TODO icp align with initial 

    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        mtx.lock();
        std::cout << "  [SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        mtx.unlock();
        return std::experimental::nullopt;
    } else {
        mtx.lock();
        std::cout << "  [SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
        KeyFrame keyframe;
        keyframe.reloScore = icp.getFitnessScore();
        keyframe.reloTargetIdx = loop_idx_target_session;
        reloKeyFrames.emplace_back(std::make_pair(loop_idx_source_session, keyframe));
        mtx.unlock();
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

    return poseFrom.between(poseTo);
} // doICPVirtualRelative


std::experimental::optional<gtsam::Pose3> MultiSession::IncreMapping::doICPGlobalRelative( // For RS loop
    Session& target_sess, Session& source_sess, 
    const int& loop_idx_target_session, const int& loop_idx_source_session){
    // parse pointclouds
    mtx.lock();
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    KeyFrame cureKeyframe;
    KeyFrame targetKeyframe;
    cureKeyframe.all_cloud = cureKeyframeCloud;
    targetKeyframe.all_cloud = targetKeyframeCloud;

    int base_key = 0; // its okay. (using the origin for sc loops' co-base) 
    int historyKeyframeSearchNum = 2; // TODO move to yaml 

    source_sess.loopFindNearKeyframesCentralCoord(cureKeyframe, loop_idx_source_session, 0);
    target_sess.loopFindNearKeyframesCentralCoord(targetKeyframe, loop_idx_target_session, historyKeyframeSearchNum); 
    mtx.unlock(); // unlock after loopFindNearKeyframesWithRespectTo because many new in the loopFindNearKeyframesWithRespectTo

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(30); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(10);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframe.all_cloud);
    icp.setInputTarget(targetKeyframe.all_cloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
 
    // TODO icp align with initial 

    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        mtx.lock();
        std::cout << "  [RS loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this RS loop." << std::endl;
        mtx.unlock();
        return std::experimental::nullopt;
    } else {
        mtx.lock();
        std::cout << "  [RS loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this RS loop." << std::endl;
        mtx.unlock();
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    Eigen::Affine3f tWrong = pclPointToAffine3f(source_sess.cloudKeyPoses6D->points[loop_idx_source_session]);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(target_sess.cloudKeyPoses6D->points[loop_idx_target_session]);

    return poseFrom.between(poseTo);
} // doICPGlobalRelative


void MultiSession::IncreMapping::detectInterSessionSCloops() // using ScanContext
{
    auto& target_sess = sessions_.at(target_sess_idx); 
    auto& source_sess = sessions_.at(source_sess_idx);

    // Detect loop closures: Find loop edge index pairs 
    SCLoopIdxPairs_.clear();
    RSLoopIdxPairs_.clear();
    auto& target_scManager = target_sess.scManager;
    auto& source_scManager = source_sess.scManager;
    TicToc time_count;
    for (int source_node_idx=0; source_node_idx < int(source_scManager.polarcontexts_.size()); source_node_idx++)
    {
        std::vector<float> source_node_key = source_scManager.polarcontext_invkeys_mat_.at(source_node_idx);
        Eigen::MatrixXd source_node_scd = source_scManager.polarcontexts_.at(source_node_idx);

        auto detectResult = target_scManager.detectLoopClosureIDBetweenSession(source_node_key, source_node_scd); // first: nn index, second: yaw diff 

        int loop_idx_source_session = source_node_idx;
        int loop_idx_target_session = detectResult.first;

        if(loop_idx_target_session == -1) { // TODO using NO_LOOP_FOUND rather using -1 
            RSLoopIdxPairs_.emplace_back(std::make_pair(-1, loop_idx_source_session)); // -1 will be later be found (nn pose). 
            continue;
        }

        SCLoopIdxPairs_.emplace_back(std::make_pair(loop_idx_target_session, loop_idx_source_session));
    }
    double ave_time = time_count.toc()/SCLoopIdxPairs_.size();
    ROS_INFO_STREAM("\033[1;32m Total " << SCLoopIdxPairs_.size() << " inter-session loops are found. Average time "<<  ave_time <<  " \033[0m" );
} // detectInterSessionSCloops


void MultiSession::IncreMapping::detectInterSessionRSloops() // using ScanContext
{
    
} // detectInterSessionRSloops


void MultiSession::IncreMapping::addAllSessionsToGraph(){
    for(auto& _sess_pair: sessions_)    
    {
        auto& _sess = _sess_pair.second;
        initTrajectoryByAnchoring(_sess);
        addSessionToCentralGraph(_sess);   
    }
} // addAllSessionsToGraph


std::vector<std::pair<int, int>> MultiSession::IncreMapping::equisampleElements(
    const std::vector<std::pair<int, int>>& _input_pair, float _gap, int _num_sampled){
    std::vector<std::pair<int, int>> sc_loop_idx_pairs_sampled;

    int equisampling_counter { 0 }; 

    std::vector<int> equisampled_idx;
    for (int i=0; i<_num_sampled; i++)
        equisampled_idx.emplace_back(std::round(float(i) * _gap));

    for (auto& _idx: equisampled_idx)
        sc_loop_idx_pairs_sampled.emplace_back(_input_pair.at(_idx));

    return sc_loop_idx_pairs_sampled;
}

void MultiSession::IncreMapping::addSCloops(){
    if(SCLoopIdxPairs_.empty()) 
        return;

    // equi-sampling sc loops 
    int num_scloops_all_found = int(SCLoopIdxPairs_.size());
    int num_scloops_to_be_added = num_scloops_all_found;
    int equisampling_gap = num_scloops_all_found / num_scloops_to_be_added;

    auto sc_loop_idx_pairs_sampled = equisampleElements(SCLoopIdxPairs_, equisampling_gap, num_scloops_to_be_added);
    auto num_scloops_sampled = sc_loop_idx_pairs_sampled.size();

    // add selected sc loops 
    auto& target_sess = sessions_.at(target_sess_idx); 
    auto& source_sess = sessions_.at(source_sess_idx);

    std::vector<int> idx_added_loops; 
    idx_added_loops.reserve(num_scloops_sampled);
    #pragma omp parallel for num_threads(numberOfCores)
    for (int ith = 0; ith < num_scloops_sampled; ith++) 
    {
        auto& _loop_idx_pair = sc_loop_idx_pairs_sampled.at(ith);
        int loop_idx_target_session = _loop_idx_pair.first;
        int loop_idx_source_session = _loop_idx_pair.second;

        auto relative_pose_optional = doICPVirtualRelative(target_sess, source_sess, loop_idx_target_session, loop_idx_source_session); 

        if(relative_pose_optional) {
            mtx.lock();
            gtsam::Pose3 relative_pose = relative_pose_optional.value();
            gtSAMgraph.add( gtsam::BetweenFactorWithAnchoring<gtsam::Pose3>(
                genGlobalNodeIdx(target_sess_idx, loop_idx_target_session), genGlobalNodeIdx(source_sess_idx, loop_idx_source_session),
                genAnchorNodeIdx(target_sess_idx), genAnchorNodeIdx(source_sess_idx), 
                relative_pose, robustNoise) );
            mtx.unlock();

            // debug msg (would be removed later)
            mtx.lock();
            idx_added_loops.emplace_back(loop_idx_target_session); 
            cout << "SCdetector found an inter-session edge between " 
                << genGlobalNodeIdx(target_sess_idx, loop_idx_target_session) << " and " << genGlobalNodeIdx(source_sess_idx, loop_idx_source_session) 
                << " (anchor nodes are " << genAnchorNodeIdx(target_sess_idx) << " and " << genAnchorNodeIdx(source_sess_idx) << ")" << endl;
            mtx.unlock();
        }
    }
} // addSCloops


double MultiSession::IncreMapping::calcInformationGainBtnTwoNodes(const int loop_idx_target_session, const int loop_idx_source_session){
    auto pose_s1 = isamCurrentEstimate.at<gtsam::Pose3>( genGlobalNodeIdx(target_sess_idx, loop_idx_target_session) ); // node: s1 is the central 
    auto pose_s2 = isamCurrentEstimate.at<gtsam::Pose3>( genGlobalNodeIdx(source_sess_idx, loop_idx_source_session) );
    auto pose_s1_anchor = isamCurrentEstimate.at<gtsam::Pose3>( genAnchorNodeIdx(target_sess_idx) );
    auto pose_s2_anchor = isamCurrentEstimate.at<gtsam::Pose3>( genAnchorNodeIdx(source_sess_idx) );

    gtsam::Pose3 hx1 = gtsam::traits<gtsam::Pose3>::Compose(pose_s1_anchor, pose_s1); // for the updated jacobian, see line 60, 219, https://gtsam.org/doxygen/a00053_source.html
    gtsam::Pose3 hx2 = gtsam::traits<gtsam::Pose3>::Compose(pose_s2_anchor, pose_s2); 
    gtsam::Pose3 estimated_relative_pose = gtsam::traits<gtsam::Pose3>::Between(hx1, hx2); 

    gtsam::Matrix H_s1, H_s2, H_s1_anchor, H_s2_anchor;
    auto loop_factor = gtsam::BetweenFactorWithAnchoring<gtsam::Pose3>(
        genGlobalNodeIdx(target_sess_idx, loop_idx_target_session), genGlobalNodeIdx(source_sess_idx, loop_idx_source_session),
        genAnchorNodeIdx(target_sess_idx), genAnchorNodeIdx(source_sess_idx), 
        estimated_relative_pose, robustNoise);
    loop_factor.evaluateError(pose_s1, pose_s2, pose_s1_anchor, pose_s2_anchor, 
                                 H_s1,    H_s2,    H_s1_anchor,    H_s2_anchor);

    gtsam::Matrix pose_s1_cov = isam->marginalCovariance(genGlobalNodeIdx(target_sess_idx, loop_idx_target_session)); // note: typedef Eigen::MatrixXd  gtsam::Matrix
    gtsam::Matrix pose_s2_cov = isam->marginalCovariance(genGlobalNodeIdx(source_sess_idx, loop_idx_source_session));

    // calc S and information gain 
    gtsam::Matrix Sy = Eigen::MatrixXd::Identity(6, 6); // measurement noise, assume fixed
    gtsam::Matrix S = Sy + (H_s1*pose_s1_cov*H_s1.transpose() + H_s2*pose_s2_cov*H_s2.transpose());
    double Sdet = S.determinant(); 
    double information_gain = 0.5 * log( Sdet / Sy.determinant());

    return information_gain;
}

void MultiSession::IncreMapping::findNearestRSLoopsTargetNodeIdx() // based-on information gain 
{
    std::vector<std::pair<int, int>> validRSLoopIdxPairs;

    for(std::size_t i=0; i<RSLoopIdxPairs_.size(); i++)
    {
        // curr query pose 
        auto rsloop_idx_pair = RSLoopIdxPairs_.at(i);
        auto rsloop_idx_source_session = rsloop_idx_pair.second;
        auto rsloop_global_idx_source_session = genGlobalNodeIdx(source_sess_idx, rsloop_idx_source_session);

        auto source_node_idx = rsloop_idx_source_session;
        auto query_pose = isamCurrentEstimate.at<gtsam::Pose3>(rsloop_global_idx_source_session);
        gtsam::Pose3 query_sess_anchor_transform = isamCurrentEstimate.at<gtsam::Pose3>(genAnchorNodeIdx(source_sess_idx));
        auto query_pose_central_coord = query_sess_anchor_transform * query_pose;

        // find nn pose idx in the target sess 
        auto& target_sess = sessions_.at(target_sess_idx); 
        std::vector<int> target_node_idxes_within_ball;
        for (int target_node_idx=0; target_node_idx < int(target_sess.nodes_.size()); target_node_idx++) {
            auto target_pose = isamCurrentEstimate.at<gtsam::Pose3>(genGlobalNodeIdx(target_sess_idx, target_node_idx));
            if( poseDistance(query_pose_central_coord, target_pose) < 10.0 ) // 10 is a hard-coding for fast test
            {
                target_node_idxes_within_ball.push_back(target_node_idx);
                // cout << "(all) RS pair detected: " << target_node_idx << " <-> " << source_node_idx << endl;    
            }
        }

        // if no nearest one, skip 
        if(target_node_idxes_within_ball.empty())
            continue;

        // selected a single one having maximum information gain  
        int selected_near_target_node_idx; 
        double max_information_gain {0.0};     
        for (int i=0; i<target_node_idxes_within_ball.size(); i++) 
        {
            auto nn_target_node_idx = target_node_idxes_within_ball.at(i);
            double this_information_gain = calcInformationGainBtnTwoNodes(nn_target_node_idx, source_node_idx);
            if(this_information_gain > max_information_gain) {
                selected_near_target_node_idx = nn_target_node_idx;
                max_information_gain = this_information_gain;
            }
        }

        // cout << "RS pair detected: " << selected_near_target_node_idx << " <-> " << source_node_idx << endl;    
        // cout << "info gain: " << max_information_gain << endl;    
             
        validRSLoopIdxPairs.emplace_back(std::pair<int, int>{selected_near_target_node_idx, source_node_idx});
    }

    // update 
    RSLoopIdxPairs_.clear();
    RSLoopIdxPairs_.resize((int)(validRSLoopIdxPairs.size()));
    std::copy( validRSLoopIdxPairs.begin(), validRSLoopIdxPairs.end(), RSLoopIdxPairs_.begin() );
}


bool MultiSession::IncreMapping::addRSloops(){

    // find nearest target node idx 
    findNearestRSLoopsTargetNodeIdx();

    // parse RS loop src idx
    int num_rsloops_all_found = int(RSLoopIdxPairs_.size());
    if( num_rsloops_all_found == 0 )
        return false;

    int num_rsloops_to_be_added = num_rsloops_all_found;
    int equisampling_gap = num_rsloops_all_found / num_rsloops_to_be_added;

    auto rs_loop_idx_pairs_sampled = equisampleElements(RSLoopIdxPairs_, equisampling_gap, num_rsloops_to_be_added);
    auto num_rsloops_sampled = rs_loop_idx_pairs_sampled.size();

    cout << "num of RS pair: " << num_rsloops_all_found << endl;         
    cout << "num of sampled RS pair: " << num_rsloops_sampled << endl;         

    // add selected rs loops 
    auto& target_sess = sessions_.at(target_sess_idx); 
    auto& source_sess = sessions_.at(source_sess_idx);

    #pragma omp parallel for num_threads(numberOfCores)
    for (int ith = 0; ith < num_rsloops_sampled; ith++) {
        auto& _loop_idx_pair = rs_loop_idx_pairs_sampled.at(ith);
        int loop_idx_target_session = _loop_idx_pair.first;
        int loop_idx_source_session = _loop_idx_pair.second;

        auto relative_pose_optional = doICPGlobalRelative(target_sess, source_sess, loop_idx_target_session, loop_idx_source_session); 

        if(relative_pose_optional) {
            mtx.lock();
            gtsam::Pose3 relative_pose = relative_pose_optional.value();
            gtSAMgraph.add( gtsam::BetweenFactorWithAnchoring<gtsam::Pose3>(
                genGlobalNodeIdx(target_sess_idx, loop_idx_target_session), genGlobalNodeIdx(source_sess_idx, loop_idx_source_session),
                genAnchorNodeIdx(target_sess_idx), genAnchorNodeIdx(source_sess_idx), 
                relative_pose, robustNoise) );
            mtx.unlock();

            // debug msg (would be removed later)
            mtx.lock();
            cout << "RS loop detector found an inter-session edge between " 
                << genGlobalNodeIdx(target_sess_idx, loop_idx_target_session) << " and " << genGlobalNodeIdx(source_sess_idx, loop_idx_source_session) 
                << " (anchor nodes are " << genAnchorNodeIdx(target_sess_idx) << " and " << genAnchorNodeIdx(source_sess_idx) << ")" << endl;
            mtx.unlock();
        }
    }

    return true;
} // addRSloops


void MultiSession::IncreMapping::initTrajectoryByAnchoring(const Session& _sess){
    int this_session_anchor_node_idx = genAnchorNodeIdx(_sess.index_);

    if(_sess.is_base_session_) {
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin, priorNoise));
    } else {
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin, largeNoise));
    }

    initialEstimate.insert(this_session_anchor_node_idx, poseOrigin);
} // initTrajectoryByAnchoring


void MultiSession::IncreMapping::addSessionToCentralGraph(const Session& _sess){
    // add nodes 
    for( auto& _node: _sess.nodes_){        
        int node_idx = _node.second.idx;
        auto& curr_pose = _node.second.initial;

        int prev_node_global_idx = genGlobalNodeIdx(_sess.index_, node_idx - 1);
        int curr_node_global_idx = genGlobalNodeIdx(_sess.index_, node_idx);

        gtsam::Vector Vector6(6);
        if(node_idx == 0) { // TODO consider later if the initial node idx is not zero (but if using SC-LIO-SAM, don't care)
            // prior node 
            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(curr_node_global_idx, curr_pose, priorNoise));
            initialEstimate.insert(curr_node_global_idx, curr_pose);
        } else { 
            // odom nodes 
            initialEstimate.insert(curr_node_global_idx, curr_pose);
        }
    }
    
    // add edges 
    for( auto& _edge: _sess.edges_){
        int from_node_idx = _edge.second.from_idx;
        int to_node_idx = _edge.second.to_idx;
        
        int from_node_global_idx = genGlobalNodeIdx(_sess.index_, from_node_idx);
        int to_node_global_idx = genGlobalNodeIdx(_sess.index_, to_node_idx);

        gtsam::Pose3 relative_pose = _edge.second.relative;
        if( std::abs(to_node_idx - from_node_idx) == 1) {
            // odom edge (temporally consecutive)
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(from_node_global_idx, to_node_global_idx, relative_pose, odomNoise));
            if(is_display_debug_msgs_) cout << "add an odom edge between " << from_node_global_idx << " and " << to_node_global_idx << endl;
        } else {
            // loop edge
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(from_node_global_idx, to_node_global_idx, relative_pose, robustNoise));
            if(is_display_debug_msgs_) cout << "add a loop edge between " << from_node_global_idx << " and " << to_node_global_idx << endl;
        }
    }
}

void MultiSession::IncreMapping::loadAllSessions() {
    // pose 
    ROS_INFO_STREAM("\033[1;32m Load sessions' pose data from: " << sessions_dir_ << "\033[0m");
    for(auto& _session_entry : fs::directory_iterator(sessions_dir_)) {
        std::string session_name = _session_entry.path().filename();        
        // std::cout << session_name << " " << central_sess_name_ << std::endl;
        if( !isTwoStringSame(session_name, central_sess_name_) & !isTwoStringSame(session_name, query_sess_name_) ) {
            continue; // jan. 2021. currently designed for two-session ver. (TODO: be generalized for N-session co-optimization)
        }

        // save a session (read graph txt flie and load nodes and edges internally)
        int session_idx;
        if(isTwoStringSame(session_name, central_sess_name_))
            session_idx = target_sess_idx;
        else
            session_idx = source_sess_idx;

        std::string session_dir_path = _session_entry.path();

        // sessions_.emplace_back(Session(session_idx, session_name, session_dir_path, isTwoStringSame(session_name, central_sess_name_)));
        sessions_.insert( std::make_pair(session_idx, 
                                         Session(session_idx, session_name, session_dir_path, isTwoStringSame(session_name, central_sess_name_))) );

        // MultiSession::IncreMapping::num_sessions++; // incr the global index // TODO: make this private and provide incrSessionIdx
    }

    std::cout << std::boolalpha;   
    ROS_INFO_STREAM("\033[1;32m Total : " << sessions_.size() << " sessions are loaded.\033[0m");
    std::for_each( sessions_.begin(), sessions_.end(), [](auto& _sess_pair) { 
                cout << " — " << _sess_pair.second.name_ << " (is central: " << _sess_pair.second.is_base_session_ << ")" << endl; 
                } );
} // loadSession


void MultiSession::IncreMapping::visualizeLoopClosure()
{
    std::string odometryFrame = "camera_init";

    if (SCLoopIdxPairs_.empty() && RSLoopIdxPairs_.empty())
        return;

    // show sc
    visualization_msgs::MarkerArray markerArray_sc;
    // 回环顶点
    visualization_msgs::Marker markerNode_sc;
    markerNode_sc.header.frame_id = odometryFrame;
    // markerNode_sc.header.stamp = timeLaserInfoStamp;
    // action对应的操作:ADD=0、MODIFY=0、DELETE=2、DELETEALL=3,即添加、修改、删除、全部删除
    markerNode_sc.action = visualization_msgs::Marker::ADD;
    // 设置形状:球体
    markerNode_sc.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode_sc.ns = "loop_nodes";
    markerNode_sc.id = 0;
    markerNode_sc.pose.orientation.w = 1;
    // 尺寸
    markerNode_sc.scale.x = 0.3;
    markerNode_sc.scale.y = 0.3;
    markerNode_sc.scale.z = 0.3;
    // 颜色
    markerNode_sc.color.r = 0.9;
    markerNode_sc.color.g = 0;
    markerNode_sc.color.b = 0;
    markerNode_sc.color.a = 1;
    // 回环边
    visualization_msgs::Marker markerEdge_sc;
    markerEdge_sc.header.frame_id = odometryFrame;
    // markerEdge_sc.header.stamp = timeLaserInfoStamp;
    markerEdge_sc.action = visualization_msgs::Marker::ADD;
    // 设置形状:线
    markerEdge_sc.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge_sc.ns = "loop_edges";
    markerEdge_sc.id = 1;
    markerEdge_sc.pose.orientation.w = 1;
    markerEdge_sc.scale.x = 0.1;
    markerEdge_sc.color.r = 0.9;
    markerEdge_sc.color.g = 0;
    markerEdge_sc.color.b = 0;
    markerEdge_sc.color.a = 1;

    for (auto& it : SCLoopIdxPairs_){
        int key_cur = it.first;
        int key_pre = it.second;
        geometry_msgs::Point p;
        p.x = sessions_.at(target_sess_idx).cloudKeyPoses6D->points[key_cur].x;
        p.y = sessions_.at(target_sess_idx).cloudKeyPoses6D->points[key_cur].y;
        p.z = sessions_.at(target_sess_idx).cloudKeyPoses6D->points[key_cur].z;
        markerNode_sc.points.push_back(p);
        markerEdge_sc.points.push_back(p);
        p.x = sessions_.at(source_sess_idx).cloudKeyPoses6D->points[key_pre].x;
        p.y = sessions_.at(source_sess_idx).cloudKeyPoses6D->points[key_pre].y;
        p.z = sessions_.at(source_sess_idx).cloudKeyPoses6D->points[key_pre].z;
        markerNode_sc.points.push_back(p);
        markerEdge_sc.points.push_back(p);
    }

    markerArray_sc.markers.push_back(markerNode_sc);
    markerArray_sc.markers.push_back(markerEdge_sc);
    pubSCLoop.publish(markerArray_sc);

    // show rs
    visualization_msgs::MarkerArray markerArray_rs;
    // rs回环顶点
    visualization_msgs::Marker markerNode_rs;
    markerNode_rs.header.frame_id = odometryFrame;
    // markerNode_rs.header.stamp = timeLaserInfoStamp;
    // action对应的操作:ADD=0、MODIFY=0、DELETE=2、DELETEALL=3,即添加、修改、删除、全部删除
    markerNode_rs.action = visualization_msgs::Marker::ADD;
    // 设置形状:球体
    markerNode_rs.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode_rs.ns = "loop_nodes";
    markerNode_rs.id = 0;
    markerNode_rs.pose.orientation.w = 1;
    // 尺寸
    markerNode_rs.scale.x = 0.3;
    markerNode_rs.scale.y = 0.3;
    markerNode_rs.scale.z = 0.3;
    // 颜色
    markerNode_rs.color.r = 0;
    markerNode_rs.color.g = 0;
    markerNode_rs.color.b = 0.9;
    markerNode_rs.color.a = 1;
    // 回环边
    visualization_msgs::Marker markerEdge_rs;
    markerEdge_rs.header.frame_id = odometryFrame;
    // markerEdge_rs.header.stamp = timeLaserInfoStamp;
    markerEdge_rs.action = visualization_msgs::Marker::ADD;
    // 设置形状:线
    markerEdge_rs.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge_rs.ns = "loop_edges";
    markerEdge_rs.id = 1;
    markerEdge_rs.pose.orientation.w = 1;
    markerEdge_rs.scale.x = 0.1;
    markerEdge_rs.color.r = 0;
    markerEdge_rs.color.g = 0;
    markerEdge_rs.color.b = 0.9;
    markerEdge_rs.color.a = 1;

    for (auto& it : RSLoopIdxPairs_){
        int key_cur = it.first;
        int key_pre = it.second;
        geometry_msgs::Point p;
        p.x = sessions_.at(target_sess_idx).cloudKeyPoses6D->points[key_cur].x;
        p.y = sessions_.at(target_sess_idx).cloudKeyPoses6D->points[key_cur].y;
        p.z = sessions_.at(target_sess_idx).cloudKeyPoses6D->points[key_cur].z;
        markerNode_rs.points.push_back(p);
        markerEdge_rs.points.push_back(p);
        p.x = sessions_.at(source_sess_idx).cloudKeyPoses6D->points[key_pre].x;
        p.y = sessions_.at(source_sess_idx).cloudKeyPoses6D->points[key_pre].y;
        p.z = sessions_.at(source_sess_idx).cloudKeyPoses6D->points[key_pre].z;
        markerNode_rs.points.push_back(p);
        markerEdge_rs.points.push_back(p);
    }

    markerArray_rs.markers.push_back(markerNode_rs);
    markerArray_rs.markers.push_back(markerEdge_rs);
    pubRSLoop.publish(markerArray_rs);

}

void MultiSession::IncreMapping::loadCentralMap(){
    std::string name = sessions_dir_ + central_sess_name_ + "/globalMap.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile<pcl::PointXYZI>(name, *map);
    for(size_t i = 0; i < map->points.size(); i++){
        PointType pt;
        pt.x = map->points[i].x;
        pt.y = map->points[i].y;
        pt.z = map->points[i].z;
        centralMap_->points.emplace_back(pt);
    }
    downSizeFilterPub.setInputCloud(centralMap_);
    downSizeFilterPub.filter(*centralMap_);
    std::cout << "load " << (sessions_dir_ + central_sess_name_ + "/globalMap.pcd") << " size: " <<  map->points.size() << std::endl;
}

// void MultiSession::IncreMapping::publish(){
//     publishCloud(&pubCentralGlobalMap, centralMap_, publishTimeStamp, "camera_init");

//     publishCloud(&pubCentralTrajectory, traj_central, publishTimeStamp, "camera_init");

//     publishCloud(&pubRegisteredTrajectory, traj_regis, publishTimeStamp, "camera_init");

//     visualizeLoopClosure();

// }

void MultiSession::IncreMapping::getReloKeyFrames(){
    for(auto& it : reloKeyFrames){
        it.second.all_cloud = transformPointCloud(sessions_.at(source_sess_idx).cloudKeyFrames[it.first].all_cloud, &sessions_.at(source_sess_idx).cloudKeyPoses6D->points[it.first]);
        it.second.scv_od = sessions_.at(source_sess_idx).cloudKeyFrames[it.first].scv_od;
        *regisMap_ += *it.second.all_cloud;
    }

    for(int i = 0; i < sessions_.at(target_sess_idx).cloudKeyPoses6D->points.size(); i++){
        PointType pt;
        pt.x = sessions_.at(target_sess_idx).cloudKeyPoses6D->points[i].x;
        pt.y = sessions_.at(target_sess_idx).cloudKeyPoses6D->points[i].y;
        pt.z = sessions_.at(target_sess_idx).cloudKeyPoses6D->points[i].z;
        traj_central->points.emplace_back(pt);
    }

    for(int i = 0; i < sessions_.at(source_sess_idx).cloudKeyPoses6D->points.size(); i++){
        PointType pt;
        pt.x = sessions_.at(source_sess_idx).cloudKeyPoses6D->points[i].x;
        pt.y = sessions_.at(source_sess_idx).cloudKeyPoses6D->points[i].y;
        pt.z = sessions_.at(source_sess_idx).cloudKeyPoses6D->points[i].z;
        traj_regis->points.emplace_back(pt);
    }
}

