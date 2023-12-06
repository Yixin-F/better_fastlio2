#include "multi-session/Incremental_mapping.h"

std::string sessions_dir;
std::string central_sess_name;
std::string query_sess_name;
std::string save_directory;

int iteration;  // >= 4

bool keyFrameSort(const std::pair<int, KeyFrame>& frame1, const std::pair<int, KeyFrame>& frame2){
    return frame1.first < frame2.first;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "multi-session");
    ros::NodeHandle nh;

    nh.param<std::string>("multi_session/sessions_dir", sessions_dir, " ");
    nh.param<std::string>("multi_session/central_sess_name", central_sess_name, " ");
    nh.param<std::string>("multi_session/query_sess_name", query_sess_name, " ");
    nh.param<std::string>("multi_session/save_directory", save_directory, " ");
    nh.param<int>("multi_session/iteration", iteration, 5);

    

    ROS_INFO("\033[1;32m----> multi-session starts.\033[0m");
    MultiSession::IncreMapping multi_session(sessions_dir, central_sess_name, query_sess_name, save_directory);

    // pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    // pcl::PointCloud<PointTypePose>::Ptr pose(new pcl::PointCloud<PointTypePose>());
    // std::string pose_name = "/home/fyx/fastlio2/src/sessions/0102/aft_tansformation2.pcd";
    // pcl::io::loadPCDFile(pose_name, *pose);
    // int i = 0;
    // for(auto& it : multi_session.sessions_.at(multi_session.source_sess_idx).cloudKeyFrames){
    //     *cloud += *transformPointCloud(it.all_cloud, &pose->points[i]);
    //     i++;
    // }
    // pcl::io::savePCDFile("/home/fyx/fastlio2/src/sessions/0102/map.pcd", *cloud);

    ROS_INFO("\033[1;32m----> pose-graph optimization.\033[0m");
    multi_session.run(iteration);  
    
    ROS_INFO("\033[1;32m----> publish cloud.\033[0m");
    std::sort(multi_session.reloKeyFrames.begin(), multi_session.reloKeyFrames.end(), keyFrameSort);
    int it = multi_session.reloKeyFrames.size();
    int i = 0;
    ros::Rate rate(0.5);
    while((ros::ok()) && (i < it)){
        ros::spinOnce();
        publishCloud(&multi_session.pubCentralGlobalMap, multi_session.centralMap_, multi_session.publishTimeStamp, "camera_init");
        publishCloud(&multi_session.pubCentralTrajectory, multi_session.traj_central, multi_session.publishTimeStamp, "camera_init");
        publishCloud(&multi_session.pubRegisteredTrajectory, multi_session.traj_regis, multi_session.publishTimeStamp, "camera_init");
        multi_session.visualizeLoopClosure();
        publishCloud(&multi_session.pubReloCloud, multi_session.reloKeyFrames[i].second.all_cloud, multi_session.publishTimeStamp, "camera_init");
        std::cout << "relo name(Idx): " << multi_session.reloKeyFrames[i].first << " target  name(Idx): " << multi_session.reloKeyFrames[i].second.reloTargetIdx
                            << " score: " << multi_session.reloKeyFrames[i].second.reloScore << std::endl; 
        i ++;
        rate.sleep();
    }
    return 0;
}