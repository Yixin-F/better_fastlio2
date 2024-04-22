#include "pose_estimator.h"
#include "../FRICP-toolkit/registeration.h"

pose_estimator::pose_estimator(){
    allocateMemory();

    nh.param<std::string>("relo/priorDir", priorDir, " ");
    nh.param<std::string>("relo/cloudTopic", cloudTopic, "/cloud_registered");
    nh.param<std::string>("relo/poseTopic", poseTopic, "/Odometry");
    cloudTopic_repub = cloudTopic + "repub";  // FIXME: no use
    poseTopic_repub = poseTopic + "repub";  // FIXME: no use
    nh.param<float>("relo/searchDis", searchDis, 10.0);
    nh.param<int>("relo/searchNum", searchNum, 3);
    nh.param<float>("relo/trustDis", trustDis, 5.0);
    nh.param<int>("relo/regMode", regMode, 5);

    nh.param<std::vector<double>>("relo/extrinsic_T", extrinT_, std::vector<double>());
    nh.param<std::vector<double>>("relo/extrinsic_R", extrinR_, std::vector<double>());

    extrinT << VEC_FROM_ARRAY(extrinT_);
    extrinR << MAT_FROM_ARRAY(extrinR_);
    std::cout << "extrinT: " << extrinT << "\t" << "extrinR: " << extrinR << std::endl;

    Eigen::Matrix<double, 3, 1> euler_ext = RotMtoEuler(extrinR);
    pose_ext.x = extrinT(0);
    pose_ext.y = extrinT(1);
    pose_ext.z = extrinT(2);
    pose_ext.roll = euler_ext(0, 0);
    pose_ext.pitch = euler_ext(1, 0);
    pose_ext.yaw = euler_ext(2, 0);

    pose_zero.x = 0.0;
    pose_zero.y = 0.0;
    pose_zero.z = 0.0;
    pose_zero.roll = 0.0;
    pose_zero.pitch = 0.0;
    pose_zero.yaw = 0.0;

    subCloud = nh.subscribe<sensor_msgs::PointCloud2>(cloudTopic, 1, &pose_estimator::cloudCBK, this);
    subPose = nh.subscribe<nav_msgs::Odometry>(poseTopic, 500, &pose_estimator::poseCBK, this);
    pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
    pubPose = nh.advertise<nav_msgs::Odometry>("/pose", 1);
    fout_relo.open(priorDir + "relo_pose.txt", ios::out);
    

    subExternalPose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 500, &pose_estimator::externalCBK, this);
    pubPriorMap = nh.advertise<sensor_msgs::PointCloud2>("/prior_map", 1);
    pubPriorPath = nh.advertise<sensor_msgs::PointCloud2>("/prior_path", 1);
    pubReloCloud = nh.advertise<sensor_msgs::PointCloud2>("/relo_cloud", 1);
    pubInitCloud = nh.advertise<sensor_msgs::PointCloud2>("/init_cloud", 1);
    pubNearCloud = nh.advertise<sensor_msgs::PointCloud2>("/near_cloud", 1);
    pubMeasurementEdge = nh.advertise<visualization_msgs::MarkerArray>("measurement", 1);
    pubPath = nh.advertise<nav_msgs::Path>("/path_loc", 1e00000);

    std::cout << ANSI_COLOR_GREEN << "rostopic is ok" << ANSI_COLOR_RESET << std::endl;
    
    sessions.push_back(MultiSession::Session(1, "priorMap", priorDir, true));
    // std::string path_6D = priorDir + "/transformations.pcd";
    // std::string path_3D = priorDir + "/trajectory.pcd";
    // std::string path_map = priorDir + "/globalMap.pcd";
    // pcl::io::loadPCDFile(path_6D, *sessions[0].cloudKeyPoses6D);
    // pcl::io::loadPCDFile(path_3D, *sessions[0].cloudKeyPoses3D);
    // pcl::io::loadPCDFile(path_map, *priorMap);

    *priorMap += *sessions[0].globalMap;
    *priorPath += *sessions[0].cloudKeyPoses3D;
    downSizeFilterPub.setLeafSize(3.0, 3.0, 3.0);

    height = priorPath->points[0].z;

    kdtreeGlobalMapPoses->setInputCloud(priorPath);
    kdtreeGlobalMapPoses_copy->setInputCloud(priorPath);
    std::cout << ANSI_COLOR_GREEN << "load prior knowledge" << ANSI_COLOR_RESET << std::endl;

    reg.push_back(Registeration(regMode));
}

void pose_estimator::allocateMemory(){
    priorMap.reset(new pcl::PointCloud<PointType>());
    priorPath.reset(new pcl::PointCloud<PointType>());
    reloCloud.reset(new pcl::PointCloud<PointType>());
    initCloud.reset(new pcl::PointCloud<PointType>());
    initCloud_.reset(new pcl::PointCloud<PointType>());
    nearCloud.reset(new pcl::PointCloud<PointType>());
    kdtreeGlobalMapPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeGlobalMapPoses_copy.reset(new pcl::KdTreeFLANN<PointType>());
}

void pose_estimator::cloudCBK(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<PointType>::Ptr msgCloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg, *msgCloud);

    if(msgCloud->points.size() == 0){
        return ;
    }

    msgCloud->width = msgCloud->points.size();
    msgCloud->height = 1;
    cloudBuffer.push_back(msgCloud);
}

void pose_estimator::poseCBK(const nav_msgs::Odometry::ConstPtr& msg){
    PointTypePose pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.z = msg->pose.pose.position.z;
    
    Eigen::Vector4d q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    quaternionNormalize(q);

    Eigen::Matrix3d rot = quaternionToRotation(q);
    Eigen::Matrix<double, 3, 1> euler = RotMtoEuler(rot);
    pose.roll = euler(0,0);
    pose.pitch = euler(1,0);
    pose.yaw = euler(2,0);

    poseBuffer_6D.push_back(pose);
    
    PointType pose3d;
    pose3d.x = msg->pose.pose.position.x;
    pose3d.y = msg->pose.pose.position.y;
    // pose3d.z = msg->pose.pose.position.z;
    pose3d.z = height;

    poseBuffer_3D.push_back(pose3d);
}

void pose_estimator::externalCBK(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    if(external_flg){
        return ;
    }
    std::cout << ANSI_COLOR_RED << "please set your external pose now ... " << ANSI_COLOR_RESET << std::endl;

    externalPose.x = msg->pose.pose.position.x;
    externalPose.y = msg->pose.pose.position.y;
    externalPose.z = 0.0;
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    externalPose.roll = 0.0;   // FIXME: it's better to choose zero
    externalPose.pitch = 0.0;
    externalPose.yaw = yaw;
    std::cout << ANSI_COLOR_GREEN << "Get initial pose: " << externalPose.x << " " << externalPose.y << " " << externalPose.z 
              << " " << externalPose.roll << " " << externalPose.pitch << " " << externalPose.yaw << ANSI_COLOR_RESET << std::endl;
    external_flg = true;
}

void pose_estimator::run(){
    ros::Rate rate(20);
    while(ros::ok()){
        ros::spinOnce();
        if(!global_flg){
            if(cout_count_ < 1)
                std::cout << ANSI_COLOR_RED << "wait for global pose initialization ... " << ANSI_COLOR_RESET << std::endl;

            global_flg = globalRelo();
            cout_count_ = 1;
            continue;
        }

        if(idx >= cloudBuffer.size()){
            // std::cout << ANSI_COLOR_RED << "relo > subscribe ... " << ANSI_COLOR_RESET << std::endl;
            publish_odometry(pubPose);
            publish_path(pubPath);
            publishCloud(&pubReloCloud, reloCloud, ros::Time().fromSec(ld_time), "world");
            continue;
        }

        ld_time = ros::Time().fromSec(ld_time).toSec();
        
        pcl::PointCloud<PointType>::Ptr relo_pt(new pcl::PointCloud<PointType>());
        relo_pt->points.emplace_back(poseBuffer_3D[idx]);
        relo_pt = transformPointCloud(relo_pt, &initPose);
        std::cout << ANSI_COLOR_GREEN << "get current relo point " << idx << ANSI_COLOR_RESET << std::endl;

        if(easyToRelo(relo_pt->points[0])){
        // if(0){
            std::cout << ANSI_COLOR_GREEN << "relo mode for frame: " << idx << ANSI_COLOR_RESET << std::endl;
            
            pcl::PointCloud<PointType>::Ptr curCloud(new pcl::PointCloud<PointType>());
            *curCloud += *transformPointCloud(cloudBuffer[idx], &initPose);
            std::cout << "current cloud size: " << curCloud->points.size() << std::endl;

            nearCloud->clear();
            for(auto& it : idxVec){
                std::cout << ANSI_COLOR_GREEN << "get near frame: " << it << ANSI_COLOR_RESET << std::endl;
                pcl::PointCloud<PointType>::Ptr nearCloud_tmp(new pcl::PointCloud<PointType>());
                *nearCloud_tmp += *transformPointCloud(sessions[0].cloudKeyFrames[it].all_cloud, &pose_ext);
                *nearCloud += *transformPointCloud(nearCloud_tmp, &sessions[0].cloudKeyPoses6D->points[it]);
            }

            Eigen::MatrixXd transform = reg[0].run(curCloud, nearCloud);
            Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
            Eigen::MatrixXd linear = transform.block(0, 3, 3, 1);
            Eigen::Matrix<double, 3, 1> euler = RotMtoEuler(rot);
            PointTypePose pose_icp;

            pose_icp.x = linear(0, 0);
            pose_icp.y = linear(1, 0);
            pose_icp.z = linear(2, 0);
            pose_icp.roll = euler(0, 0);
            pose_icp.pitch = euler(1, 0);
            pose_icp.yaw = euler(2, 0);

            // floor(pose_icp.x, 0.5);
            // floor(pose_icp.y, 0.5);
            // floor(pose_icp.z, 0.1);
            // floor(pose_icp.roll, 0.1);
            // floor(pose_icp.pitch, 0.1);
            // floor(pose_icp.yaw, 0.2);

            reloCloud->clear();
            *reloCloud += *transformPointCloud(curCloud, &pose_icp);
            publishCloud(&pubReloCloud, reloCloud, ros::Time().fromSec(ld_time), "world");

            Eigen::Affine3f trans_buffer = pcl::getTransformation(poseBuffer_6D[idx].x, poseBuffer_6D[idx].y, poseBuffer_6D[idx].z, poseBuffer_6D[idx].roll, poseBuffer_6D[idx].pitch, poseBuffer_6D[idx].yaw);
            Eigen::Affine3f trans_init = pcl::getTransformation(initPose.x, initPose.y, initPose.z, initPose.roll, initPose.pitch, initPose.yaw);
            Eigen::Affine3f trans_res = pcl::getTransformation(pose_icp.x, pose_icp.y, pose_icp.z, pose_icp.roll, pose_icp.pitch, pose_icp.yaw);
            Eigen::Affine3f trans_aft = trans_res * trans_init * trans_buffer;

            float aft[6];
            pcl::getTranslationAndEulerAngles(trans_aft, aft[0], aft[1], aft[2],
                                                                 aft[3], aft[4], aft[5]);

            PointTypePose pose_aft;
            pose_aft.x = aft[0];
            pose_aft.y = aft[1];
            pose_aft.z = aft[2];
            pose_aft.roll = aft[3];
            pose_aft.pitch = aft[4];
            pose_aft.yaw = aft[5];

            Eigen::Matrix3d R = Exp((double)pose_aft.roll, (double)pose_aft.pitch, (double)pose_aft.yaw);
            Eigen::Vector3d t((double)pose_aft.x, (double)pose_aft.y, (double)pose_aft.z);
            fout_relo << std::fixed << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t[0] << " "
                             << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << t[1] << " "
                             << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << t[2] << std::endl;
;
            reloPoseBuffer.push_back(pose_aft);

            Eigen::Matrix<double, 3, 3> ang_rot = Exp((double)pose_aft.roll, (double)pose_aft.pitch, (double)pose_aft.yaw);
            Eigen::Vector4d q = rotationToQuaternion(ang_rot);
            quaternionNormalize(q);

            odomAftMapped.pose.pose.position.x = pose_aft.x;
            odomAftMapped.pose.pose.position.y = pose_aft.y;
            odomAftMapped.pose.pose.position.z = pose_aft.z;
            odomAftMapped.pose.pose.orientation.x = q(0);
            odomAftMapped.pose.pose.orientation.y = q(1);
            odomAftMapped.pose.pose.orientation.z = q(2);
            odomAftMapped.pose.pose.orientation.w = q(3);
            publish_odometry(pubPose);

            msg_body_pose.pose.position.x = pose_aft.x;
            msg_body_pose.pose.position.y = pose_aft.y;
            msg_body_pose.pose.position.z = pose_aft.z;
            msg_body_pose.pose.orientation.x = q(0);
            msg_body_pose.pose.orientation.y = q(1);
            msg_body_pose.pose.orientation.z = q(2);
            msg_body_pose.pose.orientation.w = q(3);
            publish_path(pubPath);

            height = pose_aft.z;

            idx ++;
        }
        else{
            std::cout << ANSI_COLOR_RED << "lio mode for frame: " << idx << ANSI_COLOR_RESET << std::endl;
            
            pcl::PointCloud<PointType>::Ptr curCloud(new pcl::PointCloud<PointType>());
            *curCloud += *transformPointCloud(cloudBuffer[idx], &initPose);
            std::cout << "current cloud size: " << curCloud->points.size() << std::endl;

            reloCloud->clear();
            *reloCloud += *curCloud;
            publishCloud(&pubReloCloud, reloCloud, ros::Time().fromSec(ld_time), "world");

            // *sessions[0].globalMap += *curCloud;
            downSizeFilterPub.setInputCloud(curCloud);
            downSizeFilterPub.filter(*curCloud);
            *priorMap += *curCloud;
            
            pcl::PointCloud<PointType>::Ptr invCloud(new pcl::PointCloud<PointType>());
            *invCloud += *getBodyCloud(cloudBuffer[idx], poseBuffer_6D[idx], pose_zero);
            invCloud = getBodyCloud(invCloud, pose_ext, pose_zero);

            KeyFrame newFrame;
            newFrame.all_cloud = invCloud;
            sessions[0].cloudKeyFrames.push_back(newFrame);
            std::cout << "newFrame.all_cloud size: " << newFrame.all_cloud->points.size() << std::endl;

            sessions[0].scManager.makeAndSaveScancontextAndKeys(*invCloud);
            std::cout << "add current sc" << std::endl;

            Eigen::Affine3f trans_buffer = pcl::getTransformation(poseBuffer_6D[idx].x, poseBuffer_6D[idx].y, poseBuffer_6D[idx].z, poseBuffer_6D[idx].roll, poseBuffer_6D[idx].pitch, poseBuffer_6D[idx].yaw);
            Eigen::Affine3f trans_init = pcl::getTransformation(initPose.x, initPose.y, initPose.z, initPose.roll, initPose.pitch, initPose.yaw);
            Eigen::Affine3f trans_aft = trans_init * trans_buffer;

            float aft[6];
            pcl::getTranslationAndEulerAngles(trans_aft, aft[0], aft[1], aft[2],
                                                                 aft[3], aft[4], aft[5]);

            PointTypePose pose_aft;
            pose_aft.x = aft[0];
            pose_aft.y = aft[1];
            // pose_aft.z = aft[2];
            pose_aft.z = height;
            pose_aft.roll = aft[3];
            pose_aft.pitch = aft[4];
            pose_aft.yaw = aft[5];

            Eigen::Matrix3d R = Exp((double)pose_aft.roll, (double)pose_aft.pitch, (double)pose_aft.yaw);
            Eigen::Vector3d t((double)pose_aft.x, (double)pose_aft.y, (double)pose_aft.z);
            fout_relo << std::fixed << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t[0] << " "
                             << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << t[1] << " "
                             << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << t[2] << std::endl;

            reloPoseBuffer.push_back(pose_aft);
            std::cout << "reloPoseBuffer size: " << reloPoseBuffer.size() << std::endl;

            sessions[0].cloudKeyPoses6D->points.push_back(pose_aft);
            sessions[0].cloudKeyPoses6D->width = sessions[0].cloudKeyPoses6D->points.size();
            sessions[0].cloudKeyPoses6D->height = 1;
            std::cout << "cloudKeyPoses6D size: " << sessions[0].cloudKeyPoses6D->points.size() << std::endl;

            PointType pose3d;
            pose3d.x = pose_aft.x;
            pose3d.y = pose_aft.y;
            // pose3d.z = pose_aft.z;
            pose3d.z = height;
            sessions[0].cloudKeyPoses3D->points.push_back(pose3d);
            sessions[0].cloudKeyPoses3D->width = sessions[0].cloudKeyPoses3D->points.size();
            sessions[0].cloudKeyPoses3D->height = 1;
            priorPath->points.push_back(pose3d);
            priorPath->width = priorPath->points.size();
            priorPath->height = 1;
            kdtreeGlobalMapPoses_copy->setInputCloud(priorPath);
            std::cout << "priorPath size: " << priorPath->points.size() << std::endl;


            Eigen::Matrix<double, 3, 3> ang_rot = Exp((double)pose_aft.roll, (double)pose_aft.pitch, (double)pose_aft.yaw);
            Eigen::Vector4d q = rotationToQuaternion(ang_rot);
            quaternionNormalize(q);
            
            odomAftMapped.pose.pose.position.x = pose_aft.x;
            odomAftMapped.pose.pose.position.y = pose_aft.y;
            odomAftMapped.pose.pose.position.z = pose_aft.z;
            odomAftMapped.pose.pose.orientation.x = q(0);
            odomAftMapped.pose.pose.orientation.y = q(1);
            odomAftMapped.pose.pose.orientation.z = q(2);
            odomAftMapped.pose.pose.orientation.w = q(3);
            publish_odometry(pubPose);

            msg_body_pose.pose.position.x = pose_aft.x;
            msg_body_pose.pose.position.y = pose_aft.y;
            msg_body_pose.pose.position.z = pose_aft.z;
            msg_body_pose.pose.orientation.x = q(0);
            msg_body_pose.pose.orientation.y = q(1);
            msg_body_pose.pose.orientation.z = q(2);
            msg_body_pose.pose.orientation.w = q(3);
            publish_path(pubPath);

            idx ++;
        }

        rate.sleep();
    }
}

void pose_estimator::publishThread(){
    bool status = ros::ok();
    ros::Rate rate(10);
    while(status){
        ros::spinOnce();
        publishCloud(&pubPriorMap, priorMap, ros::Time().fromSec(ld_time), "world");
        publishCloud(&pubPriorPath, priorPath, ros::Time().fromSec(ld_time), "world");
        publishCloud(&pubInitCloud, initCloud, ros::Time().fromSec(ld_time), "world");
        publishCloud(&pubNearCloud, nearCloud, ros::Time().fromSec(ld_time), "world");
        rate.sleep();
    }
}

bool pose_estimator::easyToRelo(const PointType& pose3d){
    idxVec.clear();
    disVec.clear();
    kdtreeGlobalMapPoses->nearestKSearch(pose3d, 1, idxVec, disVec);

    if(disVec[0] > searchDis){
        // detectResult = sessions[0].scManager.detectLoopClosureID();
        // sc_new = detectResult.first;
        // if(detectResult.first != -1 && sc_new != sc_old){
        //     std::cout << ANSI_COLOR_GREEN_BG << "lio -> relo " << ANSI_COLOR_RESET << std::endl;
        //     idxVec.clear();
        //     idxVec.emplace_back(detectResult.first);
        //     sc_old = detectResult.first;
        //     return true;
        // }
        // else{
        //     return false;
        // }

        idxVec_copy.clear();
        disVec_copy.clear();
        kdtreeGlobalMapPoses_copy->radiusSearchT(pose3d, searchDis, idxVec_copy, disVec_copy);
        bool status;
        for(auto& it : idxVec_copy){
            if(priorPath->points.size() - it >= 100){  // FIXME: default 100
                std::cout << ANSI_COLOR_GREEN_BG << "lio -> relo with " << priorPath->points.size() << " to " << it << ANSI_COLOR_RESET << std::endl;
                idxVec.clear();
                idxVec.emplace_back(it);
                status = true;
                break;
            }
            else{
                status= false;
            }
        }
        if(status){  
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return true;
    }

    // pcl::PointCloud<PointType>::Ptr cloud_search(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr cloud_tmp(new pcl::PointCloud<PointType>());
    // *cloud_tmp += *transformPointCloud(sessions[0].cloudKeyFrames[idxVec[0]].all_cloud, &pose_ext);
    // *cloud_search += *transformPointCloud(cloud_tmp, &sessions[0].cloudKeyPoses6D->points[idxVec[0]]);
    // pcl::PointXYZINormal point_min, point_max;
    // pcl::getMinMax3D(*cloud_search, point_min, point_max);
    // float min_x = point_min.x;
    // float min_y = point_min.y;
    // float min_z = point_min.z;
    // float max_x = point_max.x;
    // float max_y = point_max.y;
    // float max_z = point_max.z;
    // if(pose3d.x > (min_x + 10.0) && pose3d.x < (max_x - 10.0) && pose3d.y > (min_y + 10.0) && pose3d.y < (max_y + 10.0)){
    //     return true;
    // }
    // else{
    //     return false;
    // }
    
    // kdtreeGlobalMapPoses->radiusSearch(pose3d, searchDis, idxVec, disVec);
    // if(idxVec.size() >= searchNum){
    //     // std::cout << ANSI_COLOR_GREEN << "relo mode start for frame " << idx << ANSI_COLOR_RESET << std::endl;
    //     return true;
    // }
    // else{
    //     // std::cout << ANSI_COLOR_GREEN << "lio mode start for frame " << idx << ANSI_COLOR_RESET << std::endl;
    //     return false;
    // }
}

bool pose_estimator::globalRelo(){
    if(cloudBuffer.size() < 1 || poseBuffer_6D.size() < 1){
        if(buffer_flg){
            std::cout << ANSI_COLOR_RED << "wait for cloud and pose from fast-lio2 ... " << ANSI_COLOR_RESET << std::endl;
            buffer_flg = false;
        }
        return false;
    }


    if(!sc_flg){
        // TODO: The pointcloud saved in fast-lio2 is calculated without extrinsic, but the pose ignored the extrinsic
        initCloud_->clear();
        *initCloud_ += *cloudBuffer[0];
        std::cout << "init cloud size: " << initCloud_->points.size() << std::endl;
        pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/cur_imu.pcd", *initCloud_);

        std::cout << ANSI_COLOR_GREEN << "global relo by sc ... " << ANSI_COLOR_RESET << std::endl;
        pcl::PointCloud<PointType>::Ptr cloud_lid(new pcl::PointCloud<PointType>());
        *cloud_lid += *getBodyCloud(initCloud_, pose_ext, pose_zero);
        pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/cur_lid.pcd", *cloud_lid);

        // sessions[0].scManager.makeAndSaveScancontextAndKeys(*cloud_lid);
        // detectResult = sessions[0].scManager.detectLoopClosureID();
        // sessions[0].scManager.polarcontexts_.pop_back();
        // sessions[0].scManager.polarcontext_invkeys_.pop_back();
        // sessions[0].scManager.polarcontext_vkeys_.pop_back();
        // sessions[0].scManager.polarcontext_invkeys_mat_.pop_back();

        Eigen::MatrixXd initSC = sessions[0].scManager.makeScancontext(*cloud_lid);
        Eigen::MatrixXd ringkey = sessions[0].scManager.makeRingkeyFromScancontext(initSC);
        Eigen::MatrixXd sectorkey = sessions[0].scManager.makeSectorkeyFromScancontext(initSC);
        std::vector<float> polarcontext_invkey_vec = ScanContext::eig2stdvec(ringkey);
        detectResult = sessions[0].scManager.detectLoopClosureIDBetweenSession(polarcontext_invkey_vec, initSC);

        std::cout << ANSI_COLOR_RED << "init relocalization by current SC id: " << 0 << " in prior map's SC id: " << detectResult.first 
                  << " yaw offset: " << detectResult.second << ANSI_COLOR_RESET << std::endl;

        if(detectResult.first != -1){
            PointTypePose pose_com;
            pose_com.x = 0.0;
            pose_com.y = 0.0;
            pose_com.z = 0.0;
            pose_com.roll = 0.0;
            pose_com.pitch = 0.0;
            pose_com.yaw = -detectResult.second;

            initCloud->clear();
            *initCloud += *getAddCloud(cloud_lid, pose_com, pose_ext);
            pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/cur_sc.pcd", *initCloud);

            nearCloud->clear();
            *nearCloud += *sessions[0].cloudKeyFrames[detectResult.first].all_cloud;

            pcl::PointCloud<PointType>::Ptr near_ext(new pcl::PointCloud<PointType>());
            *near_ext += *transformPointCloud(nearCloud, &pose_ext);
            pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/near_sc.pcd", *near_ext);

            nearCloud->clear();
            *nearCloud += *transformPointCloud(near_ext, &sessions[0].cloudKeyPoses6D->points[detectResult.first]);
            pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/near_world.pcd", *nearCloud);
        }
        else{
            initCloud->clear();
            *initCloud += *initCloud_;
            pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/cur_sc.pcd", *initCloud);
        }

        sc_flg = true;

        return false;
    }

    if(!external_flg){
        if(cout_count <= 0){
            std::cout << ANSI_COLOR_RED << "wait for external pose ... " << ANSI_COLOR_RESET << std::endl;
        }

        cout_count = 1;
        return false;
    }

    std::cout << ANSI_COLOR_GREEN << "global relocalization processing ... " << ANSI_COLOR_RESET << std::endl;
    
    bool trust;
    PointTypePose poseSC = sessions[0].cloudKeyPoses6D->points[detectResult.first];
    if(detectResult.first < 0){
        trust = false;
        std::cout << ANSI_COLOR_RED << "can not relo by SC ... " << ANSI_COLOR_RESET << std::endl;
    }
    else{
        float x_diff = externalPose.x - poseSC.x;
        float y_diff = externalPose.y - poseSC.y;
        float dis = std::sqrt(x_diff * x_diff + y_diff * y_diff);
        std::cout << ANSI_COLOR_GREEN << "distance between sc pose and external pose: " << dis << ANSI_COLOR_RESET << std::endl;
        trust = (dis <= trustDis) ? true : false;  // select SC-pose or extermal-pose
    }
    
    if(trust){
    // if(0){
        std::cout << ANSI_COLOR_GREEN << "init relo by SC-pose ... " << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_GREEN << "use prior frame " << detectResult.first << " to relo init cloud ..." << ANSI_COLOR_RESET << std::endl;

        nearCloud->clear();
        PointType tmp;
        tmp.x = poseSC.x;
        tmp.y = poseSC.y;
        tmp.z = poseSC.z;

        idxVec.clear();
        disVec.clear();
        kdtreeGlobalMapPoses->nearestKSearch(tmp, searchNum, idxVec, disVec);

        for(int i = 0; i < idxVec.size(); i++){
            pcl::PointCloud<PointType>::Ptr nearCloud_tmp(new pcl::PointCloud<PointType>());
            *nearCloud_tmp += *transformPointCloud(sessions[0].cloudKeyFrames[idxVec[i]].all_cloud, &pose_ext);
            *nearCloud += *transformPointCloud(nearCloud_tmp, &sessions[0].cloudKeyPoses6D->points[idxVec[i]]);
        }
        std::cout << "near cloud size: " << nearCloud->points.size() << std::endl;
        pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/near.pcd", *nearCloud);

        PointTypePose poseOffset;
        poseOffset.x = poseSC.x;
        poseOffset.y = poseSC.y;
        poseOffset.z = poseSC.z;
        poseOffset.roll = 0.0;
        poseOffset.pitch = 0.0;
        poseOffset.yaw = 0.0;
        initCloud = transformPointCloud(initCloud, &poseOffset);
        std::cout << "init cloud size: " << initCloud->points.size() << std::endl;
        pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/init_offset.pcd", *initCloud);

        std::cout << ANSI_COLOR_GREEN << "get precise pose by FR-ICP ... " << ANSI_COLOR_RESET << std::endl;
        Eigen::MatrixXd transform = reg[0].run(initCloud, nearCloud);
        Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
        Eigen::MatrixXd linear = transform.block(0, 3, 3, 1);
        Eigen::Matrix<double, 3, 1> euler = RotMtoEuler(rot);

        PointTypePose poseReg;
        poseReg.x = linear(0, 0);
        poseReg.y = linear(1, 0);
        poseReg.z = linear(2, 0);
        poseReg.roll = euler(0, 0);
        poseReg.pitch = euler(1, 0);
        poseReg.yaw = euler(2, 0);

        initCloud = transformPointCloud(initCloud, &poseReg);
        initCloud->width = initCloud->points.size();
        initCloud->height = 1;
        pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/init_result.pcd", *initCloud);

        Eigen::Affine3f trans_com = pcl::getTransformation(0.0, 0.0, 0.0, 0.0, 0.0, -detectResult.second);
        Eigen::Affine3f trans_offset = pcl::getTransformation(poseOffset.x, poseOffset.y, poseOffset.z, poseOffset.roll, poseOffset.pitch, poseOffset.yaw);
        Eigen::Affine3f trans_reg = pcl::getTransformation(poseReg.x, poseReg.y, poseReg.z, poseReg.roll, poseReg.pitch, poseReg.yaw);
        Eigen::Affine3f trans_init = trans_com * trans_offset * trans_reg;

        float pose_init[6];
        pcl::getTranslationAndEulerAngles(trans_init, pose_init[0], pose_init[1], pose_init[2],
                                                                 pose_init[3], pose_init[4], pose_init[5]);
        initPose.x = pose_init[0];
        initPose.y = pose_init[1];
        initPose.z = pose_init[2];
        initPose.roll = pose_init[3];
        initPose.pitch = pose_init[4];
        initPose.yaw = pose_init[5];
        
        global_flg = true;
        std::cout << ANSI_COLOR_GREEN << "Get optimized pose: " << initPose.x << " " << initPose.y << " " << initPose.z 
              << " " << initPose.roll << " " << initPose.pitch << " " << initPose.yaw << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_RED << "init relocalization has been finished ... " << ANSI_COLOR_RESET << std::endl;

        return true;
    }
    else{
        std::cout << ANSI_COLOR_GREEN << "init relo by external-pose ... " << ANSI_COLOR_RESET << std::endl;
        PointType tmp;
        tmp.x = externalPose.x;
        tmp.y = externalPose.y;
        tmp.z = externalPose.z;

        std::cout << ANSI_COLOR_GREEN << "use prior frame " << idxVec[0] << " to relo init cloud ..." << ANSI_COLOR_RESET << std::endl;

        PointTypePose pose_offset;
        pose_offset.x = externalPose.x;
        pose_offset.y = externalPose.y;
        pose_offset.z = externalPose.z;
        pose_offset.roll = 0.0;
        pose_offset.pitch = 0.0;
        pose_offset.yaw = externalPose.yaw;

        initCloud = transformPointCloud(initCloud, &pose_offset);
        std::cout << "init cloud size: " << initCloud->points.size() << std::endl;
        pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/init_offset.pcd", *initCloud);

        PointType tmp2;
        tmp2.x = externalPose.x;
        tmp2.y = externalPose.y;
        tmp2.z = externalPose.z;
        idxVec.clear();
        disVec.clear();
        kdtreeGlobalMapPoses->nearestKSearch(tmp2, searchNum, idxVec, disVec);
        PointTypePose pose_new = sessions[0].cloudKeyPoses6D->points[idxVec[0]];

        nearCloud->clear();
        for(int i = 0; i < idxVec.size(); i++){
            pcl::PointCloud<PointType>::Ptr nearCloud_tmp(new pcl::PointCloud<PointType>());
            *nearCloud_tmp += *transformPointCloud(sessions[0].cloudKeyFrames[idxVec[i]].all_cloud, &pose_ext);
            *nearCloud += *transformPointCloud(nearCloud_tmp, &sessions[0].cloudKeyPoses6D->points[idxVec[i]]);
            // *nearCloud += *getBodyCloud(sessions[0].cloudKeyFrames[idxVec[i]].all_cloud, pose_ext, sessions[0].cloudKeyPoses6D->points[idxVec[i]]);
        }
        std::cout << "near cloud size: " << nearCloud->points.size() << std::endl;
        pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/near.pcd", *nearCloud);
        
        std::cout << ANSI_COLOR_GREEN << "get precise pose by FR-ICP ... " << ANSI_COLOR_RESET << std::endl;
        Eigen::MatrixXd transform = reg[0].run(initCloud, nearCloud);
        Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
        Eigen::MatrixXd linear = transform.block(0, 3, 3, 1);
        // std::cout << "linear: " << linear << std::endl;

        Eigen::Matrix<double, 3, 1> euler = RotMtoEuler(rot);

        PointTypePose poseReg;
        poseReg.x = linear(0, 0);
        poseReg.y = linear(1, 0);
        poseReg.z = linear(2, 0);
        poseReg.roll = euler(0, 0);
        poseReg.pitch = euler(1, 0);
        poseReg.yaw = euler(2, 0);

        // std::cout << poseReg.x << " " << poseReg.y << " " << poseReg.z << std::endl;
        // std::cout << poseReg.roll << " " << poseReg.pitch << " " << poseReg.yaw << std::endl;

        initCloud = getAddCloud(initCloud, poseReg, pose_zero);
        initCloud->width = initCloud->points.size();
        initCloud->height = 1;
        pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_loc/init_result.pcd", *initCloud);

        Eigen::Affine3f trans_offset = pcl::getTransformation(pose_offset.x, pose_offset.y, pose_offset.z, pose_offset.roll, pose_offset.pitch, pose_offset.yaw);
        Eigen::Affine3f trans_reg = pcl::getTransformation(poseReg.x, poseReg.y, poseReg.z, poseReg.roll, poseReg.pitch, poseReg.yaw);
        Eigen::Affine3f trans_init = trans_offset * trans_reg ;

        float pose_init[6];
        pcl::getTranslationAndEulerAngles(trans_init, pose_init[0], pose_init[1], pose_init[2],
                                                                 pose_init[3], pose_init[4], pose_init[5]);
        initPose.x = pose_init[0];
        initPose.y = pose_init[1];
        initPose.z = pose_init[2];
        initPose.roll = pose_init[3];
        initPose.pitch = pose_init[4];
        initPose.yaw = pose_init[5];

        global_flg = true;
        std::cout << ANSI_COLOR_GREEN << "Get optimized pose: " << initPose.x << " " << initPose.y << " " << initPose.z 
              << " " << initPose.roll << " " << initPose.pitch << " " << initPose.yaw << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_RED << "init relocalization has been finished ... " << ANSI_COLOR_RESET << std::endl;

        return true;
    }
    
}

void pose_estimator::publish_odometry(const ros::Publisher &pubOdomAftMapped){
    pubOdomAftMapped.publish(odomAftMapped);
    odomAftMapped.header.frame_id = "world";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(ld_time);

    // static tf::TransformBroadcaster br1;
    // tf::Transform transform;
    // tf::Quaternion q;
    // transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
    //                                 odomAftMapped.pose.pose.position.y,
    //                                 odomAftMapped.pose.pose.position.z));
    // q.setW(odomAftMapped.pose.pose.orientation.w);
    // q.setX(odomAftMapped.pose.pose.orientation.x);
    // q.setY(odomAftMapped.pose.pose.orientation.y);
    // q.setZ(odomAftMapped.pose.pose.orientation.z);
    // transform.setRotation(q);
    // br1.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "world", "body"));
}

void pose_estimator::publish_path(const ros::Publisher& pubPath){
    msg_body_pose.header.frame_id = "world";
    msg_body_pose.header.stamp = ros::Time().fromSec(ld_time);
    
    path.header.frame_id = "world";
    path.header.stamp = ros::Time().fromSec(ld_time);
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}