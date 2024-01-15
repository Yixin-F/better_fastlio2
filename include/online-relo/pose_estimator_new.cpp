#include "pose_estimator_new.h"
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

    subCloud = nh.subscribe<sensor_msgs::PointCloud2>(cloudTopic, 1, &pose_estimator::cloudCBK, this);
    subPose = nh.subscribe<nav_msgs::Odometry>(poseTopic, 500, &pose_estimator::poseCBK, this);
    pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
    pubPose = nh.advertise<nav_msgs::Odometry>("/pose", 1);

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
    *priorMap += *sessions[0].globalMap;
    *priorPath += *sessions[0].cloudKeyPoses3D;
    kdtreeGlobalMapPoses->setInputCloud(priorPath);
    std::cout << ANSI_COLOR_GREEN << "load prior knowledge" << ANSI_COLOR_RESET << std::endl;

    reg.push_back(Registeration(regMode));
}

void pose_estimator::allocateMemory(){
    priorMap.reset(new pcl::PointCloud<PointType>());
    priorPath.reset(new pcl::PointCloud<PointType>());
    reloCloud.reset(new pcl::PointCloud<PointType>());
    initCloud.reset(new pcl::PointCloud<PointType>());
    nearCloud.reset(new pcl::PointCloud<PointType>());
    kdtreeGlobalMapPoses.reset(new pcl::KdTreeFLANN<PointType>());
}

void pose_estimator::cloudCBK(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<PointType>::Ptr msgCloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg, *msgCloud);
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
    pose3d.z = msg->pose.pose.position.z;

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
    ros::Rate rate(50);
    while(ros::ok()){
        ros::spinOnce();
        if(!global_flg){
            if(cout_count_ < 1)
                std::cout << ANSI_COLOR_RED << "wait for global pose initialization ... " << ANSI_COLOR_RESET << std::endl;

            global_flg = globalRelo();
            cout_count_ ++;
            continue;
        }

        if(idx > cloudBuffer.size()){
            std::cout << ANSI_COLOR_RED << "relo > subscribe ... " << ANSI_COLOR_RESET << std::endl;
            continue;
        }

        if(easyToRelo(poseBuffer_3D[idx])){
            std::cout << ANSI_COLOR_GREEN << "relo mode for frame: " << idx << ANSI_COLOR_RESET << std::endl;
            
            pcl::PointCloud<PointType>::Ptr curCloud(new pcl::PointCloud<PointType>());
            *curCloud += *transformPointCloud(cloudBuffer[idx], &initPose);
            nearCloud->clear();
            for(auto& it : idxVec){
                *nearCloud += *transformPointCloud(sessions[0].cloudKeyFrames[it].all_cloud,
                                                  &sessions[0].cloudKeyPoses6D->points[it]);
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

            PointTypePose pose_res = addPose(initPose, pose_icp); 
            PointTypePose pose_aft = addPose(pose_res, poseBuffer_6D[idx]);
            reloPoseBuffer.push_back(pose_aft);

            reloCloud->clear();
            *reloCloud += *transformPointCloud(cloudBuffer[idx], &pose_res);
            publishCloud(&pubReloCloud, reloCloud, ros::Time::now(), "camera_init");

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
        else{
            std::cout << ANSI_COLOR_GREEN << "lio mode for frame: " << idx << ANSI_COLOR_RESET << std::endl;
            
            pcl::PointCloud<PointType>::Ptr curCloud(new pcl::PointCloud<PointType>());
            *curCloud += *transformPointCloud(cloudBuffer[idx], &initPose);
            *sessions[0].globalMap += *curCloud;
            *priorMap += *curCloud;

            reloCloud->clear();
            *reloCloud += *curCloud;
            publishCloud(&pubReloCloud, reloCloud, ros::Time::now(), "camera_init");

            KeyFrame newFrame;
            *newFrame.all_cloud += *curCloud;
            sessions[0].cloudKeyFrames.push_back(newFrame);

            PointTypePose pose_inv = inversePose(poseBuffer_6D[idx]);
            pcl::PointCloud<PointType>::Ptr invCloud(new pcl::PointCloud<PointType>());
            *invCloud += *transformPointCloud(cloudBuffer[idx], &pose_inv);
            sessions[0].scManager.makeAndSaveScancontextAndKeys(*invCloud);

            PointTypePose pose_aft = addPose(initPose, poseBuffer_6D[idx]);
            sessions[0].cloudKeyPoses6D->points.push_back(pose_aft);

            PointType pose3d;
            pose3d.x = pose_aft.x;
            pose3d.y = pose_aft.y;
            pose3d.z = pose_aft.z;
            sessions[0].cloudKeyPoses3D->points.push_back(pose3d);
            priorPath->points.push_back(pose3d);

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
    ros::Rate rate(5);
    while(status){
        ros::spinOnce();
        publishCloud(&pubPriorMap, priorMap, ros::Time::now(), "camera_init");
        publishCloud(&pubPriorPath, priorPath, ros::Time::now(), "camera_init");
        publishCloud(&pubInitCloud, initCloud, ros::Time::now(), "camera_init");
        publishCloud(&pubNearCloud, nearCloud, ros::Time::now(), "camera_init");
        rate.sleep();
    }
}

bool pose_estimator::easyToRelo(const PointType& pose3d){
    idxVec.clear();
    disVec.clear();
    kdtreeGlobalMapPoses->radiusSearch(pose3d, searchDis, idxVec, disVec);
    if(idxVec.size() >= searchNum){
        // std::cout << ANSI_COLOR_GREEN << "relo mode start for frame " << idx << ANSI_COLOR_RESET << std::endl;
        return true;
    }
    else{
        // std::cout << ANSI_COLOR_GREEN << "lio mode start for frame " << idx << ANSI_COLOR_RESET << std::endl;
        return false;
    }
}

bool pose_estimator::globalRelo(){
    if(cloudBuffer.size() < 1 || poseBuffer_6D.size() < 1){
        if(buffer_flg){
            std::cout << ANSI_COLOR_RED << "wait for cloud and pose from fast-lio2 ... " << ANSI_COLOR_RESET << std::endl;
            buffer_flg = false;
        }
        return false;
    }

    if(!external_flg){
        if(cout_count <= 0){
            std::cout << ANSI_COLOR_RED << "wait for 5s external pose ... " << ANSI_COLOR_RESET << std::endl;
            
        }

        cout_count ++;
        return false;
    }

    std::cout << ANSI_COLOR_GREEN << "global relocalization processing ... " << ANSI_COLOR_RESET << std::endl;

    pcl::PointCloud<PointType>::Ptr initCloud_(new pcl::PointCloud<PointType>());
    *initCloud_ += *cloudBuffer[0];
    std::cout << "init cloud size: " << initCloud_->points.size() << std::endl;

    std::cout << ANSI_COLOR_GREEN << "global relo by sc ... " << ANSI_COLOR_RESET << std::endl;

    sessions[0].scManager.makeAndSaveScancontextAndKeys(*initCloud_);
    auto detectResult = sessions[0].scManager.detectLoopClosureID();
    sessions[0].scManager.polarcontexts_.pop_back();
    sessions[0].scManager.polarcontext_invkeys_.pop_back();
    sessions[0].scManager.polarcontext_vkeys_.pop_back();
    sessions[0].scManager.polarcontext_invkeys_mat_.pop_back();

    // Eigen::MatrixXd initSC = sessions[0].scManager.makeScancontext(*initCloud_);
    // Eigen::MatrixXd ringkey = sessions[0].scManager.makeRingkeyFromScancontext(initSC);
    // Eigen::MatrixXd sectorkey = sessions[0].scManager.makeSectorkeyFromScancontext(initSC);
    // std::vector<float> polarcontext_invkey_vec = ScanContext::eig2stdvec(ringkey);
    // auto detectResult = sessions[0].scManager.detectLoopClosureIDBetweenSession(polarcontext_invkey_vec, initSC);
    
    std::cout << "init relocalization by SC in prior map's id: " << detectResult.first << " yaw offset: " << detectResult.second << std::endl;

    if(detectResult.first != -1)
        *nearCloud += *sessions[0].cloudKeyFrames[detectResult.first].all_cloud;

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
        bool trust = (dis <= trustDis) ? false : true;  // select SC-pose or extermal-pose
    }
    
    if(trust){
        std::cout << ANSI_COLOR_GREEN << "init relo by SC-pose ... " << ANSI_COLOR_RESET << std::endl;
        pcl::PointCloud<PointType>::Ptr initCloudCompensate(new pcl::PointCloud<PointType>());
        PointTypePose compensate;
        compensate.yaw = detectResult.second;
        *initCloudCompensate += *transformPointCloud(initCloud_, &compensate);
        std::cout << "init cloud compendate size: " << initCloudCompensate->points.size() << std::endl;

        std::cout << ANSI_COLOR_GREEN << "use prior frame " << detectResult.first << " to relo init cloud ..." << ANSI_COLOR_RESET << std::endl;

        nearCloud->clear();
        PointType tmp;
        tmp.x = poseSC.x;
        tmp.y = poseSC.y;
        tmp.z = poseSC.z;
        idxVec.clear();
        disVec.clear();
        kdtreeGlobalMapPoses->nearestKSearch(tmp, searchNum, idxVec, disVec);
        *nearCloud += *sessions[0].cloudKeyFrames[idxVec[0]].all_cloud;
        for(int i = 1; i < idxVec.size(); i++){
            PointTypePose toBody0 = getBodyPose(poseSC, sessions[0].cloudKeyPoses6D->points[idxVec[i]]);
            *nearCloud += *transformPointCloud(sessions[0].cloudKeyFrames[idxVec[i]].all_cloud, &toBody0);
        }
        std::cout << "near cloud size: " << nearCloud->points.size() << std::endl;
        
        std::cout << ANSI_COLOR_GREEN << "get precise pose by FR-ICP ... " << ANSI_COLOR_RESET << std::endl;
        Eigen::MatrixXd transform = reg[0].run(initCloudCompensate, nearCloud);
        Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
        Eigen::MatrixXd linear = transform.block(0, 3, 3, 1);
        Eigen::Matrix<double, 3, 1> euler = RotMtoEuler(rot);

        initPose.x = poseSC.x + linear(0, 0);
        initPose.y = poseSC.y + linear(1, 0);
        initPose.z = poseSC.z + linear(2, 0);
        initPose.roll = poseSC.roll + euler(0, 0);
        initPose.pitch = poseSC.pitch + euler(1, 0);
        initPose.yaw = poseSC.yaw + euler(2, 0) + compensate.yaw;
        *initCloud += *transformPointCloud(initCloud_, &initPose);
        
        global_flg = true;
        return true;
    }
    else{
        std::cout << ANSI_COLOR_GREEN << "init relo by external-pose ... " << ANSI_COLOR_RESET << std::endl;
        PointType tmp;
        tmp.x = externalPose.x;
        tmp.y = externalPose.y;
        tmp.z = externalPose.z;
        if(!easyToRelo(tmp)){
            external_flg = false;
            std::cout << ANSI_COLOR_RED << "please reset external pose ... " << ANSI_COLOR_RESET << std::endl;
            return false;
        }

        pcl::PointCloud<PointType>::Ptr initCloudCompensate(new pcl::PointCloud<PointType>());
        PointTypePose compensate;
        compensate.yaw = externalPose.yaw;
        *initCloudCompensate += *transformPointCloud(initCloud_, &compensate);
        std::cout << "init cloud compendate size: " << initCloudCompensate->points.size() << std::endl;

        std::cout << ANSI_COLOR_GREEN << "use prior frame " << idxVec[0] << " to relo init cloud ..." << ANSI_COLOR_RESET << std::endl;
        PointTypePose pose_new = sessions[0].cloudKeyPoses6D->points[idxVec[0]];
        PointType tmp2;
        tmp2.x = pose_new.x;
        tmp2.y = pose_new.y;
        tmp2.z = pose_new.z;
        idxVec.clear();
        disVec.clear();
        kdtreeGlobalMapPoses->nearestKSearch(tmp2, searchNum, idxVec, disVec);
        nearCloud->clear();
        *nearCloud += *(sessions[0].cloudKeyFrames[idxVec[0]].all_cloud);
        for(int i = 1; i < idxVec.size(); i++){
            PointTypePose toBody0 = getBodyPose(pose_new, sessions[0].cloudKeyPoses6D->points[idxVec[i]]);
            *nearCloud += *transformPointCloud(sessions[0].cloudKeyFrames[idxVec[i]].all_cloud, &toBody0);
        }
        std::cout << "near cloud size: " << nearCloud->points.size() << std::endl;
        
        std::cout << ANSI_COLOR_GREEN << "get precise pose by FR-ICP ... " << ANSI_COLOR_RESET << std::endl;
        Eigen::MatrixXd transform = reg[0].run(initCloudCompensate, nearCloud);
        Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
        Eigen::MatrixXd linear = transform.block(0, 3, 3, 1);
        Eigen::Matrix<double, 3, 1> euler = RotMtoEuler(rot);

        initPose.x = pose_new.x + linear(0, 0);
        initPose.y = pose_new.y + linear(1, 0);
        initPose.z = pose_new.z + linear(2, 0);
        initPose.roll = pose_new.roll + euler(0, 0);
        initPose.pitch = pose_new.pitch + euler(1, 0);
        initPose.yaw = pose_new.yaw + euler(2, 0) + compensate.yaw;
        *initCloud += *transformPointCloud(initCloud_, &initPose);

        global_flg = true;
        return true;
    }
    
    std::cout << ANSI_COLOR_RED << "init relocalization has been finished ... " << ANSI_COLOR_RESET << std::endl;
    
}

void pose_estimator::publish_odometry(const ros::Publisher &pubOdomAftMapped){
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time::now();
    pubOdomAftMapped.publish(odomAftMapped);

    // static tf::TransformBroadcaster br;
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
    // br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
}

void pose_estimator::publish_path(const ros::Publisher& pubPath){
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "camera_init";
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}