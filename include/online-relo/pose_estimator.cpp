#include "pose_estimator.h"

pose_estimator::pose_estimator(){
    nh.param<std::string>("common/rootDir", rootDir, " ");
    nh.param<std::string>("common/pointCloudTopic", pointCloudTopic, "points_raw");
    nh.param<std::string>("common/imuTopic", imuTopic, "/imu");

    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, LIVOX);
    nh.param<int>("preprocess/livox_type", p_pre->livox_type, LIVOX_CUS);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 1);
    nh.param<bool>("preprocess/feature_extract_enable", p_pre->feature_enabled, false);

    nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
    nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());

    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);

    if(p_pre->lidar_type == LIVOX){
        subCloud = nh.subscribe(pointCloudTopic, 500, &pose_estimator::livox_pcl_cbk, this);
    }
    else{
        subCloud = nh.subscribe(pointCloudTopic, 500, &pose_estimator::standard_pcl_cbk, this);
    }
    
    subIMU = nh.subscribe(imuTopic, 20000, &pose_estimator::imuCBK, this);

    subPose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &pose_estimator::poseCBK, this);  // interaction in rviz
    subSrv = nh.subscribe<std_msgs::Bool>("/posesrv", 1, &pose_estimator::mannualCBK, this);   // TODO: external signal from your terminal
    // FIXME: rostopic pub /posesrv std_msgs/Bool "data: 1.0"

    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pubPriorMap = nh.advertise<sensor_msgs::PointCloud2>("/priorMap", 5);
    pubPriorPath = nh.advertise<sensor_msgs::PointCloud2>("/priorPath", 5);
    pubReloMap = nh.advertise<sensor_msgs::PointCloud2>("/reloMap_Ori", 5);
    pubReloDiffMap = nh.advertise<sensor_msgs::PointCloud2>("/reloMap_SC", 5);
    pubReloResMap = nh.advertise<sensor_msgs::PointCloud2>("/reloMap_Res", 5);
    pubReloNearMap = nh.advertise<sensor_msgs::PointCloud2>("/nearMap_Res", 5);
    pubReloFinalMap = nh.advertise<sensor_msgs::PointCloud2>("/reloMap_Final", 5);
    pubCurCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloudCur", 5);
    pubIkdTree = nh.advertise<sensor_msgs::PointCloud2>("/kdTree", 5);
    pubPath = nh.advertise<nav_msgs::Path>("/path", 1e00000);
    pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000); 

    std::cout << ANSI_COLOR_GREEN << "rostopic is ok" << ANSI_COLOR_RESET << std::endl;

    MultiSession::Session priorKnown_(1, "priorMap", rootDir, true);  // prior knowledge

    std::cout << ANSI_COLOR_GREEN << "create prior session" << ANSI_COLOR_RESET << std::endl;

    priorKnown = &priorKnown_;  // copy
    *priorMap += *priorKnown->globalMap;
    *priorPath += *priorKnown->cloudKeyPoses3D;
    downSizeFilterSurf.setLeafSize(0.1, 0.1, 0.1);

    std::cout << ANSI_COLOR_GREEN << "prior knowledge is loaded" << ANSI_COLOR_RESET << std::endl;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);        
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));           
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));            
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov)); 
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov)); 

    // esekf
    fill(epsi, epsi + 23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    std::cout << ANSI_COLOR_RED_BOLD << "please promise your device being static until the  relocalization finashed!!" << ANSI_COLOR_RESET << std::endl;
}

void pose_estimator::run(){
    ros::Rate rate(500);
    // std::cout << ANSI_COLOR_RED_BOLD << "please be static (but you can implement rotation), wait for the initial pose ..." << ANSI_COLOR_RESET << std::endl;
    bool status = ros::ok();
    while (status){
        if (flg_exit)
            break;
        ros::spinOnce();

        initpose_flag = getInitPose();
        
        if(initpose_flag){  //FIXME: how to transfer
            pcl::PointCloud<PointType>::Ptr pos_cloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr pos_cloud_trans(new pcl::PointCloud<PointType>());
            PointType pos;
            pos.x = pos_lid(0);
            pos.y = pos_lid(1);
            pos.z = pos_lid(2);
            pos_cloud->points.emplace_back(pos);
            *pos_cloud_trans += *transformPointCloud(pos_cloud, &finalpose);
            pos_lid(0) = pos_cloud_trans->points[0].x;
            pos_lid(1) = pos_cloud_trans->points[0].y;
            pos_lid(2) = pos_cloud_trans->points[0].z;
            initpose_flag = false;
            loc_flag = true;
        }

        if (sync_packages(Measures)){
            if (flg_first_scan){
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }
        }

        double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;
        double match_time = 0, kdtree_search_time = 0.0, solve_time = 0, solve_const_H_time = 0;
        t0 = omp_get_wtime();

        p_imu->Process(Measures, kf, feats_undistort);
        state_point = kf.get_x();  

        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

        if (feats_undistort->empty() || (feats_undistort == NULL)){
            if(cout_flg4 < 1)
                ROS_WARN("No point, skip this scan!\n");
            cout_flg4 ++;
            continue;
        }

        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

        lasermap_fov_segment();

        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body); 

        t1 = omp_get_wtime();  
        feats_down_size = feats_down_body->points.size();

        std::cout << "current raw cloud size: " << feats_down_size << std::endl;

        if (ikdtree.Root_Node == nullptr)
        {
            if (feats_down_size > 5)
            {
                ikdtree.set_downsample_param(filter_size_map_min); 
                feats_down_world->resize(feats_down_size);         
                for (int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                ikdtree.Build(feats_down_world->points); 
            }
            continue;      
        }

        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();

        /*** ICP and iterated Kalman filter update ***/
        if (feats_down_size < 5)
        {
            if(cout_flg5 < 1)
                ROS_WARN("No point, skip this scan!\n");
            cout_flg5 ++;
            continue;
        }

        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);

        // if (1) 
        // {
        //     PointVector().swap(ikdtree.PCL_Storage);                     
        //     ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD); 
        //     featsFromMap->clear();
        //     featsFromMap->points = ikdtree.PCL_Storage;
        //     publish_map(pubIkdTree);
        // }

        pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);

        int rematch_num = 0;
        bool nearest_search_en = true;

        t2 = omp_get_wtime();

        double t_update_start = omp_get_wtime();
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time); 
        state_point = kf.get_x();
        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I; // TODO: give initial pose here
        geoQuat.x = state_point.rot.coeffs()[0];                   
        geoQuat.y = state_point.rot.coeffs()[1];
        geoQuat.z = state_point.rot.coeffs()[2];
        geoQuat.w = state_point.rot.coeffs()[3];

        double t_update_end = omp_get_wtime();

        getCurPose(state_point);

        bool useRelo = easyToRelo();

        if(useRelo & loc_flag){
            std::cout << ANSI_COLOR_GREEN << "RELO MODE" << ANSI_COLOR_RESET << std::endl;
            recontructIKdTree();  // TODO: get current measurement in prior map
        }
        else{
            std::cout << ANSI_COLOR_GREEN << "LIO MODE" << ANSI_COLOR_RESET << std::endl;
            map_incremental(); // TODO: mapping
        }

        t3 = omp_get_wtime();

        publish_odometry(pubOdomAftMapped);
        publish_path(pubPath);
        publish_frame_world(pubCurCloud); 
        publish_map(pubIkdTree);

        status = ros::ok();
        rate.sleep();
    }

}

bool pose_estimator::sync_packages(MeasureGroup &meas){
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();        
        meas.lidar_beg_time = time_buffer.front(); 

        if (meas.lidar->points.size() <= 1)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num++; 
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time; 

        lidar_pushed = true; 
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    double imu_time = imu_buffer.front()->header.stamp.toSec(); 
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec(); 
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front()); 
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front(); 
    time_buffer.pop_front();  
    lidar_pushed = false;    
    return true;
}

void pose_estimator::cloudCBK(){
    // TODO: unified
}

void pose_estimator::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg){
    mtx_buffer.lock(); 
    double preprocess_start_time = omp_get_wtime();
    scan_count++; 
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu; 
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }
    pcl::PointCloud<PointType>::Ptr ptr(new pcl::PointCloud<PointType>());
    p_pre->process(msg, ptr);              
    lidar_buffer.push_back(ptr);                 
    time_buffer.push_back(last_timestamp_lidar); 

    mtx_buffer.unlock();    
    sig_buffer.notify_all();
}

void pose_estimator::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg){
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    pcl::PointCloud<PointType>::Ptr ptr(new pcl::PointCloud<PointType>());
    p_pre->process(msg, ptr);                       
    lidar_buffer.push_back(ptr);                      
    time_buffer.push_back(msg->header.stamp.toSec()); 
    last_timestamp_lidar = msg->header.stamp.toSec(); 

    mtx_buffer.unlock();     
    sig_buffer.notify_all(); 
}

void pose_estimator::imuCBK(const sensor_msgs::Imu::ConstPtr &msg_in){
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }
    double timestamp = msg->header.stamp.toSec(); // IMU时间戳

    mtx_buffer.lock(); 
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    // sensor_msgs::Imu::Ptr imu_first = imu_buffer.front();
    // sensor_msgs::Imu::Ptr imu_last = imu_buffer.back();

    // double x_acc_first = imu_first->linear_acceleration.x;
    // double y_acc_first = imu_first->linear_acceleration.y;
    // if(x_acc_first >= 8.0){
    //     ROS_ERROR("Wrong x-coordinates ...");
    //     x_acc_first = imu_first->linear_acceleration.z;
    // }

    // if(y_acc_first >= 8.0){
    //     ROS_ERROR("Wrong y-coordinates ...");
    //     y_acc_first = imu_first->linear_acceleration.z;
    // }

    // double x_acc_last = imu_last->linear_acceleration.x;
    // double y_acc_last = imu_last->linear_acceleration.y;
    // if(x_acc_last >= 8.0){
    //     ROS_ERROR("Wrong x-coordinates ...");
    //     x_acc_last = imu_last->linear_acceleration.z;
    // }

    // if(y_acc_last >= 8.0){
    //     ROS_ERROR("Wrong y-coordinates ...");
    //     y_acc_last = imu_last->linear_acceleration.z;
    // }

    // double t = timestamp - last_timestamp_imu;
    // double x_pos = 1 / 2 * (1 / 2 *(x_acc_first + x_acc_last)) * t * t;
    // double y_pos = 1 / 2 * (1 / 2 *(y_acc_first + y_acc_last)) * t * t;
    // double pos = std::sqrt(x_pos * x_pos + y_pos * y_pos);
    // if(pos >= 0.1){   // FIXME: how to get the offset of imu, we can not implement pre-intergration now 
    //     imu_static = true;
    // }

    last_timestamp_imu = timestamp; 

    imu_buffer.push_back(msg); 
    mtx_buffer.unlock();       
    sig_buffer.notify_all();   
}

void pose_estimator::poseCBK(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
    if(initpose_flag){
        return ;
    }
    PointType p;
    initpose.x = msg->pose.pose.position.x;
    initpose.y = msg->pose.pose.position.y;
    initpose.z = msg->pose.pose.position.z;
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // initpose.roll = roll;
    // initpose.pitch = pitch;
    initpose.roll = 0.0;   // FIXME: it's better to choose zero, becasue we do not believe the coarse roll and pitch from mannual set 
    initpose.pitch = 0.0;
    initpose.yaw = yaw;
    std::cout << ANSI_COLOR_RED << "Get initial pose: " << initpose.x << " " << initpose.y << " "
              << initpose.z << " " << roll << " " << pitch << " " << yaw << ANSI_COLOR_RESET << std::endl;
}

void pose_estimator::mannualCBK(const std_msgs::Bool::ConstPtr &msg){
    useMannual = msg->data;
    std::cout << ANSI_COLOR_GREEN << "revieve mannual flag comes, please set your pose ... " << ANSI_COLOR_RESET << std::endl;
}

void pose_estimator::lasermap_fov_segment(){
    cub_needrm.clear(); 
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    V3D pos_LiD = pos_lid; 

    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0; 
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;// 边界距离当前位置100米
        }
        Localmap_Initialized = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }

    if (!need_move)
        return;
    
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints); 
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;

}

void pose_estimator::publish_map(const ros::Publisher &pubLaserCloudMap){
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

void pose_estimator::getCurPose(state_ikfom cur_state){
    Eigen::Vector3d eulerAngle = cur_state.rot.matrix().eulerAngles(2, 1, 0); 
    transformTobeMapped[0] = eulerAngle(2);    
    transformTobeMapped[1] = eulerAngle(1);    
    transformTobeMapped[2] = eulerAngle(0);    
    transformTobeMapped[3] = cur_state.pos(0); 
    transformTobeMapped[4] = cur_state.pos(1); 
    transformTobeMapped[5] = cur_state.pos(2); 
}

void pose_estimator::recontructIKdTree(){
    pcl::PointCloud<PointType>::Ptr subMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr subMapKeyFramesDS(new pcl::PointCloud<PointType>());

    for(int i = 0; i < pointSearchIndGlobalMap.size(); i++){
        *subMapKeyFrames += *transformPointCloud(priorKnown->cloudKeyFrames[pointSearchIndGlobalMap[i]].all_cloud, &priorKnown->cloudKeyPoses6D->points[pointSearchIndGlobalMap[i]]);
    }

    downSizeFilterSurf.setInputCloud(subMapKeyFrames);
    downSizeFilterSurf.filter(*subMapKeyFramesDS);
    ikdtree.reconstruct(subMapKeyFramesDS->points);
    int featsFromMapNum = ikdtree.validnum();
    kdtree_size_st = ikdtree.size();

    std::cout << "current measurement in prior map: " << " featsFromMapNum  =  " << featsFromMapNum << "\t"
                  << " kdtree_size_st   =  " << kdtree_size_st << std::endl;

    featsFromMap->clear();  // publish kdtree
    featsFromMap->points = subMapKeyFramesDS->points;
}
 
void pose_estimator::pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pose_estimator::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
}

void pose_estimator::map_incremental()
{
    PointVector PointToAdd;           
    PointVector PointNoNeedDownsample; 
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++)
    {
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point); 

            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]); 
                continue;
            }

            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) 
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add){
                featsFromMap->points.push_back(feats_down_world->points[i]);
                PointToAdd.push_back(feats_down_world->points[i]);
            }
        }
        else
        {
            featsFromMap->points.push_back(feats_down_world->points[i]);
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true); 
    ikdtree.Add_Points(PointNoNeedDownsample, false);      
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;

    downSizeFilterSurf.setInputCloud(featsFromMap);
    downSizeFilterSurf.filter(*featsFromMap);

    std::cout << "map incremental: " << " add_point_size  =  " << add_point_size << "\t"
                  << " kdtree_incremental_time   =  " << kdtree_incremental_time << std::endl;
}

void pose_estimator::publish_path(const ros::Publisher pubPath){
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}

void pose_estimator::publish_frame_world(const ros::Publisher &pubLaserCloudFull){
    pcl::PointCloud<PointType>::Ptr laserCloudFullRes(feats_down_body);
    int size = laserCloudFullRes->points.size();  
    pcl::PointCloud<PointType>::Ptr laserCloudWorld(new pcl::PointCloud<PointType>(size, 1));
    for (int i = 0; i < size; i++)
    {
        pointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
    }
    *full_cloud += *laserCloudWorld;
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFull.publish(laserCloudmsg);
}

bool pose_estimator::easyToRelo(){
    pointSearchIndGlobalMap.clear();
    pointSearchSqDisGlobalMap.clear();

    kdtreeGlobalMapPoses->setInputCloud(priorPath);  // find nearest poses in prior map
    PointType curPose;
    curPose.x = transformTobeMapped[3];
    curPose.y = transformTobeMapped[4];
    curPose.z = transformTobeMapped[5];
    kdtreeGlobalMapPoses->radiusSearch(curPose, 5.0, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);  // TODO: add the radius into yaml

    if(pointSearchIndGlobalMap.size() >= 2){  // TODO: add the radius into yaml
        return true;
    }
    else{
        return false;
    }
}

bool pose_estimator::getInitPose(){
    TicToc time_count;
    if(imu_buffer.empty() || lidar_buffer.empty()){
        if(cout_flg1 < 1)
            ROS_ERROR("There are no imu or lidar inputs, can not get initial pose ...");
        cout_flg1 ++;
        return false;
    }
    
    if(lidar_buffer.size() <= 5){  // FIXME: need to be checked  5
        if(cout_flg2 < 1)
            ROS_ERROR("There are no enough lidar inputs, please wait ...");
        cout_flg2 ++;
        return false;
    }

    double x_offset = state_point.pos(0);
    double y_offset = state_point.pos(1);

    if(std::sqrt(x_offset * x_offset + y_offset * y_offset) <= 0.1 & lidar_buffer.size() <= 10){   // FIXME: need to be checked  0.1
        if(cout_flg3 < 1)
            ROS_ERROR("wait for the boost ...");
        cout_flg3 ++;
        return false;
    }

    std::cout << ANSI_COLOR_GREEN << "It is ready to get initial pose ..." << ANSI_COLOR_RESET << std::endl;

    pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>());

    for(int i = 0; i < ((lidar_buffer.size() - 1) >= 5 ? 5 : (lidar_buffer.size() - 1)); i++){
        *reloCloud += *(lidar_buffer[i]);
    }
    std::cout << ANSI_COLOR_GREEN << "Generated RELO CLOUD size: " << reloCloud->points.size() << ANSI_COLOR_RESET << std::endl;

    reloCloud->width = reloCloud->points.size();
    reloCloud->height = 1;
    Eigen::MatrixXd curSC = priorKnown->scManager.makeScancontext(*reloCloud);  // FIXME: just use a single scan !! please stay static before get accuracy pose
    std::cout << "current sc: " << curSC << std::endl;
    Eigen::MatrixXd ringkey = priorKnown->scManager.makeRingkeyFromScancontext(curSC);
    Eigen::MatrixXd sectorkey = priorKnown->scManager.makeSectorkeyFromScancontext(curSC);
    std::vector<float> polarcontext_invkey_vec = ScanContext::eig2stdvec(ringkey);

    auto detectResult = priorKnown->scManager.detectLoopClosureIDBetweenSession(polarcontext_invkey_vec, curSC);  // idx & yaw_diff
    
    reloIdx = detectResult.first;
    float yawDiff = detectResult.second;

    std::cout << ANSI_COLOR_GREEN << "Scan Context Relo at Idx: " << reloIdx << " in prior map; " 
                       " yaw diff: " << yawDiff << ANSI_COLOR_RESET << std::endl;

    PointTypePose diffPose;
    diffPose.x = priorKnown->cloudKeyPoses6D->points[reloIdx].x;
    diffPose.y = priorKnown->cloudKeyPoses6D->points[reloIdx].y;
    diffPose.z = priorKnown->cloudKeyPoses6D->points[reloIdx].z;
    diffPose.roll = priorKnown->cloudKeyPoses6D->points[reloIdx].roll;
    diffPose.pitch = priorKnown->cloudKeyPoses6D->points[reloIdx].pitch;
    diffPose.yaw = priorKnown->cloudKeyPoses6D->points[reloIdx].yaw + yawDiff;  // FIXME: check it "+" or "-"

    *reloCloud_diff += *transformPointCloud(reloCloud, &diffPose);  // just publish

    diffPose.x = 0.0;
    diffPose.y = 0.0;
    diffPose.z = 0.0;
    diffPose.roll = 0.0;
    diffPose.yaw = yawDiff; // compensatation for yaw
    finalpose.yaw = yawDiff; // FIXME: add
    *reloCloud_res += *transformPointCloud(reloCloud, &diffPose); // for registeration

    // sub2map registeration
    near_cloud->clear();
    PointTypePose ptd;
    ptd.x = -priorKnown->cloudKeyPoses6D->points[reloIdx].x;
    ptd.y = -priorKnown->cloudKeyPoses6D->points[reloIdx].y;
    ptd.z = -priorKnown->cloudKeyPoses6D->points[reloIdx].z;
    ptd.roll = -priorKnown->cloudKeyPoses6D->points[reloIdx].roll;
    ptd.pitch = -priorKnown->cloudKeyPoses6D->points[reloIdx].pitch;
    ptd.yaw = -priorKnown->cloudKeyPoses6D->points[reloIdx].yaw;
    *near_cloud += *priorKnown->cloudKeyFrames[reloIdx].all_cloud;
    if(reloIdx - 1 > 0){
        tmp_cloud->clear();
        tmp_cloud = transformPointCloud(priorKnown->cloudKeyFrames[reloIdx - 1].all_cloud, &priorKnown->cloudKeyPoses6D->points[reloIdx - 1]);
        tmp_cloud = transformPointCloud(tmp_cloud, &ptd);
        *near_cloud += *tmp_cloud;
    }
    if(reloIdx + 1 <= priorKnown->cloudKeyPoses6D->points.size() - 1){
        tmp_cloud->clear();
        tmp_cloud = transformPointCloud(priorKnown->cloudKeyFrames[reloIdx + 1].all_cloud, &priorKnown->cloudKeyPoses6D->points[reloIdx + 1]);
        tmp_cloud = transformPointCloud(tmp_cloud, &ptd);
        *near_cloud += *tmp_cloud;
    }

    // TODO: check and should we select the mannual rivz pose
    if(!useMannual){
        std::cout << ANSI_COLOR_GREEN_BOLD << "Are you sure that we do not need a external pose set from rviz ? " << "\n"
                           <<"If you wanna a mannual set, please give it in 10s in rviz ..."
                           <<  ANSI_COLOR_RESET << std::endl;
        ros::Duration(10).sleep();  // FIXME: check it
    }
    if(initpose.x != 0 || initpose.yaw != 0){
        useMannual = true;
        std::cout << ANSI_COLOR_GREEN_BOLD << "Reveieve mannual input ..." << ANSI_COLOR_RESET << std::endl;
    }

    if(!useMannual){
        std::cout << ANSI_COLOR_RED_BOLD << "There is no external pose input ..." << ANSI_COLOR_RESET << std::endl;
    }
    else{
        std::cout << ANSI_COLOR_GREEN_BOLD << "There is a external pose input ..." << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_RED_BOLD << "Scan Context Relo is skipped" << ANSI_COLOR_RESET << std::endl;

        PointType pt;
        pt.x = initpose.x;
        pt.y = initpose.y;
        pt.z = initpose.z;

        pointSearchIndGlobalMap.clear();
        pointSearchSqDisGlobalMap.clear();
        kdtreeGlobalMapPoses->setInputCloud(priorKnown->cloudKeyPoses3D);
        kdtreeGlobalMapPoses->nearestKSearch(pt, 5, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
        if(pointSearchSqDisGlobalMap[0] >= 5.0){   // FIXME: check 5.0
            std::cout << ANSI_COLOR_RED_BOLD << "The external input may have problem !! " << ANSI_COLOR_RESET << std::endl;
        }
        else{
            PointTypePose pt;
            pt.x = -priorKnown->cloudKeyPoses6D->points[pointSearchIndGlobalMap[0]].x;
            pt.y = -priorKnown->cloudKeyPoses6D->points[pointSearchIndGlobalMap[0]].y;
            pt.z = -priorKnown->cloudKeyPoses6D->points[pointSearchIndGlobalMap[0]].z;
            pt.roll = -priorKnown->cloudKeyPoses6D->points[pointSearchIndGlobalMap[0]].roll;
            pt.pitch = -priorKnown->cloudKeyPoses6D->points[pointSearchIndGlobalMap[0]].pitch;
            pt.yaw = -priorKnown->cloudKeyPoses6D->points[pointSearchIndGlobalMap[0]].yaw;

            near_cloud->clear();
            *near_cloud += *priorKnown->cloudKeyFrames[pointSearchIndGlobalMap[0]].all_cloud;
            tmp_cloud->clear();
            tmp_cloud = transformPointCloud(priorKnown->cloudKeyFrames[pointSearchIndGlobalMap[1]].all_cloud, &priorKnown->cloudKeyPoses6D->points[pointSearchIndGlobalMap[1]]);
            tmp_cloud = transformPointCloud(tmp_cloud, &pt);
            *near_cloud += *tmp_cloud;
            tmp_cloud->clear();
            tmp_cloud = transformPointCloud(priorKnown->cloudKeyFrames[pointSearchIndGlobalMap[2]].all_cloud, &priorKnown->cloudKeyPoses6D->points[pointSearchIndGlobalMap[2]]);
            tmp_cloud = transformPointCloud(tmp_cloud, &pt);
            *near_cloud += *tmp_cloud;

            std::cout << ANSI_COLOR_GREEN << "The reloCloud is changed ..." << ANSI_COLOR_RESET << std::endl;

            pt.x = 0.0;
            pt.y = 0.0;
            pt.z = 0.0;
            pt.roll = 0.0;
            pt.pitch = 0.0;
            pt.yaw = pt.yaw - initpose.yaw;  // FIXME: check it
            finalpose.yaw = pt.yaw - initpose.yaw;

            reloCloud_res->clear();
            *reloCloud_res += *transformPointCloud(reloCloud, &pt);
        }
    }

    // TODO: teaser ++ to get the precise initial pose in global map and reset it in the ieskf !!
    fpfh_teaser teaser(reloCloud_res, near_cloud);
    std::pair<double, Eigen::Isometry3f> results = teaser.match();
    Eigen::Matrix4f transMatrix = results.second.matrix();
    Eigen::Matrix3f rot = transMatrix.block<3, 3>(0, 0);
    Eigen::Matrix<float, 1, 3> trans = transMatrix.block<1, 3>(3, 0);

    Eigen::Matrix<float, 3, 1> euler = RotMtoEuler(rot);
    finalpose.x = trans(0, 0);
    finalpose.y = trans(0, 1);
    finalpose.z = trans(0, 2);
    finalpose.roll = euler(0, 0);
    finalpose.pitch = euler(1, 0);
    finalpose.yaw += euler(2, 0);

    std::cout << ANSI_COLOR_GREEN_BOLD << "FPFH Teaser Results: " << "\n"
                       << " x: " <<  finalpose.x << " y: " << finalpose.y << " z: " << finalpose.z
                       << " roll: " << finalpose.roll << " pitch: " << finalpose.pitch << " yaw: " << finalpose.yaw
                       << ANSI_COLOR_RESET << std::endl;

    if(results.first <= 1.0){
        *final_cloud += *transformPointCloud(reloCloud, &finalpose);
        return true;
    }
    else{
        finalpose = initpose;
        *final_cloud += *transformPointCloud(reloCloud, &finalpose);
        std::cout << ANSI_COLOR_RED << "FPFH Teaser is not precise !! We just use the mannual set ..." << ANSI_COLOR_RESET << std::endl;
        return true;
    }
}

void pose_estimator::publishThread(){
    ros::Rate rate(5);
    while(ros::ok()){
        ros::spinOnce();
        
        // TODO: publish relocloud 
        publishCloud(&pubReloMap, reloCloud, ros::Time::now(), "camera_init");  
        publishCloud(&pubReloDiffMap, reloCloud_diff, ros::Time::now(), "camera_init");   // sc
        publishCloud(&pubReloResMap, reloCloud_res, ros::Time::now(), "camera_init");  // to res
        publishCloud(&pubReloNearMap, near_cloud, ros::Time::now(), "camera_init");  // near
        publishCloud(&pubReloFinalMap, final_cloud, ros::Time::now(), "camera_init");  // final
        publishCloud(&pubPriorMap, priorMap, ros::Time::now(), "camera_init");  // too large
        publishCloud(&pubPriorPath, priorPath, ros::Time::now(), "camera_init");  
        publishCloud(&pubLaserCloudFull, full_cloud, ros::Time::now(), "camera_init");  

        rate.sleep();
    }
}
