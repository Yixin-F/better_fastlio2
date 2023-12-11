#include "pose_estimator.h"
  
  pose_estimator::pose_estimator()
  {
    nh_.param<std::string>("common/rootDir", rootDir, " ");
    nh_.param<std::string>("location/filedir", filename, "");
    nh_.param<std::string>("location/pointCloudTopic", pointCloudTopic, "points_raw");
    nh_.param<std::string>("location/imuTopic", imu_topic, "/livox/imu");
    nh_.param<int>("location/IMU_Mode", IMU_Mode, 0);
    nh_.param<bool>("location/use_lio", use_lio, false);
    nh_.param<double>("location/corner_leaf_", corner_leaf_, 0.2);
    nh_.param<double>("location/surf_leaf_", surf_leaf_, 0.5);

    sub_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 50, &pose_estimator::cloudHandler, this);
    if (IMU_Mode > 0)
      sub_imu_ = nh_.subscribe(imu_topic, 2000, &pose_estimator::imu_callback, this);
    if (IMU_Mode < 2)
      WINDOWSIZE = 1;
    else
      WINDOWSIZE = 20;
    sub_initial_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &pose_estimator::initialPoseCB, this);

    ds_corner_.setLeafSize(corner_leaf_, corner_leaf_, corner_leaf_);
    ds_surf_.setLeafSize(surf_leaf_, surf_leaf_, surf_leaf_);

    if (loadmap())
      std::cout << ANSI_COLOR_GREEN << "load map successful..." << ANSI_COLOR_RESET << std::endl;
    else
    {
      std::cout << ANSI_COLOR_RED_BOLD << "WARN: load map failed." << ANSI_COLOR_RESET << std::endl;
      return;
    }

    std::cout << "ROOT_DIR: " << rootDir << std::endl;

    //  create folder
    fsmkdir(rootDir);

    surround_surf.reset(new pcl::PointCloud<PointType>());
    surround_corner.reset(new pcl::PointCloud<PointType>());
    kdtree_keyposes_3d_.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_keyposes_3d_->setInputCloud(map.cloudKeyPoses3D_); // init 3d-pose kdtree

    kdtree_corner_map.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_corner_map->setInputCloud(map.globalCornerMapCloud_);
    kdtree_surf_map.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_surf_map->setInputCloud(map.globalSurfMapCloud_);

    kdtree_corner_localmap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_surf_localmap.reset(new pcl::KdTreeFLANN<PointType>());

    publishCloud(&pub_corner_map, map.globalCornerMapCloud_, ros::Time::now(), "world");

    initializedFlag = NonInitialized;

    // slide windows
    for (int i = 0; i < localMapWindowSize; i++)
    {
      localCornerMap[i].reset(new pcl::PointCloud<PointType>);
      localSurfMap[i].reset(new pcl::PointCloud<PointType>);
    }
    laserCloudCornerFromLocal.reset(new pcl::PointCloud<PointType>);
    laserCloudSurfFromLocal.reset(new pcl::PointCloud<PointType>);

    map_manager = new MAP_MANAGER(0.2, 0.3);
    lidarFrameList.reset(new std::list<LidarFrame>);
  }

  void pose_estimator::cloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (!_lidarMsgQueue.empty() && (initializedFlag != Initialized)) //  由于tf关系，导致发布时间存在滞后，tf无法显示
      _lidarMsgQueue.clear();
    _lidarMsgQueue.push_back(msg);
  }

  void pose_estimator::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
  {
    // push IMU msg to queue
    _imuMsgQueue.push_back(imu_msg);
  }

  void pose_estimator::ExtractFeature(LidarFrame &kf)
  {
    kf.corner->clear();
    kf.surf->clear();
    for (const auto &p : kf.laserCloud->points)
    {
      if (std::fabs(p.normal_z - 1.0) < 1e-5)
        kf.corner->push_back(p);
    }
    for (const auto &p : kf.laserCloud->points)
    {
      if (std::fabs(p.normal_z - 2.0) < 1e-5)
        kf.surf->push_back(p);
    }
    ds_surf_.setInputCloud(kf.surf);
    ds_surf_.filter(*kf.surf);
    ds_corner_.setInputCloud(kf.corner);
    ds_corner_.filter(*kf.corner);
  }

  void pose_estimator::initialPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
  {
    //  低频，不加锁
    PointType p;

    initpose.x = msg->pose.pose.position.x;
    initpose.y = msg->pose.pose.position.y;
    initpose.z = msg->pose.pose.position.z;
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    initpose.roll = roll;
    initpose.pitch = pitch;
    initpose.yaw = yaw;

    p.x = initpose.x;
    p.y = initpose.y;
    p.z = initpose.z;
    std::cout << ANSI_COLOR_RED << "Get initial pose: " << initpose.x << " " << initpose.y << " "
              << initpose.z << " " << roll << " " << pitch << " " << yaw << ANSI_COLOR_RESET << std::endl;
    extractSurroundKeyFrames(p);
    std::cout << ANSI_COLOR_YELLOW << "Change flat from " << initializedFlag
              << " to " << Initializing << ", start do localizating ..."
              << ANSI_COLOR_RESET << std::endl;
    if (initializedFlag != NonInitialized)
    { // TODO: 非第一次执行，需要重置部分参数
      delta_Rl = Eigen::Matrix3d::Identity();
      delta_tl = Eigen::Vector3d::Zero();
      for (int i = 0; i < localMapWindowSize; i++)
      {
        localCornerMap[i]->clear();
        localSurfMap[i]->clear();
      }
      localMapID = 0;
      laserCloudCornerFromLocal->clear();
      laserCloudSurfFromLocal->clear();
    }
    initializedFlag = Initializing;
  }

  void pose_estimator::pubOdometry(Eigen::Matrix4d pose, double &time)
  {
    Eigen::Quaterniond Q(pose.block<3, 3>(0, 0));
    ros::Time ros_time = ros::Time().fromSec(time);
    transform_.stamp_ = ros_time;
    transform_.setRotation(tf::Quaternion(Q.x(), Q.y(), Q.z(), Q.w()));
    transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
    transform_.frame_id_ = "world";
    transform_.child_frame_id_ = "base_link";
    broadcaster_.sendTransform(transform_);

    geometry_msgs::PoseStamped laserPose;
    laserPose.header.frame_id = "world";
    laserPose.header.stamp = ros_time;

    laserPose.pose.orientation.x = Q.x();
    laserPose.pose.orientation.y = Q.y();
    laserPose.pose.orientation.z = Q.z();
    laserPose.pose.orientation.w = Q.w();
    laserPose.pose.position.x = pose(0, 3);
    laserPose.pose.position.y = pose(1, 3);
    laserPose.pose.position.z = pose(2, 3);

    laserOdoPath.header.stamp = ros_time;
    laserOdoPath.poses.push_back(laserPose);
    laserOdoPath.header.frame_id = "/world";
    pubLaserOdometryPath_.publish(laserOdoPath);
  }

  void pose_estimator::pubOdometry(LidarFrame &frame)
  {
    ros::Time ros_time = ros::Time().fromSec(frame.timeStamp);
    transform_.stamp_ = ros_time;
    transform_.setRotation(tf::Quaternion(frame.Q.x(), frame.Q.y(), frame.Q.z(), frame.Q.w()));
    transform_.setOrigin(tf::Vector3(frame.P(0), frame.P(1), frame.P(2)));
    transform_.frame_id_ = "world";
    transform_.child_frame_id_ = "base_link";
    broadcaster_.sendTransform(transform_);

    geometry_msgs::PoseStamped laserPose;
    laserPose.header.frame_id = "world";
    laserPose.header.stamp = ros_time;

    laserPose.pose.orientation.x = frame.Q.x();
    laserPose.pose.orientation.y = frame.Q.y();
    laserPose.pose.orientation.z = frame.Q.z();
    laserPose.pose.orientation.w = frame.Q.w();
    laserPose.pose.position.x = frame.P(0);
    laserPose.pose.position.y = frame.P(1);
    laserPose.pose.position.z = frame.P(2);

    laserOdoPath.header.stamp = ros_time;
    laserOdoPath.poses.push_back(laserPose);
    laserOdoPath.header.frame_id = "/world";
    pubLaserOdometryPath_.publish(laserOdoPath);
  }

  void pose_estimator::vector2double(const std::list<LidarFrame> &tempFrameList)
  {
    int i = 0;
    for (const auto &l : tempFrameList)
    {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> PR(para_PR[i]);
      PR.segment<3>(0) = l.P;
      PR.segment<3>(3) = Sophus::SO3d(l.Q).log();

      Eigen::Map<Eigen::Matrix<double, 9, 1>> VBias(para_VBias[i]);
      VBias.segment<3>(0) = l.V;
      VBias.segment<3>(3) = l.bg;
      VBias.segment<3>(6) = l.ba;
      i++;
    }
  }

  void pose_estimator::double2vector(std::list<LidarFrame> &tempFrameList)
  {
    int i = 0;
    for (auto &l : tempFrameList)
    {
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> PR(para_PR[i]);
      Eigen::Map<const Eigen::Matrix<double, 9, 1>> VBias(para_VBias[i]);
      l.P = PR.segment<3>(0);
      l.Q = Sophus::SO3d::exp(PR.segment<3>(3)).unit_quaternion();
      l.V = VBias.segment<3>(0);
      l.bg = VBias.segment<3>(3);
      l.ba = VBias.segment<3>(6);
      i++;
    }
  }

  void pose_estimator::Estimate(std::list<LidarFrame> &frameList, const Eigen::Vector3d &gravity)
  {
    TicToc etc;
    int num_corner_map = 0;
    int num_surf_map = 0;
    int windowSize = frameList.size();
    double t_kd = 0, t_ext = 0;
    etc.tic();
    if (use_lio)
    {
      kdtree_corner_localmap->setInputCloud(laserCloudCornerFromLocal);
      kdtree_surf_localmap->setInputCloud(laserCloudSurfFromLocal);
    }
    t_kd = etc.toc();

    etc.tic();
    for (auto &frame : frameList)
      ExtractFeature(frame);
    t_ext = etc.toc();
    std::cout << "make kd-tree: " << t_kd << "ms, extract feature: " << t_ext << "ms" << std::endl;

    // store point to line features
    std::vector<std::vector<FeatureLine>> vLineFeatures(windowSize);
    for (auto &v : vLineFeatures)
      v.reserve(2000);

    // store point to plan features
    std::vector<std::vector<FeaturePlanVec>> vPlanFeatures(windowSize);
    for (auto &v : vPlanFeatures)
      v.reserve(2000);

    if (windowSize == SLIDEWINDOWSIZE)
    {
      plan_weight_tan = 0.0003;
      thres_dist = 1.0;
    }
    else
    {
      plan_weight_tan = 0.0;
      thres_dist = 25.0;
    }

    // excute optimize process
    const int max_iters = 5;
    int iterOpt = 0;
    for (; iterOpt < max_iters; ++iterOpt)
    {
      double t_search = 0, t_ass = 0, t_solve = 0, t_marg = 0;
      vector2double(frameList);

      // create huber loss function
      ceres::LossFunction *loss_function = NULL;
      loss_function = new ceres::HuberLoss(0.1 / IMUIntegrator::lidar_m);
      if (windowSize == SLIDEWINDOWSIZE)
      {
        loss_function = NULL;
      }
      else
      {
        loss_function = new ceres::HuberLoss(0.1 / IMUIntegrator::lidar_m);
      }

      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);

      for (int i = 0; i < windowSize; ++i)
      {
        problem.AddParameterBlock(para_PR[i], 6);
      }

      for (int i = 0; i < windowSize; ++i)
        problem.AddParameterBlock(para_VBias[i], 9);

      //  TODO: add imu here
      for (int f = 1; f < windowSize; ++f)
      {
        auto frame_curr = frameList.begin();
        std::advance(frame_curr, f);
        problem.AddResidualBlock(Cost_NavState_PRV_Bias::Create(frame_curr->imuIntegrator,
                                                                const_cast<Eigen::Vector3d &>(gravity),
                                                                Eigen::LLT<Eigen::Matrix<double, 15, 15>>(frame_curr->imuIntegrator.GetCovariance().inverse())
                                                                    .matrixL()
                                                                    .transpose()),
                                 nullptr,
                                 para_PR[f - 1],
                                 para_VBias[f - 1],
                                 para_PR[f],
                                 para_VBias[f]);
      }

      if (last_marginalization_info)
      {
        // construct new marginlization_factor
        auto *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, nullptr,
                                 last_marginalization_parameter_blocks);
      }

      Eigen::Quaterniond q_before_opti = frameList.back().Q;
      Eigen::Vector3d t_before_opti = frameList.back().P;

      std::vector<std::vector<ceres::CostFunction *>> edgesLine(windowSize);
      std::vector<std::vector<ceres::CostFunction *>> edgesPlan(windowSize);
      std::thread threads[2];

      etc.tic();
      for (int f = 0; f < windowSize; ++f)
      {
        auto frame_curr = frameList.begin();
        std::advance(frame_curr, f);
        Eigen::Matrix4d transformTobeMapped = Eigen::Matrix4d::Identity();
        transformTobeMapped.topLeftCorner(3, 3) = frame_curr->Q.toRotationMatrix();
        transformTobeMapped.topRightCorner(3, 1) = frame_curr->P;

        threads[0] = std::thread(&pose_estimator::processPointToLine, this,
                                 std::ref(edgesLine[f]),
                                 std::ref(vLineFeatures[f]),
                                 std::ref(frame_curr->corner),
                                 std::ref(map.globalCornerMapCloud_),
                                 std::ref(kdtree_corner_map),
                                 std::ref(laserCloudCornerFromLocal),
                                 std::ref(kdtree_corner_localmap),
                                 std::ref(exTlb),
                                 std::ref(transformTobeMapped));

        threads[1] = std::thread(&pose_estimator::processPointToPlanVec, this,
                                 std::ref(edgesPlan[f]),
                                 std::ref(vPlanFeatures[f]),
                                 std::ref(frame_curr->surf),
                                 std::ref(map.globalSurfMapCloud_),
                                 std::ref(kdtree_surf_map),
                                 std::ref(laserCloudSurfFromLocal),
                                 std::ref(kdtree_surf_localmap),
                                 std::ref(exTlb),
                                 std::ref(transformTobeMapped));

        threads[0].join();
        threads[1].join();
      }
      t_search = etc.toc();

      etc.tic();
      if (windowSize == SLIDEWINDOWSIZE)
      {
        thres_dist = 1.0;
        if (iterOpt == 0)
        {
          for (int f = 0; f < windowSize; ++f)
          {
            int cntFtu = 0;
            for (auto &e : edgesLine[f])
            {
              if (std::fabs(vLineFeatures[f][cntFtu].error) > 1e-5)
              {
                problem.AddResidualBlock(e, loss_function, para_PR[f]);
                vLineFeatures[f][cntFtu].valid = true;
              }
              else
              {
                vLineFeatures[f][cntFtu].valid = false;
              }
              cntFtu++;
            }

            cntFtu = 0;
            for (auto &e : edgesPlan[f])
            {
              if (std::fabs(vPlanFeatures[f][cntFtu].error) > 1e-5)
              {
                problem.AddResidualBlock(e, loss_function, para_PR[f]);
                vPlanFeatures[f][cntFtu].valid = true;
              }
              else
              {
                vPlanFeatures[f][cntFtu].valid = false;
              }
              cntFtu++;
            }
          }
        }
        else
        {
          for (int f = 0; f < windowSize; ++f)
          {
            int cntFtu = 0;
            for (auto &e : edgesLine[f])
            {
              if (vLineFeatures[f][cntFtu].valid)
              {
                problem.AddResidualBlock(e, loss_function, para_PR[f]);
              }
              cntFtu++;
            }
            cntFtu = 0;
            for (auto &e : edgesPlan[f])
            {
              if (vPlanFeatures[f][cntFtu].valid)
              {
                problem.AddResidualBlock(e, loss_function, para_PR[f]);
              }
              cntFtu++;
            }
          }
        }
      }
      else
      {
        if (iterOpt == 0)
        {
          thres_dist = 10.0;
        }
        else
        {
          thres_dist = 1.0;
        }
        for (int f = 0; f < windowSize; ++f)
        {
          int cntFtu = 0;
          for (auto &e : edgesLine[f])
          {
            if (std::fabs(vLineFeatures[f][cntFtu].error) > 1e-5)
            {
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
              vLineFeatures[f][cntFtu].valid = true;
            }
            else
            {
              vLineFeatures[f][cntFtu].valid = false;
            }
            cntFtu++;
          }
          cntFtu = 0;
          for (auto &e : edgesPlan[f])
          {
            if (std::fabs(vPlanFeatures[f][cntFtu].error) > 1e-5)
            {
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
              vPlanFeatures[f][cntFtu].valid = true;
            }
            else
            {
              vPlanFeatures[f][cntFtu].valid = false;
            }
            cntFtu++;
          }
        }
      }
      t_ass = etc.toc();

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_SCHUR;
      options.trust_region_strategy_type = ceres::DOGLEG;
      options.max_num_iterations = 5;
      options.minimizer_progress_to_stdout = false;
      options.num_threads = 6;
      ceres::Solver::Summary summary;
      etc.tic();
      ceres::Solve(options, &problem, &summary);
      t_solve = etc.toc();

      double2vector(frameList);

      Eigen::Quaterniond q_after_opti = frameList.back().Q;
      Eigen::Vector3d t_after_opti = frameList.back().P;

      double deltaR = (q_before_opti.angularDistance(q_after_opti)) * 180.0 / M_PI;
      double deltaT = (t_before_opti - t_after_opti).norm();
      std::cout << "make ass takes: " << t_search << "ms, ass takes: " << t_ass << "ms, solve takes: " << t_solve << std::endl;
      if (deltaR < 0.05 && deltaT < 0.05 || (iterOpt + 1) == max_iters)
      {
        if (windowSize != SLIDEWINDOWSIZE)
          break;
        etc.tic();
        auto *marginalization_info = new MarginalizationInfo();
        if (last_marginalization_info)
        {
          std::vector<int> drop_set;
          for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
          {
            if (last_marginalization_parameter_blocks[i] == para_PR[0] ||
                last_marginalization_parameter_blocks[i] == para_VBias[0])
              drop_set.push_back(i);
          }

          auto *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
          auto *residual_block_info = new ResidualBlockInfo(marginalization_factor, nullptr,
                                                            last_marginalization_parameter_blocks,
                                                            drop_set);
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        auto frame_curr = frameList.begin();
        std::advance(frame_curr, 1);
        ceres::CostFunction *IMU_Cost = Cost_NavState_PRV_Bias::Create(frame_curr->imuIntegrator,
                                                                       const_cast<Eigen::Vector3d &>(gravity),
                                                                       Eigen::LLT<Eigen::Matrix<double, 15, 15>>(frame_curr->imuIntegrator.GetCovariance().inverse())
                                                                           .matrixL()
                                                                           .transpose());
        auto *residual_block_info = new ResidualBlockInfo(IMU_Cost, nullptr,
                                                          std::vector<double *>{para_PR[0], para_VBias[0], para_PR[1], para_VBias[1]},
                                                          std::vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);

        int f = 0;
        Eigen::Matrix4d transformTobeMapped = Eigen::Matrix4d::Identity();
        transformTobeMapped.topLeftCorner(3, 3) = frame_curr->Q.toRotationMatrix();
        transformTobeMapped.topRightCorner(3, 1) = frame_curr->P;
        edgesLine[f].clear();
        edgesPlan[f].clear();
        threads[0] = std::thread(&pose_estimator::processPointToLine, this,
                                 std::ref(edgesLine[f]),
                                 std::ref(vLineFeatures[f]),
                                 std::ref(frame_curr->corner),
                                 std::ref(map.globalCornerMapCloud_),
                                 std::ref(kdtree_corner_map),
                                 std::ref(laserCloudCornerFromLocal),
                                 std::ref(kdtree_corner_localmap),
                                 std::ref(exTlb),
                                 std::ref(transformTobeMapped));

        threads[1] = std::thread(&pose_estimator::processPointToPlanVec, this,
                                 std::ref(edgesPlan[f]),
                                 std::ref(vPlanFeatures[f]),
                                 std::ref(frame_curr->surf),
                                 std::ref(map.globalSurfMapCloud_),
                                 std::ref(kdtree_surf_map),
                                 std::ref(laserCloudSurfFromLocal),
                                 std::ref(kdtree_surf_localmap),
                                 std::ref(exTlb),
                                 std::ref(transformTobeMapped));

        threads[0].join();
        threads[1].join();

        int cntFtu = 0;
        for (auto &e : edgesLine[f])
        {
          if (vLineFeatures[f][cntFtu].valid)
          {
            auto *residual_block_info = new ResidualBlockInfo(e, nullptr,
                                                              std::vector<double *>{para_PR[0]},
                                                              std::vector<int>{0});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          }
          cntFtu++;
        }
        cntFtu = 0;
        for (auto &e : edgesPlan[f])
        {
          if (vPlanFeatures[f][cntFtu].valid)
          {
            auto *residual_block_info = new ResidualBlockInfo(e, nullptr,
                                                              std::vector<double *>{para_PR[0]},
                                                              std::vector<int>{0});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          }
          cntFtu++;
        }

        marginalization_info->preMarginalize();
        marginalization_info->marginalize();

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i < SLIDEWINDOWSIZE; i++)
        {
          addr_shift[reinterpret_cast<long>(para_PR[i])] = para_PR[i - 1];
          addr_shift[reinterpret_cast<long>(para_VBias[i])] = para_VBias[i - 1];
        }
        std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        t_marg = etc.toc();
        std::cout << "marg takes:" << t_marg << "ms" << std::endl;
        break;
      }
      if (windowSize != SLIDEWINDOWSIZE)
      {
        for (int f = 0; f < windowSize; ++f)
        {
          edgesLine[f].clear();
          edgesPlan[f].clear();
          vLineFeatures[f].clear();
          vPlanFeatures[f].clear();
        }
      }
    }
    std::cout << "estimate iter: " << iterOpt << std::endl;
  }

  void pose_estimator::run()
  {
    double time_last_lidar = -1;
    double time_curr_lidar = -1;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;
    while (true)
    {

      if (initializedFlag == NonInitialized)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      if (_lidarMsgQueue.empty())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      //  update frame
      time_curr_lidar = _lidarMsgQueue.front()->header.stamp.toSec();
      laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
      pcl::fromROSMsg(*_lidarMsgQueue.front(), *laserCloudFullRes);
      _lidarMsgQueue.pop_front();

      if (IMU_Mode > 0 && time_last_lidar > 0)
      {
        // get IMU msg int the Specified time interval
        vimuMsg.clear();
        int countFail = 0;
        while (!fetchImuMsgs(time_last_lidar, time_curr_lidar, vimuMsg))
        {
          countFail++;
          if (countFail > 100)
          {
            if (_imuMsgQueue.empty())
              std::cout << "imu queue is empty." << std::endl;
            else
              std::cout << "imu time: " << _imuMsgQueue.front()->header.stamp.toSec() << "-->" << _imuMsgQueue.back()->header.stamp.toSec() << std::endl;
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }

      LidarFrame lidarFrame;
      lidarFrame.timeStamp = time_curr_lidar;
      lidarFrame.laserCloud = laserCloudFullRes;

      boost::shared_ptr<std::list<LidarFrame>> lidar_list;

      if (!vimuMsg.empty())
      {
        if (!LidarIMUInited)
        {
          lidarFrame.imuIntegrator.PushIMUMsg(vimuMsg);
          lidarFrame.imuIntegrator.GyroIntegration(time_last_lidar);
          delta_Rl = lidarFrame.imuIntegrator.GetDeltaQ().toRotationMatrix();

          //  predict current lidar pose
          lidarFrame.P = transformLastMapped.topLeftCorner(3, 3) * delta_tl + transformLastMapped.topRightCorner(3, 1);
          Eigen::Matrix3d m3d = transformLastMapped.topLeftCorner(3, 3) * delta_Rl;
          lidarFrame.Q = m3d;

          lidar_list.reset(new std::list<LidarFrame>);
          lidar_list->push_back(lidarFrame);
        }
        else
        {
          // if get IMU msg successfully, use pre-integration to update delta lidar pose
          lidarFrame.imuIntegrator.PushIMUMsg(vimuMsg);
          lidarFrame.imuIntegrator.PreIntegration(lidarFrameList->back().timeStamp, lidarFrameList->back().bg, lidarFrameList->back().ba);

          const Eigen::Vector3d &Pwbpre = lidarFrameList->back().P;
          const Eigen::Quaterniond &Qwbpre = lidarFrameList->back().Q;
          const Eigen::Vector3d &Vwbpre = lidarFrameList->back().V;

          const Eigen::Quaterniond &dQ = lidarFrame.imuIntegrator.GetDeltaQ();
          const Eigen::Vector3d &dP = lidarFrame.imuIntegrator.GetDeltaP();
          const Eigen::Vector3d &dV = lidarFrame.imuIntegrator.GetDeltaV();
          double dt = lidarFrame.imuIntegrator.GetDeltaTime();

          lidarFrame.Q = Qwbpre * dQ;
          lidarFrame.P = Pwbpre + Vwbpre * dt + 0.5 * GravityVector * dt * dt + Qwbpre * (dP);
          lidarFrame.V = Vwbpre + GravityVector * dt + Qwbpre * (dV);
          lidarFrame.bg = lidarFrameList->back().bg;
          lidarFrame.ba = lidarFrameList->back().ba;

          Eigen::Quaterniond Qwlpre = Qwbpre;
          Eigen::Vector3d Pwlpre = Pwbpre;

          Eigen::Quaterniond Qwl = lidarFrame.Q;
          Eigen::Vector3d Pwl = lidarFrame.P;

          delta_Rl = Qwlpre.conjugate() * Qwl;
          delta_tl = Qwlpre.conjugate() * (Pwl - Pwlpre);
          // delta_Rb = dQ.toRotationMatrix();
          // delta_tb = dP;

          lidarFrameList->push_back(lidarFrame);
          lidarFrameList->pop_front();
          lidar_list = lidarFrameList;
        }
      }
      else
      {
        if (LidarIMUInited)
          break;
        else
        {
          //  predict pose use constant velocity
          lidarFrame.P = transformLastMapped.topLeftCorner(3, 3) * delta_tl + transformLastMapped.topRightCorner(3, 1);
          Eigen::Matrix3d m3d = transformLastMapped.topLeftCorner(3, 3) * delta_Rl;
          lidarFrame.Q = m3d;

          lidar_list.reset(new std::list<LidarFrame>);
          lidar_list->push_back(lidarFrame);
        }
      }

      RemoveLidarDistortion(laserCloudFullRes, delta_Rl, delta_tl);
      //  publish cloud after remove distort
      sensor_msgs::PointCloud2 laserCloudMsg;
      pcl::toROSMsg(*lidarFrame.laserCloud, laserCloudMsg);
      laserCloudMsg.header.frame_id = "/base_link";
      laserCloudMsg.header.stamp.fromSec(lidarFrame.timeStamp);
      pubMappedPoints_.publish(laserCloudMsg);

      if (initializedFlag == Initializing)
      {
        if (ICPScanMatchGlobal(*lidar_list))
        {
          initializedFlag = Initialized;
          std::cout << ANSI_COLOR_GREEN << "icp scan match successful ..." << ANSI_COLOR_RESET << std::endl;
        }

        transformLastMapped.topLeftCorner(3, 3) = lidar_list->front().Q.toRotationMatrix();
        transformLastMapped.topRightCorner(3, 1) = lidar_list->front().P;

        pubOdometry(transformLastMapped, lidar_list->front().timeStamp);
      }
      else if (initializedFlag == Initialized)
      {
        TicToc tc;
        Eigen::Matrix4d transformAftMapped = Eigen::Matrix4d::Identity();
        double t1, t2, t3;
        //  TODO: 增加局部地图
        int laserCloudCornerFromLocalNum = laserCloudCornerFromLocal->points.size();
        int laserCloudSurfFromLocalNum = laserCloudSurfFromLocal->points.size();
        if ((kdtree_surf_map && kdtree_corner_map) ||
            (laserCloudCornerFromLocalNum > 0 && laserCloudSurfFromLocalNum > 100))
        {
          tc.tic();
          Estimate(*lidar_list, GravityVector);
          t1 = tc.toc();
          tc.tic();

          transformAftMapped = Eigen::Matrix4d::Identity();
          transformAftMapped.topLeftCorner(3, 3) = lidar_list->front().Q.toRotationMatrix();
          transformAftMapped.topRightCorner(3, 1) = lidar_list->front().P;
          pubOdometry(transformAftMapped, lidar_list->front().timeStamp);

          Eigen::Matrix3d Rwlpre = transformLastMapped.topLeftCorner(3, 3);
          Eigen::Vector3d Pwlpre = transformLastMapped.topRightCorner(3, 1);
          delta_Rl = Rwlpre.transpose() * transformAftMapped.topLeftCorner(3, 3);
          delta_tl = Rwlpre.transpose() * (transformAftMapped.topRightCorner(3, 1) - Pwlpre);
          transformLastMapped = transformAftMapped;
          t2 = tc.toc();
        }
        tc.tic();
        laserCloudCornerFromLocal->clear();
        laserCloudSurfFromLocal->clear();
        if (use_lio)
          MapIncrementLocal(lidar_list->front());
        t3 = tc.toc();
        std::cout << "LIO takes: " << t1 << ",pubodom:" << t2 << ",mapincrement:" << t3 << std::endl;
#define SAVE_TRAJ

#ifdef SAVE_TRAJ
        if (1)
        {
          static std::fstream fout(rootDir + "odom_trajectory.txt", std::ios::out);
          static std::ostringstream stamp;
          stamp.str("");
          if (fout.is_open())
          {
            // std::string tstamp = to_string(ros::Time().fromSec(laser_odometry->time));
            std::string tstamp = std::to_string(lidar_list->front().timeStamp);
            saveTrajectoryTUMformat(fout, tstamp, lidar_list->front().P(0), lidar_list->front().P(1), lidar_list->front().P(2),
                                    lidar_list->front().Q.x(), lidar_list->front().Q.y(), lidar_list->front().Q.z(), lidar_list->front().Q.w());
          }
        }
#endif

        // if tightly coupled IMU message, start IMU initialization
        if (IMU_Mode > 1 && !LidarIMUInited)
        {
          // update lidar frame pose
          lidarFrame.P = transformAftMapped.topRightCorner(3, 1);
          Eigen::Matrix3d m3d = transformAftMapped.topLeftCorner(3, 3);
          lidarFrame.Q = m3d;

          // static int pushCount = 0;
          std::cout << "lidarframelist: " << lidarFrameList->size() << std::endl;
          if (pushCount == 0)
          {
            lidarFrameList->push_back(lidarFrame);
            lidarFrameList->back().imuIntegrator.Reset();
            if (lidarFrameList->size() > WINDOWSIZE)
              lidarFrameList->pop_front();
          }
          else
          {
            lidarFrameList->back().laserCloud = lidarFrame.laserCloud;
            lidarFrameList->back().imuIntegrator.PushIMUMsg(vimuMsg);
            lidarFrameList->back().timeStamp = lidarFrame.timeStamp;
            lidarFrameList->back().P = lidarFrame.P;
            lidarFrameList->back().Q = lidarFrame.Q;
          }
          std::cout << "lidarframelist: " << lidarFrameList->size() << std::endl;

          pushCount++;
          if (pushCount >= 3)
          {
            pushCount = 0;
            if (lidarFrameList->size() > 1)
            {
              auto iterRight = std::prev(lidarFrameList->end());
              auto iterLeft = std::prev(std::prev(lidarFrameList->end()));
              iterRight->imuIntegrator.PreIntegration(iterLeft->timeStamp, iterLeft->bg, iterLeft->ba);
            }

            if (lidarFrameList->size() == int(WINDOWSIZE / 1.5))
            {
              startTime = lidarFrameList->back().timeStamp;
            }

            if (!LidarIMUInited && lidarFrameList->size() == WINDOWSIZE && lidarFrameList->front().timeStamp >= startTime)
            {
              std::cout << "**************Start MAP Initialization!!!******************" << std::endl;
              if (TryMAPInitialization())
              {
                LidarIMUInited = true;
                pushCount = 0;
                startTime = 0;
              }
              std::cout << "**************Finish MAP Initialization!!!******************" << std::endl;
            }
          }
        }
      }

      time_last_lidar = time_curr_lidar; //  update time
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  bool pose_estimator::TryMAPInitialization()
  {

    Eigen::Vector3d average_acc = -lidarFrameList->begin()->imuIntegrator.GetAverageAcc();
    double info_g = std::fabs(9.805 - average_acc.norm());
    average_acc = average_acc * 9.805 / average_acc.norm();

    // calculate the initial gravity direction
    double para_quat[4];
    para_quat[0] = 1;
    para_quat[1] = 0;
    para_quat[2] = 0;
    para_quat[3] = 0;

    ceres::LocalParameterization *quatParam = new ceres::QuaternionParameterization();
    ceres::Problem problem_quat;

    problem_quat.AddParameterBlock(para_quat, 4, quatParam);

    problem_quat.AddResidualBlock(Cost_Initial_G::Create(average_acc),
                                  nullptr,
                                  para_quat);

    ceres::Solver::Options options_quat;
    ceres::Solver::Summary summary_quat;
    ceres::Solve(options_quat, &problem_quat, &summary_quat);

    Eigen::Quaterniond q_wg(para_quat[0], para_quat[1], para_quat[2], para_quat[3]);

    // build prior factor of LIO initialization
    Eigen::Vector3d prior_r = Eigen::Vector3d::Zero();
    Eigen::Vector3d prior_ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d prior_bg = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> prior_v;
    int v_size = lidarFrameList->size();
    for (int i = 0; i < v_size; i++)
    {
      prior_v.push_back(Eigen::Vector3d::Zero());
    }
    Sophus::SO3d SO3_R_wg(q_wg.toRotationMatrix());
    prior_r = SO3_R_wg.log();

    for (int i = 1; i < v_size; i++)
    {
      auto iter = lidarFrameList->begin();
      auto iter_next = lidarFrameList->begin();
      std::advance(iter, i - 1);
      std::advance(iter_next, i);

      Eigen::Vector3d velo_imu = (iter_next->P - iter->P) / (iter_next->timeStamp - iter->timeStamp);
      prior_v[i] = velo_imu;
    }
    prior_v[0] = prior_v[1];

    double para_v[v_size][3];
    double para_r[3];
    double para_ba[3];
    double para_bg[3];

    for (int i = 0; i < 3; i++)
    {
      para_r[i] = 0;
      para_ba[i] = 0;
      para_bg[i] = 0;
    }

    for (int i = 0; i < v_size; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        para_v[i][j] = prior_v[i][j];
      }
    }

    Eigen::Matrix<double, 3, 3> sqrt_information_r = 2000.0 * Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> sqrt_information_ba = 1000.0 * Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> sqrt_information_bg = 4000.0 * Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> sqrt_information_v = 4000.0 * Eigen::Matrix<double, 3, 3>::Identity();

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(para_r, 3);
    problem.AddParameterBlock(para_ba, 3);
    problem.AddParameterBlock(para_bg, 3);
    for (int i = 0; i < v_size; i++)
    {
      problem.AddParameterBlock(para_v[i], 3);
    }

    // add CostFunction
    problem.AddResidualBlock(Cost_Initialization_Prior_R::Create(prior_r, sqrt_information_r),
                             nullptr,
                             para_r);

    problem.AddResidualBlock(Cost_Initialization_Prior_bv::Create(prior_ba, sqrt_information_ba),
                             nullptr,
                             para_ba);
    problem.AddResidualBlock(Cost_Initialization_Prior_bv::Create(prior_bg, sqrt_information_bg),
                             nullptr,
                             para_bg);

    for (int i = 0; i < v_size; i++)
    {
      problem.AddResidualBlock(Cost_Initialization_Prior_bv::Create(prior_v[i], sqrt_information_v),
                               nullptr,
                               para_v[i]);
    }

    for (int i = 1; i < v_size; i++)
    {
      auto iter = lidarFrameList->begin();
      auto iter_next = lidarFrameList->begin();
      std::advance(iter, i - 1);
      std::advance(iter_next, i);

      Eigen::Vector3d pi = iter->P;
      Sophus::SO3d SO3_Ri(iter->Q);
      Eigen::Vector3d ri = SO3_Ri.log();
      Eigen::Vector3d pj = iter_next->P;
      Sophus::SO3d SO3_Rj(iter_next->Q);
      Eigen::Vector3d rj = SO3_Rj.log();

      problem.AddResidualBlock(Cost_Initialization_IMU::Create(iter_next->imuIntegrator,
                                                               ri,
                                                               rj,
                                                               pj - pi,
                                                               Eigen::LLT<Eigen::Matrix<double, 9, 9>>(iter_next->imuIntegrator.GetCovariance().block<9, 9>(0, 0).inverse())
                                                                   .matrixL()
                                                                   .transpose()),
                               nullptr,
                               para_r,
                               para_v[i - 1],
                               para_v[i],
                               para_ba,
                               para_bg);
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_QR;
    options.num_threads = 6;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Eigen::Vector3d r_wg(para_r[0], para_r[1], para_r[2]);
    GravityVector = Sophus::SO3d::exp(r_wg) * Eigen::Vector3d(0, 0, -9.805);

    Eigen::Vector3d ba_vec(para_ba[0], para_ba[1], para_ba[2]);
    Eigen::Vector3d bg_vec(para_bg[0], para_bg[1], para_bg[2]);

    std::cout << "after opt,ba:" << ba_vec.transpose() << ",bg: " << bg_vec.transpose()
              << ",g: " << GravityVector.transpose() << std::endl;

    if (ba_vec.norm() > 0.5 || bg_vec.norm() > 0.5)
    {
      ROS_WARN("Too Large Biases! Initialization Failed!");
      return false;
    }

    for (int i = 0; i < v_size; i++)
    {
      auto iter = lidarFrameList->begin();
      std::advance(iter, i);
      iter->ba = ba_vec;
      iter->bg = bg_vec;
      Eigen::Vector3d bv_vec(para_v[i][0], para_v[i][1], para_v[i][2]);
      if ((bv_vec - prior_v[i]).norm() > 2.0)
      {
        ROS_WARN("Too Large Velocity! Initialization Failed!");
        std::cout << "delta v norm: " << (bv_vec - prior_v[i]).norm() << std::endl;
        return false;
      }
      iter->V = bv_vec;
    }

    for (size_t i = 0; i < v_size - 1; i++)
    {
      auto laser_trans_i = lidarFrameList->begin();
      auto laser_trans_j = lidarFrameList->begin();
      std::advance(laser_trans_i, i);
      std::advance(laser_trans_j, i + 1);
      laser_trans_j->imuIntegrator.PreIntegration(laser_trans_i->timeStamp, laser_trans_i->bg, laser_trans_i->ba);
    }

    // //if IMU success initialized
    WINDOWSIZE = SLIDEWINDOWSIZE;
    while (lidarFrameList->size() > WINDOWSIZE)
    {
      lidarFrameList->pop_front();
    }
    Eigen::Vector3d Pwl = lidarFrameList->back().P;
    Eigen::Quaterniond Qwl = lidarFrameList->back().Q;
    lidarFrameList->back().P = Pwl;
    lidarFrameList->back().Q = Qwl;

    std::cout << "\n=============================================\n|         Initialization Successful         |"
              << "\n=============================================\n"
              << std::endl;
    return true;
  }

  void pose_estimator::RemoveLidarDistortion(pcl::PointCloud<PointType>::Ptr &laserCloud,
                             const Eigen::Matrix3d &dRlc, const Eigen::Vector3d &dtlc)
  {
    int PointsNum = laserCloud->points.size();
    for (int i = 0; i < PointsNum; i++)
    {
      Eigen::Vector3d startP;
      float s = laserCloud->points[i].normal_x; //  time intervel
      Eigen::Quaterniond qlc = Eigen::Quaterniond(dRlc).normalized();
      Eigen::Quaterniond delta_qlc = Eigen::Quaterniond::Identity().slerp(s, qlc).normalized(); // 插值
      const Eigen::Vector3d delta_Plc = s * dtlc;
      startP = delta_qlc * Eigen::Vector3d(laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z) + delta_Plc;
      Eigen::Vector3d _po = dRlc.transpose() * (startP - dtlc);

      laserCloud->points[i].x = _po(0);
      laserCloud->points[i].y = _po(1);
      laserCloud->points[i].z = _po(2);
      laserCloud->points[i].normal_x = 1.0;

      // if (std::fabs(kf.laserCloud->points[i].normal_z - 1.0) < 1e-5)
      //   kf.corner->push_back(kf.laserCloud->points[i]);
      // if (std::fabs(kf.laserCloud->points[i].normal_z - 2.0) < 1e-5)
      //   kf.surf->push_back(kf.laserCloud->points[i]);
    }
    /*std::cout << "bef-surf: " << kf.surf->size() << ",corner: " << kf.corner->size() << std::endl;
    ds_surf_.setInputCloud(kf.surf);
    ds_surf_.filter(*kf.surf);
    ds_corner_.setInputCloud(kf.corner);
    ds_corner_.filter(*kf.corner);
    std::cout << "aft-surf: " << kf.surf->size() << ",corner: " << kf.corner->size() << std::endl;*/
  }

  bool pose_estimator::fetchImuMsgs(double startTime, double endTime, std::vector<sensor_msgs::ImuConstPtr> &vimuMsg)
  {
    double current_time = 0;
    vimuMsg.clear();
    while (true)
    {
      if (_imuMsgQueue.empty())
        break;
      if (_imuMsgQueue.back()->header.stamp.toSec() < endTime ||
          _imuMsgQueue.front()->header.stamp.toSec() >= endTime)
        break;
      sensor_msgs::ImuConstPtr tmpimumsg = _imuMsgQueue.front();
      double time = tmpimumsg->header.stamp.toSec();
      if (time <= endTime && time > startTime)
      {
        vimuMsg.push_back(tmpimumsg);
        current_time = time;
        _imuMsgQueue.pop_front();
        if (time == endTime)
          break;
      }
      else
      {
        if (time <= startTime)
        {
          _imuMsgQueue.pop_front();
        }
        else
        {
          double dt_1 = endTime - current_time;
          double dt_2 = time - endTime;
          ROS_ASSERT(dt_1 >= 0);
          ROS_ASSERT(dt_2 >= 0);
          ROS_ASSERT(dt_1 + dt_2 > 0);
          double w1 = dt_2 / (dt_1 + dt_2);
          double w2 = dt_1 / (dt_1 + dt_2);
          sensor_msgs::ImuPtr theLastIMU(new sensor_msgs::Imu);
          theLastIMU->linear_acceleration.x = w1 * vimuMsg.back()->linear_acceleration.x + w2 * tmpimumsg->linear_acceleration.x;
          theLastIMU->linear_acceleration.y = w1 * vimuMsg.back()->linear_acceleration.y + w2 * tmpimumsg->linear_acceleration.y;
          theLastIMU->linear_acceleration.z = w1 * vimuMsg.back()->linear_acceleration.z + w2 * tmpimumsg->linear_acceleration.z;
          theLastIMU->angular_velocity.x = w1 * vimuMsg.back()->angular_velocity.x + w2 * tmpimumsg->angular_velocity.x;
          theLastIMU->angular_velocity.y = w1 * vimuMsg.back()->angular_velocity.y + w2 * tmpimumsg->angular_velocity.y;
          theLastIMU->angular_velocity.z = w1 * vimuMsg.back()->angular_velocity.z + w2 * tmpimumsg->angular_velocity.z;
          theLastIMU->header.stamp.fromSec(endTime);
          vimuMsg.emplace_back(theLastIMU);
          break;
        }
      }
    }
    return !vimuMsg.empty();
  }

  void pose_estimator::processPointToLine(std::vector<ceres::CostFunction *> &edges,
                          std::vector<FeatureLine> &vLineFeatures,
                          const pcl::PointCloud<PointType>::Ptr &laserCloudCorner,
                          const pcl::PointCloud<PointType>::Ptr &laserCloudCornerGlobal,
                          const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeGlobal,
                          const pcl::PointCloud<PointType>::Ptr &laserCloudCornerLocal,
                          const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeLocal,
                          const Eigen::Matrix4d &exTlb,
                          const Eigen::Matrix4d &m4d)
  {

    Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
    Tbl.topLeftCorner(3, 3) = exTlb.topLeftCorner(3, 3).transpose();
    Tbl.topRightCorner(3, 1) = -1.0 * Tbl.topLeftCorner(3, 3) * exTlb.topRightCorner(3, 1);
    if (!vLineFeatures.empty())
    {
      for (const auto &l : vLineFeatures)
      {
        auto *e = Cost_NavState_IMU_Line::Create(l.pointOri,
                                                 l.lineP1,
                                                 l.lineP2,
                                                 Tbl,
                                                 Eigen::Matrix<double, 1, 1>(1 / IMUIntegrator::lidar_m));
        edges.push_back(e);
      }
      return;
    }
    PointType _pointOri, _pointSel, _coeff;
    std::vector<int> _pointSearchInd;
    std::vector<float> _pointSearchSqDis;
    std::vector<int> _pointSearchInd2;
    std::vector<float> _pointSearchSqDis2;

    Eigen::Matrix<double, 3, 3> _matA1;
    _matA1.setZero();

    int laserCloudCornerStackNum = laserCloudCorner->points.size();
    pcl::PointCloud<PointType>::Ptr kd_pointcloud(new pcl::PointCloud<PointType>);
    int debug_num1 = 0;
    int debug_num2 = 0;
    int debug_num12 = 0;
    int debug_num22 = 0;
    for (int i = 0; i < laserCloudCornerStackNum; i++)
    {
      _pointOri = laserCloudCorner->points[i];
      MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);

      //  for global
      if (laserCloudCornerGlobal->points.size() > 100)
      {
        kdtreeGlobal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
        if (_pointSearchSqDis2[4] < thres_dist)
        {

          debug_num2++;
          float cx = 0;
          float cy = 0;
          float cz = 0;
          for (int j = 0; j < 5; j++)
          {
            cx += laserCloudCornerGlobal->points[_pointSearchInd2[j]].x;
            cy += laserCloudCornerGlobal->points[_pointSearchInd2[j]].y;
            cz += laserCloudCornerGlobal->points[_pointSearchInd2[j]].z;
          }
          cx /= 5;
          cy /= 5;
          cz /= 5;

          float a11 = 0;
          float a12 = 0;
          float a13 = 0;
          float a22 = 0;
          float a23 = 0;
          float a33 = 0;
          for (int j = 0; j < 5; j++)
          {
            float ax = laserCloudCornerGlobal->points[_pointSearchInd2[j]].x - cx;
            float ay = laserCloudCornerGlobal->points[_pointSearchInd2[j]].y - cy;
            float az = laserCloudCornerGlobal->points[_pointSearchInd2[j]].z - cz;

            a11 += ax * ax;
            a12 += ax * ay;
            a13 += ax * az;
            a22 += ay * ay;
            a23 += ay * az;
            a33 += az * az;
          }
          a11 /= 5;
          a12 /= 5;
          a13 /= 5;
          a22 /= 5;
          a23 /= 5;
          a33 /= 5;

          _matA1(0, 0) = a11;
          _matA1(0, 1) = a12;
          _matA1(0, 2) = a13;
          _matA1(1, 0) = a12;
          _matA1(1, 1) = a22;
          _matA1(1, 2) = a23;
          _matA1(2, 0) = a13;
          _matA1(2, 1) = a23;
          _matA1(2, 2) = a33;

          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(_matA1);
          Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);

          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
          {
            debug_num22++;
            float x1 = cx + 0.1 * unit_direction[0];
            float y1 = cy + 0.1 * unit_direction[1];
            float z1 = cz + 0.1 * unit_direction[2];
            float x2 = cx - 0.1 * unit_direction[0];
            float y2 = cy - 0.1 * unit_direction[1];
            float z2 = cz - 0.1 * unit_direction[2];

            Eigen::Vector3d tripod1(x1, y1, z1);
            Eigen::Vector3d tripod2(x2, y2, z2);
            auto *e = Cost_NavState_IMU_Line::Create(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                                     tripod1,
                                                     tripod2,
                                                     Tbl,
                                                     Eigen::Matrix<double, 1, 1>(1 / IMUIntegrator::lidar_m));
            edges.push_back(e);
            vLineFeatures.emplace_back(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                       tripod1,
                                       tripod2);
            vLineFeatures.back().ComputeError(m4d);
          }
        }
      }

      if (laserCloudCornerLocal->points.size() > 20)
      {
        kdtreeLocal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
        if (_pointSearchSqDis2[4] < thres_dist)
        {

          debug_num2++;
          float cx = 0;
          float cy = 0;
          float cz = 0;
          for (int j = 0; j < 5; j++)
          {
            cx += laserCloudCornerLocal->points[_pointSearchInd2[j]].x;
            cy += laserCloudCornerLocal->points[_pointSearchInd2[j]].y;
            cz += laserCloudCornerLocal->points[_pointSearchInd2[j]].z;
          }
          cx /= 5;
          cy /= 5;
          cz /= 5;

          float a11 = 0;
          float a12 = 0;
          float a13 = 0;
          float a22 = 0;
          float a23 = 0;
          float a33 = 0;
          for (int j = 0; j < 5; j++)
          {
            float ax = laserCloudCornerLocal->points[_pointSearchInd2[j]].x - cx;
            float ay = laserCloudCornerLocal->points[_pointSearchInd2[j]].y - cy;
            float az = laserCloudCornerLocal->points[_pointSearchInd2[j]].z - cz;

            a11 += ax * ax;
            a12 += ax * ay;
            a13 += ax * az;
            a22 += ay * ay;
            a23 += ay * az;
            a33 += az * az;
          }
          a11 /= 5;
          a12 /= 5;
          a13 /= 5;
          a22 /= 5;
          a23 /= 5;
          a33 /= 5;

          _matA1(0, 0) = a11;
          _matA1(0, 1) = a12;
          _matA1(0, 2) = a13;
          _matA1(1, 0) = a12;
          _matA1(1, 1) = a22;
          _matA1(1, 2) = a23;
          _matA1(2, 0) = a13;
          _matA1(2, 1) = a23;
          _matA1(2, 2) = a33;

          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(_matA1);
          Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);

          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
          {
            debug_num22++;
            float x1 = cx + 0.1 * unit_direction[0];
            float y1 = cy + 0.1 * unit_direction[1];
            float z1 = cz + 0.1 * unit_direction[2];
            float x2 = cx - 0.1 * unit_direction[0];
            float y2 = cy - 0.1 * unit_direction[1];
            float z2 = cz - 0.1 * unit_direction[2];

            Eigen::Vector3d tripod1(x1, y1, z1);
            Eigen::Vector3d tripod2(x2, y2, z2);
            auto *e = Cost_NavState_IMU_Line::Create(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                                     tripod1,
                                                     tripod2,
                                                     Tbl,
                                                     Eigen::Matrix<double, 1, 1>(1 / IMUIntegrator::lidar_m));
            edges.push_back(e);
            vLineFeatures.emplace_back(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                       tripod1,
                                       tripod2);
            vLineFeatures.back().ComputeError(m4d);
          }
        }
      }
    }
  }

  void pose_estimator::processPointToPlanVec(std::vector<ceres::CostFunction *> &edges,
                             std::vector<FeaturePlanVec> &vPlanFeatures,
                             const pcl::PointCloud<PointType>::Ptr &laserCloudSurf,
                             const pcl::PointCloud<PointType>::Ptr &laserCloudSurfGlobal,
                             const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeGlobal,
                             const pcl::PointCloud<PointType>::Ptr &laserCloudSurfLocal,
                             const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeLocal,
                             const Eigen::Matrix4d &exTlb,
                             const Eigen::Matrix4d &m4d)
  {
    Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
    Tbl.topLeftCorner(3, 3) = exTlb.topLeftCorner(3, 3).transpose();
    Tbl.topRightCorner(3, 1) = -1.0 * Tbl.topLeftCorner(3, 3) * exTlb.topRightCorner(3, 1);
    if (!vPlanFeatures.empty())
    {
      for (const auto &p : vPlanFeatures)
      {
        auto *e = Cost_NavState_IMU_Plan_Vec::Create(p.pointOri,
                                                     p.pointProj,
                                                     Tbl,
                                                     p.sqrt_info);
        edges.push_back(e);
      }
      return;
    }
    PointType _pointOri, _pointSel, _coeff;
    std::vector<int> _pointSearchInd;
    std::vector<float> _pointSearchSqDis;
    std::vector<int> _pointSearchInd2;
    std::vector<float> _pointSearchSqDis2;

    Eigen::Matrix<double, 5, 3> _matA0;
    _matA0.setZero();
    Eigen::Matrix<double, 5, 1> _matB0;
    _matB0.setOnes();
    _matB0 *= -1;
    Eigen::Matrix<double, 3, 1> _matX0;
    _matX0.setZero();
    int laserCloudSurfStackNum = laserCloudSurf->points.size();

    int debug_num1 = 0;
    int debug_num2 = 0;
    int debug_num12 = 0;
    int debug_num22 = 0;
    for (int i = 0; i < laserCloudSurfStackNum; i++)
    {
      _pointOri = laserCloudSurf->points[i];
      MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);
      //  for global
      if (laserCloudSurfGlobal->points.size() > 200)
      {
        kdtreeGlobal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
        if (_pointSearchSqDis2[4] < thres_dist)
        {
          debug_num2++;
          for (int j = 0; j < 5; j++)
          {
            _matA0(j, 0) = laserCloudSurfGlobal->points[_pointSearchInd2[j]].x;
            _matA0(j, 1) = laserCloudSurfGlobal->points[_pointSearchInd2[j]].y;
            _matA0(j, 2) = laserCloudSurfGlobal->points[_pointSearchInd2[j]].z;
          }
          _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

          float pa = _matX0(0, 0);
          float pb = _matX0(1, 0);
          float pc = _matX0(2, 0);
          float pd = 1;

          float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps;
          pb /= ps;
          pc /= ps;
          pd /= ps;

          bool planeValid = true;
          for (int j = 0; j < 5; j++)
          {
            if (std::fabs(pa * laserCloudSurfGlobal->points[_pointSearchInd2[j]].x +
                          pb * laserCloudSurfGlobal->points[_pointSearchInd2[j]].y +
                          pc * laserCloudSurfGlobal->points[_pointSearchInd2[j]].z + pd) > 0.2)
            {
              planeValid = false;
              break;
            }
          }

          if (planeValid)
          {
            debug_num22++;
            double dist = pa * _pointSel.x +
                          pb * _pointSel.y +
                          pc * _pointSel.z + pd;
            Eigen::Vector3d omega(pa, pb, pc);
            Eigen::Vector3d point_proj = Eigen::Vector3d(_pointSel.x, _pointSel.y, _pointSel.z) - (dist * omega);
            Eigen::Vector3d e1(1, 0, 0);
            Eigen::Matrix3d J = e1 * omega.transpose();
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Matrix3d R_svd = svd.matrixV() * svd.matrixU().transpose();
            Eigen::Matrix3d info = (1.0 / IMUIntegrator::lidar_m) * Eigen::Matrix3d::Identity();
            info(1, 1) *= plan_weight_tan;
            info(2, 2) *= plan_weight_tan;
            Eigen::Matrix3d sqrt_info = info * R_svd.transpose();

            auto *e = Cost_NavState_IMU_Plan_Vec::Create(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                                         point_proj,
                                                         Tbl,
                                                         sqrt_info);
            edges.push_back(e);
            vPlanFeatures.emplace_back(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                       point_proj,
                                       sqrt_info);
            vPlanFeatures.back().ComputeError(m4d);
          }
        }
      }

      if (laserCloudSurfLocal->points.size() > 20)
      {
        kdtreeLocal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
        if (_pointSearchSqDis2[4] < thres_dist)
        {
          debug_num2++;
          for (int j = 0; j < 5; j++)
          {
            _matA0(j, 0) = laserCloudSurfLocal->points[_pointSearchInd2[j]].x;
            _matA0(j, 1) = laserCloudSurfLocal->points[_pointSearchInd2[j]].y;
            _matA0(j, 2) = laserCloudSurfLocal->points[_pointSearchInd2[j]].z;
          }
          _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

          float pa = _matX0(0, 0);
          float pb = _matX0(1, 0);
          float pc = _matX0(2, 0);
          float pd = 1;

          float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps;
          pb /= ps;
          pc /= ps;
          pd /= ps;

          bool planeValid = true;
          for (int j = 0; j < 5; j++)
          {
            if (std::fabs(pa * laserCloudSurfLocal->points[_pointSearchInd2[j]].x +
                          pb * laserCloudSurfLocal->points[_pointSearchInd2[j]].y +
                          pc * laserCloudSurfLocal->points[_pointSearchInd2[j]].z + pd) > 0.2)
            {
              planeValid = false;
              break;
            }
          }

          if (planeValid)
          {
            debug_num22++;
            double dist = pa * _pointSel.x +
                          pb * _pointSel.y +
                          pc * _pointSel.z + pd;
            Eigen::Vector3d omega(pa, pb, pc);
            Eigen::Vector3d point_proj = Eigen::Vector3d(_pointSel.x, _pointSel.y, _pointSel.z) - (dist * omega);
            Eigen::Vector3d e1(1, 0, 0);
            Eigen::Matrix3d J = e1 * omega.transpose();
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Matrix3d R_svd = svd.matrixV() * svd.matrixU().transpose();
            Eigen::Matrix3d info = (1.0 / IMUIntegrator::lidar_m) * Eigen::Matrix3d::Identity();
            info(1, 1) *= plan_weight_tan;
            info(2, 2) *= plan_weight_tan;
            Eigen::Matrix3d sqrt_info = info * R_svd.transpose();

            auto *e = Cost_NavState_IMU_Plan_Vec::Create(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                                         point_proj,
                                                         Tbl,
                                                         sqrt_info);
            edges.push_back(e);
            vPlanFeatures.emplace_back(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                       point_proj,
                                       sqrt_info);
            vPlanFeatures.back().ComputeError(m4d);
          }
        }
      }
    }
  }

  void pose_estimator::MapIncrementLocal(LidarFrame &kframe)
  {
    int laserCloudCornerStackNum = kframe.corner->points.size();
    int laserCloudSurfStackNum = kframe.surf->points.size();
    Eigen::Matrix4d pose_in_map = Eigen::Matrix4d::Identity();
    pose_in_map.topLeftCorner(3, 3) = kframe.Q.toRotationMatrix();
    pose_in_map.topRightCorner(3, 1) = kframe.P;
    PointType pointSel;
    PointType pointSel2;
    size_t Id = localMapID % localMapWindowSize;
    localCornerMap[Id]->clear();
    localSurfMap[Id]->clear();
    for (int i = 0; i < laserCloudCornerStackNum; i++)
    {
      MAP_MANAGER::pointAssociateToMap(&kframe.corner->points[i], &pointSel, pose_in_map);
      localCornerMap[Id]->push_back(pointSel);
    }
    for (int i = 0; i < laserCloudSurfStackNum; i++)
    {
      MAP_MANAGER::pointAssociateToMap(&kframe.surf->points[i], &pointSel2, pose_in_map);
      localSurfMap[Id]->push_back(pointSel2);
    }

    for (int i = 0; i < localMapWindowSize; i++)
    {
      *laserCloudCornerFromLocal += *localCornerMap[i];
      *laserCloudSurfFromLocal += *localSurfMap[i];
    }
    pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>());
    ds_corner_.setInputCloud(laserCloudCornerFromLocal);
    ds_corner_.filter(*temp);
    laserCloudCornerFromLocal = temp;
    pcl::PointCloud<PointType>::Ptr temp2(new pcl::PointCloud<PointType>());
    ds_surf_.setInputCloud(laserCloudSurfFromLocal);
    ds_surf_.filter(*temp2);
    laserCloudSurfFromLocal = temp2;

    localMapID++;
  }

  bool pose_estimator::ICPScanMatchGlobal(std::list<LidarFrame> &kframeList)
  {
    if (kframeList.size() != 1)
      std::cout << "may error,only process one lidar frame" << std::endl;

    auto &kframe = kframeList.front();
    pcl::PointCloud<PointType>::Ptr surf(new pcl::PointCloud<PointType>());
    for (const auto &p : kframe.laserCloud->points)
    {
      if (std::fabs(p.normal_z - 2.0) < 1e-5)
        surf->push_back(p);
    }
    ds_surf_.setInputCloud(surf);
    ds_surf_.filter(*surf);

    TicToc tc;
    tc.tic();
    pcl::PointCloud<PointType>::Ptr cloud_icp(new pcl::PointCloud<PointType>());
    // *cloud_icp += *TransformPointCloud(corner, &initpose);
    *cloud_icp += *TransformPointCloud(surf, &initpose);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(50);
    icp.setMaximumIterations(30);
    icp.setTransformationEpsilon(1e-4);
    icp.setEuclideanFitnessEpsilon(1e-4);
    icp.setRANSACIterations(0);

    icp.setInputSource(cloud_icp);
    icp.setInputTarget(surround_surf);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > 0.4)
    {
      std::cout << ANSI_COLOR_RED << "initial loc failed...,score: " << icp.getFitnessScore() << ANSI_COLOR_RESET << std::endl;
      return false;
    }
    Eigen::Affine3f correct_transform;
    correct_transform = icp.getFinalTransformation();
    Eigen::Matrix4d curr_pose = toMatrix(initpose);

    Eigen::Matrix4d pose = correct_transform.matrix().cast<double>() * curr_pose;

    kframe.Q = pose.block<3, 3>(0, 0);
    kframe.P = pose.topRightCorner(3, 1); //  update pose here

    double tt = tc.toc();
    std::cout << "icp takes: " << tt << "ms" << std::endl;
    pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>());

    pcl::transformPointCloud(*surf, *output, pose);
    sensor_msgs::PointCloud2 msg_target;
    // pcl::toROSMsg(*cloud_icp, msg_target);
    pcl::toROSMsg(*output, msg_target);
    msg_target.header.stamp = ros::Time::now();
    msg_target.header.frame_id = "world";
    pub_surf_.publish(msg_target);

    return true;
  }

  Eigen::Matrix4d pose_estimator::toMatrix(PointTypePose &p)
  {
    Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(p.roll, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond rotation = yawAngle * pitchAngle * rollAngle;
    odom.block(0, 0, 3, 3) = rotation.toRotationMatrix();
    odom(0, 3) = p.x, odom(1, 3) = p.y, odom(2, 3) = p.z;
    return odom;
  }

  bool pose_estimator::extractSurroundKeyFrames(const PointType &p)
  {
    TicToc tc;
    tc.tic();
    std::cout << "-----extract surround keyframes ------ " << std::endl;
    std::vector<int> point_search_idx_;
    std::vector<float> point_search_dist_;
    double surround_search_radius_ = 5.0;
    kdtree_keyposes_3d_->radiusSearch(p, surround_search_radius_, point_search_idx_, point_search_dist_, 0);
    surround_surf->clear();
    surround_corner->clear();
    for (int i = 0; i < point_search_idx_.size(); ++i)
    {
      *surround_surf += *map.surf_keyframes_[i];
      *surround_corner += *map.corner_keyframes_[i];
    }
    ds_corner_.setInputCloud(surround_corner);
    ds_corner_.filter(*surround_corner);
    ds_surf_.setInputCloud(surround_surf);
    ds_surf_.filter(*surround_surf);
    double tt = tc.toc();
    std::cout << __FUNCTION__ << ",takes: " << tt << "ms" << std::endl;
  }

  bool pose_estimator::loadmap()
  {
    std::cout << ANSI_COLOR_YELLOW << "file dir: " << filename << ANSI_COLOR_RESET << std::endl;
    pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());

    std::string fn_poses_ = filename + "/trajectory.pcd";
    std::string fn_corner_ = filename + "/CornerMap.pcd";
    std::string fn_surf_ = filename + "/SurfMap.pcd";
    std::string fn_global_ = filename + "/GlobalMap.pcd";

    if (pcl::io::loadPCDFile(fn_poses_, *map.cloudKeyPoses3D_) == -1 ||
        pcl::io::loadPCDFile(fn_corner_, *globalCornerCloud) == -1 ||
        pcl::io::loadPCDFile(fn_surf_, *globalSurfCloud) == -1 ||
        pcl::io::loadPCDFile(fn_global_, *map.globalMapCloud_))
    {
      std::cout << ANSI_COLOR_RED << "couldn't load pcd file" << ANSI_COLOR_RESET << std::endl;
      return false;
    }

    map.corner_keyframes_.resize(map.cloudKeyPoses3D_->points.size());
    map.surf_keyframes_.resize(map.cloudKeyPoses3D_->points.size());
    for (int i = 0; i < map.cloudKeyPoses3D_->points.size(); ++i)
    {
      map.corner_keyframes_[i] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
      map.surf_keyframes_[i] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }

    for (int i = 0; i < globalCornerCloud->points.size(); ++i)
    {
      const auto &p = globalCornerCloud->points[i];
      map.corner_keyframes_[int(p.intensity)]->points.push_back(p);
    }
    for (int i = 0; i < globalSurfCloud->points.size(); ++i)
    {
      const auto &p = globalSurfCloud->points[i];
      map.surf_keyframes_[int(p.intensity)]->points.push_back(p);
    }
    ds_corner_.setInputCloud(globalCornerCloud);
    ds_corner_.filter(*map.globalCornerMapCloud_);
    std::cout << globalCornerCloud->size() << ", " << map.globalCornerMapCloud_->size() << std::endl;
    ds_surf_.setInputCloud(globalSurfCloud);
    ds_surf_.filter(*map.globalSurfMapCloud_);
    return true;
  }

  void pose_estimator::saveTrajectoryTUMformat(std::fstream &fout, std::string &stamp, Eigen::Vector3d &xyz, Eigen::Quaterniond &xyzw)
  {
    fout << stamp << " " << xyz[0] << " " << xyz[1] << " " << xyz[2] << " " << xyzw.x() << " " << xyzw.y() << " " << xyzw.z() << " " << xyzw.w() << std::endl;
  }

  void pose_estimator::saveTrajectoryTUMformat(std::fstream &fout, std::string &stamp, double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    fout << stamp << " " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
  }




