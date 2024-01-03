#include "fpfh_teaser.hpp"

fpfh_teaser::fpfh_teaser(const pcl::PointCloud<PointType>::Ptr& source, const pcl::PointCloud<PointType>::Ptr& target){
    allocateMemory();

    ROS_INFO("fpfh_teaser initialized ...");
    std::cout << " NORMAL_ESTIMATION_RADIUS " << NORMAL_ESTIMATION_RADIUS << "\n"
                       << " MATCH_DISTANCE " << MATCH_DISTANCE << "\n"
                       << " FPFH_SEARCH_RADIUS " << FPFH_SEARCH_RADIUS << std::endl;
    
    downSizeFilterSurf.setLeafSize(0.1, 0.1, 0.1);
    downSizeFilterSurf.setInputCloud(source);
    downSizeFilterSurf.filter(*source);
    downSizeFilterSurf.setInputCloud(target);
    downSizeFilterSurf.filter(*target);

    set_source(source);
    set_target(target);
    std::cout << " source cloud size: " << source->points.size() << "\n"
                       << " target cloud size: " << target->points.size() << std::endl;
}

void fpfh_teaser::allocateMemory(){
    tree.reset(new pcl::KdTreeFLANN<PointType>());
    cloud_source.reset(new pcl::PointCloud<PointType>());
    cloud_target.reset(new pcl::PointCloud<PointType>());
    cloud_source_transformed.reset(new pcl::PointCloud<PointType>());
}

void fpfh_teaser::set_source(const pcl::PointCloud<PointType>::Ptr& cloud){
    cloud_source->clear();
    *cloud_source += *cloud;
}

void fpfh_teaser::set_target(const pcl::PointCloud<PointType>::Ptr& cloud){
    cloud_target->clear();
    *cloud_target += *cloud;
    tree->setInputCloud(cloud_target);
}

double fpfh_teaser::calc_matching_error(const pcl::PointCloud<PointType>::Ptr& cloud, const Eigen::Matrix4f& transformation){
    int num_inliers = 0;
    double matching_error = 0.0;

    pcl::PointCloud<PointType> transformed;
    pcl::transformPointCloud(*cloud, transformed, transformation);

    std::vector<int> indices;
    std::vector<float> sq_dists;
    for (int i = 0; i < transformed.size(); i++) {
        tree->nearestKSearch(transformed[i], 1, indices, sq_dists);
        if (sq_dists[0] < MATCH_DISTANCE) {
            num_inliers++;
            matching_error += sq_dists[0];
        }
    }

    return num_inliers ? matching_error / num_inliers : std::numeric_limits<double>::max();
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_teaser::extract_fpfh(const pcl::PointCloud<PointType>::Ptr& cloud){
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*cloud, *tmp);
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> nest;
    nest.setRadiusSearch(NORMAL_ESTIMATION_RADIUS);
    nest.setInputCloud(tmp);
    nest.compute(*normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fest;
    fest.setInputCloud(tmp);
    fest.setRadiusSearch(FPFH_SEARCH_RADIUS);
    fest.setInputNormals(normals);
    fest.compute(*features);

    std::cout << "fpfhs are extracted ... " << std::endl;

    return features;
}

std::pair<double, Eigen::Isometry3f> fpfh_teaser::match(){
    auto fpfh_source = extract_fpfh(cloud_source);
    auto fpfh_target = extract_fpfh(cloud_target);
    std::cout << "source :" << " fpfh: " << fpfh_source->size() << " cloud: " << cloud_source->size() << std::endl;
    std::cout << "target :" << " fpfh: " << fpfh_target->size() << " cloud: " << cloud_target->size() << std::endl;

    teaser::PointCloud target_teaser, source_teaser;
    teaser::FPFHCloud target_fpfh, source_fpfh;

    // target_teaser.reserve(cloud_target->size());
    // target_fpfh.reserve(cloud_target->size());
    for(int i = 0; i < cloud_target->size(); i++){
        target_teaser.push_back({cloud_target->at(i).x, cloud_target->at(i).y, cloud_target->at(i).z});
        target_fpfh.push_back(fpfh_target->at(i));
    }

    // source_teaser.reserve(cloud_source->size());
    // source_fpfh.reserve(cloud_source->size());
    for(int i = 0; i < cloud_source->size(); i++){
        source_teaser.push_back({cloud_source->at(i).x, cloud_source->at(i).y, cloud_source->at(i).z});
        source_fpfh.push_back(fpfh_source->at(i));
    }

    ROS_INFO("Find Correspondences ... ");
    teaser::Matcher matcher;
    auto correspondences = matcher.calculateCorrespondences(
        source_teaser,
        target_teaser,
        source_fpfh,
        target_fpfh,
        USE_ABSOLUTE_SCALE,
        CROSS_CHECK,
        TUPLE_TEST,
        TUPLE_SCALE
    );

    ROS_INFO("Run Teaser++ ... ");
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = NOISE_BOUND;
    params.cbar2 = CBAR2;
    params.estimate_scaling = false;
    params.rotation_max_iterations = ROTATION_MAX_ITERATIONS;
    params.rotation_gnc_factor = ROTATION_GNC_FACTOR;
    params.rotation_cost_threshold = ROTATION_COST_THRESHOLD;
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;

    teaser::RobustRegistrationSolver solver(params);
    solver.solve(source_teaser, target_teaser, correspondences);
    auto solution = solver.getSolution();

    Eigen::Isometry3f transformation = Eigen::Isometry3f::Identity();
    transformation.linear() = solution.rotation.cast<float>();
    transformation.translation() = solution.translation.cast<float>();
    Eigen::Matrix4f transMatrix = transformation.matrix();

    double error = calc_matching_error(cloud_source, transMatrix);

    return std::make_pair(error, transformation);
}