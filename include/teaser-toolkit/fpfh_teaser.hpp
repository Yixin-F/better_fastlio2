// Teaser accepted by CVPR, it has been rewritten by @Yixin Fang

#pragma once

#include <ros/ros.h>

#include <atomic>
#include <vector>
#include <random>
#include <unordered_set>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/voxel_grid.h>

#include "teaser/matcher.h"
#include "teaser/registration.h"

// FIXME: set your params
// common
#define NORMAL_ESTIMATION_RADIUS (2.0)  // FIXME? ??
#define MATCH_DISTANCE (2.0)
#define FPFH_SEARCH_RADIUS (4.0)

// fpfh correspondence
#define USE_ABSOLUTE_SCALE (false)
#define CROSS_CHECK (true)
#define TUPLE_TEST (true)
#define TUPLE_SCALE (0.95)

// teaser registeration
#define NOISE_BOUND (0.5)
#define CBAR2 (1.0)
#define ROTATION_MAX_ITERATIONS (20)
#define ROTATION_GNC_FACTOR (1.4)
#define ROTATION_COST_THRESHOLD (0.005)

typedef pcl::PointXYZINormal PointType;

class fpfh_teaser
{
public:
    fpfh_teaser(const pcl::PointCloud<PointType>::Ptr& source, const pcl::PointCloud<PointType>::Ptr& target);
    ~fpfh_teaser() {}

    pcl::VoxelGrid<PointType> downSizeFilterSurf; 
    pcl::KdTreeFLANN<PointType>::Ptr tree;
    pcl::PointCloud<PointType>::Ptr cloud_source;
    pcl::PointCloud<PointType>::Ptr cloud_target;
    pcl::PointCloud<PointType>::Ptr cloud_source_transformed;
    
    void allocateMemory();

    void set_source(const pcl::PointCloud<PointType>::Ptr& cloud);
    void set_target(const pcl::PointCloud<PointType>::Ptr& cloud);
    double calc_matching_error(const pcl::PointCloud<PointType>::Ptr& cloud, const Eigen::Matrix4f& transformation);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr extract_fpfh(const pcl::PointCloud<PointType>::Ptr& cloud);
    std::pair<double, Eigen::Isometry3f> match();
};
