#pragma once

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <set>
#include <map>
#include <algorithm>
#include <utility>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include<experimental/filesystem>
     
namespace fs=std::experimental::filesystem;
using std::ios;
using std::cout;
using std::cerr;
using std::endl;

// pose -> trajectory
struct PointXYZIRPYTRGB
{
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYTRGB,
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYTRGB PointTypePose;
typedef pcl::PointCloud<PointTypePose>::Ptr Trajectory;  // load

// point -> scan
typedef pcl::PointXYZINormal PointType;  // Normal仅为了适配fastlio2的点云类型
struct keyFrame
{
    pcl::PointCloud<PointType>::Ptr all_cloud;  // origianl pointcloud
    std::vector<std::pair<int, std::vector<int>>> object_cloud; // TODO: segmented object <object_id, ptIdx>
    Eigen::MatrixXd scv_od;  // T-GRS paper is a variant of scan context
};

// edge
struct Edge {
    int from_idx;
    int to_idx;
    gtsam::Pose3 relative;
};

// node
struct Node {
    int idx;
    gtsam::Pose3 initial;
};

using SessionNodes = std::multimap<int, Node>; // from_idx, Edge
using SessionEdges = std::multimap<int, Edge>; // from_idx, Edge


// g2o的图存储形式
struct G2oLineInfo 
{
    std::string type;

    int prev_idx = -1; // for vertex, this member is null
    int curr_idx;

    std::vector<double> trans;
    std::vector<double> quat;

    inline static const std::string kVertexTypeName = "VERTEX_SE3:QUAT";
    inline static const std::string kEdgeTypeName = "EDGE_SE3:QUAT";
}; // G2oLine

struct SphericalPoint
{
    float az; // azimuth 
    float el; // elevation
    float r; // radius
};

std::vector<std::pair<double, int>> sortVecWithIdx(const std::vector<double>& arr);

namespace LTslamParam {

    const static inline int kSessionStartIdxOffset = 1000000; // int max 2147483647 so ok.

    int genGlobalNodeIdx (const int& _session_idx, const int& _node_offset);
    int genAnchorNodeIdx (const int& _session_idx);
} // namespace LTslamParam


pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);

float rad2deg(float radians); 
float deg2rad(float degrees);
void fsmkdir(std::string _path);

void readBin(std::string _bin_path, pcl::PointCloud<PointType>::Ptr _pcd_ptr);

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame);

std::vector<double> splitPoseLine(std::string _str_line, char _delimiter);

G2oLineInfo splitG2oFileLine(std::string _str_line);

SphericalPoint cart2sph(const PointType & _cp);

std::pair<int, int> resetRimgSize(const std::pair<float, float> _fov, const float _resize_ratio);

template<typename T>
cv::Mat convertColorMappedImg (const cv::Mat &_src, std::pair<T, T> _caxis)
{
  T min_color_val = _caxis.first;
  T max_color_val = _caxis.second;

  cv::Mat image_dst;
  image_dst = 255 * (_src - min_color_val) / (max_color_val - min_color_val);
  image_dst.convertTo(image_dst, CV_8UC1);
  
  cv::applyColorMap(image_dst, image_dst, cv::COLORMAP_JET);

  return image_dst;
}

std::set<int> convertIntVecToSet(const std::vector<int> & v);

template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N-1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

sensor_msgs::ImagePtr cvmat2msg(const cv::Mat &_img);

void pubRangeImg(cv::Mat& _rimg, sensor_msgs::ImagePtr& _msg, image_transport::Publisher& _publiser, std::pair<float, float> _caxis);
void publishPointcloud2FromPCLptr(const ros::Publisher& _scan_publisher, const pcl::PointCloud<PointType>::Ptr _scan);
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame);

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);

float pointDistance(PointType p);
float pointDistance(PointType p1, PointType p2);

bool isTwoStringSame(std::string _str1, std::string _str2);

void collect_digits(std::vector<int>& digits, int num);

void writePose3ToStream(std::fstream& _stream, gtsam::Pose3 _pose);

std::vector<int> linspace(int a, int b, int N);

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter);
Eigen::MatrixXd readSCD(std::string fileToOpen);

float poseDistance(const gtsam::Pose3& p1, const gtsam::Pose3& p2);