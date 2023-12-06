#pragma once

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

// ros
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

#include <geometry_msgs/Vector3.h>

// eigen
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// opencv
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/range_image/range_image.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 

// tf
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// gtsam
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

// TODO: rviz 

// std
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
#include <omp.h>
#include <mutex>
#include <math.h>
#include <csignal>
#include <unistd.h>
#include <condition_variable>

#include <experimental/filesystem> // file gcc>=8
#include <experimental/optional>
#include <Python.h>

// user
#include <fast_lio_sam/Pose6D.h>
#include "fast_lio_sam/save_map.h"
#include "fast_lio_sam/save_pose.h"
#include <so3_math.h>

// namesapce
using namespace std;
using namespace Eigen;
namespace fs=std::experimental::filesystem;

// define
#define USE_IKFOM

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)         // Gravaty const in GuangDong/China
#define DIM_STATE (18)        // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N (12)       // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN  (6.0)
#define LIDAR_SP_LEN    (2)
#define INIT_COV   (1)
#define NUM_MATCH_POINTS    (5)
#define MAX_MEAS_DIM        (10000)

#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define CONSTRAIN(v,min,max)     ((v>min)?((v<max)?v:max):min)
#define ARRAY_FROM_EIGEN(mat)    mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)  vector<decltype(mat)::Scalar> (mat.data(), mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))

typedef fast_lio_sam::Pose6D Pose6D;
typedef pcl::PointXYZINormal PointType;
// typedef pcl::PointCloud<PointType> PointCloudXYZI; // just for fast-lio2
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a,b)  Matrix<double, (a), (b)>
#define VD(a)    Matrix<double, (a), 1>
#define MF(a,b)  Matrix<float, (a), (b)>
#define VF(a)    Matrix<float, (a), 1>

M3D Eye3d(M3D::Identity());
M3F Eye3f(M3F::Identity());
V3D Zero3d(0, 0, 0);
V3F Zero3f(0, 0, 0);

// debug
bool cout_debug = false;

// structure
// trajectory
struct PointXYZIRPYTRGB{
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
typedef pcl::PointCloud<PointTypePose>::Ptr Trajectory;

// keyframe
struct KeyFrame{
    pcl::PointCloud<PointType>::Ptr all_cloud;  // origianl pointcloud
    std::vector<std::pair<int, std::vector<int>>> object_cloud; // TODO: segmented object <object_id, ptIdx>
    Eigen::MatrixXd scv_od;  // TODO: T-GRS paper is a variant of scan context
    int reloTargetIdx;
    float reloScore;
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

using SessionNodes = std::multimap<int, Node>; // from_idx, Node
using SessionEdges = std::multimap<int, Edge>; // from_idx, Edge

// g2o
struct G2oLineInfo {
    std::string type;

    int prev_idx = -1; // for vertex, this member is null
    int curr_idx;

    std::vector<double> trans;
    std::vector<double> quat;

    inline static const std::string kVertexTypeName = "VERTEX_SE3:QUAT";
    inline static const std::string kEdgeTypeName = "EDGE_SE3:QUAT";
}; 

// spherical point
struct SphericalPoint{
    float az; // azimuth 
    float el; // elevation
    float r; // radius
};

// measurement for fast-lio2: lidar and imu in a single keyframe
struct MeasureGroup     // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new pcl::PointCloud<PointType>());
    };
    double lidar_beg_time; // lidar data begin time in the MeasureGroup
    double lidar_end_time; // lidar data end time in the MeasureGroup
    pcl::PointCloud<PointType>::Ptr lidar;
    deque<sensor_msgs::Imu::ConstPtr> imu;
};

// states to be estimated in fast-lio2
struct StatesGroup
{
    StatesGroup() {
		this->rot_end = M3D::Identity();
		this->pos_end = Zero3d;
        this->vel_end = Zero3d;
        this->bias_g  = Zero3d;
        this->bias_a  = Zero3d;
        this->gravity = Zero3d;
        this->cov     = MD(DIM_STATE,DIM_STATE)::Identity() * INIT_COV;
        this->cov.block<9,9>(9,9) = MD(9,9)::Identity() * 0.00001;
	};

    StatesGroup(const StatesGroup& b) {
		this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        this->cov     = b.cov;
	};

    StatesGroup& operator=(const StatesGroup& b)
	{
        this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        this->cov     = b.cov;
        return *this;
	};

    StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
	{
        StatesGroup a;
		a.rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		a.pos_end = this->pos_end + state_add.block<3,1>(3,0);
        a.vel_end = this->vel_end + state_add.block<3,1>(6,0);
        a.bias_g  = this->bias_g  + state_add.block<3,1>(9,0);
        a.bias_a  = this->bias_a  + state_add.block<3,1>(12,0);
        a.gravity = this->gravity + state_add.block<3,1>(15,0);
        a.cov     = this->cov;
		return a;
	};

    StatesGroup& operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
	{
        this->rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		this->pos_end += state_add.block<3,1>(3,0);
        this->vel_end += state_add.block<3,1>(6,0);
        this->bias_g  += state_add.block<3,1>(9,0);
        this->bias_a  += state_add.block<3,1>(12,0);
        this->gravity += state_add.block<3,1>(15,0);
		return *this;
	};

    Matrix<double, DIM_STATE, 1> operator-(const StatesGroup& b)
	{
        Matrix<double, DIM_STATE, 1> a;
        M3D rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3,1>(0,0)  = Log(rotd);
        a.block<3,1>(3,0)  = this->pos_end - b.pos_end;
        a.block<3,1>(6,0)  = this->vel_end - b.vel_end;
        a.block<3,1>(9,0)  = this->bias_g  - b.bias_g;
        a.block<3,1>(12,0) = this->bias_a  - b.bias_a;
        a.block<3,1>(15,0) = this->gravity - b.gravity;
		return a;
	};

    void resetpose()
    {
        this->rot_end = M3D::Identity();
		this->pos_end = Zero3d;
        this->vel_end = Zero3d;
    }

	M3D rot_end;      // the estimated attitude (rotation matrix) at the end lidar point
    V3D pos_end;      // the estimated position at the end lidar point (world frame)
    V3D vel_end;      // the estimated velocity at the end lidar point (world frame)
    V3D bias_g;       // gyroscope bias
    V3D bias_a;       // accelerator bias
    V3D gravity;      // the estimated gravity acceleration
    Matrix<double, DIM_STATE, DIM_STATE>  cov;     // states covariance
};

// functions
template<typename T>
inline T rad2deg(T radians) // rad2deg
{
  return radians * 180.0 / PI_M;
}

template<typename T>
inline T deg2rad(T degrees) // deg2rad
{
  return degrees * PI_M / 180.0;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

template<typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, \
                const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{  // Pose6D.msg
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
    }
    return move(rot_kp);
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
template<typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num){
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);
    
    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

inline float calc_dist(PointType p1, PointType p2){ // 3d-distance
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

inline float calc_dist_(PointType p1, PointType p2){ // 2d-distance
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
    return d;
}

template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold){  // plane
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++){
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

// padding with zero for invalid parts in scd (SC)
inline std::string padZeros(int val, int num_digits = 6) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

// save scd for each keyframe
inline void writeSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " "){
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

// read scd
Eigen::MatrixXd readSCD(std::string fileToOpen)
{
	// ref: https://aleksandarhaber.com/eigen-matrix-library-c-tutorial-saving-and-loading-data-in-from-a-csv-file/
	using namespace Eigen;

    std::vector<double> matrixEntries;
    std::ifstream matrixDataFile(fileToOpen);
    std::string matrixRowString;
	std::string matrixEntry;

    int matrixRowNumber = 0;
    while (getline(matrixDataFile, matrixRowString)) {
        std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.
        while (getline(matrixRowStringStream, matrixEntry, ' ')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
            matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries

        matrixRowNumber++; //update the column numbers
    }
    return Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

// TODO: pubilsh any-type pointcloud
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame){ 
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

// read bin
void readBin(std::string _bin_path, pcl::PointCloud<PointType>::Ptr _pcd_ptr)
{
 	std::fstream input(_bin_path.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << _bin_path << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
  
	for (int ii=0; input.good() && !input.eof(); ii++) {
		PointType point;

		input.read((char *) &point.x, sizeof(float));
		input.read((char *) &point.y, sizeof(float));
		input.read((char *) &point.z, sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));

		_pcd_ptr->push_back(point);
	}
	input.close();
}

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

const static inline int kSessionStartIdxOffset = 1000000; // int max 2147483647 so ok.

int ungenGlobalNodeIdx (const int& _session_idx, const int& _idx_in_graph)
{
    return (_idx_in_graph - 1) / (_session_idx * kSessionStartIdxOffset);
} // ungenGlobalNodeIdx

int genGlobalNodeIdx (const int& _session_idx, const int& _node_offset)
{
    return (_session_idx * kSessionStartIdxOffset) + _node_offset + 1;
} // genGlobalNodeIdx

int genAnchorNodeIdx (const int& _session_idx)
{
    return (_session_idx * kSessionStartIdxOffset);
} // genAnchorNodeIdx

gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

PointTypePose trans2PointTypePose(float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    return thisPose6D;
}

float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

void fsmkdir(std::string _path)
{
    if(fs::is_directory(_path) && fs::exists(_path))
        fs::remove_all(_path);
    if (!fs::is_directory(_path) || !fs::exists(_path)) 
        fs::create_directories(_path); // create src folder
} //fsmkdir

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, 
                                                transformIn->roll, transformIn->pitch, transformIn->yaw);
    
    int numberOfCores = 8; // TODO: move to yaml
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
}

// // cannot use
// pcl::PointCloud<PointType>::Ptr convertPointTypePose(const Trajectory& pose_){
//     pcl::PointCloud<PointType>::Ptr convert(new pcl::PointCloud<PointType>());

//     int numberOfCores = 8; // TODO: move to yaml
//     #pragma omp parallel for num_threads(numberOfCores)
//     for(int i = 0; i < pose_->points.size(); i++){
//         PointType pt;
//         pt.x = pose_->points[i].x;
//         pt.y = pose_->points[i].y;
//         pt.z = pose_->points[i].z;
//         convert->points.emplace_back(pt);
//     }
//     return convert;
// }

// pcl::PointCloud<PointType>::Ptr convertPointXYZI(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_){
//     pcl::PointCloud<PointType>::Ptr convert(new pcl::PointCloud<PointType>());

//     int numberOfCores = 8; // TODO: move to yaml
//     #pragma omp parallel for num_threads(numberOfCores)
//     for(int i = 0; i < cloud_->points.size(); i++){
//         PointType pt;
//         pt.x = cloud_->points[i].x;
//         pt.y = cloud_->points[i].y;
//         pt.z = cloud_->points[i].z;
//         convert->points.emplace_back(pt);
//     }
//     return convert;
// }

std::vector<std::pair<double, int>> sortVecWithIdx(const std::vector<double>& arr) 
{ 
    std::vector<std::pair<double, int> > vp; 
    for (int i = 0; i < arr.size(); ++i)
        vp.push_back(std::make_pair(arr[i], i)); 
  
    std::sort(vp.begin(), vp.end(), std::greater<>()); 
    return vp;
} 

std::vector<double> splitPoseLine(std::string _str_line, char _delimiter) {
    std::vector<double> parsed;
    std::stringstream ss(_str_line);
    std::string temp;
    while (getline(ss, temp, _delimiter)) {
        parsed.push_back(std::stod(temp)); // convert string to "double"
    }
    return parsed;
}

bool isTwoStringSame(std::string _str1, std::string _str2)
{
	return !(_str1.compare(_str2));
}

void collect_digits(std::vector<int>& digits, int num) {
    if (num > 9) {
        collect_digits(digits, num / 10);
    }
    digits.push_back(num % 10);
}

// record poses
void writePose3ToStream(std::fstream& _stream, gtsam::Pose3 _pose)
{
    gtsam::Point3 t = _pose.translation();
    gtsam::Rot3 R = _pose.rotation();

    // r1 means column 1 (see https://gtsam.org/doxygen/a02759.html)
    std::string sep = " "; // separator
    _stream << R.r1().x() << sep << R.r2().x() << sep << R.r3().x() << sep << t.x() << sep 
            << R.r1().y() << sep << R.r2().y() << sep << R.r3().y() << sep << t.y() << sep
            << R.r1().z() << sep << R.r2().z() << sep << R.r3().z() << sep << t.z() << std::endl;
}

std::vector<int> linspace(int a, int b, int N) {
    int h = (b - a) / static_cast<int>(N-1);
    std::vector<int> xs(N);
    typename std::vector<int>::iterator x;
    int val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

float poseDistance(const gtsam::Pose3& p1, const gtsam::Pose3& p2)
{
    auto p1x = p1.translation().x();
    auto p1y = p1.translation().y();
    auto p1z = p1.translation().z();
    auto p2x = p2.translation().x();
    auto p2y = p2.translation().y();
    auto p2z = p2.translation().z();
    
    return sqrt((p1x-p2x)*(p1x-p2x) + (p1y-p2y)*(p1y-p2y) + (p1z-p2z)*(p1z-p2z));
}

Eigen::Quaterniond EulerToQuat(float roll_, float pitch_, float yaw_)
{
    Eigen::Quaterniond q; // 四元数q和-q是相等的
    Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
    q = yaw * pitch * roll;
    q.normalize();
    return q;
}

std::set<int> convertIntVecToSet(const std::vector<int> & v) 
{ 
    std::set<int> s; 
    for (int x : v) { 
        s.insert(x); 
    } 
    return s; 
} 

sensor_msgs::ImagePtr cvmat2msg(const cv::Mat &_img)
{
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _img).toImageMsg();
  return msg;
}

void writeVertex(const int _node_idx, const gtsam::Pose3& _initPose, std::vector<std::string>& vertices_str){
    gtsam::Point3 t = _initPose.translation();
    gtsam::Rot3 R = _initPose.rotation();

    std::string curVertexInfo {
        "VERTEX_SE3:QUAT " + std::to_string(_node_idx) + " "
         + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
        + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
        + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

    // pgVertexSaveStream << curVertexInfo << std::endl;
    vertices_str.emplace_back(curVertexInfo);
}

void writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose, std::vector<std::string>& edges_str){
    gtsam::Point3 t = _relPose.translation();
    gtsam::Rot3 R = _relPose.rotation();

    std::string curEdgeInfo {
        "EDGE_SE3:QUAT " + std::to_string(_node_idx_pair.first) + " " + std::to_string(_node_idx_pair.second) + " "
        + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
        + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
        + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

    // pgEdgeSaveStream << curEdgeInfo << std::endl;
    edges_str.emplace_back(curEdgeInfo);  
}

// read g2o
// example：VERTEX_SE3:QUAT 99 -61.332581 -9.253125 0.131973 -0.004256 -0.005810 -0.625732 0.780005
G2oLineInfo splitG2oFileLine(std::string _str_line) {

    std::stringstream ss(_str_line);

	std::vector<std::string> parsed_elms ;
    std::string elm;
	char delimiter = ' ';
    while (getline(ss, elm, delimiter)) {
        parsed_elms.push_back(elm); // convert string to "double"
    }

	G2oLineInfo parsed;
    // determine whether edge or node
	if( isTwoStringSame(parsed_elms.at(0), G2oLineInfo::kVertexTypeName) )
	{
		parsed.type = parsed_elms.at(0);
		parsed.curr_idx = std::stoi(parsed_elms.at(1));
		parsed.trans.push_back(std::stod(parsed_elms.at(2)));
		parsed.trans.push_back(std::stod(parsed_elms.at(3)));
		parsed.trans.push_back(std::stod(parsed_elms.at(4)));
		parsed.quat.push_back(std::stod(parsed_elms.at(5)));
		parsed.quat.push_back(std::stod(parsed_elms.at(6)));
		parsed.quat.push_back(std::stod(parsed_elms.at(7)));
		parsed.quat.push_back(std::stod(parsed_elms.at(8)));
	}
	if( isTwoStringSame(parsed_elms.at(0), G2oLineInfo::kEdgeTypeName) )
	{
		parsed.type = parsed_elms.at(0);
		parsed.prev_idx = std::stoi(parsed_elms.at(1));
		parsed.curr_idx = std::stoi(parsed_elms.at(2));
		parsed.trans.push_back(std::stod(parsed_elms.at(3)));
		parsed.trans.push_back(std::stod(parsed_elms.at(4)));
		parsed.trans.push_back(std::stod(parsed_elms.at(5)));
		parsed.quat.push_back(std::stod(parsed_elms.at(6)));
		parsed.quat.push_back(std::stod(parsed_elms.at(7)));
		parsed.quat.push_back(std::stod(parsed_elms.at(8)));
		parsed.quat.push_back(std::stod(parsed_elms.at(9)));
	}

	return parsed;
}

SphericalPoint cart2sph(const PointType & _cp)
{ // _cp means cartesian point

    if(cout_debug){
        cout << "Cartesian Point [x, y, z]: [" << _cp.x << ", " << _cp.y << ", " << _cp.z << endl;
    }

    SphericalPoint sph_point {
         std::atan2(_cp.y, _cp.x), 
         std::atan2(_cp.z, std::sqrt(_cp.x*_cp.x + _cp.y*_cp.y)),
         std::sqrt(_cp.x*_cp.x + _cp.y*_cp.y + _cp.z*_cp.z)
    };    
    return sph_point;
}

// multi-resolution range image
std::pair<int, int> resetRimgSize(const std::pair<float, float> _fov, const float _resize_ratio)
{
    // default is 1 deg x 1 deg 
    float alpha_vfov = _resize_ratio;    
    float alpha_hfov = _resize_ratio;    

    float V_FOV = _fov.first;
    float H_FOV = _fov.second;

    int NUM_RANGE_IMG_ROW = std::round(V_FOV*alpha_vfov);
    int NUM_RANGE_IMG_COL = std::round(H_FOV*alpha_hfov);

    std::pair<int, int> rimg {NUM_RANGE_IMG_ROW, NUM_RANGE_IMG_COL};
    return rimg;
}

void pubRangeImg(cv::Mat& _rimg, 
                sensor_msgs::ImagePtr& _msg,
                image_transport::Publisher& _publiser,
                std::pair<float, float> _caxis)
{
    cv::Mat scan_rimg_viz = convertColorMappedImg(_rimg, _caxis);
    _msg = cvmat2msg(scan_rimg_viz);
    if (_publiser.getNumSubscribers() != 0)
        _publiser.publish(_msg);    
} // pubRangeImg

#endif
