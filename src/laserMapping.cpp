#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
// gstam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

// save map
#include "fast_lio_sam/save_map.h"
#include "fast_lio_sam/save_pose.h"
// save data in kitti format
#include <sstream>
#include <fstream>
#include <iomanip>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

// kdtree建立时间;kdtree搜索时间;kdtree删除时间;
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
// T1雷达初始时间戳;s_plot为整个流程耗时;2特征点数量;3kdtree增量时间;4kdtree搜索耗时;5kdtree删除点数量;
// 6kdtree删除耗时;7kdtree初始大小;8kdtree结束大小;9平均消耗时间;10添加点数量;11点云预处理的总时间;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
// 匹配时间;求解时间;求解H矩阵时间;
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
// ikdtree获得的节点数;ikdtree结束时的节点数;添加点的数量;删除点的数量;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
// log是否开启;是否保存pcd文件;是否同步时间;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;

float res_last[100000] = {0.0};   // 残差,点到面距离平方和;
float DET_RANGE = 300.0f;         // 当前雷达系中心到各个地图边缘的距离;
const float MOV_THRESHOLD = 1.5f; // 当前雷达系中心到各个地图边缘的权重;

mutex mtx_buffer;              // 互斥锁;
condition_variable sig_buffer; // 条件变量;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, repub_topic, imu_topic; // 地图文件路径;雷达topic;imu topic;

// 残差平均值;残差和;
double res_mean_last = 0.05, total_residual = 0.0;
// 雷达时间戳;imu时间戳;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
// imu的角速度协方差;加速度协方差;角速度协方差偏置;加速度协方差偏置;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
// 滤波器的最小尺寸;地图的最小尺寸;视野角度;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
// 立方体长度;视野一半的角度;视野总角度;总距离;雷达结束时间;雷达初始时间;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
// 有效特征点数;时间log计数器;接收到的激光雷达Msg的总数;接收到的IMU的Msg的总数;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
// 迭代次数;下采样的点数;最大迭代次数;有效点数;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool point_selected_surf[100000] = {0}; // 是否为平面特征点
// 是否把雷达从buffer送到meas中;是否第一帧扫描;是否收到退出的中断;flg_EKF_inited判断EKF是否初始化完成;
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
// 是否发布激光雷达数据;是否发布稠密数据;是否发布激光雷达数据的车体数据;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>> pointSearchInd_surf;      // 每个点的索引,暂时没用到
vector<BoxPointType> cub_needrm;              // ikdtree中,地图需要移除的包围盒序列
vector<PointVector> Nearest_Points;           // 每个点的最近点序列
vector<double> extrinT(3, 0.0);               // 雷达相对于IMU的外参T
vector<double> extrinR(9, 0.0);               // 雷达相对于IMU的外参R
deque<double> time_buffer;                    // 激光雷达数据时间戳缓存队列
deque<PointCloudXYZI::Ptr> lidar_buffer;      // 记录特征提取或间隔采样后的lidar(特征)数据
deque<sensor_msgs::Imu::ConstPtr> imu_buffer; // IMU数据缓存队列

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());           // 提取地图中的特征点,ikdtree获得
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());        // 去畸变的点云,lidar系
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());        // 畸变纠正后降采样的单帧点云,lidar系
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());       // 畸变纠正后降采样的单帧点云,world系
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));       // 特征点在地图中对应点的,局部平面参数,world系
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1)); // 畸变纠正后降采样的单帧点云,imu(body)系
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1)); // 对应点法相量
PointCloudXYZI::Ptr _featsArray;                                  // ikdtree中,map需要移除的点云

pcl::VoxelGrid<PointType> downSizeFilterSurf; // 单帧内降采样使用voxel grid
pcl::VoxelGrid<PointType> downSizeFilterMap;  // 未使用

KD_TREE ikdtree; // ikdtree类

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;               // 当前的欧拉角
V3D position_last(Zero3d);   // 上一帧的位置
V3D Lidar_T_wrt_IMU(Zero3d); // T lidar to imu (imu = r * lidar + t)
M3D Lidar_R_wrt_IMU(Eye3d);  // R lidar to imu (imu = r * lidar + t)

// EKF inputs and output
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf; // 状态,噪声维度,输入
state_ikfom state_point;                         // 状态
vect3 pos_lid;                                   // world系下lidar坐标

// 输出的路径参数
nav_msgs::Path path;                      // 包含了一系列位姿
nav_msgs::Odometry odomAftMapped;         // 只包含了一个位姿
geometry_msgs::Quaternion geoQuat;        // 四元数
geometry_msgs::PoseStamped msg_body_pose; // 位姿

shared_ptr<Preprocess> p_pre(new Preprocess()); // 定义指向激光雷达数据的预处理类Preprocess的智能指针
shared_ptr<ImuProcess> p_imu(new ImuProcess()); // 定义指向IMU数据预处理类ImuProcess的智能指针

// back end
vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; // 历史所有关键帧的角点集合(降采样)
vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;   // 历史所有关键帧的平面点集合(降采样)

pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D(new pcl::PointCloud<PointType>());         // 历史关键帧位姿(位置)
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); // 历史关键帧位姿
pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());

pcl::PointCloud<PointTypePose>::Ptr fastlio_unoptimized_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); // 存储fastlio 未优化的位姿

// voxel filter paprams
float odometrySurfLeafSize;
float mappingCornerLeafSize;
float mappingSurfLeafSize;

float z_tollerance;
float rotation_tollerance;

// CPU Params
int numberOfCores = 4;
double mappingProcessInterval;

/*loop clousre*/
bool startFlag = true;
bool loopClosureEnableFlag;
float loopClosureFrequency; // 回环检测频率
int surroundingKeyframeSize;
float historyKeyframeSearchRadius;   // 回环检测radius kdtree搜索半径
float historyKeyframeSearchTimeDiff; // 帧间时间阈值
int historyKeyframeSearchNum;        // 回环时多少个keyframe拼成submap
float historyKeyframeFitnessScore;   // icp匹配阈值
bool potentialLoopFlag = false;

ros::Publisher pubHistoryKeyFrames; // 发布loop history keyframe submap
ros::Publisher pubIcpKeyFrames;
ros::Publisher pubRecentKeyFrames;
ros::Publisher pubRecentKeyFrame;
ros::Publisher pubCloudRegisteredRaw;
ros::Publisher pubLoopConstraintEdge;
ros::Publisher pubLidarPCL;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_msg;

bool aLoopIsClosed = false;
map<int, int> loopIndexContainer;                               // from new to old
vector<pair<int, int>> loopIndexQueue;                          // 回环索引队列
vector<gtsam::Pose3> loopPoseQueue;                             // 回环位姿队列
vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue; // 回环噪声队列
deque<std_msgs::Float64MultiArray> loopInfoVec;

nav_msgs::Path globalPath;

// 局部关键帧构建的map点云,对应kdtree,用于scan-to-map找相邻点
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());

// 降采样
pcl::VoxelGrid<PointType> downSizeFilterCorner;
// pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterICP;
pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

float transformTobeMapped[6]; // 当前帧的位姿(world系下),x,y,z,roll,pitch,yaw

std::mutex mtx;
std::mutex mtxLoopInfo;

// Surrounding map
float surroundingkeyframeAddingDistThreshold;  // 判断是否为关键帧的距离阈值,yaml
float surroundingkeyframeAddingAngleThreshold; // 判断是否为关键帧的角度阈值,yaml
float surroundingKeyframeDensity;
float surroundingKeyframeSearchRadius;

// gtsam
gtsam::NonlinearFactorGraph gtSAMgraph; // 实例化一个空的因子图
gtsam::Values initialEstimate;
gtsam::Values optimizedEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;
Eigen::MatrixXd poseCovariance;

ros::Publisher pubLaserCloudSurround;
ros::Publisher pubOptimizedGlobalMap; // 发布最后优化的地图

bool recontructKdTree = false;
int updateKdtreeCount = 0;        // 每100次更新一次
bool visulize_IkdtreeMap = false; // visual iktree submap

// global map visualization radius
float globalMapVisualizationSearchRadius;
float globalMapVisualizationPoseDensity;
float globalMapVisualizationLeafSize;

// saveMap
ros::ServiceServer srvSaveMap;
ros::ServiceServer srvSavePose;
bool savePCD;            // 是否保存地图
string savePCDDirectory; // 保存路径

/*------------------------------------------*/
#define Hmax 720
#define Wmax 1280
vector<double> cam_ex(16, 0.0);
vector<double> cam_in(12, 0.0);
cv::Vec3b image_color[Hmax][Wmax];                            // 每个像素点的BGR值用一个三维向量表示
cv::Mat externalMat(4, 4, cv::DataType<double>::type);        // 外参旋转矩阵3*3和平移向量3*1
cv::Mat internalMatProject(3, 4, cv::DataType<double>::type); // 内参3*4的投影矩阵,最后一列是三个零
struct Box
{
    int xmin;
    int xmax;
    int ymin;
    int ymax;
};
std::vector<Box> box;

/**
 * @brief camera话题rgb回调函数
 *
 */
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    for (int row = 0; row < Hmax; row++)
    {
        for (int col = 0; col < Wmax; col++)
        {
            image_color[row][col] = (cv::Vec3b)image.at<cv::Vec3b>(row, col);
        }
    }

    // for (int i = 0; i < box.size(); i++)
    // {
    //     for (int row = 0; row < Hmax; row++)
    //     {
    //         for (int col = 0; col < Wmax; col++)
    //         {
    //             if (col > box[i].xmin - 20 && col < box[i].xmax + 220 && row > box[i].ymin - 20 && row < box[i].ymax + 20)
    //             {
    //                 image_color[row][col] = {0, 0, 255};
    //             }
    //         }
    //     }
    // }
}

/**
 * @brief 参数设置:camera内参、lidar系到camera系的外参
 *
 */
void paramSetting(void)
{
    externalMat = (cv::Mat_<double>(4, 4) << cam_ex[0], cam_ex[1], cam_ex[2], cam_ex[3],
                   cam_ex[4], cam_ex[5], cam_ex[6], cam_ex[7],
                   cam_ex[8], cam_ex[9], cam_ex[10], cam_ex[11],
                   cam_ex[12], cam_ex[13], cam_ex[14], cam_ex[15]);

    internalMatProject = (cv::Mat_<double>(3, 4) << cam_in[0], cam_in[1], cam_in[2], cam_in[3],
                          cam_in[4], cam_in[5], cam_in[6], cam_in[7],
                          cam_in[8], cam_in[9], cam_in[10], cam_in[11]);
}

/**
 * @brief 目标检测投影框回调函数
 */
void BoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    box.clear();
    for (int i = 0; i < msg->bounding_boxes.size(); i++)
    {
        if (msg->bounding_boxes[i].Class == "person" && msg->bounding_boxes[i].probability > 0.6)
        {
            Box boxi;
            boxi.xmin = msg->bounding_boxes[i].xmin;
            boxi.xmax = msg->bounding_boxes[i].xmax;
            boxi.ymin = msg->bounding_boxes[i].ymin;
            boxi.ymax = msg->bounding_boxes[i].ymax;
            box.push_back(boxi);
        }
    }
}

/**
 * @brief 发布彩色点云到world系
 *
 */
void publish_frame_world_color(const ros::Publisher &pubLaserCloudColor)
{
    if (p_pre->lidar_type == LIVOX)
    {
        if (scan_pub_en) // 是否发布激光雷达数据
        {
            // 发布稠密点云还是降采样点云
            PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
            sensor_msgs::PointCloud2 laserCloudFullResmsg;

            pcl::PointCloud<livox_ros::Point>::Ptr livox_color_ptr(new pcl::PointCloud<livox_ros::Point>); // 带颜色的点云
            pcl::PointCloud<livox_ros::Point>::Ptr livox_raw_ptr(new pcl::PointCloud<livox_ros::Point>);   // 不带颜色的点云

            for (int i = 0; i < laserCloudFullRes->size(); i++)
            {
                livox_ros::Point livox_raw_point;

                livox_raw_point.x = laserCloudFullRes->points[i].x;
                livox_raw_point.y = laserCloudFullRes->points[i].y;
                livox_raw_point.z = laserCloudFullRes->points[i].z;
                livox_raw_point.intensity = laserCloudFullRes->points[i].intensity;

                livox_raw_ptr->points.push_back(livox_raw_point);
            }

            cv::Mat coordinateCloud(4, 1, cv::DataType<double>::type);  // 点云坐标系下的x,y,z,1向量
            cv::Mat coordinateCamera(3, 1, cv::DataType<double>::type); // 相机坐标系下的x,y,z向量

            for (int i = 0; i < livox_raw_ptr->points.size(); i++)
            {
                coordinateCloud.at<double>(0, 0) = livox_raw_ptr->points[i].x;
                coordinateCloud.at<double>(1, 0) = livox_raw_ptr->points[i].y;
                coordinateCloud.at<double>(2, 0) = livox_raw_ptr->points[i].z;
                coordinateCloud.at<double>(3, 0) = 1;
                coordinateCamera = internalMatProject * externalMat * coordinateCloud; // 点云坐标转相机坐标

                cv::Point pixel; // 像素坐标
                pixel.x = coordinateCamera.at<double>(0, 0) / coordinateCamera.at<double>(0, 2);
                pixel.y = coordinateCamera.at<double>(1, 0) / coordinateCamera.at<double>(0, 2);

                if (pixel.x >= 0 && pixel.x < Wmax && pixel.y >= 0 && pixel.y < Hmax && livox_raw_ptr->points[i].x > 0) // && livox_raw_ptr->points[i].x>0去掉图像后方的点云
                {
                    livox_ros::Point livoxPoint;

                    livoxPoint.x = livox_raw_ptr->points[i].x;
                    livoxPoint.y = livox_raw_ptr->points[i].y;
                    livoxPoint.z = livox_raw_ptr->points[i].z;

                    livoxPoint.b = image_color[pixel.y][pixel.x][0];
                    livoxPoint.g = image_color[pixel.y][pixel.x][1];
                    livoxPoint.r = image_color[pixel.y][pixel.x][2];

                    livox_color_ptr->points.push_back(livoxPoint);

                    // if (livoxPoint.b != 0 && livoxPoint.g != 0 && livoxPoint.r != 255)
                    // {
                    //     livox_color_ptr->points.push_back(livoxPoint);
                    // }
                }
            }

            pcl::PointCloud<livox_ros::Point>::Ptr livox_color_world_ptr(livox_color_ptr);
            // 转换到world系
            for (int i = 0; i < livox_color_ptr->points.size(); i++)
            {
                V3D p_body(livox_color_ptr->points[i].x, livox_color_ptr->points[i].y, livox_color_ptr->points[i].z);
                V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

                livox_color_world_ptr->points[i].x = p_global(0);
                livox_color_world_ptr->points[i].y = p_global(1);
                livox_color_world_ptr->points[i].z = p_global(2);
            }

            sensor_msgs::PointCloud2 livox_color_world_msg;
            livox_color_world_ptr->width = 1;
            livox_color_world_ptr->height = livox_color_world_ptr->points.size();
            pcl::toROSMsg(*livox_color_world_ptr, livox_color_world_msg);
            livox_color_world_msg.header.frame_id = "camera_init";
            pubLaserCloudColor.publish(livox_color_world_msg);
            publish_count -= PUBFRAME_PERIOD;
        }
    }
}

/**
 * @brief 更新里程计轨迹
 *
 */
void updatePath(const PointTypePose &pose_in)
{
    string odometryFrame = "camera_init";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);

    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
}

/**
 * @brief
 * 对点云cloudIn进行变换,返回结果点云transformIn
 * cloudKeyPoses6D存储的是T_w_b,而点云是lidar系下的,构建icp的submap时,需要通过外参数T_b_lidar转换
 */
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I); // 获取body2lidar外参
    T_b_lidar.pretranslate(state_point.offset_T_L_I);      // pretranslate之前的pre表示的是平移在旋转之前的坐标原点上平移

    Eigen::Affine3f T_w_b_ = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    Eigen::Isometry3d T_w_b; // world2body
    T_w_b.matrix() = T_w_b_.matrix().cast<double>();

    Eigen::Isometry3d T_w_lidar = T_w_b * T_b_lidar; // T_w_lidar转换矩阵

    Eigen::Isometry3d transCur = T_w_lidar;

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

/**
 * @brief 位姿格式变换:pcl->gtsamPose3
 *
 */
gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

/**
 * @brief 位姿格式变换:float[]->gtsamPose
 *
 */
gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

/**
 * @brief 格式的位姿格式变换:pcl->Affine3f
 *
 */
Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

/**
 * @brief 格式的位姿格式变换:float[]->Affine3f
 *
 */
Eigen::Affine3f trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

/**
 * @brief 位姿格式变换:float[]->pcl
 *
 */
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

/**
 * @brief 发布thisCloud,返回thisCloud对应msg格式
 *
 */
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

/**
 * @brief 点到坐标系原点距离
 *
 */
float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/**
 * @brief 两点之间距离
 *
 */
float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

/**
 * @brief 初始化
 *
 */
void allocateMemory()
{
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudOri.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    for (int i = 0; i < 6; ++i)
    {
        transformTobeMapped[i] = 0;
    }
}

/**
 * @brief EulerAngle转Quaterniond
 *
 */
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

/**
 * @brief Get the Cur Pose object将更新的pose赋值到transformTobeMapped中来表示位姿（因子图要用）
 *
 */
void getCurPose(state_ikfom cur_state)
{
    // 欧拉角是没有群的性质,所以从SO3还是一般的rotation matrix,转换过来的结果一样
    Eigen::Vector3d eulerAngle = cur_state.rot.matrix().eulerAngles(2, 1, 0); // yaw-pitch-roll,单位:弧度

    transformTobeMapped[0] = eulerAngle(2);    // roll
    transformTobeMapped[1] = eulerAngle(1);    // pitch
    transformTobeMapped[2] = eulerAngle(0);    // yaw
    transformTobeMapped[3] = cur_state.pos(0); // x
    transformTobeMapped[4] = cur_state.pos(1); // y
    transformTobeMapped[5] = cur_state.pos(2); // z
}

/**
 * @brief rviz展示回环边
 *
 */
void visualizeLoopClosure()
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); // 时间戳
    string odometryFrame = "camera_init";

    if (loopIndexContainer.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // 回环顶点
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    // action对应的操作:ADD=0、MODIFY=0、DELETE=2、DELETEALL=3,即添加、修改、删除、全部删除
    markerNode.action = visualization_msgs::Marker::ADD;
    // 设置形状:球体
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    // 尺寸
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    // 颜色
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // 回环边
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    // 设置形状:线
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    // 遍历回环
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::Point p;
        p.x = copy_cloudKeyPoses6D->points[key_cur].x;
        p.y = copy_cloudKeyPoses6D->points[key_cur].y;
        p.z = copy_cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = copy_cloudKeyPoses6D->points[key_pre].x;
        p.y = copy_cloudKeyPoses6D->points[key_pre].y;
        p.z = copy_cloudKeyPoses6D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);
}

/**
 * @brief 计算当前帧与前一帧位姿变换,如果变化太小,不设为关键帧,反之设为关键帧
 *
 */
bool saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    // 前一帧位姿,注:最开始没有的时候,在函数extractCloud里面有
    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    // 当前帧位姿
    Eigen::Affine3f transFinal = trans2Affine3f(transformTobeMapped);
    // 位姿变换增量
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    // pcl::getTranslationAndEulerAngles是根据仿射矩阵计算x,y,z,roll,pitch,yaw
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw); // 获取上一帧相对当前帧的位姿

    // 旋转和平移量都较小,当前帧不设为关键帧
    if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
        return false;
    return true;
}

/**
 * @brief 添加激光里程计因子
 *
 */
void addOdomFactor()
{
    // 如果是第一帧
    if (cloudKeyPoses3D->points.empty())
    {
        // 给出一个噪声模型,也就是协方差矩阵
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
        // 加入先验因子PriorFactor,固定这个顶点,对第0个节点增加约束
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        // 节点设置初始值,将这个顶点的值加入初始值中
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }
    // 不是第一帧,增加帧间约束
    else
    {
        // 添加激光里程计因子
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back()); // 上一个位姿
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);                   // 当前位姿
        // 参数:前一帧id;当前帧id;前一帧与当前帧的位姿变换poseFrom.between(poseTo) = poseFrom.inverse()*poseTo;噪声协方差;
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        // 变量节点设置初始值,将这个顶点的值加入初始值中
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

/**
 * @brief 添加回环因子
 *
 */
void addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    // 把队列里面所有的回环约束添加进行
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        int indexFrom = loopIndexQueue[i].first; // 回环帧索引
        int indexTo = loopIndexQueue[i].second;  // 当前帧索引
        // 两帧的位姿变换（帧间约束）
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        // 回环的置信度就是icp的得分？
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        // 加入约束
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }
    // 清空回环相关队列
    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

/**
 * @brief
 * 设置当前帧为关键帧并执行因子图优化
 * 1、计算当前帧与前一帧位姿变换,如果变化太小,不设为关键帧,反之设为关键帧
 * 2、添加激光里程计因子、回环因子
 * 3、执行因子图优化
 * 4、得到当前帧优化后位姿,位姿协方差
 * 5、添加cloudKeyPoses3D,cloudKeyPoses6D,更新transformTobeMapped,添加当前关键帧的角点、平面点集合
 */
void saveKeyFramesAndFactor()
{
    // 计算当前帧与前一帧位姿变换,如果变化太小,不设为关键帧,反之设为关键帧
    if (saveFrame() == false)
        return;
    // 激光里程计因子(from fast-lio)
    addOdomFactor();
    // 回环因子
    // addLoopFactor();
    // 执行优化,更新图模型
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    // 如果加入了回环约束,isam需要进行更多次的优化
    if (aLoopIsClosed == true)
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }
    // update之后要清空一下保存的因子图,注:清空不会影响优化,ISAM保存起来了
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    // 通过接口获得所以变量的优化结果
    isamCurrentEstimate = isam->calculateBestEstimate();
    // 取出优化后的当前帧位姿结果
    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
    // 位移信息取出来保存进clouKeyPoses3D这个结构中
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    // 其中索引作为intensity
    thisPose3D.intensity = cloudKeyPoses3D->size(); // 使用intensity作为该帧点云的index
    cloudKeyPoses3D->push_back(thisPose3D);         // 新关键帧帧放入队列中
    // 同样6D的位姿也保存下来
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = lidar_end_time;
    cloudKeyPoses6D->push_back(thisPose6D);
    // 保存当前位姿的位姿协方差（置信度）
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

    // ESKF状态和方差更新
    state_ikfom state_updated = kf.get_x(); // 获取cur_pose(还没修正)
    Eigen::Vector3d pos(latestEstimate.translation().x(), latestEstimate.translation().y(), latestEstimate.translation().z());
    Eigen::Quaterniond q = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());

    // 更新状态量
    state_updated.pos = pos;
    state_updated.rot = q;
    state_point = state_updated; // 对state_point进行更新,state_point可视化用到
    // if(aLoopIsClosed == true )
    kf.change_x(state_updated); // 对cur_pose进行isam2优化后的修正
    // 当前帧激光角点、平面点,降采样集合
    // pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    // pcl::copyPointCloud(*feats_undistort,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*feats_undistort, *thisSurfKeyFrame); // 存储关键帧,没有降采样的点云

    // 保存特征点降采样集合
    // cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    updatePath(thisPose6D); // 可视化update后的最新位姿
}

void recontructIKdTree()
{
    if (recontructKdTree && updateKdtreeCount > 0)
    {
        /*** if path is too large, the rvis will crash ***/
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kdtree查找最近一帧关键帧相邻的关键帧集合
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        mtx.lock();
        kdtreeGlobalMapPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMapPoses->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            subMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]); // subMap的pose集合
        // 降采样
        pcl::VoxelGrid<PointType> downSizeFilterSubMapKeyPoses;
        downSizeFilterSubMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterSubMapKeyPoses.setInputCloud(subMapKeyPoses);
        downSizeFilterSubMapKeyPoses.filter(*subMapKeyPosesDS); // subMap poses  downsample
        // 提取局部相邻关键帧对应的特征点云
        for (int i = 0; i < (int)subMapKeyPosesDS->size(); ++i)
        {
            // 距离过大
            if (pointDistance(subMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)subMapKeyPosesDS->points[i].intensity;
            // *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *subMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]); //  fast_lio only use  surfCloud
        }
        // 降采样，发布
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;                                                                                   // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(subMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*subMapKeyFramesDS);

        std::cout << "subMapKeyFramesDS sizes  =  " << subMapKeyFramesDS->points.size() << std::endl;

        ikdtree.reconstruct(subMapKeyFramesDS->points);
        updateKdtreeCount = 0;
        ROS_INFO("Reconstructed  ikdtree ");
        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();
        std::cout << "featsFromMapNum  =  " << featsFromMapNum << "\t"
                  << " kdtree_size_st   =  " << kdtree_size_st << std::endl;
    }
    updateKdtreeCount++;
}

/**
 * @brief 更新因子图中所有变量节点的位姿,也就是所有历史关键帧的位姿,调整全局轨迹,重构ikdtree
 *
 */
void correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;
    // 只有回环以及才会触发全局调整
    if (aLoopIsClosed == true)
    {
        // 清空里程计轨迹
        globalPath.poses.clear();
        // 更新因子图中所有变量节点的位姿,也就是所有历史关键帧的位姿
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();

            // 更新里程计轨迹
            updatePath(cloudKeyPoses6D->points[i]);
        }
        // 清空局部map, reconstruct  ikdtree submap
        recontructIKdTree();
        ROS_INFO("ISMA2 Update");
        aLoopIsClosed = false;
    }
}

/**
 * @brief
 * 检测最新帧是否和其它帧形成回环
 * 回环检测三大要素
 * 1.设置最小时间差,太近没必要
 * 2.控制回环的频率,避免频繁检测,每检测一次,就做一次等待
 * 3.根据当前最小距离重新计算等待时间
 */
bool detectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1; // 当前关键帧索引
    int loopKeyPre = -1;

    // 检查一下当前帧是否和别的形成了回环,如果已经有回环就不再继续
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;
    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
    std::vector<int> pointSearchIndLoop;                        // 候选关键帧索引
    std::vector<float> pointSearchSqDisLoop;                    // 候选关键帧距离
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D); // 把只包含关键帧位移信息的点云填充kdtree
    // 根据最后一个关键帧的平移信息,寻找离他一定距离内的其它关键帧
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    // 在候选关键帧集合中,找到与当前帧时间相隔较远的帧,设为候选匹配帧
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        // 历史帧必须比当前帧间隔historyKeyframeSearchTimeDiff以上,必须满足时间阈值,才是一个有效回环,一次找一个回环帧就行了
        if (abs(copy_cloudKeyPoses6D->points[id].time - lidar_end_time) > historyKeyframeSearchTimeDiff)
        {
            loopKeyPre = id;
            break;
        }
    }
    // 如果没有找到回环或者回环找到自己身上去了,就认为是本次回环寻找失败
    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;
    // 赋值当前帧和历史回环帧的id
    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    ROS_INFO("Find loop clousre frame ");
    return true;
}

/**
 * @brief 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合,降采样
 *
 */
void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
{
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    // searchNum是搜索范围,遍历帧的范围
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        // 超出范围,退出
        if (keyNear < 0 || keyNear >= cloudSize)
            continue;
        // 注意:cloudKeyPoses6D存储的是T_w_b,而点云是lidar系下的,构建icp的submap时,要把点云转到world系
        *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]); // fast-lio没有进行特征提取,默认点云就是surf
    }
    if (nearKeyframes->empty())
        return;

    // 降采样
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

/**
 * @brief 回环检测函数
 *
 */
void performLoopClosure()
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); // 时间戳
    string odometryFrame = "camera_init";
    // 没有关键帧,没法进行回环检测
    if (cloudKeyPoses3D->points.empty() == true)
    {
        return;
    }
    // 把存储关键帧位姿的点云copy出来,避免线程冲突,cloudKeyPoses3D就是关键帧的位置,cloudKeyPoses6D就是关键帧的位姿
    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    int loopKeyCur; // 当前关键帧索引
    int loopKeyPre; // 候选回环匹配帧索引
    // 根据里程计的距离来检测回环,如果还没有回环则直接返回
    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
    {
        return;
    }
    // 提取
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>()); // 当前关键帧的点云
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>()); // 历史回环帧周围的点云（局部地图）
    {
        // 提取当前关键帧特征点集合,降采样
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0); // 将cureKeyframeCloud转换到world系下
        // 提取历史回环帧周围的点云特征点集合,降采样
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum); // 选取historyKeyframeSearchNum个keyframe拼成submap
        // 发布回环局部地图submap
        if (pubHistoryKeyFrames.getNumSubscribers() != 0)
            publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    }
    // ICP设置
    pcl::IterativeClosestPoint<PointType, PointType> icp; // 使用icp来进行帧到局部地图的配准
    icp.setMaxCorrespondenceDistance(150);                // 设置对应点最大欧式距离,只有对应点之间的距离小于该设置值的对应点才作为ICP计算的点对
    icp.setMaximumIterations(100);                        // 迭代停止条件一:设置最大的迭代次数
    icp.setTransformationEpsilon(1e-6);                   // 迭代停止条件二:设置前后两次迭代的转换矩阵的最大容差,一旦两次迭代小于最大容差,则认为收敛到最优解,迭代停止
    icp.setEuclideanFitnessEpsilon(1e-6);                 // 迭代终止条件三:设置前后两次迭代的点对的欧式距离均值的最大容差
    icp.setRANSACIterations(0);                           // 设置RANSAC运行次数

    // scan-to-map,调用icp匹配
    icp.setInputSource(cureKeyframeCloud); // 设置原始点云
    icp.setInputTarget(prevKeyframeCloud); // 设置目标点云
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result); // 进行ICP配准,输出变换后点云

    // 检测icp是否收敛以及得分是否满足要求
    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

    std::cout << "icp  success  " << std::endl;

    // 发布当前关键帧经过回环优化后的位姿变换之后的特征点云供可视化使用
    if (pubIcpKeyFrames.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
        publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    }

    // 回环优化得到的当前关键帧与回环关键帧之间的位姿变换
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation(); // 获得两个点云的变换矩阵结果
    // 回环优化前的当前帧位姿
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // 回环优化后的当前帧位姿:将icp结果补偿过去,就是当前帧的更为准确的位姿结果
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;

    // 将回环优化后的当前帧位姿换成平移和旋转
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    // 将回环优化后的当前帧位姿换成gtsam的形式,From和To相当于帧间约束的因子,To是历史回环帧的位姿
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    // 使用icp的得分作为他们的约束噪声项
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore(); // 获得icp的得分
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    std::cout << "loopNoiseQueue   =   " << noiseScore << std::endl;

    // 添加回环因子需要的数据:将两帧索引,两帧相对位姿和噪声作为回环约束送入对列
    mtx.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtx.unlock();

    loopIndexContainer[loopKeyCur] = loopKeyPre; // 使用hash map存储回环对
}

/**
 * @brief 回环检测线程
 *
 */
void loopClosureThread()
{
    // 不进行回环检测
    if (loopClosureEnableFlag == false)
    {
        std::cout << "loopClosureEnableFlag   ==  false " << endl;
        return;
    }

    ros::Rate rate(loopClosureFrequency); // 设置回环检测的频率loopClosureFrequency默认为1hz
    while (ros::ok() && startFlag)
    {
        // 执行完一次就必须sleep一段时间,否则该线程的cpu占用会非常高
        rate.sleep();
        performLoopClosure();   // 回环检测函数
        visualizeLoopClosure(); // rviz展示回环边
    }
}

/**
 * @brief 按下ctrl+c后唤醒所有线程
 *
 */
void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    // 唤醒所有等待队列中阻塞的线程,线程被唤醒后,会通过轮询方式获得锁,获得锁前也一直处理运行状态,不会被再次阻塞.
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                            // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));    // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));    // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));       // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));       // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

/**
 * @brief 按当前body(lidar)的状态,将局部点转换到world系下
 *
 */
void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    // 先转到imu(body)系,再转到world系
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

/**
 * @brief 含有RGB的点云从body(lidar)系转到world系
 *
 */
void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

/**
 * @brief 含有RGB的点云从body(lidar)系转到imu(body)系
 *
 */
void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    for (int i = 0; i < points_history.size(); i++)
        _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;      // ikdtree中,局部地图的包围盒角点
bool Localmap_Initialized = false; // 局部地图是否初始化
/**
 * @brief 根据lidar的FoV分割场景,动态调整地图区域,防止地图过大而内存溢出
 *
 */
void lasermap_fov_segment()
{
    cub_needrm.clear(); // 清空需要移除的区域
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    V3D pos_LiD = pos_lid; // globalidar系lidar位置

    // 初始化局部地图包围盒角点,以world系下lidar位置为中心,得到长宽高200*200*200的局部地图
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0; // 边界距离当前位置100米
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0; // 边界距离当前位置100米
        }
        Localmap_Initialized = true;
        return;
    }

    float dist_to_map_edge[3][2]; // 各个方向与局部地图边界的距离,或者说是lidar与立方体盒子六个面的距离
    bool need_move = false;
    // 当前雷达系中心到各个地图边缘的距离
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);

        // 与某个方向上的边界距离(例如1.5*300m)太小,标记需要移动need_move
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    // 不需要挪动就直接返回了,否则需要计算移动的距离
    if (!need_move)
        return;
    // 新的局部地图盒子边界点
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        // 与包围盒最小值角点距离
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints); // 移除较远包围盒
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
    // 使用Boxs删除指定盒内的点
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

/**
 * @brief sensor_msgs::PointCloud2格式点云数据的回调函数,将数据引入buffer中
 *
 */
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock(); // 加锁
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    // 如果当前扫描的lidar数据比上一次早,则lidar数据有误,需要将lidar数据缓存队列清空
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);                         // 点云预处理
    lidar_buffer.push_back(ptr);                      // 点云存入缓冲区
    time_buffer.push_back(msg->header.stamp.toSec()); // 时间存入缓冲区
    last_timestamp_lidar = msg->header.stamp.toSec(); // 记录最后一个雷达时间
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();     // 解锁
    sig_buffer.notify_all(); // 唤醒所有线程
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false; // 标记是否已经进行了时间同步,false不进行时间同步
/**
 * @brief livox_ros_driver::CustomMsg格式点云数据的回调函数,将数据引入buffer中
 *
 */
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock(); // 加锁
    double preprocess_start_time = omp_get_wtime();
    scan_count++; // 激光雷达扫描的次数
    // 如果当前扫描的lidar数据比上一次早,则lidar数据有误,需要将lidar数据缓存队列清空
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec(); // 记录最后一个雷达时间
    // 如果不需要进行时间同步,而imu时间戳和雷达时间戳相差大于10s,则输出错误信息
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }
    // time_sync_en为true时,当imu时间戳和雷达时间戳相差大于1s时,进行时间同步
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu; //????
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

    p_pre->process(msg, ptr);                    // 点云预处理
    lidar_buffer.push_back(ptr);                 // 点云存入缓冲区
    time_buffer.push_back(last_timestamp_lidar); // 时间存入缓冲区

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();     // 解锁
    sig_buffer.notify_all(); // 唤醒所有线程
}

/**
 * @brief livox的pointcloud2格式数据的回调函数,将数据引入buffer中
 *
 */
void livox_ros_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock(); // 加锁
    scan_count++;

    // 如果当前扫描的lidar数据比上一次早,则lidar数据有误,需要将lidar数据缓存队列清空
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    last_timestamp_lidar = msg->header.stamp.toSec(); // 记录最后一个雷达时间
    // 如果不需要进行时间同步,而imu时间戳和雷达时间戳相差大于10s,则输出错误信息
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }
    // time_sync_en为true时,当imu时间戳和雷达时间戳相差大于1s时,进行时间同步
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu; // ????
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);                         // 点云预处理
    lidar_buffer.push_back(ptr);                      // 点云存入缓冲区
    time_buffer.push_back(msg->header.stamp.toSec()); // 时间存入缓冲区
    last_timestamp_lidar = msg->header.stamp.toSec(); // 记录最后一个雷达时间
    mtx_buffer.unlock();                              // 解锁
    sig_buffer.notify_all();                          // 唤醒所有线程
}

/**
 * @brief sensor_msgs::Imu格式IMU数据的回调函数,将数据引入buffer中
 *
 */
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // lidar和imu时间差过大且开启时间同步时,纠正当前输入imu的时间
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        // 将imu时间纠正为时间差+原始时间(激光雷达时间)
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec(); // IMU时间戳

    mtx_buffer.lock(); // 加锁
    // 如果当前IMU的时间戳小于上一个时刻,则IMU数据有误,将IMU数据缓存队列清空
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp; // 记录最后一个IMU时间

    imu_buffer.push_back(msg); // IMU存入缓冲区
    mtx_buffer.unlock();       // 解锁
    sig_buffer.notify_all();   // 唤醒所有线程
}

/**
 * @brief livox custom转pcl
 *
 */
void LivoxRepubCallback(const livox_ros_driver::CustomMsgConstPtr &livox_msg_in)
{
    livox_msg.push_back(livox_msg_in); // 读入livox数据
    if (livox_msg.size() < 1)
        return;

    pcl::PointCloud<livox_ros::Point> pcl_in; // livox_ros::Point点云

    for (size_t j = 0; j < livox_msg.size(); j++)
    {
        auto &livox_data = livox_msg[j];                         // 当前帧点云
        auto time_end = livox_data->points.back().offset_time;   // 当前帧点云最后一个点的时间偏移
        for (unsigned int i = 0; i < livox_data->point_num; ++i) // 遍历当前帧点云的点
        {
            /* 转换数据类型 */
            livox_ros::Point pointPCL;            // livox_ros::Point
            pointPCL.x = livox_data->points[i].x; // X
            pointPCL.y = livox_data->points[i].y; // Y
            pointPCL.z = livox_data->points[i].z; // Z
            pointPCL.intensity = livox_data->points[i].reflectivity;
            pointPCL.line = livox_data->points[i].line;
            pointPCL.tag = livox_data->points[i].tag;
            pointPCL.offset_time = livox_data->points[i].offset_time;
            pcl_in.push_back(pointPCL);
        }
    }

    /// timebase 5ms ~ 50000000, so 10 ~ 1ns
    unsigned long timebase_ns = livox_msg[0]->timebase;
    ros::Time timestamp;
    timestamp.fromNSec(timebase_ns);

    sensor_msgs::PointCloud2 pcl_msg_out;
    pcl::toROSMsg(pcl_in, pcl_msg_out);
    pcl_msg_out.header.stamp.fromNSec(timebase_ns);
    pcl_msg_out.header.frame_id = "/livox_frame";
    pubLidarPCL.publish(pcl_msg_out);
    livox_msg.clear();
}

double lidar_mean_scantime = 0.0; // 单帧扫描时间
int scan_num = 0;
/**
 * @brief 处理缓冲区中的数据,将两帧激光雷达点云数据时间内的IMU数据从缓存队列中取出,进行时间对齐,并保存到meas中
 *
 */
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    // 如果还没有把雷达数据放到meas中的话,执行
    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();         // lidar指针指向最旧的lidar数据
        meas.lidar_beg_time = time_buffer.front(); // 记录最早时间

        // 时间太短,点数不足
        if (meas.lidar->points.size() <= 1)
        {
            // 记录lidar结束时间为起始时间+单帧扫描时间
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        // 最后一个点的时间小于单帧扫描时间的一半
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            // 记录lidar结束时间为起始时间+单帧扫描时间
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num++; // 扫描数增加
            // 结束时间设置为起始时间+最后一个点的时间偏移
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            // 动态更新每帧lidar数据平均扫描时间
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time; // 点云结束时间

        lidar_pushed = true; // 成功提取到lidar测量的标志
    }
    // 最新的IMU时间戳不能早于雷达的结束时间戳,因为last_timestamp_imu比较时是加了0.1的,所以要比较大于雷达的end时间戳
    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    // 压入imu数据,并从imu缓冲区弹出
    double imu_time = imu_buffer.front()->header.stamp.toSec(); // 最旧IMU时间
    meas.imu.clear();
    // 拿出lidar_beg_time到lidar_end_time之间的所有IMU数据
    // 如果imu缓存队列中的数据时间戳小于雷达结束时间戳,则将该数据放到meas中,代表了这一帧中的imu数据
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec(); // 获取imu数据的时间戳
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front()); // 记录当前lidar帧内的imu数据到meas.imu
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front(); // 将lidar数据弹出
    time_buffer.pop_front();  // 将时间戳弹出
    lidar_pushed = false;     // 将lidar_pushed置为false,代表lidar数据已经被放到meas中了
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;            // 需要加入到ikd-tree中的点云
    PointVector PointNoNeedDownsample; // 加入ikd-tree时，不需要降采样的点云
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    // 根据点与所在包围盒中心点的距离，分类是否需要降采样
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point); // 当前点与box中心的距离

            // 判断最近点在x、y、z三个方向上，与中心的距离，判断是否加入时需要降采样
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]); // 若三个方向距离都大于地图珊格半轴长，无需降采样
                continue;
            }

            // 判断当前点的NUM_MATCH_POINTS个邻近点与包围盒中心的范围
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) // 如果邻近点到中心的距离小于当前点到中心的距离，则不需要添加当前点
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true); // 加入点时需要降采样
    ikdtree.Add_Points(PointNoNeedDownsample, false);      // 加入点时不需要降采样
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1)); // 创建一个点云用于存储等待发布的点云
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());         // 创建一个点云用于存储等待保存的点云

/**
 * @brief 消息发布到world系
 *
 */
void publish_frame_world(const ros::Publisher &pubLaserCloudFull)
{
    if (scan_pub_en) // 是否发布激光雷达数据
    {
        // 发布稠密点云还是降采样点云
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();                      // 获取待转换点云的大小
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1)); // W坐标系的点云
        // 转换到world系
        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }
        // 发布到sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i],
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
        {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

/**
 * @brief 消息发布到imu(body)系
 *
 */
void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1)); // imu(body)系的点云
                                                                        // 转换到imu(body)系
    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }
    // 发布到sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher &pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i],
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

/**
 * @brief 发布ikdtreeMap
 *
 */
void publish_map(const ros::Publisher &pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template <typename T>
/**
 * @brief 置输出的p、q,在publish_odometry,publish_path调用
 *
 */
void set_posestamp(T &out)
{
    // 将eskf求得的位置传入
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    // 将eskf求得的姿态传入
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

/**
 * @brief 发布里程计
 *
 */
void publish_odometry(const ros::Publisher &pubOdomAftMapped)
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
        // 设置协方差P,P先是旋转后是位置,pose先是位置后是旋转,所以对应的协方差要对调一下
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
    // 发布tf变换
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
}

/**
 * @brief 每隔10个发布一下位姿(未优化)
 *
 */
void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);

        // 保存未优化的姿态
        V3D rot_ang(Log(state_point.rot.toRotationMatrix())); // 旋转向量
        PointTypePose thisPose6D;
        thisPose6D.x = msg_body_pose.pose.position.x;
        thisPose6D.y = msg_body_pose.pose.position.y;
        thisPose6D.z = msg_body_pose.pose.position.z;
        thisPose6D.roll = rot_ang(0);
        thisPose6D.pitch = rot_ang(1);
        thisPose6D.yaw = rot_ang(2);
        fastlio_unoptimized_cloudKeyPoses6D->push_back(thisPose6D);
    }
}

/**
 * @brief 每隔10个发布一下位姿(优化)
 *
 */
void publish_path_update(const ros::Publisher pubPath)
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); // 时间戳
    string odometryFrame = "camera_init";
    if (pubPath.getNumSubscribers() != 0)
    {
        /*** if path is too large, the rvis will crash ***/
        static int kkk = 0;
        kkk++;
        if (kkk % 10 == 0)
        {
            // path.poses.push_back(globalPath);
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
    }
}

/**
 * @brief 定义pose结构体
 *
 */
struct pose
{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
};

bool CreateFile(std::ofstream &ofs, std::string file_path)
{
    ofs.open(file_path, std::ios::out); // 使用std::ios::out可实现覆盖
    if (!ofs)
    {
        std::cout << "open csv file error " << std::endl;
        return false;
    }
    return true;
}

/* write2txt format KITTI*/
void WriteText(std::ofstream &ofs, pose data)
{
    ofs << std::fixed << data.R(0, 0) << " " << data.R(0, 1) << " " << data.R(0, 2) << " " << data.t[0] << " "
        << data.R(1, 0) << " " << data.R(1, 1) << " " << data.R(1, 2) << " " << data.t[1] << " "
        << data.R(2, 0) << " " << data.R(2, 1) << " " << data.R(2, 2) << " " << data.t[2] << std::endl;
}

bool savePoseService(fast_lio_sam::save_poseRequest &req, fast_lio_sam::save_poseResponse &res)
{
    pose pose_optimized;
    pose pose_without_optimized;

    std::ofstream file_pose_optimized;
    std::ofstream file_pose_without_optimized;

    string savePoseDirectory;
    cout << "****************************************************" << endl;
    cout << "Saving poses to pose files ..." << endl;
    if (req.destination.empty())
        savePoseDirectory = std::getenv("HOME") + savePCDDirectory;
    else
        savePoseDirectory = std::getenv("HOME") + req.destination;
    cout << "Save destination: " << savePoseDirectory << endl;

    // create file
    CreateFile(file_pose_optimized, savePoseDirectory + "/optimized_pose.txt");
    CreateFile(file_pose_without_optimized, savePoseDirectory + "/without_optimized_pose.txt");

    // save optimize data
    for (int i = 0; i < cloudKeyPoses6D->size(); i++)
    {
        pose_optimized.t = Eigen::Vector3d(cloudKeyPoses6D->points[i].x, cloudKeyPoses6D->points[i].y, cloudKeyPoses6D->points[i].z);
        pose_optimized.R = Exp(double(cloudKeyPoses6D->points[i].roll), double(cloudKeyPoses6D->points[i].pitch), double(cloudKeyPoses6D->points[i].yaw));
        WriteText(file_pose_optimized, pose_optimized);
    }
    cout << "Sucess global optimized  poses to pose files ..." << endl;

    for (int i = 0; i < fastlio_unoptimized_cloudKeyPoses6D->size(); i++)
    {
        pose_without_optimized.t = Eigen::Vector3d(fastlio_unoptimized_cloudKeyPoses6D->points[i].x, fastlio_unoptimized_cloudKeyPoses6D->points[i].y, fastlio_unoptimized_cloudKeyPoses6D->points[i].z);
        pose_without_optimized.R = Exp(double(fastlio_unoptimized_cloudKeyPoses6D->points[i].roll), double(fastlio_unoptimized_cloudKeyPoses6D->points[i].pitch), double(fastlio_unoptimized_cloudKeyPoses6D->points[i].yaw));
        WriteText(file_pose_without_optimized, pose_without_optimized);
    }
    cout << "Sucess unoptimized  poses to pose files ..." << endl;

    file_pose_optimized.close();
    file_pose_without_optimized.close();
    return true;
}

/**
 * 保存全局关键帧特征点集合
 */
bool saveMapService(fast_lio_sam::save_mapRequest &req, fast_lio_sam::save_mapResponse &res)
{
    string saveMapDirectory;

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files ..." << endl;
    if (req.destination.empty())
        saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
    else
        saveMapDirectory = std::getenv("HOME") + req.destination;
    cout << "Save destination: " << saveMapDirectory << endl;
    // 这个代码太坑了！！注释掉
    //   int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
    //   unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());
    // 保存历史关键帧位姿
    pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);      // 关键帧位置
    pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D); // 关键帧位姿
    // 提取历史关键帧角点、平面点集合
    //   pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    //   pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

    // 注意：拼接地图时，keyframe是lidar系，而fastlio更新后的存到的cloudKeyPoses6D 关键帧位姿是body系下的，需要把
    // cloudKeyPoses6D  转换为T_world_lidar 。 T_world_lidar = T_world_body * T_body_lidar , T_body_lidar 是外参
    for (int i = 0; i < (int)cloudKeyPoses6D->size(); i++)
    {
        //   *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
        *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
    }

    if (req.resolution != 0)
    {
        cout << "\n\nSave resolution: " << req.resolution << endl;

        // 降采样
        // downSizeFilterCorner.setInputCloud(globalCornerCloud);
        // downSizeFilterCorner.setLeafSize(req.resolution, req.resolution, req.resolution);
        // downSizeFilterCorner.filter(*globalCornerCloudDS);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);
        // 降采样
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
    }
    else
    {
        //   downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloud);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);           //  稠密点云地图
    }

    // 保存到一起，全局关键帧特征点集合
    //   *globalMapCloud += *globalCornerCloud;
    *globalMapCloud += *globalSurfCloud;
    pcl::io::savePCDFileBinary(saveMapDirectory + "/filterGlobalMap.pcd", *globalSurfCloudDS);  //  滤波后地图
    int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud); //  稠密地图
    res.success = ret == 0;

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed\n"
         << endl;

    // visial optimize global map on viz
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";
    publishCloud(&pubOptimizedGlobalMap, globalSurfCloudDS, timeLaserInfoStamp, odometryFrame);

    return true;
}

void saveMap()
{
    fast_lio_sam::save_mapRequest req;
    fast_lio_sam::save_mapResponse res;
    // 保存全局关键帧特征点集合
    if (!saveMapService(req, res))
    {
        cout << "Fail to save map" << endl;
    }
}

/**
 * @brief 发布局部关键帧map的特征点云
 *
 */
void publishGlobalMap()
{
    /*** if path is too large, the rvis will crash ***/
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";
    if (pubLaserCloudSurround.getNumSubscribers() == 0)
        return;

    if (cloudKeyPoses3D->points.empty() == true)
        return;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
    ;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

    // kdtree查找最近一帧关键帧相邻的关键帧集合
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    // 降采样
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
    downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    // 提取局部相邻关键帧对应的特征点云
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
    {
        // 距离过大
        if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
            continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        // *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
        *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]); //  fast_lio only use  surfCloud
    }
    // 降采样,发布
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;                                                                                   // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(&pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
}

/**
 * @brief 构造H矩阵,计算残差
 *
 */
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    laserCloudOri->clear(); // 将body系的有效点云存储清空
    corr_normvect->clear(); // 将对应的法向量清空
    total_residual = 0.0;

/** 最接近曲面搜索和残差计算  **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    // 对降采样后的每个特征点进行残差计算
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];   // 获取降采样后的特征点在lidar系下坐标
        PointType &point_world = feats_down_world->points[i]; // 获取降采样后的特征点在world系下坐标

        // 将点转换到世界坐标系下
        V3D p_body(point_body.x, point_body.y, point_body.z);                     // lidar系下坐标
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos); // world系下坐标
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        // 如果收敛了
        if (ekfom_data.converge)
        {
            // world系下从ikdtree已构造的地图上找5个最近点用于平面拟合
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            // 如果最近邻的点数小于NUM_MATCH_POINTS或者最近邻的点到特征点的距离大于5m，则认为该点不是有效点
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        // 无效点
        if (!point_selected_surf[i])
            continue;

        VF(4)
        pabcd;                          // 平面点信息
        point_selected_surf[i] = false; // 将该点设置为无效点，用来计算是否为平面点
        // 拟合平面方程ax+by+cz+d=0并求解点到平面距离,返回:是否有内点大于距离阈值
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            // 计算点到平面的距离
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            // 计算残差，残差公式 1 - 0.9 * (点到平面距离 / 点到lidar原点距离)
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            // 如果残差大于阈值，则认为该点是有效点
            if (s > 0.9)
            {
                point_selected_surf[i] = true;   // 再次回复为有效点
                normvec->points[i].x = pabcd(0); // 将法向量存储至normvec
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2; // 将点到平面的距离存储至normvec的intensit中
                res_last[i] = abs(pd2);             // 将残差存储至res_last
            }
        }
    }

    effct_feat_num = 0; // 有效特征点数

    for (int i = 0; i < feats_down_size; i++)
    {
        // 根据point_selected_surf状态判断哪些点是可用的
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i]; // 将降采样后的每个特征点存储至laserCloudOri
            corr_normvect->points[effct_feat_num] = normvec->points[i];         // 对应点法相量
            total_residual += res_last[i];                                      // 计算总残差
            effct_feat_num++;                                                   // 有效特征点数加1
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num; // 残差均值(距离)
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //定义H维度
    ekfom_data.h.resize(effct_feat_num);                 //有效方程个数

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i]; // lidar系 平面特征点
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I; // 当前状态imu系下 点坐标
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this); // 当前状态imu系下 点坐标反对称矩阵

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z); //对应局部法相量, world系下

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec); // 将对应局部法相量旋转到imu系下 corr_normal_I
        V3D A(point_crossmat * C);           //残差对角度求导系数 P(IMU)^ [R(imu <-- w) * normal_w]
        //添加数据到矩阵
        if (extrinsic_est_en)
        {
            // B = lidar_p^ R(L <-- I) * corr_normal_I
            // B = lidar_p^ R(L <-- I) * R(I <-- W) * normal_W
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); // s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int main(int argc, char **argv)
{
    // allocateMemory();
    for (int i = 0; i < 6; ++i)
    {
        transformTobeMapped[i] = 0;
    }

    ros::init(argc, argv, "laserMapping"); // 初始化ros节点,节点名为laserMapping
    ros::NodeHandle nh;

    // 从参数服务器读取参数值赋给变量
    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
    nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
    nh.param<string>("map_file_path", map_file_path, "");
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/repub_topic", repub_topic, "/livox/lidar_repub");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("cube_side_length", cube_len, 200);
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
    nh.param<double>("mapping/fov_degree", fov_deg, 180);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, LIVOX);
    nh.param<int>("preprocess/livox_type", p_pre->livox_type, LIVOX_CUS);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 1);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<vector<double>>("camera/camera_external", cam_ex, vector<double>());
    nh.param<vector<double>>("camera/camera_internal", cam_in, vector<double>());

    nh.param<float>("odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
    nh.param<float>("mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
    nh.param<float>("mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

    nh.param<float>("z_tollerance", z_tollerance, FLT_MAX);
    nh.param<float>("rotation_tollerance", rotation_tollerance, FLT_MAX);

    nh.param<int>("numberOfCores", numberOfCores, 2);
    nh.param<double>("mappingProcessInterval", mappingProcessInterval, 0.15);

    // save keyframes
    nh.param<float>("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 20.0);
    nh.param<float>("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
    nh.param<float>("surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    nh.param<float>("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

    // loop clousre
    nh.param<bool>("loopClosureEnableFlag", loopClosureEnableFlag, true);
    nh.param<float>("loopClosureFrequency", loopClosureFrequency, 1.0);
    nh.param<int>("surroundingKeyframeSize", surroundingKeyframeSize, 50);
    nh.param<float>("historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    nh.param<float>("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    nh.param<int>("historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<float>("historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

    // Visualization
    nh.param<float>("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
    nh.param<float>("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
    nh.param<float>("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

    // visual ikdtree map
    nh.param<bool>("visulize_IkdtreeMap", visulize_IkdtreeMap, false);

    // reconstruct ikdtree
    nh.param<bool>("recontructKdTree", recontructKdTree, false);

    // savMap
    nh.param<bool>("savePCD", savePCD, false);
    nh.param<std::string>("savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

    paramSetting(); // 设置相机的内参、相机到hap的外参

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    // downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    // ISAM2参数
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    int effect_feat_num = 0;                  // 有效的特征点数量
    int frame_num = 0;                        // 雷达总帧数
    double deltaT;                            // 平移增量
    double deltaR;                            // 旋转增量
    double aver_time_consu = 0;               // 每帧平均的处理总时间
    double aver_time_icp = 0;                 // 每帧中icp的平均时间
    double aver_time_match = 0;               // 每帧中匹配的平均时间
    double aver_time_incre = 0;               // 每帧中ikdtree增量处理的平均时间
    double aver_time_solve = 0;               // 每帧中计算的平均时间
    double aver_time_const_H_time = 0;        // 每帧中计算的平均时间(当H)
    bool flg_EKF_converged, EKF_stop_flg = 0; // 卡尔曼滤波收敛标志,卡尔曼滤波停止标志

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0); // 视场角度
    HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);              // 视场角度半值的cos值

    _featsArray.reset(new PointCloudXYZI()); // 特征点数组

    // 将数组point_selected_surf内元素的值全部设为true,数组point_selected_surf用于选择平面点
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    // 将数组res_last内元素的值全部设置为-1000.0f,数组res_last用于平面拟合中
    memset(res_last, -1000.0f, sizeof(res_last));
    // VoxelGrid滤波器参数,即进行滤波时的创建的体素边长为filter_size_surf_min
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    // VoxelGrid滤波器参数,即进行滤波时的创建的体素边长为filter_size_map_min
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    // 设置imu和lidar外参和imu参数等
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);        // p_imu为ImuProcess的智能指针
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));            // 角速度协方差
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));            // 加速度协方差
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov)); // 角速度bias协方差
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov)); // 加速度bias协方差

    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001); // 全部数组置0.001
    // 将函数地址传入kf对象中用于初始化,通过函数(h_dyn_share_in)同时计算测量(z)、估计测量(h)、偏微分矩阵(h_x,h_v)和噪声协方差(R)
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

    // ROS订阅器和发布器的定义和初始化
    ros::Subscriber sub_pcl = p_pre->lidar_type == LIVOX ? (p_pre->livox_type == LIVOX_CUS ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : nh.subscribe(repub_topic, 200000, livox_ros_cbk)) : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Subscriber subLivoxMsg = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100000, LivoxRepubCallback);
    ros::Subscriber subBox = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 100, BoxCallback);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, &imageCallback);

    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);           // world系下稠密点云
    ros::Publisher pubLaserCloudColor = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_color", 100000);    // world系下稠密彩色点云
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000); // imu(body)系下稠密点云
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);           // no used
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);                   // no used
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 1e00000);
    ros::Publisher pubPathUpdate = nh.advertise<nav_msgs::Path>("fast_lio_sam/path_update", 100000); // isam更新后的path

    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/keyframe_submap", 1);      // 发布局部关键帧map的特征点云
    pubOptimizedGlobalMap = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/map_global_optimized", 1); // 发布局部关键帧map的特征点云
    pubLidarPCL = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_repub", 100000);
    // 发布回环匹配关键帧局部map
    pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/icp_loop_closure_history_cloud", 1);
    // 发布当前关键帧经过回环优化后的位姿变换之后的特征点云
    pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
    // 发布回环边,rviz中表现为回环帧之间的连线
    pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/fast_lio_sam/mapping/loop_closure_constraints", 1);

    // saveMap发布地图保存服务
    srvSaveMap = nh.advertiseService("/save_map", &saveMapService);
    // savePose发布轨迹保存服务
    srvSavePose = nh.advertiseService("/save_pose", &savePoseService);
    // 回环检测线程
    std::thread loopthread(&loopClosureThread);

    // 中断处理函数,如果有中断信号(比如Ctrl+C),则执行第二个参数里面的SigHandle函数
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        // 如果有中断产生,则结束主循环
        if (flg_exit)
            break;
        ros::spinOnce(); // ROS消息回调处理函数

        // 将当前lidar数据及lidar扫描时间内对应的imu数据从缓存队列中取出,进行时间对齐,并保存到Measures中
        if (sync_packages(Measures))
        {
            // 第一帧lidar数据
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time; // 记录第一帧绝对时间
                p_imu->first_lidar_time = first_lidar_time; // 记录第一帧绝对时间
                flg_first_scan = false;
                continue;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();

            // 根据imu数据序列和lidar数据,向前传播纠正点云的畸变
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();                                               // 获取kf预测的全局状态
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I; // W下雷达lidar系的位置,W^p_L=W^p_I+W^R_I*I^t_L

            // 如果去畸变点云数据为空,则代表了激光雷达没有完成去畸变,此时还不能初始化成功
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            // 判断是否初始化完成,需要满足第一次扫描的时间和第一个点云时间的差值大于INIT_TIME
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            // 在拿到eskf前馈结果后,动态调整局部地图
            lasermap_fov_segment(); // 根据L在world系下的位置,重新确定局部地图的包围盒角点,移除远端的点

            downSizeFilterSurf.setInputCloud(feats_undistort); // 获得去畸变后的点云数据
            downSizeFilterSurf.filter(*feats_down_body);       // 滤波降采样后的点云数据
            t1 = omp_get_wtime();                              // 记录时间
            feats_down_size = feats_down_body->points.size();  // 当前帧降采样后点数

            // 建立ikdtree
            if (ikdtree.Root_Node == nullptr)
            {
                if (feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min); // 设置ikdtree的降采样参数
                    feats_down_world->resize(feats_down_size);         // 将下采样得到的地图点大小于imu(body)系大小一致
                    // 将降采样得到的地图点转换为world系下的点云
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points); // world系下对当前帧降采样后的点云,初始化ikdtree
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum(); // 获取ikdtree中的有效节点数,无效点就是被打了deleted标签的点
            kdtree_size_st = ikdtree.size();          // 获取ikdtree中的节点数

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            // lidar-->imu的外参,旋转矩阵转欧拉角
            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                     << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << endl;

            if (visulize_IkdtreeMap) // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);                             // 释放PCL_Storage的内存
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD); // 把树展平用于展示
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
                publish_map(pubLaserCloudMap);
            }

            pointSearchInd_surf.resize(feats_down_size); // 搜索索引
            Nearest_Points.resize(feats_down_size);      // 将降采样处理后的点云用于搜索最近
            int rematch_num = 0;
            bool nearest_search_en = true;

            t2 = omp_get_wtime();

            // 迭代状态估计
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time); // 迭代卡尔曼滤波
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I; // world系下L坐标
            geoQuat.x = state_point.rot.coeffs()[0];                                // world系下当前IMU的姿态四元数
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            getCurPose(state_point); // 更新transformTobeMapped
            // 后端
            saveKeyFramesAndFactor();
            // 更新因子图中所有变量节点的位姿,也就是所有历史关键帧的位姿,调整全局轨迹,重构ikdtree
            correctPoses();
            // 发布里程计
            publish_odometry(pubOdomAftMapped);
            // 向映射ikdtree添加特性点
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

            // 发布轨迹和点
            if (path_en)
            {
                publish_path(pubPath);
                publish_path_update(pubPathUpdate); // 发布经过isam2优化后的路径
                static int jjj = 0;
                jjj++;
                if (jjj % 100 == 0)
                {
                    publishGlobalMap(); // 发布局部点云特征地图
                }
            }
            if (scan_pub_en || pcd_save_en)
            {
                publish_frame_world(pubLaserCloudFull);        // 发布world系下的点云
                publish_frame_world_color(pubLaserCloudColor); // 发布world系下的彩色点云
            }
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body); // 发布imu(body)系下的点云

            // if(savePCD)  saveMap();

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;                         // 整个流程总时间
                s_plot2[time_log_counter] = feats_undistort->points.size(); // 特征点数量
                s_plot3[time_log_counter] = kdtree_incremental_time;        // ikdtree增量时间
                s_plot4[time_log_counter] = kdtree_search_time;             // ikdtree搜索耗时
                s_plot5[time_log_counter] = kdtree_delete_counter;          // ikdtree删除点数量
                s_plot6[time_log_counter] = kdtree_delete_time;             // ikdtree删除耗时
                s_plot7[time_log_counter] = kdtree_size_st;                 // ikdtree初始大小
                s_plot8[time_log_counter] = kdtree_size_end;                // ikdtree结束大小
                s_plot9[time_log_counter] = aver_time_consu;                // 平均消耗时间
                s_plot10[time_log_counter] = add_point_size;                // 添加点数量
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n", t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                         << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << " " << feats_undistort->points.size() << endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2, "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++)
        {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", T1[i], s_plot[i], int(s_plot2[i]), s_plot3[i], s_plot4[i], int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    startFlag = false;
    loopthread.join(); // 分离线程

    return 0;
}
