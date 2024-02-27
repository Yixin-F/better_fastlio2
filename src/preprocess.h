#include "common_lib.h"
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false) // 是否是有效值

enum LID_TYPE
{
  LIVOX = 1, 
  VELO16, 
  OUST64,
  RS
}; //{1, 2, 3, 4}

enum LIVOX_TYPE
{
  LIVOX_CUS = 1,
  LIVOX_ROS
};

enum TIME_UNIT
{
  SEC = 0, 
  MS = 1, 
  US = 2, 
  NS = 3
};

enum Feature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
}; // 未判断，可能平面，平面，跳跃边，平面交接边,细线

enum Surround
{
  Prev,
  Next
};

enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
}; // 未判断，接近0度，接近180度，接近远端，接近近端

// 用于记录每个点的距离、角度、特征种类等属性
struct orgtype
{
  double range;     // 平面距离
  double dista;     // 与后一个点的间距平方
  double angle[2];  // cos(当前点指向前一点或后一点的向量, ray)
  double intersect; // 当前点与相邻两点的夹角cos值
  E_jump edj[2];    // 点前后两个方向的edge_jump类型
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace livox_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型,xyz+padding,float padding用于填补位数,以满足存储对齐要求
    float intensity;                // Reflectivity
    uint8_t tag;                    // Livox point tag
    uint8_t line;                   // Laser line id
    uint8_t reflectivity;           // reflectivity, 0~255
    uint32_t offset_time;           // offset time relative to the base time
    PCL_ADD_RGB;                    // RGB
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
}
// 注册livox_ros的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)(std::uint8_t, tag, tag)(std::uint8_t, line, line)(std::uint8_t, reflectivity, reflectivity)(std::uint32_t, offset_time, offset_time)(float, rgb, rgb))

namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型,xyz+padding,float padding用于填补位数,以满足存储对齐要求
    float intensity;                // 强度
    float time;                     // 时间
    uint16_t ring;                  // 线束
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))

namespace ouster_ros 
{
  struct EIGEN_ALIGN16 Point
  {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)(std::uint16_t, reflectivity, reflectivity)(std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)(std::uint32_t, range, range)
)

namespace rslidar_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace rslidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, curvature)(float, time, normal_x)(uint16_t, ring, ring))

/**
 * @brief Preproscess类:用于对激光雷达点云数据进行预处理
 *
 */
class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();  // 构造函数
  ~Preprocess(); // 析构函数

  // 对Livox自定义Msg格式的激光雷达数据进行处理
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, pcl::PointCloud<PointType>::Ptr &pcl_out);
  // 对ros的Msg格式的激光雷达数据进行处理
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType>::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  pcl::PointCloud<PointType> pl_full, pl_corn, pl_surf;                         // 全部点、边缘点、平面点
  pcl::PointCloud<PointType> pl_buff[128];                                      // maximum 128 line lidar
  vector<orgtype> typess[128];                                      // maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, livox_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit; // 雷达类型、livox数据类型,采样间隔、扫描线数、扫描频率
  double blind;                                                     // 最小距离阈值(盲区),小于此阈值不计算特征
  bool feature_enabled, given_offset_time;                          // 是否进行特征提取、是否进行时间偏移
  ros::Publisher pub_full, pub_surf, pub_corn;

private:
  void livox_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg); // 用于对Livox激光雷达数据进行处理
  void livoxros_handler(const sensor_msgs::PointCloud2::ConstPtr &msg); 
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg); // 用于对velodyne激光雷达数据进行处理
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types);        // 当前扫描线点云，扫描点属性
  void pub_func(pcl::PointCloud<PointType> &pl, const ros::Time &ct);
  int plane_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size; // 计算平面特征时需要的最少局部点数
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
