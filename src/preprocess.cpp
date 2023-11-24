#include <omp.h>
#include "preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

/**
 * @brief 构造函数
 *
 */
Preprocess::Preprocess()
    : feature_enabled(0), lidar_type(LIVOX), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS = 6;    // 激光雷达的线束
  SCAN_RATE = 10; // 扫描频率
  group_size = 8;
  disA = 0.01;
  disA = 0.1;
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false; // 是否进行时间偏移

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

/**
 * @brief 析构函数
 *
 */
Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

/**
 * @brief Livox激光雷达点云预处理函数
 *
 * @param msg livox激光雷达点云数据,格式为livox_ros_driver::CustomMsg
 * @param pcl_out 输出处理后的点云数据,格式为pcl::PointCloud<pcl::PointXYZINormal>
 */
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  livox_handler(msg);
  *pcl_out = pl_surf; // 储存间隔采样点
}

/**
 * @brief velodyne激光雷达点云预处理函数
 *
 * @param msg velodyne激光雷达点云数据,格式为sensor_msgs::PointCloud2
 * @param pcl_out 输出处理后的点云数据,格式为pcl::PointCloud<pcl::PointXYZINormal>
 */
void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (lidar_type)
  {
  case VELO16:
    velodyne_handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

/**
 * @brief livox雷达数据预处理
 *
 */
void Preprocess::livox_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  // 清除之前的点云缓存
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  int plsize = msg->point_num; // 一帧点云中的点数

  double t1 = omp_get_wtime();

  // 分配空间
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  // 清空缓存内的点云并预留足够空间
  for (int i = 0; i < N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize); // 预分配每一个scan保存的点数
  }
  uint valid_num = 0; // 有效的点数

  // 特征提取
  if (feature_enabled)
  {
    // 按照line划分点云
    for (uint i = 1; i < plsize; i++)
    {
      if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points

        bool is_new = false;
        // 与前一点间距太小则忽略该点，间距太小不利于特征提取
        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count++;
    double t0 = omp_get_wtime();
    for (int j = 0; j < N_SCANS; j++)
    {
      if (pl_buff[j].size() <= 5)
        continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j]; // 当前line的点云
      plsize = pl.size();
      vector<orgtype> &types = typess[j]; // 用于记录每个点的距离、角度、特征种类等属性
      types.clear();
      types.resize(plsize);
      plsize--;
      for (uint i = 0; i < plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      give_feature(pl, types); // 通过点和每个点的属性，计算特征
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  else
  {
    // 不进行特征处理,分别对每个点进行处理
    for (uint i = 1; i < plsize; i++)
    {
      // 只取线数在0~N_SCANS内并且回波次序(tag标签bit5和bit4)为0或者1的点云
      if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num++; // 满足回波序列01和00的计数有效的点数
        // 等间隔降采样,选取降采样的点
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg->points[i].x;                                    // 点的x轴坐标
          pl_full[i].y = msg->points[i].y;                                    // 点的y轴坐标
          pl_full[i].z = msg->points[i].z;                                    // 点的z轴坐标
          pl_full[i].intensity = msg->points[i].reflectivity;                 // 点的强度
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // 使用曲率作为每个激光点的时间？

          // 间距太小不利于特征提取,只有当当前点和上一点的间距>1e-7,并且在最小距离阈值之外,才认为是有用点,加入到pl_surf队列中
          if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7) && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            pl_surf.push_back(pl_full[i]); // 将当前点加入到对应line的pl_fuff队列中
          }
        }
      }
    }
  }
}

/**
 * @brief velodyne雷达数据预处理
 *
 */
void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  /* 清除之前的点云缓存 */
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);     // fromROSMsg读数据
  int plsize = pl_orig.points.size(); // 一帧点云中的点数
  if (plsize == 0)
    return;
  /* 分配空间 */
  pl_surf.reserve(plsize);

  /* 这些变量仅在没有给出点时间戳时才有效 */
  double omega_l = 0.361 * SCAN_RATE;         // 扫描角速度
  std::vector<bool> is_first(N_SCANS, true);  // 是否是第一个点
  std::vector<double> yaw_fp(N_SCANS, 0.0);   // 第一个扫描点的偏航角
  std::vector<float> yaw_last(N_SCANS, 0.0);  // 最后一个扫描点的偏航角
  std::vector<float> time_last(N_SCANS, 0.0); // 最后偏移(offset)时间

  if (pl_orig.points[plsize - 1].time > 0) // todo check pl_orig.points[plsize - 1].time
  {
    given_offset_time = true;
  }
  else
  {
    given_offset_time = false;
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578; // 记录第一个点(index 0)的偏航角yaw, to degree
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring; // 第一个点(index 0)的线束layer序号
    /* 倒序遍历,找到与第一个点相同线束的最后一个点 */
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578; // 与第一个点相同layer的最后一个点的偏航角yaw
        break;
      }
    }
  }

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    // 计算时间、转换点云格式为PointType，正序遍历
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = pl_orig.points[i].ring;
      if (layer >= N_SCANS)
        continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms

      if (!given_offset_time)
      {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957; // 但前点yaw, to degree
        if (is_first[layer])                                        // 如果当前点是其对应layer的第一个点
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle; // 记录为当前点对应layer的起始yaw
          is_first[layer] = false;
          added_pt.curvature = 0.0;    // 当前点curvature（时间）置零
          yaw_last[layer] = yaw_angle; // 暂时记录为当前点对应layer的结束yaw
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;           // 记录当前layer最后一个点的yaw
        time_last[layer] = added_pt.curvature; // 记录当前layer最后一个点的时间
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j]; // points_line
      int linesize = pl.size();
      if (linesize < 2)
        continue;
      vector<orgtype> &types = typess[j]; // 用于记录当前扫描线上每个点的参数
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    // 不进行特征提取
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0; // curvature unit: ms

      if (!given_offset_time)
      {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
}

/**
 * @brief livoxros雷达数据预处理
 *
 */
void Preprocess::livoxros_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  /* 清除之前的点云缓存 */
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<livox_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);     // fromROSMsg读数据
  int plsize = pl_orig.points.size(); // 一帧点云中的点数
  if (plsize == 0)
    return;

  // 分配空间
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  // 清空缓存内的点云并预留足够空间
  for (int i = 0; i < N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize); // 预分配每一个scan保存的点数
  }
  uint valid_num = 0; // 有效的点数

  // 不进行特征处理,分别对每个点进行处理
  for (uint i = 1; i < plsize; i++)
  {
    // 只取线数在0~N_SCANS内并且回波次序(tag标签bit5和bit4)为0或者1的点云
    if ((pl_orig.points[i].line < N_SCANS) && ((pl_orig.points[i].tag & 0x30) == 0x10 || (pl_orig.points[i].tag & 0x30) == 0x00))
    {
      valid_num++; // 满足回波序列01和00的计数有效的点数
      // 等间隔降采样,选取降采样的点
      if (valid_num % point_filter_num == 0)
      {
        pl_full[i].x = pl_orig.points[i].x;                 // 点的x轴坐标
        pl_full[i].y = pl_orig.points[i].y;                 // 点的y轴坐标
        pl_full[i].z = pl_orig.points[i].z;                 // 点的z轴坐标
        pl_full[i].intensity = pl_orig.points[i].intensity; // 点的强度

        // 间距太小不利于特征提取,只有当当前点和上一点的间距>1e-7,并且在最小距离阈值之外,才认为是有用点,加入到pl_surf队列中
        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7) && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
        {
          pl_surf.push_back(pl_full[i]); // 将当前点加入到对应line的pl_fuff队列中
        }
      }
    }
  }
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size(); // 输入扫描线的原始点数
  int plsize2;            // 用于估计特征的点数
  if (plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;
  // 更新head为第一个大于blind范围的点索引
  while (types[head].range < blind)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0; // 本layer除group_size（局部平面点）之外的剩余点数

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0;
  uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  // 第一个大于blind范围的点索引开始遍历,判断平面特征
  for (uint i = head; i < plsize2; i++)
  {
    if (types[i].range < blind) // blind范围内直接跳过
    {
      continue;
    }

    i2 = i; // i2记录当前点索引

    // 输出
    // i_nex 局部最后最后一个点的索引
    // curr_direct 归一化后的，局部范围最后一个点与第一个点的坐标差值，即向量（i_cur --> i_nex)
    // return 1 正常退出， 0 中途break，curr_direct置零
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if (plane_type == 1) // plane_judge正常退出
    {
      for (uint j = i; j <= i_nex; j++)
      {
        if (j != i && j != i_nex)
        {
          types[j].ftype = Real_Plane; // 局部范围内部点定义为real平面点
        }
        else
        {
          types[j].ftype = Poss_Plane; // 局部范围边界点定义可能平面
        }
      }

      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if (last_state == 1 && last_direct.norm() > 0.1) // 根据上一状态（局部是平面）和长度（向量模长），决定起始点的类型
      {
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707) // 平面夹角30度
        {
          types[i].ftype = Edge_Plane; //平面交接的边
        }
        else
        {
          types[i].ftype = Real_Plane; //平面
        }
      }

      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  // 从head+3向后遍历，判断非平面点是不是edge，以及判断edge的类型
  for (uint i = head + 3; i < plsize2; i++)
  {
    if (types[i].range < blind || types[i].ftype >= Real_Plane) // 距离太近或这已经判断为edge或者平面
    {
      continue;
    }

    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) // 前两个点间距太小
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z); // sensor到当前点ray
    Eigen::Vector3d vecs[2];                          // 从当前点指向前后相邻点的两个向量

    for (int j = 0; j < 2; j++) // 计算当前点与前一点、后一点的向量，判断当前点前后两个方向的edge属性
    {
      int m = -1;
      if (j == 1)
      {
        m = 1;
      }

      if (types[i + m].range < blind) // 索引i的前一个点（m = -1) 或者后一个点（m = 1) ，距离很小
      {
        if (types[i].range > inf_bound) // 根据当前点平面距离，判断edge jump属性
        {
          types[i].edj[j] = Nr_inf; // 靠近远端
        }
        else
        {
          types[i].edj[j] = Nr_blind; // 靠近近端
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z); // sensor指向 当前点的前一点或后一点的向量
      vecs[j] = vecs[j] - vec_a;                                        // 当前点指向前一点或后一点的向量

      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm(); // cos(当前点指向前一点或后一点的向量, ray)
      if (types[i].angle[j] < jump_up_limit)                                  // jump_up_limit 默认cos170度
      {
        types[i].edj[j] = Nr_180; // ray与当前点指向端点外一点的向量 夹角接近180度
      }
      else if (types[i].angle[j] > jump_down_limit) // jump_down_limit 默认8度
      {
        types[i].edj[j] = Nr_zero; // ray与当前点指向端点外一点的向量 夹角接近0度
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm(); // 当前点与相邻两点的夹角cos值
    // 根据前端点edgejump类型，后端点edgejump类型，与后一点间距，与前一点间距的4倍，判断edge的类型
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      if (edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      if (edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      if (types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize - 1;
  double ratio;                             // 点的前后间距比，大：小
  for (uint i = head + 1; i < plsize2; i++) // 对为分类点，判断特征类型
  {
    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind)
    {
      continue;
    }

    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8)
    {
      continue;
    }

    if (types[i].ftype == Nor)
    {
      if (types[i - 1].dista > types[i].dista)
      {
        ratio = types[i - 1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i - 1].dista;
      }

      // smallp_intersect：默认172.5度余弦值
      // smallp_intersect：默认1.2
      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio) // 前后夹角大、间距接近，认为是真平面
      {
        if (types[i - 1].ftype == Nor)
        {
          types[i - 1].ftype = Real_Plane;
        }
        if (types[i + 1].ftype == Nor)
        {
          types[i + 1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1; // 用于记录上一面特征的索引
  // 从头遍历，
  for (uint j = head; j < plsize; j++)
  {
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
      if (last_surface == -1)
      {
        last_surface = j;
      }

      if (j == uint(last_surface + point_filter_num - 1)) // 按索引间距取平面点索引
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap); // 记录

        last_surface = -1;
      }
    }
    else
    {
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
      {
        pl_corn.push_back(pl[j]); // 记录edge特征
      }
      if (last_surface != -1)
      {
        // edge特征和上一个surface特征之间取所有点均值，作为面特征
        // todo 检查超限
        PointType ap;
        for (uint k = last_surface; k < j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

// （line点云，点属性， 当前点索引，当前点索引，当前方向）
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA * types[i_cur].range + disB; // disB?? 用于限制特征点计算时，局部范围的阈值
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr; // 与后一个点的距离，（line内相邻有效点间距）
  disarr.reserve(20);

  // group_size(计算特征需要的点数，默认8）， 索引范围内遍历
  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
  {
    if (types[i_nex].range < blind) // group_size存在点的平面距离小于blind， 方向置零，直接返回2
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista); // 保存与后一个点的距离
  }

  for (;;) // 继续向后遍历，保存group_dis范围以内，与后一个点的间距,记录最后一个局部点索引为i_nex
  {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size()))
      break;

    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz; // 最后一个点与当前点的距离平方和
    if (two_dis >= group_dis)              // 超出局部间距距离范围约束时，跳出
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0; // 记录局部范围内，叉乘的最大模长，即局部范围内最大平行四边形面积
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++) // 局部范围内，从当前点向后遍历，i_nex为局部范围内索引最大值
  {
    if ((j >= pl.size()) || (i_cur >= pl.size()))
      break;
    // 计算向量（i_cur --> j）
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;
    // vx,vy,vz为局部范围最后一个点与第一个点的坐标差值，即向量（i_cur --> i_nex)，以下为叉乘
    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]; // 叉乘的模长，平行四边形面积
    if (lw > leng_wid)
    {
      leng_wid = lw;
    }
  }

  if ((two_dis * two_dis / leng_wid) < p2l_ratio) // 最大距离平方和的乘积与最大平行四边形面积比，判断比例，返回0
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++) // 排序，disarr按从大到小，leng_wid为最小的相邻点间隔距离
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid; // j、k互换
      }
    }
  }

  if (disarr[disarr.size() - 2] < 1e-16) // 第二小的点间距
  {
    curr_direct.setZero();
    return 0;
  }

  if (lidar_type == LIVOX)
  {
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize - 2]; // 最大最小间距比
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  curr_direct << vx, vy, vz; // vx,vy,vz为局部范围最后一个点与第一个点的坐标差值，即向量（i_cur --> i_nex)
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if (nor_dir == 0)
  {
    if (types[i - 1].range < blind || types[i - 2].range < blind)
    {
      return false;
    }
  }
  else if (nor_dir == 1)
  {
    if (types[i + 1].range < blind || types[i + 2].range < blind)
    {
      return false;
    }
  }
  // d1,d2为向前或向后的相邻点间距，通过nor_dir控制方向
  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  double d;

  // d1为较大间距
  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  // 间距大于edgea（2）倍 或 间距差大于edgeb（0.1）
  if (d1 > edgea * d2 || (d1 - d2) > edgeb)
  {
    return false;
  }

  return true;
}
