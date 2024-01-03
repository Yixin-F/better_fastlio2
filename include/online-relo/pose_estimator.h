#pragma once

#include "../common_lib.h"
#include "../src/preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include "../src/IMU_Processing.hpp"
#include "../multi-session/Incremental_mapping.hpp"
#include "../teaser-toolkit/fpfh_teaser.hpp"

#include "../mutexDeque.h"
#include "../tool_color_printf.h"
#include "../tictoc.hpp"

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

bool flg_exit = false;
mutex mtx_buffer;              
condition_variable sig_buffer; 
int scan_count = 0, scan_num = 0, publish_count = 0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0, timediff_lidar_wrt_imu = 0.0, lidar_end_time = 0.0, lidar_mean_scantime = 0.0, first_lidar_time = 0.0;
bool time_sync_en = false, timediff_set_flg = false;
bool lidar_pushed, flg_first_scan = true, flg_EKF_inited;
int kdtree_delete_counter = 0, feats_down_size = 0, kdtree_size_st = 0, add_point_size = 0;
double kdtree_delete_time = 0.0, kdtree_incremental_time = 0.0;
double filter_size_map_min = 0.1;

bool flg_EKF_converged, EKF_stop_flg = 0;
double res_mean_last = 0.05, total_residual = 0.0;
int effct_feat_num = 0;
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
bool point_selected_surf[100000] = {0};
float res_last[100000] = {0.0}; 

double epsi[23] = {0.001};
const int NUM_MAX_ITERATIONS = 4;

int cout_flg1 = 0, cout_flg2 = 0, cout_flg3 = 0, cout_flg4 = 0, cout_flg5 = 0;

std::string rootDir;
std::string pointCloudTopic;
std::string imuTopic;

double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;

std::vector<double> extrinT(3, 0.0);            
std::vector<double> extrinR(9, 0.0); 

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;               
V3D position_last(Zero3d);   
V3D Lidar_T_wrt_IMU(Zero3d); 
M3D Lidar_R_wrt_IMU(Eye3d);  

nav_msgs::Path path;                    
nav_msgs::Odometry odomAftMapped;         
geometry_msgs::Quaternion geoQuat;        
geometry_msgs::PoseStamped msg_body_pose; 

shared_ptr<Preprocess> p_pre(new Preprocess()); 
shared_ptr<ImuProcess> p_imu(new ImuProcess()); 

deque<pcl::PointCloud<PointType>::Ptr> lidar_buffer;      
deque<sensor_msgs::Imu::ConstPtr> imu_buffer; 
deque<double> time_buffer;   

pcl::PointCloud<PointType>::Ptr feats_undistort(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr feats_down_body(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr feats_down_world(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr featsFromMap(new pcl::PointCloud<PointType>());  
pcl::PointCloud<PointType>::Ptr normvec(new pcl::PointCloud<PointType>(100000, 1));
pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>(100000, 1)); 
pcl::PointCloud<PointType>::Ptr corr_normvect(new pcl::PointCloud<PointType>(100000, 1)); 

pcl::PointCloud<PointType>::Ptr reloCloud_diff(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr reloCloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr reloCloud_res(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr near_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr final_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr priorMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr priorPath(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr full_cloud(new pcl::PointCloud<PointType>());

vector<vector<int>> pointSearchInd_surf;     
vector<PointVector> Nearest_Points;

BoxPointType LocalMap_Points;     
bool Localmap_Initialized = false;
vector<BoxPointType> cub_needrm;  

const float MOV_THRESHOLD = 1.5f;
const float DET_RANGE = 260.0f;
const double cube_len = 200.0;
int process_increments = 0;

pcl::VoxelGrid<PointType> downSizeFilterSurf; 

float transformTobeMapped[6];

std::vector<int> pointSearchIndGlobalMap;
std::vector<float> pointSearchSqDisGlobalMap;
pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses(new pcl::KdTreeFLANN<PointType>());

// EKF inputs and output
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf; 
state_ikfom state_point;                         
vect3 pos_lid;   

KD_TREE ikdtree;

// measurement function
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0;

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif

    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];   
        PointType &point_world = feats_down_world->points[i]; 

        V3D p_body(point_body.x, point_body.y, point_body.z);                     
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos); 
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4) pabcd;                     
        point_selected_surf[i] = false; 
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            if (s > 0.9)
            {
                point_selected_surf[i] = true;   
                normvec->points[i].x = pabcd(0); 
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2; 
                res_last[i] = abs(pd2);         
            }
        }
    }

    effct_feat_num = 0; 

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i]; 
            corr_normvect->points[effct_feat_num] = normvec->points[i];       
            total_residual += res_last[i];      
            effct_feat_num++;                                             
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num; 
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); 
    ekfom_data.h.resize(effct_feat_num);               

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i]; 
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I; 
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this); 

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec); 
        V3D A(point_crossmat * C);           
        if (0)  // FIXME: extrinsic_est_en
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

bool imu_static = false;

class pose_estimator{
public:
    ros::NodeHandle nh;

    ros::Subscriber subCloud;
    ros::Subscriber subIMU;
    ros::Subscriber subPose;
    ros::Subscriber subSrv;
    ros::Publisher pubPriorMap;
    ros::Publisher pubPriorPath;
    ros::Publisher pubReloMap;
    ros::Publisher pubReloDiffMap;
    ros::Publisher pubReloResMap;
    ros::Publisher pubReloNearMap;
    ros::Publisher pubReloFinalMap;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubCurCloud;
    ros::Publisher pubPath;
    ros::Publisher pubIkdTree;
    ros::Publisher pubLaserCloudFull;
    // ros::Publisher pubConstraintEdge;

    PointTypePose initpose;
    PointTypePose finalpose;
    int reloIdx;
    Eigen::Isometry3f fromTeaser;
    bool initpose_flag = false;
    bool loc_flag = false;

    pose_estimator(); 
    ~pose_estimator(){}

    void cloudCBK();
    void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void imuCBK(const sensor_msgs::Imu::ConstPtr &msg_in);
    void poseCBK(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    bool getInitPose();
    bool useMannual = false;
    void mannualCBK(const std_msgs::Bool::ConstPtr &msg);

    void run();
    bool sync_packages(MeasureGroup &meas);
    void lasermap_fov_segment();
    void publish_map(const ros::Publisher &pubLaserCloudMap);
    void getCurPose(state_ikfom cur_state);
    void recontructIKdTree();
    // void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
    void pointBodyToWorld(PointType const *const pi, PointType *const po);
    void publish_odometry(const ros::Publisher &pubOdomAftMapped);
    void map_incremental();
    void publish_path(const ros::Publisher pubPath);
    void publish_frame_world(const ros::Publisher &pubLaserCloudFull);
    bool easyToRelo();

    void publishThread();

    template <typename T>
    void set_posestamp(T &out)
    {
        out.pose.position.x = state_point.pos(0);
        out.pose.position.y = state_point.pos(1);
        out.pose.position.z = state_point.pos(2);
        out.pose.orientation.x = geoQuat.x;
        out.pose.orientation.y = geoQuat.y;
        out.pose.orientation.z = geoQuat.z;
        out.pose.orientation.w = geoQuat.w;
    }

};