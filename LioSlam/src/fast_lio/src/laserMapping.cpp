#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp> // Replaced ros/ros.h
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp> // Updated header
#include <nav_msgs/msg/path.hpp> // Updated header
#include <visualization_msgs/msg/marker.hpp> // Updated header
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp> // Updated header
#include <sensor_msgs/msg/imu.hpp> // Added explicitly
#include <tf2/LinearMath/Quaternion.h> // TF2
#include <tf2_ros/transform_broadcaster.h> // TF2
#include <geometry_msgs/msg/vector3.hpp> // Updated header
#include <geometry_msgs/msg/transform_stamped.hpp> // Added for TF2
#include <livox_ros_driver2/msg/custom_msg.hpp> // Assumed ROS2 driver package
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <voxel_map/voxel_map.h>
#include "FMT_Relocalizer.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include "DebugLogger.hpp"
#include "UniversalLogger.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>  // 通常也需要这个
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Eigenvalues>
// [新增] 用于通知底层 EKF 进行 ZUPT 虚拟量测
bool is_zupt_active = false;
// [新增] 一致性检测缓冲区
struct RelocCandidate {
    double time;
    Eigen::Vector3d pos;
    double yaw;
};
std::deque<RelocCandidate> reloc_buffer; // 滑动窗口
const int CONSISTENCY_COUNT = 3;         // 需要连续 3 帧一致才采纳
const double MAX_JUMP_DIST = 0.5;        // 帧间位移允许的最大误差 (米)
// 定义一个结构体存储待更新的数据
// [修改] 结构体定义
struct MapUpdatePacket {
    double timestamp;
    // 不要存 PointXYZINormal，要存 pointWithVar
    // 因为你的 UpdateVoxelMap 需要协方差矩阵
    std::vector<pointWithVar> cached_pv_list; 
    bool skip_map_update;
};

// 定义延迟队列
std::deque<MapUpdatePacket> map_update_queue;
double boot_start_time = -1.0;     // 系统启动时间
const double BOOT_COOLDOWN = 3.0;  // 启动保护期 (秒)，给 EKF 3秒时间初始化重力
// 定义延迟时间 (比如 2.0 秒)
// 这个时间必须大于重定位算法的平均运行时间 (0.5s) + 线程调度时间
const double MAP_UPDATE_DELAY = 1.3;
double system_lost_time = 0.0;
// 在类定义中添加成员变量
std::mutex reloc_mutex;          // 保护共享数据
std::thread reloc_thread;        // 后台线程
const double RELOC_COOLDOWN = 1.0;
bool has_new_correction = false; // 标志位：是否有新的修正结果
Eigen::Vector3d correction_pos;  // 存储修正量
double correction_yaw;           // 存储修正量
VoxelMapManagerPtr voxelmap_manager;
FMTRelocalizer relocalizer;
bool need_relocalize = false; // 可以通过 ROS Topic 回调修改此变量
// Global Node Pointer (Preserved as requested)
std::shared_ptr<rclcpp::Node> node_ptr;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
VoxelMapConfig voxel_config;
#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables (Global - Preserved) ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = false, path_en = true;
// 在 laserMapping.cpp 的全局区域添加：
    // ================= [全局变量] 启动保护计数器 =================
    // 防止程序刚启动时 P 阵未收敛导致的误触发
    static int startup_protection_counter = 0;
    const int PROTECTION_FRAMES = 50; // 保护前 50 帧 (约 5 秒)
std::vector<pointWithVar> pv_list;      // 定义 pv_list
std::vector<PointToPlane> ptpl_list;    // 定义 ptpl_list
/**************************/
V3D extT;
M3D extR;
float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;
bool lidar_map_inited = false;
mutex mtx_buffer;
condition_variable sig_buffer;
Eigen::Vector3d n_up;
string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;
double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
int  frame_num = 0;
vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer; 

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());
    // ================= [双坐标系核心全局变量] =================
    Eigen::Vector3d t_map_odom_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_map_odom_ = Eigen::Quaterniond::Identity();
    std::mutex drift_mutex_;
    // =======================================================
    // ================= [新增：重定位快照全局变量] =================
Eigen::Vector3d map_center_snapshot_ = Eigen::Vector3d::Zero(); // 记录生成地图时的中心
Eigen::Vector3d odom_pos_snapshot_   = Eigen::Vector3d::Zero(); // 记录当时的里程计位置
double          odom_yaw_snapshot_   = 0.0;                     // 记录当时的里程计Yaw
// ==========================================================
// Global Functions (Preserved)
void SigHandle(int sig)
{
    flg_exit = true;
    RCLCPP_WARN(node_ptr->get_logger(), "catch sig %d", sig);
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); 
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); 
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); 
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
}
bool Localmap_Initialized = false;
BoxPointType LocalMap_Points;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    double msg_time_sec = rclcpp::Time(msg->header.stamp).seconds();

    if (msg_time_sec < last_timestamp_lidar)
    {
        RCLCPP_ERROR(node_ptr->get_logger(), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg_time_sec);
    last_timestamp_lidar = msg_time_sec;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu_g = 0.0; // Renamed slightly to distinguish if needed
bool   timediff_set_flg_g = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    double msg_time_sec = rclcpp::Time(msg->header.stamp).seconds();

    if (msg_time_sec < last_timestamp_lidar)
    {
        RCLCPP_ERROR(node_ptr->get_logger(), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg_time_sec;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg_g && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg_g = true;
        timediff_lidar_wrt_imu_g = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu_g);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in) 
{
    publish_count ++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    double msg_time_sec = rclcpp::Time(msg_in->header.stamp).seconds();
    
    msg->header.stamp = rclcpp::Time(static_cast<int64_t>((msg_time_sec - time_diff_lidar_to_imu) * 1e9));
    
    if (abs(timediff_lidar_wrt_imu_g) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        rclcpp::Time(static_cast<int64_t>((timediff_lidar_wrt_imu_g + msg_time_sec) * 1e9));
    }

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        RCLCPP_WARN(node_ptr->get_logger(), "imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            RCLCPP_WARN(node_ptr->get_logger(), "Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
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

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudMap);
}
template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
    
    // 恢复：直接使用内置函数赋值位姿，不进行任何数学转换
    set_posestamp(odomAftMapped.pose);
    
    // 立即发布 Odom
    pubOdomAftMapped->publish(odomAftMapped);
    
    // 恢复：协方差赋值逻辑（保持不变）
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    // 恢复：TF 发布逻辑，直接从刚刚赋好值的 odomAftMapped 里面提取数据
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = odomAftMapped.header.stamp;
    transform_stamped.header.frame_id = "camera_init";
    transform_stamped.child_frame_id = "body";
    
    transform_stamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform_stamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform_stamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
    
    transform_stamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transform_stamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;

    tf_broadcaster->sendTransform(transform_stamped);
}
void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}
// 确保引入了VoxelMap相关的结构体定义，例如 pointWithVar, pointWithCov 等
// 假设全局变量或成员变量 voxel_map_manager 已经初始化完毕

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    // 获取当前帧特征点数量
    int feats_down_size = feats_down_body->size();
    
    // 预留空间
    pv_list.clear();
    pv_list.resize(feats_down_size);
    ptpl_list.clear();

    // 1. 准备数据：计算Body系协方差并传播到World系
    M3D rot_cur = (s.rot * s.offset_R_L_I).toRotationMatrix();
    V3D pos_cur = s.rot * s.offset_T_L_I + s.pos;
    
    double dept_err = 0.05; // 示例值，请替换为 config_setting_.dept_err_
    double beam_err = 0.1;  // 示例值，请替换为 config_setting_.beam_err_

    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body_pcl = feats_down_body->points[i];
        V3D point_body(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
        
        // 1.1 计算 Body 系下的测量协方差
        double range = point_body.norm();
        double range_var = dept_err * dept_err;
        double angle_var = (beam_err * M_PI / 180.0) * (beam_err * M_PI / 180.0);
        
        M3D direction_var;
        direction_var.setZero();
        direction_var(2, 2) = range_var; // Z轴方向为径向
        direction_var(0, 0) = angle_var * range * range;
        direction_var(1, 1) = angle_var * range * range;
        
        // 构建旋转矩阵将方差转到点云方向
        V3D norm_vec(0, 0, 1);
        V3D point_dir = point_body / range;
        V3D axis = norm_vec.cross(point_dir);
        double angle = acos(norm_vec.dot(point_dir));
        M3D R_point;
        if (axis.norm() < 1e-4) R_point.setIdentity();
        else R_point = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
        
        M3D cov_body = R_point * direction_var * R_point.transpose();

        // 1.2 转换点和协方差到 World 系
        V3D point_world = rot_cur * point_body + pos_cur;
        M3D cov_world = rot_cur * cov_body * rot_cur.transpose();

        // 1.3 填充 pv_list
        pointWithVar &pv = pv_list[i];
        pv.point_b = point_body;
        pv.point_w = point_world;
        pv.var = cov_world;      
        pv.body_var = cov_body;  
    }

    // 2. VoxelMap 核心：构建残差列表
    voxelmap_manager->BuildResidualListOMP(pv_list, ptpl_list);

    int effct_feat_num = ptpl_list.size();
    
    // 3. 构建 Jacobian (H) 和 残差 (h)
    if (effct_feat_num < 10) 
    {
        ekfom_data.valid = false;
        return;
    }
    ekfom_data.valid = true;

    // 【极其关键】雷达匹配的雅可比矩阵必须死死锁在 12 维！
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12); 
    ekfom_data.h.resize(effct_feat_num);

    double total_residual = 0.0;
    double max_weight_found = 0.0;
    double max_measure_noise = 0.0;
    double min_measure_noise = 1e6;
    // --- 点云遍历循环开始 ---
    for (int i = 0; i < effct_feat_num; i++)
    {
        auto &ptpl = ptpl_list[i]; 
        
        V3D norm_vec = ptpl.normal_;
        V3D point_body = ptpl.point_b_;
        
        V3D point_imu = s.offset_R_L_I * point_body + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_imu);

        // 【关键偏导数公式修正】
        V3D C = s.rot.conjugate() * norm_vec;       // 这是 外参平移 (ExtT) 的偏导数
        V3D A = point_crossmat * C;                 // 这是 姿态旋转 (Rot) 的偏导数

        V3D B(0,0,0);
        if (extrinsic_est_en)
        {
             M3D point_be_crossmat;
             point_be_crossmat << SKEW_SYM_MATRX(point_body);
             B = point_be_crossmat * s.offset_R_L_I.conjugate() * C; // 这是 外参旋转 (ExtR) 的偏导数
        }

        M3D point_world_cov = rot_cur * ptpl.body_cov_ * rot_cur.transpose(); 
        double measure_noise = ptpl.normal_.transpose() * point_world_cov * ptpl.normal_;
        max_measure_noise = std::max(max_measure_noise, measure_noise);
        min_measure_noise = std::min(min_measure_noise, measure_noise);
        double weight = 1.0 / (0.05 + measure_noise); 
        double sqrt_weight = sqrt(weight);

        // ================== [核心修改：严格对齐 12 维状态向量顺序] ==================
        // 状态向量定义顺序: [Pos(0~2), Rot(3~5), ExtR(6~8), ExtT(9~11)]
        ekfom_data.h_x.block<1, 12>(i, 0) << 
            // 0~2: Pos (位置的偏导数是法向量 norm_vec)
            norm_vec.x() * sqrt_weight, norm_vec.y() * sqrt_weight, norm_vec.z() * sqrt_weight, 
            
            // 3~5: Rot (旋转的偏导数是 A)
            A.x() * sqrt_weight, A.y() * sqrt_weight, A.z() * sqrt_weight,
            
            // 6~8: ExtR 外参旋转 (偏导数必须是 B！)
            (extrinsic_est_en ? B.x() * sqrt_weight : 0.0), 
            (extrinsic_est_en ? B.y() * sqrt_weight : 0.0), 
            (extrinsic_est_en ? B.z() * sqrt_weight : 0.0),
            
            // 9~11: ExtT 外参平移 (偏导数必须是 C！)
            (extrinsic_est_en ? C.x() * sqrt_weight : 0.0), 
            (extrinsic_est_en ? C.y() * sqrt_weight : 0.0), 
            (extrinsic_est_en ? C.z() * sqrt_weight : 0.0);

        // 填充残差 (h)
        ekfom_data.h(i) = -ptpl.dis_to_plane_ * sqrt_weight;

        // 统计残差总和必须放在这里
        total_residual += fabs(ptpl.dis_to_plane_);
        
    } // --- 点云遍历循环结束 ---

    // 统计和日志部分
    double mean_resid = total_residual / effct_feat_num;
    // --- 点云遍历循环结束 ---
    // ...

    // 【新增探针 1】：监控送入 IKFoM 的观测矩阵极限值
    if (effct_feat_num > 0) {
        // 1. 提取雅可比矩阵 (H_x) 中的最大绝对值
        // 如果这个值极大，说明偏导数算爆了（可能是协方差或 S2 投影导致）
        double max_H_val = ekfom_data.h_x.cwiseAbs().maxCoeff();
        
        // 2. 提取残差向量 (h) 中的最大绝对值
        // 如果残差极大，说明点云匹配完全错位，或者权重白化公式异常
        double max_res_val = ekfom_data.h.cwiseAbs().maxCoeff();

        // 3. 记录白化权重本身（复用循环里的局部变量逻辑，找出最大的权重）
        // (注：你需要在这个 for 循环外定一个 double max_sqrt_weight = 0.0; 在循环内 max_sqrt_weight = std::max(max_sqrt_weight, sqrt_weight);)

        ULOG_PLOT("IKFoM_Input", "Max_Jacobian_Hx", max_H_val);
        ULOG_PLOT("IKFoM_Input", "Max_Residual_h", max_res_val);
        
        // 可选：看看有效点数是否突然骤降
        ULOG_PLOT("IKFoM_Input", "Valid_Points", (double)effct_feat_num);
    }
    // 【新增探针上报逻辑】
    if (effct_feat_num > 0) {
        // 1. 监控雅可比矩阵的绝对峰值 (决定了 K 增益的量级)
        double max_H_val = ekfom_data.h_x.cwiseAbs().maxCoeff();
        ULOG_PLOT("IKFoM_Input", "Max_Jacobian_Hx", max_H_val);

        // 2. 监控经过白化处理后的最终残差峰值 (决定了位姿拉扯的幅度)
        double max_res_val = ekfom_data.h.cwiseAbs().maxCoeff();
        ULOG_PLOT("IKFoM_Input", "Max_Residual_h", max_res_val);

        // 3. 监控白化权重 (重点！如果这个值 > 31.6，说明 0.001 的基底噪声可能太小了)
        // 31.6 = sqrt(1 / 0.001)
        ULOG_PLOT("IKFoM_Weight", "Max_Sqrt_Weight", max_weight_found);

        // 4. 监控测量的几何噪声 (反映点云在法向量方向上的扩散程度)
        ULOG_PLOT("IKFoM_Weight", "Min_Meas_Noise", min_measure_noise);
        ULOG_PLOT("IKFoM_Weight", "Max_Meas_Noise", max_measure_noise);

        // 5. 有效点数 (反映 ICP 匹配质量)
        ULOG_PLOT("IKFoM_Input", "Valid_Points", (double)effct_feat_num);
    }

}// [新增] 用于存储历史帧

struct FrameData {
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_body; // 车身系点云
    state_ikfom state; // 当时的位姿
};
// 状态量在误差协方差矩阵 P 中的起始索引 (严格对应 state_ikfom 的定义)
namespace StateIdx {
    constexpr int POS    = 0;
    constexpr int ROT    = 3;
    constexpr int EXT_R  = 6;
    constexpr int EXT_T  = 9;
    constexpr int VEL    = 12;
    constexpr int BG     = 15;
    constexpr int BA     = 18;
    constexpr int GRAV   = 21;
}
class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode() : Node("laserMapping")
    {   
        LOG_INIT("/home/robotlab/fast_lio_debug.csv");
        ULOG_INIT("/home/robotlab/universal_debug.csv");
        // Declare and get parameters
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200);
        this->declare_parameter<float>("mapping.det_range", 300.f);
        this->declare_parameter<double>("mapping.fov_degree", 180);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", false);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->get_parameter("publish.path_en", path_en);
        this->get_parameter("publish.scan_publish_en", scan_pub_en);
        this->get_parameter("publish.dense_publish_en", dense_pub_en);
        this->get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en);
        this->get_parameter("max_iteration", NUM_MAX_ITERATIONS);
        this->get_parameter("map_file_path", map_file_path);
        this->get_parameter("common.lid_topic", lid_topic);
        this->get_parameter("common.imu_topic", imu_topic);
        this->get_parameter("common.time_sync_en", time_sync_en);
        this->get_parameter("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu);
        this->get_parameter("filter_size_corner", filter_size_corner_min);
        this->get_parameter("filter_size_surf", filter_size_surf_min);
        this->get_parameter("filter_size_map", filter_size_map_min);
        this->get_parameter("cube_side_length", cube_len);
        this->get_parameter("mapping.det_range", DET_RANGE);
        this->get_parameter("mapping.fov_degree", fov_deg);
        this->get_parameter("mapping.gyr_cov", gyr_cov);
        this->get_parameter("mapping.acc_cov", acc_cov);
        this->get_parameter("mapping.b_gyr_cov", b_gyr_cov);
        this->get_parameter("mapping.b_acc_cov", b_acc_cov);
        this->get_parameter("preprocess.blind", p_pre->blind);
        this->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
        this->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
        this->get_parameter("preprocess.timestamp_unit", p_pre->time_unit);
        this->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);
        this->get_parameter("point_filter_num", p_pre->point_filter_num);
        this->get_parameter("feature_extract_enable", p_pre->feature_enabled);
        this->get_parameter("runtime_pos_log_enable", runtime_pos_log);
        this->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);
        this->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
        this->get_parameter("pcd_save.interval", pcd_save_interval);
        this->get_parameter("mapping.extrinsic_T", extrinT);
        this->get_parameter("mapping.extrinsic_R", extrinR);
        
        cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
        
        path.header.stamp = this->now();
        path.header.frame_id ="camera_init";
        // Initialization logic
        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        double epsi[23] = {0.001};
        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        // Debug files initialization
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp_ = fopen(pos_log_dir.c_str(),"w");

        fout_pre_.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out_.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg_.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre_ && fout_out_)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

        // Subscribers
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic, 200000, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, 200000, standard_pcl_cbk);
        }

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 200000, imu_cbk);

        
        // Publishers
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100000);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100000);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100000);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100000);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 100000);
        //在 pub_odom_ = ... 附近添加：
        pub_debug_scan_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug_scan", 1);
        pub_debug_map_cloud  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug_map", 1);
        // 在 pub_debug_scan_cloud = ... 后面添加：
        pub_debug_scan_rejected = this->create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug_scan_rejected", 1);
        pub_debug_map_rejected  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug_map_rejected", 1);

        // Timer for main processing loop (Replaces while loop)
        // 5000Hz (0.2ms) to match the original rate logic, 
        // though actual rate depends on sync_packages success.
        // 话题名叫 /debug_three_views
        pub_image_debug_ = this->create_publisher<sensor_msgs::msg::Image>("/debug_three_views", 10);   
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(200), 
            std::bind(&LaserMappingNode::run_callback, this));
        gravity_init_done = false;
        // ... 在初始化代码的最后 ...

    // 1. 声明参数 (默认值为 0.0)
    this->declare_parameter("calib_roll", 0.0);
    this->declare_parameter("calib_pitch", 0.0);
    this->declare_parameter("calib_yaw", 0.0);

    // 2. 绑定回调函数
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            
            for (const auto & param : parameters) {
                if (param.get_name() == "calib_roll") {
                    manual_roll_ = param.as_double();
                    RCLCPP_INFO(get_logger(), "Update Roll: %.2f", manual_roll_);
                } else if (param.get_name() == "calib_pitch") {
                    manual_pitch_ = param.as_double();
                    RCLCPP_INFO(get_logger(), "Update Pitch: %.2f", manual_pitch_);
                } else if (param.get_name() == "calib_yaw") {
                    manual_yaw_ = param.as_double();
                    RCLCPP_INFO(get_logger(), "Update Yaw: %.2f", manual_yaw_);
                }
            }
            return result;
        });
    
    RCLCPP_WARN(get_logger(), ">>> Dynamic Calibration Ready! Use 'rqt' to adjust calib_roll/pitch/yaw <<<");
    }

    ~LaserMappingNode()
    {
        if (fp_) fclose(fp_);
        fout_out_.close();
        fout_pre_.close();
        fout_dbg_.close();
    }
    void init_voxelmap_extrinsics() {
        if (voxelmap_manager) {
            voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
            voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);
            RCLCPP_INFO(this->get_logger(), "VoxelMap Extrinsics Set.");
        }
    }
    // Public getters for publishers to be used by global functions
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr get_pubLaserCloudFull() { return pubLaserCloudFull_; }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr get_pubLaserCloudFull_body() { return pubLaserCloudFull_body_; }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr get_pubLaserCloudEffect() { return pubLaserCloudEffect_; }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr get_pubLaserCloudMap() { return pubLaserCloudMap_; }
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr get_pubOdomAftMapped() { return pubOdomAftMapped_; }
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr get_pubPath() { return pubPath_; }
    std::atomic<bool> is_reloc_running{false};
    FILE* get_fp() { return fp_; }
    ofstream& get_fout_pre() { return fout_pre_; }
    ofstream& get_fout_out() { return fout_out_; }
    // 在 LaserMappingNode 类定义中
    bool is_emergency_mode = false;     // 标记是否处于冲撞后的紧急状态
    double emergency_start_time = 0.0;  // 记录冲撞发生的时刻
private:

    // [新增] 用于时空对齐的快照变量
    Eigen::Vector3d odom_pos_snapshot_ = Eigen::Vector3d::Zero();
    double odom_yaw_snapshot_ = 0.0;
    double correction_uncertainty = 1.0;
    double correction_score = 0.0; // [新增]           
    Eigen::Matrix3d cached_R_IG_ = Eigen::Matrix3d::Identity();
    // [新增] IMU 单位自动锁定机制
    bool imu_scale_determined = false;
    double imu_scale_factor = 1.0; // 默认为1，如果检测到是G则改为9.81
    int imu_init_count = 0;
    double imu_init_accum_norm = 0.0;
    // [新增] ZUPT 闭锁机制所需的变量
    // 必须在这里声明，run_callback 才能用！
    double last_lidar_update_time_ = 0.0;
    Eigen::Vector3d last_lidar_pos_ = Eigen::Vector3d::Zero();
    double zupt_lockout_timer_ = 0.0;
    // [新增] 用于重力对齐的变量
    std::vector<Eigen::Vector3d> imu_init_buffer;  // 缓存 IMU 数据
    Eigen::Vector3d g_camera_init = Eigen::Vector3d(0, 0, 1); // 默认初始化 
    bool gravity_init_done;
    Eigen::Matrix3d R_install = Eigen::Matrix3d::Identity(); // 最终算出的安装修正矩阵  
    std::deque<FrameData> cloud_buffer;
    int buffer_size = 30; // 调试建议30，比赛建议8
    Eigen::Vector3d last_buffer_pos = Eigen::Vector3d::Zero();
    std::atomic<bool> debug_dump_images{false};
    double last_reloc_time = 0.0;     // 上次重定位时间    
    bool flg_reset = false;

    rclcpp::SubscriptionBase::SharedPtr sub_pcl_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    // [新增] 可视化发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_scan_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_map_cloud;
    // [新增] 专门用于可视化的 Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_scan_rejected; // 雷达扔掉的点
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_map_rejected;  // 地图扔掉的点
    rclcpp::TimerBase::SharedPtr timer_;
    // [新增] 用于动态调参的变量 (单位: 度)
    double manual_roll_ = 0.0;
    double manual_pitch_ = 0.0;
    double manual_yaw_ = 0;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_debug_;

    // [新增] 参数回调句柄
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    FILE *fp_;
    ofstream fout_pre_, fout_out_, fout_dbg_;
/**
     * @brief 全光滑稳态门控策略 (C-Infinity Smooth Gating)
     * 使用 Sigmoid 和 Gaussian 函数替代所有硬阈值，保证权重的任意阶导数连续。
     */
    double CalculateDualLayerAlpha(double kf_trace_P,       // EKF 不确定度
                                   double reloc_qual,       // 重定位质量 (0~1, 越大越好)
                                   double dist_gap,         // 距离偏差 (m)
                                   double shock_metric,     // 震动强度 (m/s^2)
                                   double rot_speed)        // 旋转速度 (rad/s)
    {
        // ================= [数学工具：光滑激活函数] =================
        // 1. 负向抑制函数 (输入越大，权重越趋近0)
        // 用于震动和旋转抑制。形式：Sigmoid 的镜像
        // k: 陡峭度 (越大越陡), x0: 软阈值中心
        auto SmoothSuppress = [](double x, double k, double x0) -> double {
            return 1.0 / (1.0 + std::exp(k * (x - x0)));
        };

        // 2. 正向激活函数 (输入越大，权重越趋近1)
        // 用于不确定度激活。
        auto SmoothActivate = [](double x, double k, double x0) -> double {
            return 1.0 / (1.0 + std::exp(-k * (x - x0)));
        };

        // ================= [第一层：物理稳定性门控 (Physics Gating)] =================
        // 震动抑制：软阈值 1.0 m/s^2，陡度 5.0
        // 当 shock=1.0 时权重 0.5；shock=2.0 时权重 0.006 (几乎为0)
        double w_shock = SmoothSuppress(shock_metric, 5.0, 1.0);

        // 旋转抑制：软阈值 0.5 rad/s，陡度 10.0
        // 当 rot=0.5 时权重 0.5；rot=1.0 时权重 0.006
        double w_rot = SmoothSuppress(rot_speed, 10.0, 0.5);

        // 物理总权重 (乘法原理)
        double w_physics = w_shock * w_rot;

        // ================= [第二层：系统需求门控 (Demand Gating)] =================
        // EKF 迷茫度激活：仅当 TraceP 较大时才允许大幅更新
        // 软阈值 0.5 (约0.7m精度)，陡度 5.0
        // P=0.1 -> w=0.11; P=1.0 -> w=0.92
        double w_sys_lost = SmoothActivate(kf_trace_P, 5.0, 0.5);

        // ================= [第三层：数据质量门控 (Quality Gating)] =================
        // 重定位质量激活：软阈值 0.5，陡度 10.0
        double w_qual = SmoothActivate(reloc_qual, 10.0, 0.5);

        // ================= [第四层：几何一致性门控 (Geometry Gating)] =================
        // 这是一个动态高斯分布：如果不确定度(P)大，我们允许更大的距离误差(dist)
        // Sigma = Base + Gain * P_Lost
        // 这种设计保证了“迷路时容忍度变大，自信时容忍度变小”的平滑过渡
        double sigma_dist = 0.5 + 5.0 * w_sys_lost; 
        double w_geo = std::exp( - (dist_gap * dist_gap) / (2.0 * sigma_dist * sigma_dist));

        // ================= [最终合成] =================
        // 基础 Alpha (0.001)：用于消除微小的累积漂移，始终存在
        const double ALPHA_BASE = 0.001;
        // 动态 Alpha (最大 0.1)：用于纠正大偏差
        const double ALPHA_DYNAMIC_MAX = 0.1;

        // 核心公式：基础值 + 动态增益 * 所有权重的乘积
        // 乘法保证了“一票否决权”：只要任意一个环节不可靠（如震动大、质量差），增益项就会平滑消失
        double alpha = ALPHA_BASE + ALPHA_DYNAMIC_MAX * (w_physics * w_sys_lost * w_qual * w_geo);

        // 再次经过一个 Sigmoid 软限幅，防止极端情况下数值溢出 (虽然理论上不会超过 0.101)
        // 这里只是为了数学上的严谨性
        return 0.101 / (1.0 + std::exp(-100.0 * (alpha - 0.05))) * (alpha / 0.05); // 简化版：直接返回即可
        // 实际上上面的公式已经保证了范围，直接返回更平滑：
        return alpha; 
    }
    
    
    void PerformSoftBiasConstraint(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf, 
                                   double shock_metric, 
                                   double gyr_metric) 
    {
        // 1. 计算环境能量 (熵的度量)
        const double sigma_shock = 3.0; // 阈值放宽，允许正常行驶震动
        const double sigma_gyr = 2.0;

        double E_shock = (shock_metric * shock_metric) / (sigma_shock * sigma_shock);
        double E_gyr   = (gyr_metric * gyr_metric) / (sigma_gyr * sigma_gyr);
        double Energy = E_shock + E_gyr;

        // 2. 计算置信度权重 (Weight)
        // Energy 大 -> Weight 趋近 0 (当前环境不可信)
        double Weight = std::exp(-Energy); 

        // [核心修改]：如果环境太乱 (Weight < 0.1)，什么都不要做！
        // 物理意义：此时 Bias 不可观测，强行更新只会引入噪声。
        // "只管理信息，不创造信息" -> 没有信息时，保持现状。
        if (Weight < 0.1) return; 

        // 3. 仅在平稳时施加微弱约束 (防止 Bias 长期漂移)
        const double R_base = 1.0; // 这是一个很弱的约束
        double R_curr = R_base / Weight;

        // 获取当前状态
        state_ikfom x = kf.get_x();
        auto P = kf.get_P();

        // 4. 更新 Bg (陀螺仪 Bias)
        Eigen::Matrix3d P_bg = P.block<3, 3>(9, 9);
        Eigen::Matrix3d K_bg = P_bg * (P_bg + Eigen::Matrix3d::Identity() * R_curr).inverse();
        x.bg += K_bg * (Eigen::Vector3d::Zero() - x.bg);
        P.block<3, 3>(9, 9) = (Eigen::Matrix3d::Identity() - K_bg) * P_bg;

        // 5. 更新 Ba (加速度计 Bias)
        Eigen::Matrix3d P_ba = P.block<3, 3>(12, 12);
        Eigen::Matrix3d K_ba = P_ba * (P_ba + Eigen::Matrix3d::Identity() * R_curr).inverse();
        x.ba += K_ba * (Eigen::Vector3d::Zero() - x.ba);
        P.block<3, 3>(12, 12) = (Eigen::Matrix3d::Identity() - K_ba) * P_ba;

        // 写回
        kf.change_x(x);
        kf.change_P(P);
    }
    double GetGravityYaw(const Eigen::Quaterniond& q_wb, const Eigen::Vector3d& n_up) {
        Eigen::Matrix3d R_WB = q_wb.toRotationMatrix();
        
        // 1. 构造 World 系下的水平参考轴 (World X 投影到水平面)
        Eigen::Vector3d world_x = Eigen::Vector3d::UnitX();
        Eigen::Vector3d world_x_horiz = world_x - (world_x.dot(n_up)) * n_up;
        if (world_x_horiz.norm() < 1e-3) {
            world_x_horiz = Eigen::Vector3d::UnitY() - (Eigen::Vector3d::UnitY().dot(n_up)) * n_up;
        }
        world_x_horiz.normalize();

        // 2. 构造 Body 系下的水平参考轴 (Body X 投影到水平面)
        Eigen::Vector3d body_x = R_WB.col(0); // Body X axis in World Frame
        Eigen::Vector3d body_x_horiz = body_x - (body_x.dot(n_up)) * n_up;
        body_x_horiz.normalize();

        // 3. 计算夹角 (atan2 返回弧度)
        double yaw_rad = std::atan2(
            (world_x_horiz.cross(body_x_horiz)).dot(n_up), 
            world_x_horiz.dot(body_x_horiz)
        );
        return yaw_rad;
    }
    /**
     * @brief 一致性校验函数
     * 检查 buffer 中最近的 count 帧结果是否在空间上聚类（防止单帧误匹配）
     */
    bool CheckConsistency(const std::deque<RelocCandidate>& buffer, int count) {
        if (buffer.size() < count) return false;

        // 获取最新的一帧作为基准
        const auto& latest = buffer.back();
        
        // 向前检查 (count-1) 帧
        for (int i = 1; i < count; ++i) {
            // 获取倒数第 i+1 帧
            const auto& prev = buffer[buffer.size() - 1 - i];
            
            // 计算平面距离差异
            double dx = prev.pos.x() - latest.pos.x();
            double dy = prev.pos.y() - latest.pos.y();
            double dist = std::sqrt(dx*dx + dy*dy);

            // 如果任意一帧与最新帧的距离超过 0.5m，则认为不一致
            if (dist > 0.5) { 
                return false;
            }
        }
        
        // 所有帧都通过了距离检查
        return true;
    }
    /**
     * @brief 计算统合权重泛函 lambda (泛函大脑)
     * [修改版] 同时考虑位置误差和角度误差
     */
    double CalculateUnifiedWeight(const esekfom::esekf<state_ikfom, 12, input_ikfom> &kf, 
                                  const Eigen::Vector3d& target_pos, 
                                  double target_yaw_rad, // [新增参数] 目标Yaw
                                  double reloc_cov_trace) 
    {
        state_ikfom x_k = kf.get_x();
        
        // 1. [物理一致性] R_IG 构建
        Eigen::Vector3d g_vec = (Eigen::Vector3d)x_k.grav;
        if (g_vec.norm() < 1e-3) g_vec = -Eigen::Vector3d::UnitZ();
        Eigen::Vector3d z_w = -g_vec.normalized(); 
        Eigen::Vector3d x_ref = Eigen::Vector3d::UnitX();
        if (std::abs(z_w.dot(x_ref)) > 0.9) x_ref = Eigen::Vector3d::UnitY();
        Eigen::Vector3d y_w = (z_w.cross(x_ref)).normalized();
        Eigen::Vector3d x_w = y_w.cross(z_w);
        Eigen::Matrix3d R_IG; R_IG << x_w, y_w, z_w;

        // 2. [位置误差]
        Eigen::Vector3d p_err_phys = R_IG.transpose() * (target_pos - x_k.pos);
        double dist_pos = p_err_phys.head<2>().norm();

        // 3. [角度误差] (新增)
        // 获取当前物理 Yaw
        Eigen::Quaterniond q_wb = x_k.rot.inverse();
        double curr_yaw = GetGravityYaw(q_wb, z_w); // z_w is n_up
        double yaw_err = std::abs(target_yaw_rad - curr_yaw);
        while(yaw_err > M_PI) yaw_err -= 2*M_PI;
        yaw_err = std::abs(yaw_err);

        // 4. [统合马氏距离]
        // 将角度误差折算为距离误差 (假设 1度 ~ 0.05m 的关注度)
        // 权重配比：位置 1.0 : 角度 0.5 (m/rad)
        double dist_ang_equiv = yaw_err * 0.5; 
        
        // 综合误差范数
        double total_dist = std::sqrt(dist_pos*dist_pos + dist_ang_equiv*dist_ang_equiv);

        double p_trace = kf.get_P().block<2,2>(0,0).trace(); 
        double sigma_joint = std::sqrt(p_trace + reloc_cov_trace);
        
        // [关键] 用综合误差计算 D_M
        double D_M = total_dist / (sigma_joint + 1e-6);

        // 5. [算子 A] 死区激活 (3 sigma)
        double alpha = 1.0 / (1.0 + std::exp(-5.0 * (D_M - 3.0)));
        
        // 6. [算子 B] 自信度
        double beta = p_trace / (p_trace + reloc_cov_trace + 1e-6);
        beta = std::pow(beta, 2.0); 

        // 7. [算子 C] 飞掉惩罚
        double gamma = 1.0;
        if (D_M > 10.0) {
            gamma = 1.0 + 100.0 * std::exp(std::min(5.0, D_M - 10.0)) - 100.0;
        }

        return alpha * beta * gamma;
    }   // 【新增】全状态诊断函数
    void LogEKFState(const std::string& stage, const state_ikfom& s, const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf) {
        // 1. 基础状态
        ULOG_PLOT(stage, "Pos_X", s.pos.x());
        ULOG_PLOT(stage, "Pos_Y", s.pos.y());
        ULOG_PLOT(stage, "Vel_X", s.vel.x());
        ULOG_PLOT(stage, "Bg_Z",  s.bg.z()); // 陀螺仪零偏
        ULOG_PLOT(stage, "Ba_Z",  s.ba.z()); // 加计零偏

    auto P = kf.get_P();
    
    // 【修改点】加上偏移量即可，彻底杜绝写错
    ULOG_PLOT(stage, "Cov_Pos_X", P(StateIdx::POS + 0, StateIdx::POS + 0));
    ULOG_PLOT(stage, "Cov_Pos_Y", P(StateIdx::POS + 1, StateIdx::POS + 1));
    ULOG_PLOT(stage, "Cov_Rot_Z", P(StateIdx::ROT + 2, StateIdx::ROT + 2)); 
    ULOG_PLOT(stage, "Cov_Vel_X", P(StateIdx::VEL + 0, StateIdx::VEL + 0));
    ULOG_PLOT(stage, "Cov_Vel_Z", P(StateIdx::VEL + 2, StateIdx::VEL + 2)); 
    ULOG_PLOT(stage, "Cov_Bg_Z",  P(StateIdx::BG  + 2, StateIdx::BG  + 2));
    ULOG_PLOT(stage, "Cov_Ba_Z",  P(StateIdx::BA  + 2, StateIdx::BA  + 2));
    }
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr TransformToBody(
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in, 
        state_ikfom state)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZINormal>());
        cloud_out->resize(cloud_in->size());
        
        // 获取外参 (必须准确!)
        Eigen::Matrix3d R_L_B = state.offset_R_L_I.toRotationMatrix();
        Eigen::Vector3d t_L_B = state.offset_T_L_I;

        #pragma omp parallel for num_threads(4)
        for (size_t i = 0; i < cloud_in->size(); i++)
        {
            Eigen::Vector3d p_lidar(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
            Eigen::Vector3d p_body = R_L_B * p_lidar + t_L_B;
            
            cloud_out->points[i].x = p_body.x();
            cloud_out->points[i].y = p_body.y();
            cloud_out->points[i].z = p_body.z();
            cloud_out->points[i].intensity = cloud_in->points[i].intensity;
        }
        return cloud_out;
    }
    
    // [新增] 采样相关变量
    double last_buffer_time_ = 0.0; // 上次存图的时间

void GenerateScanImage(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_lidar_in, 
                                         state_ikfom state, 
                                         cv::Mat &img_scan, 
                                         double resolution, 
                                         double range, 
                                         pcl::PointCloud<pcl::PointXYZINormal>::Ptr rejected_cloud_out, 
                                         pcl::PointCloud<pcl::PointXYZINormal>::Ptr kept_cloud_out)
{
    // [诊断变量]
    int cnt_total = 0;
    int cnt_pass_height = 0;
    int cnt_final = 0;
    double min_h = 1000.0, max_h = -1000.0;

    // 1. 获取物理重力方向 (Init系)
    Eigen::Vector3d g_init = (Eigen::Vector3d)state.grav;
    if (g_init.norm() < 1e-3) g_init = -Eigen::Vector3d::UnitZ();
    Eigen::Vector3d n_up = -g_init.normalized(); // 指向物理天空

    // [关键调试] 记录使用的重力轴，检查 Z 是否为正 (通常应指向天)
    ULOG_PLOT("Scan_Gen_Debug", "n_up_z", n_up.z());

    // 2. 矩阵准备
    double rad_y = manual_yaw_ * M_PI / 180.0; 
    Eigen::Matrix3d R_manual = Eigen::AngleAxisd(rad_y, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d R_LI = state.offset_R_L_I.toRotationMatrix() * R_manual;
    Eigen::Vector3d T_LI = state.offset_T_L_I;
    Eigen::Matrix3d R_WB = state.rot.toRotationMatrix().transpose();
    Eigen::Vector3d T_WB = state.pos;

    // 3. 构建重力对齐坐标系 R_IG
    Eigen::Vector3d z_w = n_up; 
    Eigen::Vector3d x_init = Eigen::Vector3d::UnitX();
    if (std::abs(z_w.dot(x_init)) > 0.99) x_init = Eigen::Vector3d::UnitY();
    Eigen::Vector3d y_w = (z_w.cross(x_init)).normalized();
    Eigen::Vector3d x_w = y_w.cross(z_w);
    Eigen::Matrix3d R_IG; R_IG << x_w, y_w, z_w; 

    // 计算车体在重力系下的高度
    Eigen::Vector3d body_pos_grav = R_IG.transpose() * T_WB;

    // 4. 画布准备
    int size = std::ceil(range * 2.0 / resolution); if (size % 2 != 0) size++;
    int center = size / 2;
    img_scan = cv::Mat::zeros(size, size * 3, CV_8UC1);
    cv::line(img_scan, cv::Point(0, center), cv::Point(size*3, center), cv::Scalar(60));

    // 5. 遍历点云
    for (size_t i = 0; i < cloud_lidar_in->size(); i++)
    {
        cnt_total++;
        Eigen::Vector3d p_lidar(cloud_lidar_in->points[i].x, cloud_lidar_in->points[i].y, cloud_lidar_in->points[i].z);
        Eigen::Vector3d p_init = R_WB * (R_LI * p_lidar + T_LI) + T_WB;

        // [核心] 转到重力系
        Eigen::Vector3d p_grav = R_IG.transpose() * p_init;
        
        // 计算相对高度
        double relative_height = p_grav.z() - body_pos_grav.z();

        // [统计]
        if (relative_height < min_h) min_h = relative_height;
        if (relative_height > max_h) max_h = relative_height;
        if (relative_height > 0.5 && relative_height < 1.2) {
            cnt_pass_height++;
            
            // 画图
            Eigen::Vector3d p_viz = p_grav - body_pos_grav;
            int u1 = std::round((p_viz.x() / resolution) + center);
            int v1 = std::round((p_viz.y() / resolution) + center);
            
            if (u1 >= 0 && u1 < size && v1 >= 0 && v1 < size) {
                img_scan.at<uchar>(v1, u1) = 255;
                cnt_final++;
            }
            
            
            if (kept_cloud_out) {
                pcl::PointXYZINormal p_out = cloud_lidar_in->points[i];
                p_out.x = p_init.x(); p_out.y = p_init.y(); p_out.z = p_init.z();
                kept_cloud_out->push_back(p_out);
            }
        } else {
            if (rejected_cloud_out) {
                pcl::PointXYZINormal p_out = cloud_lidar_in->points[i];
                p_out.x = p_init.x(); p_out.y = p_init.y(); p_out.z = p_init.z();
                rejected_cloud_out->push_back(p_out);
            }
        }
    }

    // [统一记录] 这一步能直接告诉你为什么图像是黑的
    ULOG_PLOT("Scan_Gen_Debug", "cnt_total", (double)cnt_total);
    ULOG_PLOT("Scan_Gen_Debug", "cnt_pass_height", (double)cnt_pass_height);
    ULOG_PLOT("Scan_Gen_Debug", "cnt_final", (double)cnt_final);
    ULOG_PLOT("Scan_Gen_Debug", "min_h", min_h);
    ULOG_PLOT("Scan_Gen_Debug", "max_h", max_h);

    if (pub_image_debug_ && pub_image_debug_->get_subscription_count() > 0 && !img_scan.empty()) {
        std_msgs::msg::Header header; header.stamp = this->get_clock()->now(); header.frame_id = "camera_init"; 
        try { pub_image_debug_->publish(*cv_bridge::CvImage(header, "mono8", img_scan).toImageMsg()); } catch (...) {}
    }
}
  
    void run_callback()
    {
        // 【新增】补上这两行定义和自增
        static int log_downsample_cnt = 0;
        log_downsample_cnt++;
        bool need_log = (flg_reset) || (log_downsample_cnt % 20 == 0);

        if (need_log) {
            if (std::isnan(state_point.pos.x())) {
                RCLCPP_ERROR(get_logger(), "[DIAG] NaN detected at Start of Frame!");
            }
            LogEKFState("Frame_Start", state_point, kf);
        }
        static std::deque<double> acc_var_window;
        static std::deque<double> gyr_var_window;
        const int WINDOW_LEN = 15; // 150ms 滑动窗口，捕捉震动特征
        ULOG_SET_TIME(Measures.lidar_beg_time);
        //std::cout<<1<<endl;
        if (flg_exit) return;
        if (!voxelmap_manager) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "VoxelMap Manager NOT Initialized!");
            return;
        }
        if(sync_packages(Measures)) 
        {   // ================= [核心修改 1: 全局单位统一 (修复编译错误版)] =================
            // 这一步是让"枪上膛"。一旦单位确定了，每一帧进来的数据都要先乘系数。
            // [Defense Layer 1] 雷达速度闭锁计时器更新
            double dt_lidar = Measures.lidar_beg_time - last_lidar_update_time_;
            if (dt_lidar > 0 && dt_lidar < 1.0) { 
                zupt_lockout_timer_ -= dt_lidar;
                if (zupt_lockout_timer_ < 0) zupt_lockout_timer_ = 0;
            }
            
            // Log: 只有当计时器处于激活状态时才打印，避免刷屏
            if (zupt_lockout_timer_ > 0.0) {
                // ULOG_PLOT 记录到 CSV
                ULOG_PLOT("Defense", "Lockout_Timer", zupt_lockout_timer_); 
            }
            double signal_energy = 0.0;
            double gyr_energy = 0.0;
            double p_static = 0.0;
            Eigen::Vector3d acc_mean_vec = Eigen::Vector3d::Zero();
            double robust_peak = 0.0; // [v26.1 新增] 用于存储去极值后的震动峰值
            // ================= [步骤 1: 捕捉永恒重力向量 & 锁定IMU单位] =================
            if (!gravity_init_done) 
            {
                // [新增] 检查 EKF 是否已经启动并有了位姿输出
                if (geoQuat.w == 0 && geoQuat.x == 0 && geoQuat.y == 0 && geoQuat.z == 0) {
                    return; // 等待下一帧
                }

                if (Measures.imu.empty()) return;
                auto imu_msg = Measures.imu.back();
                
                // 这里先拿原始数据，不做任何乘法，保持原汁原味用于判断
                Eigen::Vector3d acc_raw(imu_msg->linear_acceleration.x, 
                                        imu_msg->linear_acceleration.y, 
                                        imu_msg->linear_acceleration.z);

                // 存入 buffer
                imu_init_buffer.push_back(acc_raw);
                if (imu_init_buffer.size() > 30) imu_init_buffer.erase(imu_init_buffer.begin());

                // 2. 如果攒够了数据且方差很小（静止）
                if (imu_init_buffer.size() >= 20) {
                    // 算均值
                    Eigen::Vector3d acc_avg = Eigen::Vector3d::Zero();
                    for(auto& a : imu_init_buffer) acc_avg += a;
                    acc_avg /= imu_init_buffer.size();
                    
                    double avg_norm = acc_avg.norm();

                    // ================= [新增核心逻辑: 单位自动锁定] =================
                    // 逻辑：利用静止时的模长来判断单位，并永久锁定缩放因子
                    if (!imu_scale_determined) {
                        if (std::abs(avg_norm - 1.0) < 0.1) {
                            // 情况 A: 单位是 G (1.0)
                            imu_scale_factor = 9.81;
                            imu_scale_determined = true;
                            RCLCPP_WARN(this->get_logger(), ">>> IMU Unit Detected: [G]. Scale Factor Locked to 9.81 <<<");
                        } 
                        else if (std::abs(avg_norm - 9.81) < 1.0) {
                            // 情况 B: 单位是 m/s^2 (9.8)
                            imu_scale_factor = 1.0;
                            imu_scale_determined = true;
                            RCLCPP_WARN(this->get_logger(), ">>> IMU Unit Detected: [m/s^2]. Scale Factor Locked to 1.0 <<<");
                        }
                    }

                    // 只有当单位确定后，才进行重力捕获
                    if (imu_scale_determined) {
                        // 统一转换成 m/s^2 进行后续计算 (这一步很重要，保证 acc_avg 是物理真值)
                        Eigen::Vector3d acc_avg_metric = acc_avg * imu_scale_factor;

                        // 再次检查转换后的模长是否接近 9.81 (双重保险)
                        if (std::abs(acc_avg_metric.norm() - 9.81) < 0.5) {
                            
                            // [核心逻辑] 
                            // 此时 IMU 测到的是 Body 系下的重力
                            // 我们要把它转到 camera_init 系下保存起来
                            
                            // 获取当前时刻的姿态 (Body -> camera_init)
                            Eigen::Quaterniond q_curr(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
                            q_curr.normalize();
                            Eigen::Matrix3d R_curr = q_curr.toRotationMatrix();

                            // g_camera_init = R * g_body
                            // 这里保存归一化的方向向量即可
                            g_camera_init = R_curr * acc_avg_metric.normalized();

                            gravity_init_done = true;
                            RCLCPP_WARN(get_logger(), "[Gravity Logic] Captured constant G in camera_init: [%.3f, %.3f, %.3f]", 
                                g_camera_init.x(), g_camera_init.y(), g_camera_init.z());
                        }
                    }
                }
            }
            Eigen::Vector3d g_vec = g_camera_init.normalized();
            // 显式调用构造函数
            // 自动获取 state_point.grav 的类型，并调用其构造函数
            state_point.grav = std::decay_t<decltype(state_point.grav)>( -g_vec * 9.81 );
            n_up = g_camera_init.normalized();

            // 2. 算出雷达(Lidar)在当前坐标系下的绝对位置
            // P_lidar = P_imu + R_imu * T_lidar_to_imu
            Eigen::Matrix3d R_WB_real = state_point.rot.toRotationMatrix().transpose();
            Eigen::Vector3d P_radar = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            Eigen::Vector3d P_radar_world = state_point.pos + R_WB_real * state_point.offset_T_L_I;
            // 3. 核心：通过内积算出"雷达真实高度"
            // 几何意义：将雷达的位置向量投影到垂直方向上
            double real_radar_height = P_radar_world.dot(n_up);
            ULOG_PLOT("Height_Debug", "Calc_Real_Height", real_radar_height);
            ULOG_PLOT("Height_Debug", "N_Up_Z_Comp", n_up.z()); // 检查天空向量是否指向上方
            ULOG_PLOT("Height_Debug", "Raw_Pos_Z", state_point.pos.z()); // 对比原始Z
            // ==========================================
            // 现在的应用：用这个真实高度去解决你之前的"飞车"判定
            // ==========================================

            // 设定雷达正常离地高度范围 (比如 0.3m ~ 0.6m)
            // 如果算出来高度 > 2.0m，说明真的飞了，和坐标系歪不歪没关系
            if (real_radar_height > 2.0) {
                // 触发安全复位逻辑...
                RCLCPP_WARN(get_logger(), "[Safety] Radar Height Abnormal: %.2f", real_radar_height);
            }
            // 2. 简单的方向修正
            // 如果重力是指向地下的 (通常 IMU 加速度计测出的G在静止时是指向'上'的支持力，但有些是指向'下'的)
            // 我们希望 n_vec 指向"天空" (Z+)，这样 rel_height 才是符合直觉的 (-1 ~ +2)
            // 假设标准 World Z 是向上的，我们判断一下是否同向
            if (g_vec.dot(Eigen::Vector3d(0,0,1)) < 0) {
                g_vec = -g_vec;
            }
            // [关键操作 1] 全局更新时间
            // 这一行之后，任何地方调用的 LOG 宏都会自动打上这个时间戳
            LOG_UPDATE_TIME(Measures.lidar_beg_time);
            
            // [关键操作 2] 帧数递增
            LOG_FRAME();
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }
            // [新增] 防御层 1: 预判与自适应 Q (必须在 Process 之前!)
            if (!Measures.imu.empty()) {
                std::vector<double> acc_mags, gyr_mags;
                acc_mags.reserve(Measures.imu.size());
                gyr_mags.reserve(Measures.imu.size());
                
                // [v26.1 新增] 震动采样缓冲区
                std::vector<double> shock_samples;
                shock_samples.reserve(Measures.imu.size());

                for(const auto& imu : Measures.imu) {
                    double ax = imu->linear_acceleration.x;
                    double ay = imu->linear_acceleration.y;
                    double az = imu->linear_acceleration.z;
                    // ---------------- [核心修正：单位对齐] ----------------
                    // 判定：如果当前读到的原始模长 < 3.0，说明是 G 单位，必须转为 m/s^2
                    // 否则会导致 (1.0 - 9.81) = 8.8 的虚假地震，致盲雷达
                    double check_norm = std::sqrt(ax*ax + ay*ay + az*az);
                    if (check_norm < 3.0) {
                        ax *= 9.81; 
                        ay *= 9.81; 
                        az *= 9.81;
                    }
                    // ----------------------------------------------------
                    // [关键] 累加加速度向量
                    acc_mean_vec += Eigen::Vector3d(ax, ay, az);
                    double curr_norm = std::sqrt(ax*ax + ay*ay + az*az);
                    acc_mags.push_back(curr_norm);

                    // [v26.1 新增] 采集单点震动样本
                    double curr_shock = std::abs(curr_norm - 9.81);
                    shock_samples.push_back(curr_shock);
                    ULOG_PLOT("Shock_Test", "Calc_Curr_Norm", curr_norm);
                    ULOG_PLOT("Shock_Test", "Calc_Curr_Shock", curr_shock);
                    double gx = imu->angular_velocity.x;
                    double gy = imu->angular_velocity.y;
                    double gz = imu->angular_velocity.z;
                    gyr_mags.push_back(std::sqrt(gx*gx + gy*gy + gz*gz));
                }
                
                // [关键] 求均值
                acc_mean_vec /= Measures.imu.size();

                // ---------------- [v26.1 核心算法：鲁棒能量提取] ----------------
                if (!shock_samples.empty()) {
                    // Step A: 排序
                    std::sort(shock_samples.begin(), shock_samples.end());
                    
                    // Step B: 去除极大值 (Outlier Removal)
                    // 只有当样本足够多时才去极值，避免数据太少误删真信号
                    if (shock_samples.size() > 4) {
                        shock_samples.pop_back(); // 去掉最大值 (防止单点电子故障)
                        // shock_samples.pop_back(); // 可选：去掉第二大
                    }
                    
                    // Step C: 获取去噪后的峰值
                    robust_peak = shock_samples.back();
                }
                // -------------------------------------------------------------

                if (acc_mags.size() > 1) {
                    double acc_mean = 0, acc_sq_sum = 0;
                    for(double v : acc_mags) acc_mean += v; acc_mean /= acc_mags.size();
                    for(double v : acc_mags) acc_sq_sum += (v - acc_mean)*(v - acc_mean);
                    double acc_var = acc_sq_sum / acc_mags.size();

                    double gyr_mean = 0, gyr_sq_sum = 0;
                    for(double v : gyr_mags) gyr_mean += v; gyr_mean /= gyr_mags.size();
                    for(double v : gyr_mags) gyr_sq_sum += (v - gyr_mean)*(v - gyr_mean);
                    double gyr_var = gyr_sq_sum / gyr_mags.size();

                    signal_energy = acc_var + 10.0 * gyr_var;
                    gyr_energy = gyr_mean; 
                }

                // ... (原有的 P 阵调整逻辑保持不变) ...
                bool is_shock = (signal_energy > 5.0);
                bool is_fast_rot = (gyr_energy > 2.0);

                if (is_shock || is_fast_rot) {
                    Eigen::Matrix<double, 23, 23> P = kf.get_P();
                    // [修正版] 物理有界协方差注入
                    if (is_shock) {
                        // 定义物理极限（紧集边界）
                        // 0.5 的方差意味着我们认为速度误差在 0.7m/s 以内，这是合理的颠簸误差范围
                        const double MAX_VEL_COV = 0.5; 
                        const double MAX_ROT_COV = 0.1; // 约 18度

                        // 1. 速度协方差注入 (Index 12~14)
                        for(int k=0; k<3; k++) {
                            // 【核心逻辑】: 距离上限越远，加得越快；接近上限时，停止增加。
                            // 这构造了一个平滑的收敛边界，而不是无限发散。
                            double current_val = P(12+k, 12+k);
                            if(current_val < MAX_VEL_COV) {
                                // 注入量 = 固定步长 (或者可以使用 (Max - Curr) * 0.1 做渐近趋近)
                                P(12+k, 12+k) += 0.05; 
                            }
                        }

                        // 2. 姿态协方差注入 (Index 3~5)
                        for(int k=0; k<3; k++) {
                            if(P(3+k, 3+k) < MAX_ROT_COV) {
                                P(3+k, 3+k) += 0.01;
                            }
                        }
                        
                        // 绝对安全的强制硬限幅 (基于对角化缩放)
                        for(int i=0; i<23; i++){
                            if(P(i,i) > 10.0) {
                                double scale = std::sqrt(10.0 / P(i,i));
                                P.row(i) *= scale;
                                P.col(i) *= scale;
                                P(i,i) = 10.0;
                            }
                        }
                    }
                    if (is_fast_rot) {
                        // 1. 计算惩罚量（随转速增加，但有绝对上限）
                        double rot_penalty = std::min(0.01, 0.005 * gyr_energy * gyr_energy);
                        
                        // 2. 为姿态(Rot: 3~5)注入不确定性，并加上【硬锁】
                        for(int k=0; k<3; k++) {
                            P(3+k, 3+k) += rot_penalty;
                            // 【绝对防飞锁】：姿态方差绝不允许超过 0.05 (约12度误差)
                            // 只要不超过 1.0，卡尔曼增益 K 就永远不会算爆
                            if (P(3+k, 3+k) > 0.05) P(3+k, 3+k) = 0.05; 
                        }
                        
                        // 3. 为速度(Vel: 12~14)连带注入不确定性，并加上【硬锁】
                        for(int k=0; k<3; k++) {
                            P(12+k, 12+k) += rot_penalty * 2.0;
                            // 【绝对防飞锁】：速度方差绝不允许超过 0.2
                            if (P(12+k, 12+k) > 0.2) P(12+k, 12+k) = 0.2; 
                        }
                        
                        ULOG_PLOT("Defense_Action", "Mode", 3.0); 
                    }
                    kf.change_P(P);
                } else {
                    ULOG_PLOT("Defense_Action", "Mode", 0.0); 
                }
                
                const double VAR_THRESH = 0.05; 
                p_static = 1.0 / (1.0 + std::exp(50.0 * (signal_energy - VAR_THRESH)));
                
                ULOG_PLOT("Logic_Input", "Energy", signal_energy);
                ULOG_PLOT("Logic_Input", "Gyr_Energy", gyr_energy);
                ULOG_PLOT("Logic_Input", "P_Static", p_static);
            }
 
            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            ULOG_CHECK_PCL(feats_undistort, "1_After_Undistort");
            //voxelmap_manager->state_ = _state;
            voxelmap_manager->feats_undistort_ = feats_undistort;
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan0!\n");
                return;
            }
            state_point = kf.get_x(); 
            if (need_log) {
                LogEKFState("After_Predict", state_point, kf);
            }
            // ================= [Defense Layer 1.5: 柔性物理势能场 (v22.0)] =================
            // 彻底摒弃硬阈值。利用物理风险构建连续的抑制函数。
            
            // 1. 物理量计算
            double acc_norm_now = acc_mean_vec.norm();
            double shock_val = std::abs(acc_norm_now - 9.81); // 震动风险 (m/s^2)
            double gyr_val = std::abs(gyr_energy);            // 旋转风险 (rad/s)
            
            // [核心创新]：估算向心力风险 (Centripetal Risk)
            // 假设机器人最大线速度 v_max = 3.0 m/s (保守估计)
            // 向心加速度 a_c = v * omega。
            // 这是一个"最坏情况"估计，确保在高速+高转时风险值爆炸。
            double centripetal_risk = 3.0 * gyr_val; 

            // 2. 构建"风险势能" (Risk Potential)
            // 我们希望：
            // - shock < 0.5 && gyr < 0.2 -> 安全区 (Risk < 1)
            // - shock > 2.0 || gyr > 1.0 -> 危险区 (Risk > 5)
            // 使用平方项来产生非线性抑制效果 (类似于动能 E = 1/2 mv^2)
            
            double E_shock = std::pow(shock_val / 1.0, 2);      // 归一化：1.0 m/s^2 为软边界
            double E_spin  = std::pow(centripetal_risk / 1.0, 2); // 归一化：1.0 m/s^2 为软边界
            
            double total_risk = E_shock + E_spin;

            // 3. 计算自适应增益 (Gaussian Decay)
            // 公式：Gain = Base * exp(-Risk)
            // 特性：Risk=0 -> Gain=1; Risk=1 -> Gain=0.36; Risk=5 -> Gain=0.006 (几乎为0)
            // 这就是你要的"柔性过渡" + "硬限制"：
            // 小风险时慢慢降，大风险时断崖式降，绝对不会因为参数偏差而让错误修正漏过去。
            
            double adaptive_gain = 0.05 * std::exp(-total_risk);

            // 4. 执行修正 (无条件执行，因为 Gain 会自动变为 0)
            // 只要 Gain > 1e-4 (微小量)，就执行，避免 if-else 的跳变
            // if (adaptive_gain > 1e-4) {
            //     // ... (保留之前的重力向量计算逻辑，完全不变) ...
                
            //     // 1. 获取当前 EKF 认为的 World 重力方向 (Up)
            //     Eigen::Vector3d g_world_up = -((Eigen::Vector3d)state_point.grav); // S2强转修复
                
            //     // 2. 转到 Body 系
            //     Eigen::Vector3d g_est_body = state_point.rot.toRotationMatrix().transpose() * g_world_up;
                
            //     // 3. 实际观测值
            //     Eigen::Vector3d g_meas_body = acc_mean_vec.normalized();

            //     // 4. 计算误差四元数
            //     Eigen::Quaterniond q_error = Eigen::Quaterniond::FromTwoVectors(g_est_body, g_meas_body);
                
            //     // 5. 应用柔性增益
            //     // 这里的 adaptive_gain 包含了所有物理约束
            //     Eigen::Quaterniond q_corr = Eigen::Quaterniond::Identity().slerp(adaptive_gain, q_error);
                
            //     state_point.rot = state_point.rot * q_corr;
            //     state_point.rot.normalize();
            //     kf.change_x(state_point);
                
            //     // 【新增：向 P 阵坦白外部干涉，注入过程噪声】
            //     // 告诉 EKF：“我不小心动了姿态，你别太相信之前的预测了”
            //     Eigen::Matrix<double, 23, 23> P_anchor = kf.get_P();
            //     // 索引 3,4,5 对应 Rot。注入的噪声量与干涉幅度成正比
            //     P_anchor.block<3,3>(3,3) += Eigen::Matrix3d::Identity() * (adaptive_gain * 0.01);
            //     kf.change_P(P_anchor);
                
            //     ULOG_PLOT("Defense_Action", "Anchor_Gain", adaptive_gain);
            // } else {
            //     ULOG_PLOT("Defense_Action", "Anchor_Gain", 0.0);
            // }
            
            // 更新外参位置
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            // ================= [Defense Layer 2: 互锁 ZUPT (After Process)] =================
            
            // 获取当前角速度
            double gyr_norm_now = 0.0;
            if (!Measures.imu.empty()) {
                auto last = Measures.imu.back();
                gyr_norm_now = std::sqrt(last->angular_velocity.x*last->angular_velocity.x + 
                                         last->angular_velocity.y*last->angular_velocity.y + 
                                         last->angular_velocity.z*last->angular_velocity.z);
            }
            
// 策略 B: 互锁 ZUPT
            // 前置条件：没有冲击 (Energy < 5.0) 且 没有快转 (Gyr < 2.0)
            if (signal_energy < 5.0 && gyr_energy < 2.0) {
                double current_vel_norm = state_point.vel.norm();
                bool is_moving_locked = (zupt_lockout_timer_ > 0.0);
                // 旋转互锁：角速度 > 0.015 即为动
                bool is_rotating = (gyr_norm_now > 0.015);

                if (!is_moving_locked && p_static > 0.8 && current_vel_norm < 0.2 && !is_rotating) {
                    
                    Eigen::Vector3d z_vel = Eigen::Vector3d::Zero();
                    double r_val = 0.01 / (p_static * p_static + 1e-4); 
                    Eigen::Matrix3d R_zupt = Eigen::Matrix3d::Identity() * r_val;
                    
                    auto P = kf.get_P();
                    const int VEL_IDX = 12; // 确保索引 12 是 velocity
                    Eigen::Matrix3d P_vel = P.block<3,3>(VEL_IDX, VEL_IDX);
                    
                    // 计算卡尔曼增益 K
                    Eigen::Matrix3d K = P_vel * (P_vel + R_zupt).inverse();
                    
                    // 1. 更新速度状态
                    state_point.vel += K * (z_vel - state_point.vel);
                    
                    // 2. 【核心数学修正：Joseph Form】
                    // 使用 P = (I-K)*P*(I-K)^T + K*R*K^T
                    // 这种对称乘法形式从代数上绝对保证了方差永远大于 0！
                    Eigen::Matrix3d I_minus_K = Eigen::Matrix3d::Identity() - K;
                    Eigen::Matrix3d P_vel_new = I_minus_K * P_vel * I_minus_K.transpose() + K * R_zupt * K.transpose();
                    
                    // 3. 强制对称化，抹杀任何浮点数微小截断误差
                    P.block<3,3>(VEL_IDX, VEL_IDX) = 0.5 * (P_vel_new + P_vel_new.transpose());
                    
                    // 4. 断绝非对角线污染，维持整个 23x23 矩阵的正定性
                    P.block(0, VEL_IDX, VEL_IDX, 3).setZero();                 // 清上方
                    P.block(VEL_IDX, 0, 3, VEL_IDX).setZero();                 // 清左方
                    P.block(15, VEL_IDX, 23 - 15, 3).setZero();                // 清下方 (零偏和重力部分)
                    P.block(VEL_IDX, 15, 3, 23 - 15).setZero();                // 清右方
                    
                    // 5. 隐式 Bias 微小学习
                    P.block<3,3>(15, 15) += Eigen::Matrix3d::Identity() * 0.00001; // Bg
                    P.block<3,3>(18, 18) += Eigen::Matrix3d::Identity() * 0.00001; // Ba
                    
                    kf.change_x(state_point);
                    kf.change_P(P);
                    ULOG_PLOT("Defense_Action", "Mode", 1.0); 
                } else {
                    ULOG_PLOT("Defense_Action", "Mode", 0.0);
                }
            } else {
                ULOG_PLOT("Defense_Action", "Mode", 2.0); // Shock/Rot
            }
            
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan1!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            //lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            // [修改后]：自适应下采样

            // 点充足时才降采样
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            ULOG_CHECK_PCL(feats_down_body, "2_After_Downsample");
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();

            // ================= [DEBUG END] =================
            voxelmap_manager->feats_down_body_ = feats_down_body;
            transformLidar(state_point.rot.toRotationMatrix(), state_point.pos, feats_down_body, feats_down_world);
            ULOG_CHECK_PCL(feats_down_world, "3_After_TransformWorld");
            voxelmap_manager->feats_down_world_ = feats_down_world;
            voxelmap_manager->feats_down_size_ = feats_down_size;
            //std::cout<<3<<endl;
            if (!lidar_map_inited) 
            {
                lidar_map_inited = true;
                voxelmap_manager->BuildVoxelMap();
            }
            /*** initialize the map kdtree ***/
            // if(ikdtree.Root_Node == nullptr)
            // {
            //     if(feats_down_size > 5)
            //     {
            //         ikdtree.set_downsample_param(filter_size_map_min);
            //         feats_down_world->resize(feats_down_size);
            //         for(int i = 0; i < feats_down_size; i++)
            //         {
            //             pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            //         }
            //         ikdtree.Build(feats_down_world->points);
            //     }
            //     return;
            // }
            // int featsFromMapNum = ikdtree.validnum();
            // kdtree_size_st = ikdtree.size();
            // 1. [关键] 设置主线程当前时间
            double current_time = Measures.lidar_beg_time;
            
            ULOG_SET_TIME(current_time);






            // 2. 记录基础状态 (画图组: Pose_Monitor)
            // 直接传入 Vector3d，Logger 会自动拆分成 _x, _y, _z
            ULOG_PLOT("Pose_Monitor", "pos", state_point.pos);
            ULOG_PLOT("Pose_Monitor", "vel", state_point.vel);
            ULOG_PLOT("Pose_Monitor", "bias_acc", state_point.ba);

            static int debug_save_cnt = 0;
            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                return;
            }
            static int frame_count = 0;
            frame_count++;
            const double PIXEL_RES = 0.1;   // 10cm 分辨率
            const double MAP_RANGE = 30.0;  // 30米范围 (生成 60m x 60m 的图)
            // ================= [核心修改：速度-时间-距离 三维互锁采样策略] =================
            // 目的：兼顾 3m/s 高速时的覆盖率和 0m/s 静止时的低冗余
            
            double vel_norm = state_point.vel.norm();
            
            // A. 动态距离阈值：速度越快，允许的单帧位移越大
            // 0m/s -> 0.1m; 3m/s -> 0.4m
            double dynamic_dist_thresh = 0.1 + (vel_norm * 0.1); 

            // B. 动态时间阈值：速度越快，强制采样的时间间隔越短
            // 0m/s -> 0.5s; 3m/s -> 0.1s
            double dynamic_time_thresh = std::max(0.1, 0.5 - (vel_norm * 0.15));

            double dist_moved = (state_point.pos - last_buffer_pos).norm();
            double time_elapsed = current_time - last_buffer_time_;

            // C. 触发逻辑：(位移达标 OR (高速且时间达标))
            bool need_save = (dist_moved > dynamic_dist_thresh);
            if (vel_norm > 1.0 && time_elapsed > dynamic_time_thresh) {
                need_save = true;
            }

            pcl::PointCloud<pcl::PointXYZINormal>::Ptr current_body_cloud = TransformToBody(feats_undistort, state_point);

            if (is_emergency_mode || need_save) 
            {
                FrameData frame_data;
                frame_data.cloud_body = current_body_cloud;
                frame_data.state = state_point;
                cloud_buffer.push_back(frame_data);
                
                last_buffer_pos = state_point.pos;
                last_buffer_time_ = current_time; 
                
                // 动态 Buffer 大小：高速时只存最近的 15 帧，防止拖影过长
                int dynamic_buffer_size = (vel_norm > 2.0) ? 15 : 30;
                if (cloud_buffer.size() > dynamic_buffer_size) cloud_buffer.pop_front();
            }

            // [重力对齐矩阵计算 R_IG] (全局通用)
            // 这一步确保后续所有计算都在"物理水平面"上进行，免疫歪坐标系
            Eigen::Vector3d g_init = (Eigen::Vector3d)state_point.grav;
            if (g_init.norm() < 1e-3) g_init = -Eigen::Vector3d::UnitZ();
            Eigen::Vector3d z_w = -g_init.normalized();
            Eigen::Vector3d x_init = Eigen::Vector3d::UnitX();
            Eigen::Vector3d y_w = (z_w.cross(x_init)).normalized();
            Eigen::Vector3d x_w = y_w.cross(z_w);
            Eigen::Matrix3d R_IG; R_IG << x_w, y_w, z_w;

            // ================= [A. 结果处理：极简增量修正策略 (Debug版)] =================
            {
                std::lock_guard<std::mutex> lock(reloc_mutex); 
                
            // ================= [A. 结果处理：全微分平滑修正策略] =================
            if (has_new_correction) 
            {
                LOG_MSG("RELOC", "EVENT", "TRIGGERED");

                // [Step 1: 还原真实场景]
                Eigen::Vector3d rel_offset = correction_pos; 
                Eigen::Vector3d t_target_map = map_center_snapshot_ + rel_offset;
                t_target_map.z() = odom_pos_snapshot_.z(); 

                // [Step 2: 反解理想漂移 (Ideal Target)]
                Eigen::Vector3d t_drift_ideal = t_target_map - odom_pos_snapshot_;
                {
                    std::lock_guard<std::mutex> lock(drift_mutex_);
                    t_drift_ideal.z() = t_map_odom_.z(); // Z轴锁定
                }

                // 旋转部分平滑处理
                double yaw_map_body = correction_yaw * M_PI / 180.0;
                double yaw_drift_ideal = yaw_map_body - odom_yaw_snapshot_;
                while (yaw_drift_ideal > M_PI) yaw_drift_ideal -= 2*M_PI;
                while (yaw_drift_ideal < -M_PI) yaw_drift_ideal += 2*M_PI;
                Eigen::Quaterniond q_drift_ideal = Eigen::AngleAxisd(yaw_drift_ideal, Eigen::Vector3d::UnitZ())
                                                 * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                                                 * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

                // [Step 3: 计算全光滑权重 Alpha]
                Eigen::Vector3d t_drift_current;
                {
                    std::lock_guard<std::mutex> lock(drift_mutex_);
                    t_drift_current = t_map_odom_;
                }
                double dist_gap = (t_drift_ideal - t_drift_current).norm();
                double trace_P = kf.get_P().trace();

                // 调用之前的全光滑策略函数 (假设 correction_score 越大越好)
                double alpha = CalculateDualLayerAlpha(
                    trace_P, 
                    correction_score, 
                    dist_gap, 
                    robust_peak,        
                    std::abs(gyr_energy)
                );

                // [Step 4: 软限幅爬升 (Soft Slew Rate Limiter)]
                // 这是对 "硬截断" 的终极替代。
                // 作用：将单帧修正量平滑地限制在物理极限内，同时保留导数连续性。
                {
                    std::lock_guard<std::mutex> drift_lock(drift_mutex_);
                    
                    // 1. 计算原始期望增量 (Raw Delta)
                    Eigen::Vector3d raw_delta = alpha * (t_drift_ideal - t_map_odom_);
                    
                    // 2. 定义最大步长 (物理极限，如 0.15m/frame)
                    const double MAX_STEP = 0.15;

                    // 3. 应用 Soft Tanh Limiting
                    // 公式：v_out = Max * tanh(v_in / Max)
                    // 效果：小误差线性修正，大误差平滑饱和，永远不会出现导数突变
                    double raw_norm = raw_delta.norm();
                    double soft_scale = 1.0;
                    if (raw_norm > 1e-6) {
                        soft_scale = (MAX_STEP * std::tanh(raw_norm / MAX_STEP)) / raw_norm;
                    }
                    Eigen::Vector3d soft_delta = raw_delta * soft_scale;

                    // 4. 执行更新
                    t_map_odom_ += soft_delta;

                    // 旋转同理：球面插值本身就是平滑的，Alpha 已经是平滑的了，无需额外处理
                    q_map_odom_ = q_map_odom_.slerp(alpha, q_drift_ideal);
                    q_map_odom_.normalize();

                    // [LOG] 监控平滑效果
                    ULOG_PLOT("Reloc_Process", "5_Final_Alpha", alpha);
                    ULOG_PLOT("Reloc_Process", "5_Raw_Step", raw_norm);
                    ULOG_PLOT("Reloc_Process", "5_Soft_Step", soft_delta.norm());
                    ULOG_PLOT("Reloc_Process", "6_New_Drift_X", t_map_odom_.x());
                }

                has_new_correction = false; 
            }
            }

            // B. 触发逻辑 (自适应频率)
            bool should_trigger = false; // 【必须补上这一行定义！】
            double trace_P = kf.get_P().trace();
            bool system_unstable = (state_point.ba.norm() > 0.3) || (trace_P > 0.5); 
            double interval = system_unstable ? 1.5 : 10.0; // 不稳定时 1.5s，稳定时 10s

            if (current_time - last_reloc_time > interval && !is_reloc_running) {
                should_trigger = true;
            }

            if (is_emergency_mode && !is_reloc_running) {
                kf.set_max_iter(15);
                if (current_time - emergency_start_time > 0.2) {
                    should_trigger = true;
                    is_emergency_mode = false; 
                }
            }

            if (current_time - boot_start_time < BOOT_COOLDOWN) should_trigger = false;

            // ================= [B. 触发逻辑 (带全量日志 & 快照记录)] =================
            if (should_trigger) 
            {
                if (feats_down_body->size() >= 30) 
                {
                    last_reloc_time = current_time;
                    
                    // 1. 准备点云 (保持原逻辑)
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_ptr_final(new pcl::PointCloud<pcl::PointXYZINormal>());
                    if (cloud_buffer.size() >= 2) {
                        Eigen::Matrix3d R_curr_inv = state_point.rot.toRotationMatrix().transpose();
                        Eigen::Vector3d t_curr = state_point.pos;
                        for (const auto& past_frame : cloud_buffer) {
                            Eigen::Matrix3d R_rel = R_curr_inv * past_frame.state.rot.toRotationMatrix();
                            Eigen::Vector3d t_rel = R_curr_inv * (past_frame.state.pos - t_curr);
                            for (const auto& pt : past_frame.cloud_body->points) {
                                Eigen::Vector3d p_curr = R_rel * Eigen::Vector3d(pt.x, pt.y, pt.z) + t_rel;
                                pcl::PointXYZINormal p_out; p_out.x=p_curr.x(); p_out.y=p_curr.y(); p_out.z=p_curr.z(); p_out.intensity=pt.intensity;
                                scan_ptr_final->push_back(p_out);
                            }
                        }
                    } else {
                        scan_ptr_final = TransformToBody(feats_undistort, state_point); 
                    }

                    // 2. 生成图像 & 记录快照
                    cv::Mat img_scan_main, img_map_main;
                    
                    // [关键] 记录生成地图时所用的中心坐标
                    // 无论你这里用的是 state_point.pos 还是修正后的 pos，我们都把它存下来
                    // 这样结算时我们就能确切知道 correction 是相对于哪个点的
                    Eigen::Vector3d map_center_used = state_point.pos; 
                    
                    // [LOG] 记录触发时的状态
                    ULOG_PLOT("Reloc_Process", "0_Trigger_MapCenter_X", map_center_used.x());
                    ULOG_PLOT("Reloc_Process", "0_Trigger_MapCenter_Y", map_center_used.y());
                    
                    this->GenerateScanImage(scan_ptr_final, state_point, img_scan_main, PIXEL_RES, MAP_RANGE, nullptr, nullptr);
                    voxelmap_manager->Generate2DImage(img_map_main, map_center_used, MAP_RANGE, PIXEL_RES, g_init);

                    // 3. 启动线程
                    if (cv::countNonZero(img_scan_main) >= 40) 
                    {
                        RCLCPP_WARN(get_logger(), "Starting Reloc Thread...");
                        
                        // [快照修正] 必须保存两组数据：
                        // 1. 地图是基于哪里生成的 (map_center_snapshot_) -> 用于算 Target
                        // 2. 里程计当时读数是多少 (odom_pos_snapshot_) -> 用于算 Drift
                        // 在开环模式下，这两个值通常相等，但逻辑上必须分开
                        map_center_snapshot_ = map_center_used; 
                        odom_pos_snapshot_   = state_point.pos; 
                        
                        Eigen::Quaterniond q_wb = state_point.rot.inverse(); 
                        odom_yaw_snapshot_ = GetGravityYaw(q_wb, n_up);
                        
                        if (reloc_thread.joinable()) reloc_thread.join();
                        is_reloc_running = true;

                        double search_rad = 10.0; 
                        int roi_px = std::ceil(search_rad * 2.0 / PIXEL_RES);
                        int x_st = std::max(0, img_map_main.cols/2 - roi_px/2);
                        int y_st = std::max(0, img_map_main.rows/2 - roi_px/2);
                        int w_cr = std::min(img_map_main.cols - x_st, roi_px);
                        int h_cr = std::min(img_map_main.rows - y_st, roi_px);
                        cv::Rect roi(x_st, y_st, w_cr, h_cr);
                        cv::Mat m_crop = img_map_main(roi).clone();
                        cv::Mat s_img = img_scan_main(roi).clone();
                        double f_time = Measures.lidar_beg_time;

                        reloc_thread = std::thread([this, s_img, m_crop, PIXEL_RES, f_time]() {    
                            struct FlagGuard { std::atomic<bool>& f; FlagGuard(std::atomic<bool>& _f):f(_f){} ~FlagGuard(){f=false;} };
                            FlagGuard guard(this->is_reloc_running);
                            try {
                                relocalizer.SetInput(s_img, m_crop, f_time);
                                auto res = relocalizer.Solve(PIXEL_RES);
                                if (res.success) {
                                    std::lock_guard<std::mutex> lock(reloc_mutex);
                                    correction_pos = Eigen::Vector3d(res.x, res.y, 0); 
                                    correction_yaw = res.yaw_deg;
                                    correction_score = res.score;
                                    correction_uncertainty = res.uncertainty;
                                    has_new_correction = true;
                                }
                            } catch (...) {}
                        });
                    }
                    else{
                        RCLCPP_WARN(get_logger(), "Img Empty! Pixels: %d", cv::countNonZero(img_scan_main));
                    }
                }
            }
            // ================= [Defense Layer 4: 拓扑流形保护 (最终版)] =================
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            // ================= [Defense Layer 4: 仅数值熔断保护] =================
            // 检查：是否出现 NaN (非数字) 或 Inf (无穷大)
            bool is_nan = std::isnan(state_point.pos.x()) || std::isnan(state_point.rot.w());
            bool is_inf = std::isinf(state_point.pos.norm());

            if (is_nan || is_inf) {
                 RCLCPP_ERROR(get_logger(), "[CRITICAL] EKF Math Breakdown! Hard Resetting EKF.");
                 
                 // 只有在数学崩溃时，才允许硬复位 EKF
                 state_ikfom s_reset = kf.get_x();
                 s_reset.pos = Eigen::Vector3d::Zero();
                 s_reset.vel = Eigen::Vector3d::Zero();
                 s_reset.rot = Eigen::Quaterniond::Identity();
                 s_reset.ba.setZero(); 
                 s_reset.bg.setZero();
                 kf.change_x(s_reset);
                 
                 // 重置协方差
                 auto P_reset = kf.get_P();
                 P_reset.setIdentity(); P_reset *= 0.01;
                 kf.change_P(P_reset);
                 
                 // 既然 EKF 都崩了，之前的漂移量也没意义了，归零
                 {
                     std::lock_guard<std::mutex> lock(drift_mutex_);
                     t_map_odom_.setZero();
                     q_map_odom_.setIdentity();
                 }
                 if (p_imu) p_imu->first_lidar_time = Measures.lidar_beg_time;
                 return;
            }
            // 【重点】删除了所有关于 real_radar_height, tilt_angle, is_pos_exploded 的 if 判断
            // 这些几何异常现在完全交给后面的 "双坐标系漂移更新" 来平滑修正，绝不硬复位！
            // ================= [Defense Layer 3: 带死区的指数熔断 (v26.1 终极版)] =================
            // 策略：抬高门槛，避免正常震动误触发。保留 P 阵钳制作为最后防线。

            // ---------------- [Step 1: 计算柔性动态因子] ----------------
            // 1. 旋转风险 (RMS)
            // 注意：gyr_energy 之前计算的是均值(Mean)，这里直接用作风险评估
            // 阈值 1.0 rad/s (60deg/s)
            gyr_val = std::abs(gyr_energy);
            double risk_gyr = std::pow(gyr_val / 1.0, 2); 
            
            // 2. 震动风险 (Robust Peak)
            // [关键修改] 引入 2.0 m/s^2 的硬性死区 (避免路面颠簸误触发)
            // - 震动 < 2.0 (0.2g): Risk = 0.0 -> Scaler = 1.0 (雷达满血)
            // - 震动 = 5.0 (0.5g): Risk = (3/3)^2 = 1.0 -> Scaler ~ 13.0 (迅速响应)
            // - 震动 = 25.0 (Crash): Risk = (23/3)^2 = 58 -> Scaler = 20.0 (封顶)
            
            double shock_excess = std::max(0.0, robust_peak - 2.0);
            double risk_shock = std::pow(shock_excess / 3.0, 2); 

            total_risk = risk_shock;

            // ---------------- [Step 2: 指数熔断计算] ----------------
            // 公式：S = 1 + (Max - 1) * (1 - exp(-Risk))
            double max_scaler = 8.0; 
            double fuse_ratio = 1.0 - std::exp(-total_risk);
            double scaler = 1.0 + (max_scaler - 1.0) * fuse_ratio;
            ULOG_PLOT("Shock_Test", "Final_Robust_Peak", robust_peak);
            ULOG_PLOT("Shock_Test", "Final_R_Scaler", scaler);
            // 双重硬限幅 (防止数学溢出)
            if (scaler > 25.0) scaler = 25.0;
            
            
            // 调试日志
            ULOG_PLOT("Defense_Action", "Robust_Peak", robust_peak);
            ULOG_PLOT("Defense_Action", "R_Scaler", scaler);
            PerformSoftBiasConstraint(kf, robust_peak, gyr_energy);
            // ---------------- [Step 3: 执行 EKF 更新 (必须保留)] ----------------
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            // ================= [核心复位逻辑：适配 Pos在前，Rot在后] =================
    
            // 1. 获取当前状态检查
            state_ikfom st_check = kf.get_x();
            Eigen::Matrix<double, 23, 23> P_check = kf.get_P();

            // 2. 更新启动保护计数
            if (startup_protection_counter < PROTECTION_FRAMES) {
                startup_protection_counter++;
            }

            // 3. 定义熔断判据
            // A. 速度熔断: > 5.0 m/s (正常极速的 ~1.5 倍)
            bool vel_crash = (st_check.vel.norm() > 10.0);

            // B. 协方差熔断: 检查速度协方差 (Index 12-14)
            //    必须检查 trace 是否为 NaN，防止 NaN 传染
            double cov_vel_trace = P_check.block<3,3>(12,12).trace();
            bool cov_crash = std::isnan(cov_vel_trace);

            // C. 数值熔断: 位置或速度出现 NaN (最高优先级)
            bool nan_crash = std::isnan(st_check.pos.norm()) || std::isnan(st_check.vel.norm());

            // 4. 综合触发开关
            // 逻辑：(非保护期 && (飞车 || 协方差爆炸)) || (任何时候出现了NaN)
            bool is_flying = (vel_crash || cov_crash) && (startup_protection_counter >= PROTECTION_FRAMES);
            if (nan_crash) is_flying = true;
            //std::cout<<vel_crash<<"\t"<<cov_vel_trace<<"\t"<<cov_crash<<"\t"<<"\n"<<nan_crash<<"\t"<<is_flying<<"\n";
            // 5. 执行复位手术
            if (is_flying) {
                RCLCPP_WARN(rclcpp::get_logger("LaserMapping"), 
                    "\033[1;31m[RESET TRIGGERED] Vel:%.2f, Cov:%.2f, NaN:%d. System Resetting...\033[0m", 
                    st_check.vel.norm(), cov_vel_trace, nan_crash);
                
                // --- 步骤 A: 状态量重置 (State Reset) ---
                // 目标：将所有状态恢复到“静止、水平、无偏置”的各种安全初值
                
                st_check.vel.setZero();      // 速度归零
                st_check.pos.setZero();      // 位置归零 (或者保留上一帧有效值，但在NaN时必须归零)
                st_check.rot.setIdentity();  // 【关键】姿态重置为单位四元数 [1,0,0,0]，防止NaN残留
                st_check.bg.setZero();       // 陀螺仪零偏归零
                st_check.ba.setZero();       // 加速度计零偏归零
                // ✅ 正确代码：显式构造 S2 对象
                // S2 会自动归一化输入的向量，所以 (0, 0, -9.81) 会被转换成垂直向下的单位方向
                st_check.grav = std::decay_t<decltype(st_check.grav)>(Eigen::Vector3d(0.0, 0.0, -9.81));
                
                // --- 步骤 B: 协方差重置 (P-Matrix Surgery) ---
                // 【注意】这里严格按照您的结构体顺序: 0-2=Pos, 3-5=Rot, 12-14=Vel
                
                Eigen::Matrix<double, 23, 23> P_reset;
                P_reset.setIdentity();

                // 用语义代替数字
                P_reset.block<3,3>(StateIdx::POS, StateIdx::POS) *= 0.1;
                P_reset.block<3,3>(StateIdx::ROT, StateIdx::ROT) *= 0.0001;
                P_reset.block<3,3>(StateIdx::VEL, StateIdx::VEL) *= 1000.0;
                P_reset.block<3,3>(StateIdx::BG,  StateIdx::BG)  *= 0.0001; 
                P_reset.block<3,3>(StateIdx::BA,  StateIdx::BA)  *= 0.0001;
                                
                // --- 步骤 C: 应用变更 ---
                kf.change_x(st_check);
                kf.change_P(P_reset);
                
                // --- 步骤 D: 紧急避险 ---
                // 如果是因为 NaN 复位的，当前帧的数据已经不可信，
                // 直接 return 跳过本次 Update，等待下一帧健康的 IMU/Lidar 数据。
                if (nan_crash) {
                    return; 
                }
            }
            // 找到并删除这两行：
            // double base_cov = 0.001; 
            // double final_update_noise = base_cov * scaler;

            // 替换为：
            // 【核心修复：白化后的基底噪声必须是 1.0，绝不能是 0.001】
            double final_update_noise = 1.0; 
            if (scaler > 1.0) {
                final_update_noise = scaler; // 发生撞击时，降低对雷达的信任
            }
            // 【新增探针 2】：记录 Update 前的 EKF 状态
            state_ikfom pre_update_state = kf.get_x();
            Eigen::Matrix<double, 23, 23> pre_P = kf.get_P();
            
            // A. 记录重力流形 S2 的实际模长（极其关键！）
            // 如果这个值输出是 9.81，这就证明了流形空间被撕裂，雅可比会被放大
            Eigen::Vector3d pre_grav = pre_update_state.grav; 
            ULOG_PLOT("EKF_Update_Check", "Pre_Grav_Norm", pre_grav.norm());

            // B. 记录即将送入底层的观测噪声权重 (R)
            // 确认是否因为传入了极小的值 (如 0.001) 导致系统盲目自信
            double current_noise_scaler = final_update_noise; // 使用我们修复后的新变量名
            ULOG_PLOT("EKF_Update_Check", "Input_Noise_R", current_noise_scaler);

            // C. 记录先验协方差矩阵 P 的健康度 (对角线之和)
            ULOG_PLOT("EKF_Weight", "IMU_Pre_P_Trace", pre_P.trace());

            // ---------------------------------------------------------
            // 执行核心更新 (原有代码)
            ULOG_PLOT("EKF_Weight", "Lidar_Noise_R", final_update_noise);
            kf.update_iterated_dyn_share_modified(final_update_noise, solve_H_time);        
            // ---------------------------------------------------------
            // kf.update_iterated_dyn_share_modified(..., ...); (原有代码)
            // ---------------------------------------------------------

            // 【新增探针 3】：计算并记录单次 Update 导致的状态突变量 (Delta X)
            state_ikfom post_update_state = kf.get_x();
            
            // A. 速度的瞬间突变模长 (单位: m/s)
            double delta_vel = (post_update_state.vel - pre_update_state.vel).norm();
            ULOG_PLOT("EKF_Update_Check", "Delta_Vel_Norm", delta_vel);

            // B. 位置的瞬间突变模长 (单位: m)
            double delta_pos = (post_update_state.pos - pre_update_state.pos).norm();
            ULOG_PLOT("EKF_Update_Check", "Delta_Pos_Norm", delta_pos);

            // C. 姿态的瞬间扭转角度 (单位: 度)
            // 两个旋转的差值：R_diff = R_old^T * R_new
            Eigen::AngleAxisd rot_diff(pre_update_state.rot.conjugate() * post_update_state.rot);
            double delta_rot_deg = rot_diff.angle() * 180.0 / M_PI;
            ULOG_PLOT("EKF_Update_Check", "Delta_Rot_Deg", delta_rot_deg);

            // D. 协方差的健康度
            ULOG_PLOT("EKF_Update_Check", "Post_P_Trace", kf.get_P().trace());
            Eigen::Matrix<double, 23, 23> P_curr = kf.get_P();
            // bool p_clamped = false;
            
            // // 位置方差钳制 (约30cm) -> 使用严格的对称缩放法 S*P*S^T
            // for(int i=0; i<3; i++) {
            //     if (P_curr(i,i) > 0.1) { 
            //         double scale = std::sqrt(0.1 / P_curr(i,i));
            //         P_curr.row(i) *= scale;  // 缩放整行
            //         P_curr.col(i) *= scale;  // 缩放整列
            //         P_curr(i,i) = 0.1;       // 抹除精度误差
            //         p_clamped = true; 
            //     }
            // }
            // // 姿态方差钳制 (约5.7度)
            // for(int i=3; i<6; i++) {
            //     if (P_curr(i,i) > 0.01) { 
            //         double scale = std::sqrt(0.01 / P_curr(i,i));
            //         P_curr.row(i) *= scale;
            //         P_curr.col(i) *= scale;
            //         P_curr(i,i) = 0.01; 
            //         p_clamped = true; 
            //     }
            // }
            
            // if (p_clamped) {
            //     kf.change_P(P_curr);
            // }
            // ---------------- [Step 5: 状态更新与 NaN 熔断 (必须保留)] ----------------
            state_point = kf.get_x();
            
            // NaN 熔断保护
            if (std::isnan(state_point.pos.x()) || std::isnan(state_point.rot.w())) {
                 RCLCPP_ERROR(this->get_logger(), "[CRITICAL] EKF NaN Detected! Reset.");
                 state_point.pos = Eigen::Vector3d::Zero();
                 state_point.vel = Eigen::Vector3d::Zero();
                 state_point.rot = Eigen::Quaterniond::Identity(); 
                 state_point.ba.setZero();
                 state_point.bg.setZero();
                 kf.change_x(state_point);
                 
                 Eigen::Matrix<double, 23, 23> P_reset = Eigen::Matrix<double, 23, 23>::Identity() * 0.01;
                 kf.change_P(P_reset);
            }

            last_lidar_pos_ = state_point.pos;
            last_lidar_update_time_ = Measures.lidar_beg_time;
            
            // ---------------- [Step 6: ZUPT 闭锁联动] ----------------
            // 注意：因为 v24.0 的 scaler 最大只有 15.0，所以阈值要相应调整。
            // 之前的阈值是 10.0 (在 10000 的量级下)。
            // 现在建议：只要 scaler > 3.0 (说明有显著动态)，就锁死 ZUPT。
            if (scaler > 3.0) zupt_lockout_timer_ = 1.0;
            // 恢复常规迭代次数 (重要)
            if ( kf.get_max_iter() > 3) {
                 kf.set_max_iter(3);
            }
            
            // 更新 state_point 供后续使用
            state_point = kf.get_x();
            // ================= [调试代码插入开始] =================
            // 目的：在高动态旋转中监控 Rot 的表现，验证是否需要复位 Rot

            // 1. 获取欧拉角 (Roll, Pitch, Yaw) - 假设你有 SO3ToEuler 工具函数
            // 如果没有，请使用 Eigen 的转换: state_point.rot.toRotationMatrix().eulerAngles(0, 1, 2);
            Eigen::Vector3d euler_deg = SO3ToEuler(state_point.rot) * 57.29578; // 弧度转度
            
            ULOG_PLOT("Rot_Analysis", "Euler_Roll",  euler_deg.x());
            ULOG_PLOT("Rot_Analysis", "Euler_Pitch", euler_deg.y());
            ULOG_PLOT("Rot_Analysis", "Euler_Yaw",   euler_deg.z());

            // 2. 记录四元数 (原始姿态数据)
            ULOG_PLOT("Rot_Analysis", "Quat_w", state_point.rot.w());
            ULOG_PLOT("Rot_Analysis", "Quat_x", state_point.rot.x());
            ULOG_PLOT("Rot_Analysis", "Quat_y", state_point.rot.y());
            ULOG_PLOT("Rot_Analysis", "Quat_z", state_point.rot.z());

            // 3. [关键判定指标] 机体 Z 轴指向哪里？
            // 计算 Body 坐标系的 Z 轴 (0,0,1) 在 World 坐标系下的向量表示
            // Z_body_in_world = R_WB * [0,0,1]^T = R_WB 的第三列
            Eigen::Matrix3d R_WB_debug = state_point.rot.toRotationMatrix();
            Eigen::Vector3d Z_axis_world = R_WB_debug.col(2);

            ULOG_PLOT("Rot_Analysis", "Body_Z_in_World_x", Z_axis_world.x());
            ULOG_PLOT("Rot_Analysis", "Body_Z_in_World_y", Z_axis_world.y());
            ULOG_PLOT("Rot_Analysis", "Body_Z_in_World_z", Z_axis_world.z());

            // 4. 对比：速度与姿态的关系
            // 如果姿态歪了，通常伴随着那个方向的速度激增
            ULOG_PLOT("Rot_Analysis", "Vel_x", state_point.vel.x());
            ULOG_PLOT("Rot_Analysis", "Vel_y", state_point.vel.y());
            ULOG_PLOT("Rot_Analysis", "Vel_z", state_point.vel.z());
            
            // 尝试使用下标 [0], [1], [2] 代替 .x(), .y(), .z()
            ULOG_PLOT("Rot_Analysis", "Grav_x", state_point.grav[0]);
            ULOG_PLOT("Rot_Analysis", "Grav_y", state_point.grav[1]);
            ULOG_PLOT("Rot_Analysis", "Grav_z", state_point.grav[2]);
            
            // ================= [调试代码插入结束] =================
            LOG_STATE("UPDATE", state_point);

            // 关键：检查 Bias 是否在这个瞬间突变
            LOG_VEC("UPDATE", "BIAS_ACC", state_point.ba);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            // 找到您原本给 geoQuat 赋值的地方：
            // geoQuat.x = state_point.rot.coeffs()[0];
            // geoQuat.y = state_point.rot.coeffs()[1];
            // geoQuat.z = state_point.rot.coeffs()[2];
            // geoQuat.w = state_point.rot.coeffs()[3];

            // ---------------- [修复补丁：四元数连续性保证] ----------------
            Eigen::Quaterniond curr_q;
            curr_q.x() = state_point.rot.coeffs()[0];
            curr_q.y() = state_point.rot.coeffs()[1];
            curr_q.z() = state_point.rot.coeffs()[2];
            curr_q.w() = state_point.rot.coeffs()[3];

            // 静态变量，保存上一帧的四元数
            static Eigen::Quaterniond last_pub_q(1.0, 0.0, 0.0, 0.0);

            // 计算当前四元数与上一帧的点积
            double dot_product = last_pub_q.w() * curr_q.w() + 
                                last_pub_q.x() * curr_q.x() + 
                                last_pub_q.y() * curr_q.y() + 
                                last_pub_q.z() * curr_q.z();

            // 如果点积小于 0，说明发生了符号翻转（走向了对立半球）
            // 强行将当前四元数取反，保持数学表达的连续性，但不改变物理旋转
            if (dot_product < 0.0) {
                curr_q.w() = -curr_q.w();
                curr_q.x() = -curr_q.x();
                curr_q.y() = -curr_q.y();
                curr_q.z() = -curr_q.z();
            }

            // 记录本次的四元数供下一帧比较
            last_pub_q = curr_q;

            // 输出给 ROS 的 message
            geoQuat.w = curr_q.w();
            geoQuat.x = curr_q.x();
            geoQuat.y = curr_q.y();
            geoQuat.z = curr_q.z();
            // -------------------------------------------------------------

            double t_update_end = omp_get_wtime();
            
            //std::cout<<5<<endl;
            /******* Publish odometry *******/
            // Using getters to access publishers from global functions if needed, 
            // but for simplicity calling global wrapper functions which need update to take args
            // OR passing member publishers to modified global functions.
            // Since requirements say "don't destroy global nature", we use the members here.
            publish_odometry(pubOdomAftMapped_);

            /*** add the feature points to map kdtree ***/
            //std::cout<<6.4<<endl;
            transformLidar(state_point.rot.toRotationMatrix(), state_point.pos, feats_down_body, feats_down_world);
            // ------------------ 修改开始 ------------------

            // 1. 获取当前点云的大小
            size_t current_points_size = feats_down_world->points.size();

            // 2. [关键修复] 调整 vector 大小，分配内存！
            // 如果不resize，直接用下标[i]访问必定崩溃
            voxelmap_manager->pv_list_.resize(current_points_size);

            // 3. 同时也建议检查一下其他依赖的 list 是否大小匹配 (防止下一行崩溃)
            if (voxelmap_manager->cross_mat_list_.size() < current_points_size || 
                voxelmap_manager->body_cov_list_.size() < current_points_size) {
                // 这是一个保护措施，如果 EKF 阶段计算的列表大小不对，resize 它们或报错
                // 这里假设默认用单位阵或者零填充，防止 crash
                voxelmap_manager->cross_mat_list_.resize(current_points_size, Eigen::Matrix3d::Zero());
                voxelmap_manager->body_cov_list_.resize(current_points_size, Eigen::Matrix3d::Identity());
            }

            // << "6.3" << endl;
            for (size_t i = 0; i < current_points_size; i++)
            {
                //std::cout << "6.31" << endl;
                
                // 现在这里安全了，因为 pv_list_ 已经分配了内存
                voxelmap_manager->pv_list_[i].point_w << feats_down_world->points[i].x, 
                                                        feats_down_world->points[i].y, 
                                                        feats_down_world->points[i].z;
                
                //std::cout << "6.32" << endl;
                
                // 注意：确保 cross_mat_list_ 和 body_cov_list_ 在之前的步骤(h_share_model)中已经被正确填充了
                // 否则下面这几行虽然不会段错误，但可能会计算错误
                M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
                //std::cout << "6.33" << endl;
                
                M3D var = voxelmap_manager->body_cov_list_[i];
                //std::cout << "6.34" << endl;

                var = (state_point.rot.toRotationMatrix() * extR) * var * (state_point.rot.toRotationMatrix() * extR).transpose() +
                    (-point_crossmat) * kf.get_P().block<3, 3>(StateIdx::ROT, StateIdx::ROT) * (-point_crossmat).transpose() + 
                    kf.get_P().block<3, 3>(StateIdx::POS, StateIdx::POS);   
                //std::cout << "6.35" << endl;
                voxelmap_manager->pv_list_[i].var = var;
            }
            // ------------------ 修改结束 ------------------
            // ... (EKF Update 结束) ...
            
            state_point = kf.get_x();
            {
                // 重新计算一遍更新后的高度 (逻辑与上面保持一致)
                Eigen::Matrix3d R_WB_new = state_point.rot.toRotationMatrix().transpose();
                Eigen::Vector3d n_up_new = g_camera_init.normalized(); // 假设 state.grav 指向地心
                Eigen::Vector3d P_radar_new = state_point.pos + R_WB_new * state_point.offset_T_L_I;
                double height_after_update = P_radar_new.dot(n_up_new);

                ULOG_PLOT("Height_Debug", "Post_Update_Height", height_after_update);
                ULOG_PLOT("Height_Debug", "Post_Update_Vel_Z", state_point.vel.z());
            }
            LOG_STATE("UPDATE", state_point);
            // =========================================================
            // [模块：智能地图更新门控]
            // =========================================================
            
            // 1. 封装当前帧数据
            MapUpdatePacket packet;
            packet.timestamp = current_time;
            
            // [判断]：是否应该跳过地图写入？
            // 逻辑：如果数学算法判定当前是"绝对静止"（p_static > 0.5），它会将速度强制归零。
            // 此时写入地图不仅多余，而且如果系统处于"漂移被按住"的状态，写入的点云可能是歪的。
            // 所以：速度极小 = 认为是静止/被按住 = 跳过建图（但定位依然在跑）
            bool is_static_locked = (state_point.vel.norm() < 0.001); 
            
            packet.skip_map_update = is_static_locked; // 标记该帧

            // [修复核心] 直接拷贝 pv_list_ 
            packet.cached_pv_list = voxelmap_manager->pv_list_; 
            
            // 2. 入队缓存 (必须入队，因为要保持时间顺序)
            map_update_queue.push_back(packet);

            // 3. 延迟处理队列
            while (!map_update_queue.empty()) {
                MapUpdatePacket& front_packet = map_update_queue.front();
                
                // 延迟 2.0 秒写入，等待状态稳定
                if (current_time - front_packet.timestamp > MAP_UPDATE_DELAY) {
                    
                    // [核心门控] 只有非静止锁定的帧，才写入全局地图
                    if (!front_packet.skip_map_update) {
                        voxelmap_manager->UpdateVoxelMap(front_packet.cached_pv_list);
                        
                        // 地图滑窗 (仅在有效写入时执行)
                        if(voxelmap_manager->config_setting_.map_sliding_en) {
                            voxelmap_manager->mapSliding();
                        }
                    } 
                    // else { 静止帧，直接丢弃，节省算力，保护地图 }
                    
                    map_update_queue.pop_front();
                    
                } 
                else {
                    break; // 还没到时间
                }
            }
           

            t3 = omp_get_wtime();
            //map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            // publish_effect_world(pubLaserCloudEffect_);
            // publish_map(pubLaserCloudMap_);
            // ================= [Defense Layer 4: 物理边界钳制] =================
            state_point = kf.get_x(); 
            bool need_reset = false;
            
            // 记录 Bias 的模长，方便事后看是否接近临界值
            ULOG_PLOT("Bias_Monitor", "Acc_Bias_Norm", state_point.ba.norm());
            ULOG_PLOT("Bias_Monitor", "Gyr_Bias_Norm", state_point.bg.norm());

            // 1. Bias 钳制
            if (state_point.ba.norm() > 0.5) {
                RCLCPP_ERROR(this->get_logger(), "[Safety] ACC BIAS EXPLODED (%.2f)! CLAMPING...", state_point.ba.norm());
                state_point.ba = state_point.ba.normalized() * 0.5;
                need_reset = true;
            }
            if (state_point.bg.norm() > 0.1) {
                // Gyr Bias 一般很小，超过 0.1 肯定不对
                state_point.bg = state_point.bg.normalized() * 0.1;
                need_reset = true;
            }

            // 2. 速度钳制
            if (state_point.vel.norm() > 4.0) {
                RCLCPP_ERROR(this->get_logger(), "[Safety] VELOCITY EXPLOSION (%.2f)! CLAMPING...", state_point.vel.norm());
                state_point.vel = state_point.vel.normalized() * 4.0;
                need_reset = true;
            }

            if (need_reset) {
                kf.change_x(state_point);
                auto P = kf.get_P();
                P.block<3,3>(StateIdx::VEL, StateIdx::VEL) += Eigen::Matrix3d::Identity() * 0.01;
                kf.change_P(P);
                // 记录一次 Reset 事件
                ULOG_PLOT("Defense_Action", "Safety_Reset", 1.0);
            } else {
                ULOG_PLOT("Defense_Action", "Safety_Reset", 0.0);
            }
            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                Eigen::Vector3d ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out_ << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp_);
            }
        }
    

    }

};

std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
// 全局变量区域保持不变...
// VoxelMapManagerPtr voxelmap_manager; // 确保这是 shared_ptr

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // 1. 创建节点 (构造函数运行，读取参数到 extrinT，但不操作 voxelmap_manager)
    auto node = std::make_shared<LaserMappingNode>();
    node_ptr = node;
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    // 2. 加载 VoxelMap 配置
    // 假设 loadVoxelConfig 从 node 获取参数并填充 voxel_config
    loadVoxelConfig(node, voxel_config); 
    
    // 3. [关键修复] 初始化 voxelmap_manager 实例 (分配内存)
    // 必须在 node 创建之后 (有了参数)，且在使用 manager 之前
    voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
    
    // 4. [关键修复] 现在 manager 存在了，手动调用函数设置外参
    // 之前这段代码在构造函数里导致崩溃，现在挪到这里
    node->init_voxelmap_extrinsics(); // 调用我们在 Node 中新增的辅助函数

    // 处理全局变量 (可选，如果其他地方用到)
    extT << VEC_FROM_ARRAY(extrinT);
    extR << MAT_FROM_ARRAY(extrinR);

    // 绑定信号
    signal(SIGINT, SigHandle);

    // 5. 开始循环
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}


// std::shared_ptr<rclcpp::Node> node_ptr;
// std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
// #define INIT_TIME           (0.1)
// #define LASER_POINT_COV     (0.001)
// #define MAXN                (720000)
// #define PUBFRAME_PERIOD     (20)
// /*** Time Log Variables ***/
// double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
// double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
// double match_time = 0, solve_time = 0, solve_const_H_time = 0;
// int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
// bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
// /**************************/
// float res_last[100000] = {0.0};
// float DET_RANGE = 300.0f;
// const float MOV_THRESHOLD = 1.5f;
// double time_diff_lidar_to_imu = 0.0;
// mutex mtx_buffer;
// condition_variable sig_buffer;
// string root_dir = ROOT_DIR;
// string map_file_path, lid_topic, imu_topic;
// double res_mean_last = 0.05, total_residual = 0.0;
// double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
// double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
// double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
// double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
// int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
// int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
// bool   point_selected_surf[100000] = {0};
// bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
// bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
// vector<vector<int>>  pointSearchInd_surf; 
// vector<BoxPointType> cub_needrm;
// vector<PointVector>  Nearest_Points; 
// vector<double>       extrinT(3, 0.0);
// vector<double>       extrinR(9, 0.0);
// deque<double>                     time_buffer;
// deque<PointCloudXYZI::Ptr>        lidar_buffer;
// deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer; // ROS 2 SharedPtr
// PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
// PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
// PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
// PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
// PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
// PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
// PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
// PointCloudXYZI::Ptr _featsArray;
// pcl::VoxelGrid<PointType> downSizeFilterSurf;
// pcl::VoxelGrid<PointType> downSizeFilterMap;
// KD_TREE<PointType> ikdtree;
// V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
// V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
// V3D euler_cur;
// V3D position_last(Zero3d);
// V3D Lidar_T_wrt_IMU(Zero3d);
// M3D Lidar_R_wrt_IMU(Eye3d);
// /*** EKF inputs and output ***/
// MeasureGroup Measures;
// esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
// state_ikfom state_point;
// vect3 pos_lid;
// nav_msgs::msg::Path path;
// nav_msgs::msg::Odometry odomAftMapped;
// geometry_msgs::msg::Quaternion geoQuat;
// geometry_msgs::msg::PoseStamped msg_body_pose;
// shared_ptr<Preprocess> p_pre(new Preprocess());
// shared_ptr<ImuProcess> p_imu(new ImuProcess());
// void SigHandle(int sig)
// {
//     flg_exit = true;
//     RCLCPP_WARN(node_ptr->get_logger(), "catch sig %d", sig);
//     sig_buffer.notify_all();
//     rclcpp::shutdown();
// }
// inline void dump_lio_state_to_log(FILE *fp)  
// {
//     V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
//     fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
//     fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
//     fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
//     fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
//     fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
//     fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
//     fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
//     fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
//     fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
//     fprintf(fp, "\r\n");  
//     fflush(fp);
// }
// void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
// {
//     V3D p_body(pi->x, pi->y, pi->z);
//     V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
//     po->x = p_global(0);
//     po->y = p_global(1);
//     po->z = p_global(2);
//     po->intensity = pi->intensity;
// }
// void pointBodyToWorld(PointType const * const pi, PointType * const po)
// {
//     V3D p_body(pi->x, pi->y, pi->z);
//     V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
//     po->x = p_global(0);
//     po->y = p_global(1);
//     po->z = p_global(2);
//     po->intensity = pi->intensity;
// }
// template<typename T>
// void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
// {
//     V3D p_body(pi[0], pi[1], pi[2]);
//     V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
//     po[0] = p_global(0);
//     po[1] = p_global(1);
//     po[2] = p_global(2);
// }
// void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
// {
//     V3D p_body(pi->x, pi->y, pi->z);
//     V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
//     po->x = p_global(0);
//     po->y = p_global(1);
//     po->z = p_global(2);
//     po->intensity = pi->intensity;
// }
// void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
// {
//     V3D p_body_lidar(pi->x, pi->y, pi->z);
//     V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);
//     po->x = p_body_imu(0);
//     po->y = p_body_imu(1);
//     po->z = p_body_imu(2);
//     po->intensity = pi->intensity;
// }
// void points_cache_collect()
// {
//     PointVector points_history;
//     ikdtree.acquire_removed_points(points_history);
//     // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
// }
// BoxPointType LocalMap_Points;
// bool Localmap_Initialized = false;
// void lasermap_fov_segment()
// {
//     cub_needrm.clear();
//     kdtree_delete_counter = 0;
//     kdtree_delete_time = 0.0;    
//     pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
//     V3D pos_LiD = pos_lid;
//     if (!Localmap_Initialized){
//         for (int i = 0; i < 3; i++){
//             LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
//             LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
//         }
//         Localmap_Initialized = true;
//         return;
//     }
//     float dist_to_map_edge[3][2];
//     bool need_move = false;
//     for (int i = 0; i < 3; i++){
//         dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
//         dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
//         if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
//     }
//     if (!need_move) return;
//     BoxPointType New_LocalMap_Points, tmp_boxpoints;
//     New_LocalMap_Points = LocalMap_Points;
//     float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
//     for (int i = 0; i < 3; i++){
//         tmp_boxpoints = LocalMap_Points;
//         if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
//             New_LocalMap_Points.vertex_max[i] -= mov_dist;
//             New_LocalMap_Points.vertex_min[i] -= mov_dist;
//             tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
//             cub_needrm.push_back(tmp_boxpoints);
//         } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
//             New_LocalMap_Points.vertex_max[i] += mov_dist;
//             New_LocalMap_Points.vertex_min[i] += mov_dist;
//             tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
//             cub_needrm.push_back(tmp_boxpoints);
//         }
//     }
//     LocalMap_Points = New_LocalMap_Points;
//     points_cache_collect();
//     double delete_begin = omp_get_wtime();
//     if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
//     kdtree_delete_time = omp_get_wtime() - delete_begin;
// }
// void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) 
// {
//     mtx_buffer.lock();
//     scan_count ++;
//     double preprocess_start_time = omp_get_wtime();
//     double msg_time_sec = rclcpp::Time(msg->header.stamp).seconds();
//     if (msg_time_sec < last_timestamp_lidar)
//     {
//         RCLCPP_ERROR(node_ptr->get_logger(), "lidar loop back, clear buffer");
//         lidar_buffer.clear();
//     }
//     PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
//     p_pre->process(msg, ptr);
//     lidar_buffer.push_back(ptr);
//     time_buffer.push_back(msg_time_sec);
//     last_timestamp_lidar = msg_time_sec;
//     s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
//     mtx_buffer.unlock();
//     sig_buffer.notify_all();
// }
// double timediff_lidar_wrt_imu = 0.0;
// bool   timediff_set_flg = false;
// void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) 
// {
//     mtx_buffer.lock();
//     double preprocess_start_time = omp_get_wtime();
//     scan_count ++;
//     double msg_time_sec = rclcpp::Time(msg->header.stamp).seconds();
//     if (msg_time_sec < last_timestamp_lidar)
//     {
//         RCLCPP_ERROR(node_ptr->get_logger(), "lidar loop back, clear buffer");
//         lidar_buffer.clear();
//     }
//     last_timestamp_lidar = msg_time_sec;
//     if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
//     {
//         printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
//     }
//     if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
//     {
//         timediff_set_flg = true;
//         timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
//         printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
//     }
//     PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
//     p_pre->process(msg, ptr);
//     lidar_buffer.push_back(ptr);
//     time_buffer.push_back(last_timestamp_lidar);
//     s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
//     mtx_buffer.unlock();
//     sig_buffer.notify_all();
// }
// void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in) 
// {
//     publish_count ++;
//     // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
//     sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
//     double msg_time_sec = rclcpp::Time(msg_in->header.stamp).seconds();
//     // Note: ROS 2 Time construction from seconds
//     msg->header.stamp = rclcpp::Time(static_cast<int64_t>((msg_time_sec - time_diff_lidar_to_imu) * 1e9));
//     if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
//     {
//         msg->header.stamp = \
//         rclcpp::Time(static_cast<int64_t>((timediff_lidar_wrt_imu + msg_time_sec) * 1e9));
//     }
//     double timestamp = rclcpp::Time(msg->header.stamp).seconds();
//     mtx_buffer.lock();
//     if (timestamp < last_timestamp_imu)
//     {
//         RCLCPP_WARN(node_ptr->get_logger(), "imu loop back, clear buffer");
//         imu_buffer.clear();
//     }
//     last_timestamp_imu = timestamp;
//     imu_buffer.push_back(msg);
//     mtx_buffer.unlock();
//     sig_buffer.notify_all();
// }
// double lidar_mean_scantime = 0.0;
// int    scan_num = 0;
// bool sync_packages(MeasureGroup &meas)
// {
//     if (lidar_buffer.empty() || imu_buffer.empty()) {
//         return false;
//     }
//     /*** push a lidar scan ***/
//     if(!lidar_pushed)
//     {
//         meas.lidar = lidar_buffer.front();
//         meas.lidar_beg_time = time_buffer.front();
//         if (meas.lidar->points.size() <= 1) // time too little
//         {
//             lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
//             RCLCPP_WARN(node_ptr->get_logger(), "Too few input point cloud!\n");
//         }
//         else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
//         {
//             lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
//         }
//         else
//         {
//             scan_num ++;
//             lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
//             lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
//         }
//         meas.lidar_end_time = lidar_end_time;
//         lidar_pushed = true;
//     }
//     if (last_timestamp_imu < lidar_end_time)
//     {
//         return false;
//     }
//     /*** push imu data, and pop from imu buffer ***/
//     double imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
//     meas.imu.clear();
//     while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
//     {
//         imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
//         if(imu_time > lidar_end_time) break;
//         meas.imu.push_back(imu_buffer.front());
//         imu_buffer.pop_front();
//     }
//     lidar_buffer.pop_front();
//     time_buffer.pop_front();
//     lidar_pushed = false;
//     return true;
// }
// int process_increments = 0;
// void map_incremental()
// {
//     PointVector PointToAdd;
//     PointVector PointNoNeedDownsample;
//     PointToAdd.reserve(feats_down_size);
//     PointNoNeedDownsample.reserve(feats_down_size);
//     for (int i = 0; i < feats_down_size; i++)
//     {
//         /* transform to world frame */
//         pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
//         /* decide if need add to map */
//         if (!Nearest_Points[i].empty() && flg_EKF_inited)
//         {
//             const PointVector &points_near = Nearest_Points[i];
//             bool need_add = true;
//             BoxPointType Box_of_Point;
//             PointType downsample_result, mid_point; 
//             mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
//             mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
//             mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
//             float dist  = calc_dist(feats_down_world->points[i],mid_point);
//             if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
//                 PointNoNeedDownsample.push_back(feats_down_world->points[i]);
//                 continue;
//             }
//             for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
//             {
//                 if (points_near.size() < NUM_MATCH_POINTS) break;
//                 if (calc_dist(points_near[readd_i], mid_point) < dist)
//                 {
//                     need_add = false;
//                     break;
//                 }
//             }
//             if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
//         }
//         else
//         {
//             PointToAdd.push_back(feats_down_world->points[i]);
//         }
//     }
//     double st_time = omp_get_wtime();
//     add_point_size = ikdtree.Add_Points(PointToAdd, true);
//     ikdtree.Add_Points(PointNoNeedDownsample, false); 
//     add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
//     kdtree_incremental_time = omp_get_wtime() - st_time;
// }
// PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
// PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
// void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull)
// {
//     if(scan_pub_en)
//     {
//         PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
//         int size = laserCloudFullRes->points.size();
//         PointCloudXYZI::Ptr laserCloudWorld( \
//                         new PointCloudXYZI(size, 1));
//         for (int i = 0; i < size; i++)
//         {
//             RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
//                                 &laserCloudWorld->points[i]);
//         }
//         sensor_msgs::msg::PointCloud2 laserCloudmsg;
//         pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
//         laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
//         laserCloudmsg.header.frame_id = "camera_init";
//         pubLaserCloudFull->publish(laserCloudmsg);
//         publish_count -= PUBFRAME_PERIOD;
//     }
//     /**************** save map ****************/
//     /* 1. make sure you have enough memories
//     /* 2. noted that pcd save will influence the real-time performences **/
//     if (pcd_save_en)
//     {
//         int size = feats_undistort->points.size();
//         PointCloudXYZI::Ptr laserCloudWorld( \
//                         new PointCloudXYZI(size, 1));
//         for (int i = 0; i < size; i++)
//         {
//             RGBpointBodyToWorld(&feats_undistort->points[i], \
//                                 &laserCloudWorld->points[i]);
//         }
//         *pcl_wait_save += *laserCloudWorld;
//         static int scan_wait_num = 0;
//         scan_wait_num ++;
//         if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
//         {
//             pcd_index ++;
//             string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
//             pcl::PCDWriter pcd_writer;
//             cout << "current scan saved to /PCD/" << all_points_dir << endl;
//             pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
//             pcl_wait_save->clear();
//             scan_wait_num = 0;
//         }
//     }
// }
// void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
// {
//     int size = feats_undistort->points.size();
//     PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));
//     for (int i = 0; i < size; i++)
//     {
//         RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
//                             &laserCloudIMUBody->points[i]);
//     }
//     sensor_msgs::msg::PointCloud2 laserCloudmsg;
//     pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
//     laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
//     laserCloudmsg.header.frame_id = "body";
//     pubLaserCloudFull_body->publish(laserCloudmsg);
//     publish_count -= PUBFRAME_PERIOD;
// }
// void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudEffect)
// {
//     PointCloudXYZI::Ptr laserCloudWorld( \
//                     new PointCloudXYZI(effct_feat_num, 1));
//     for (int i = 0; i < effct_feat_num; i++)
//     {
//         RGBpointBodyToWorld(&laserCloudOri->points[i], \
//                             &laserCloudWorld->points[i]);
//     }
//     sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
//     pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
//     laserCloudFullRes3.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
//     laserCloudFullRes3.header.frame_id = "camera_init";
//     pubLaserCloudEffect->publish(laserCloudFullRes3);
// }
// void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudMap)
// {
//     sensor_msgs::msg::PointCloud2 laserCloudMap;
//     pcl::toROSMsg(*featsFromMap, laserCloudMap);
//     laserCloudMap.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
//     laserCloudMap.header.frame_id = "camera_init";
//     pubLaserCloudMap->publish(laserCloudMap);
// }
// template<typename T>
// void set_posestamp(T & out)
// {
//     out.pose.position.x = state_point.pos(0);
//     out.pose.position.y = state_point.pos(1);
//     out.pose.position.z = state_point.pos(2);
//     out.pose.orientation.x = geoQuat.x;
//     out.pose.orientation.y = geoQuat.y;
//     out.pose.orientation.z = geoQuat.z;
//     out.pose.orientation.w = geoQuat.w;
// }
// void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped)
// {
//     odomAftMapped.header.frame_id = "camera_init";
//     odomAftMapped.child_frame_id = "body";
//     odomAftMapped.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
//     set_posestamp(odomAftMapped.pose);
//     pubOdomAftMapped->publish(odomAftMapped);
//     auto P = kf.get_P();
//     for (int i = 0; i < 6; i ++)
//     {
//         int k = i < 3 ? i + 3 : i - 3;
//         odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
//         odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
//         odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
//         odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
//         odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
//         odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
//     }
//     geometry_msgs::msg::TransformStamped transform_stamped;
//     transform_stamped.header.stamp = odomAftMapped.header.stamp;
//     transform_stamped.header.frame_id = "camera_init";
//     transform_stamped.child_frame_id = "body";
//     transform_stamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
//     transform_stamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
//     transform_stamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
//     transform_stamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
//     transform_stamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
//     transform_stamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
//     transform_stamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
//     tf_broadcaster->sendTransform(transform_stamped);
// }
// void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
// {
//     set_posestamp(msg_body_pose);
//     msg_body_pose.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
//     msg_body_pose.header.frame_id = "camera_init";
//     /*** if path is too large, the rvis will crash ***/
//     static int jjj = 0;
//     jjj++;
//     if (jjj % 10 == 0) 
//     {
//         path.poses.push_back(msg_body_pose);
//         pubPath->publish(path);
//     }
// }
// void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
// {
//     double match_start = omp_get_wtime();
//     laserCloudOri->clear(); 
//     corr_normvect->clear(); 
//     total_residual = 0.0; 
//     /** closest surface search and residual computation **/
//     #ifdef MP_EN
//         omp_set_num_threads(MP_PROC_NUM);
//         #pragma omp parallel for
//     #endif
//     for (int i = 0; i < feats_down_size; i++)
//     {
//         PointType &point_body  = feats_down_body->points[i]; 
//         PointType &point_world = feats_down_world->points[i]; 
//         /* transform to world frame */
//         V3D p_body(point_body.x, point_body.y, point_body.z);
//         V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
//         point_world.x = p_global(0);
//         point_world.y = p_global(1);
//         point_world.z = p_global(2);
//         point_world.intensity = point_body.intensity;
//         vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
//         auto &points_near = Nearest_Points[i];
//         if (ekfom_data.converge)
//         {
//             /** Find the closest surfaces in the map **/
//             ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
//             point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
//         }
//         if (!point_selected_surf[i]) continue;
//         VF(4) pabcd;
//         point_selected_surf[i] = false;
//         if (esti_plane(pabcd, points_near, 0.1f))
//         {
//             float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
//             float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
//             if (s > 0.9)
//             {
//                 point_selected_surf[i] = true;
//                 normvec->points[i].x = pabcd(0);
//                 normvec->points[i].y = pabcd(1);
//                 normvec->points[i].z = pabcd(2);
//                 normvec->points[i].intensity = pd2;
//                 res_last[i] = abs(pd2);
//             }
//         }
//     }
//     effct_feat_num = 0;
//     for (int i = 0; i < feats_down_size; i++)
//     {
//         if (point_selected_surf[i])
//         {
//             laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
//             corr_normvect->points[effct_feat_num] = normvec->points[i];
//             total_residual += res_last[i];
//             effct_feat_num ++;
//         }
//     }
//     if (effct_feat_num < 1)
//     {
//         ekfom_data.valid = false;
//         RCLCPP_WARN(node_ptr->get_logger(), "No Effective Points! \n");
//         return;
//     }
//     res_mean_last = total_residual / effct_feat_num;
//     match_time  += omp_get_wtime() - match_start;
//     double solve_start_  = omp_get_wtime();
//     /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
//     ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
//     ekfom_data.h.resize(effct_feat_num);
//     for (int i = 0; i < effct_feat_num; i++)
//     {
//         const PointType &laser_p  = laserCloudOri->points[i];
//         V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
//         M3D point_be_crossmat;
//         point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
//         V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
//         M3D point_crossmat;
//         point_crossmat<<SKEW_SYM_MATRX(point_this);
//         /*** get the normal vector of closest surface/corner ***/
//         const PointType &norm_p = corr_normvect->points[i];
//         V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
//         /*** calculate the Measuremnt Jacobian matrix H ***/
//         V3D C(s.rot.conjugate() *norm_vec);
//         V3D A(point_crossmat * C);
//         if (extrinsic_est_en)
//         {
//             V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
//             ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
//         }
//         else
//         {
//             ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//         }
//         /*** Measuremnt: distance to the closest surface/corner ***/
//         ekfom_data.h(i) = -norm_p.intensity;
//     }
//     solve_time += omp_get_wtime() - solve_start_;
// }
// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     node_ptr = rclcpp::Node::make_shared("laserMapping");
//     tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node_ptr);
//     node_ptr->declare_parameter<bool>("publish.path_en", true);
//     node_ptr->declare_parameter<bool>("publish.scan_publish_en", true);
//     node_ptr->declare_parameter<bool>("publish.dense_publish_en", true);
//     node_ptr->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
//     node_ptr->declare_parameter<int>("max_iteration", 4);
//     node_ptr->declare_parameter<string>("map_file_path", "");
//     node_ptr->declare_parameter<string>("common.lid_topic", "/livox/lidar");
//     node_ptr->declare_parameter<string>("common.imu_topic", "/livox/imu");
//     node_ptr->declare_parameter<bool>("common.time_sync_en", false);
//     node_ptr->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
//     node_ptr->declare_parameter<double>("filter_size_corner", 0.5);
//     node_ptr->declare_parameter<double>("filter_size_surf", 0.5);
//     node_ptr->declare_parameter<double>("filter_size_map", 0.5);
//     node_ptr->declare_parameter<double>("cube_side_length", 200);
//     node_ptr->declare_parameter<float>("mapping.det_range", 300.f);
//     node_ptr->declare_parameter<double>("mapping.fov_degree", 180);
//     node_ptr->declare_parameter<double>("mapping.gyr_cov", 0.1);
//     node_ptr->declare_parameter<double>("mapping.acc_cov", 0.1);
//     node_ptr->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
//     node_ptr->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
//     node_ptr->declare_parameter<double>("preprocess.blind", 0.01);
//     node_ptr->declare_parameter<int>("preprocess.lidar_type", AVIA);
//     node_ptr->declare_parameter<int>("preprocess.scan_line", 16);
//     node_ptr->declare_parameter<int>("preprocess.timestamp_unit", US);
//     node_ptr->declare_parameter<int>("preprocess.scan_rate", 10);
//     node_ptr->declare_parameter<int>("point_filter_num", 2);
//     node_ptr->declare_parameter<bool>("feature_extract_enable", false);
//     node_ptr->declare_parameter<bool>("runtime_pos_log_enable", false);
//     node_ptr->declare_parameter<bool>("mapping.extrinsic_est_en", true);
//     node_ptr->declare_parameter<bool>("pcd_save.pcd_save_en", false);
//     node_ptr->declare_parameter<int>("pcd_save.interval", -1);
//     node_ptr->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
//     node_ptr->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
//     node_ptr->get_parameter("publish.path_en", path_en);
//     node_ptr->get_parameter("publish.scan_publish_en", scan_pub_en);
//     node_ptr->get_parameter("publish.dense_publish_en", dense_pub_en);
//     node_ptr->get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en);
//     node_ptr->get_parameter("max_iteration", NUM_MAX_ITERATIONS);
//     node_ptr->get_parameter("map_file_path", map_file_path);
//     node_ptr->get_parameter("common.lid_topic", lid_topic);
//     node_ptr->get_parameter("common.imu_topic", imu_topic);
//     node_ptr->get_parameter("common.time_sync_en", time_sync_en);
//     node_ptr->get_parameter("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu);
//     node_ptr->get_parameter("filter_size_corner", filter_size_corner_min);
//     node_ptr->get_parameter("filter_size_surf", filter_size_surf_min);
//     node_ptr->get_parameter("filter_size_map", filter_size_map_min);
//     node_ptr->get_parameter("cube_side_length", cube_len);
//     node_ptr->get_parameter("mapping.det_range", DET_RANGE);
//     node_ptr->get_parameter("mapping.fov_degree", fov_deg);
//     node_ptr->get_parameter("mapping.gyr_cov", gyr_cov);
//     node_ptr->get_parameter("mapping.acc_cov", acc_cov);
//     node_ptr->get_parameter("mapping.b_gyr_cov", b_gyr_cov);
//     node_ptr->get_parameter("mapping.b_acc_cov", b_acc_cov);
//     node_ptr->get_parameter("preprocess.blind", p_pre->blind);
//     node_ptr->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
//     node_ptr->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
//     node_ptr->get_parameter("preprocess.timestamp_unit", p_pre->time_unit);
//     node_ptr->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);
//     node_ptr->get_parameter("point_filter_num", p_pre->point_filter_num);
//     node_ptr->get_parameter("feature_extract_enable", p_pre->feature_enabled);
//     node_ptr->get_parameter("runtime_pos_log_enable", runtime_pos_log);
//     node_ptr->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);
//     node_ptr->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
//     node_ptr->get_parameter("pcd_save.interval", pcd_save_interval);
//     node_ptr->get_parameter("mapping.extrinsic_T", extrinT);
//     node_ptr->get_parameter("mapping.extrinsic_R", extrinR);
//     cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
//     path.header.stamp    = node_ptr->now();
//     path.header.frame_id ="camera_init";
//     /*** variables definition ***/
//     int effect_feat_num = 0, frame_num = 0;
//     double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
//     bool flg_EKF_converged, EKF_stop_flg = 0;
//     FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
//     HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);
//     _featsArray.reset(new PointCloudXYZI());
//     memset(point_selected_surf, true, sizeof(point_selected_surf));
//     memset(res_last, -1000.0f, sizeof(res_last));
//     downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
//     downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
//     memset(point_selected_surf, true, sizeof(point_selected_surf));
//     memset(res_last, -1000.0f, sizeof(res_last));
//     Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
//     Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
//     p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
//     p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
//     p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
//     p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
//     p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
//     double epsi[23] = {0.001};
//     fill(epsi, epsi+23, 0.001);
//     kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
//     /*** debug record ***/
//     FILE *fp;
//     string pos_log_dir = root_dir + "/Log/pos_log.txt";
//     fp = fopen(pos_log_dir.c_str(),"w");
//     ofstream fout_pre, fout_out, fout_dbg;
//     fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
//     fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
//     fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
//     if (fout_pre && fout_out)
//         cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
//     else
//         cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;
//     /*** ROS subscribe initialization ***/
//     rclcpp::SubscriptionBase::SharedPtr sub_pcl;
//     if (p_pre->lidar_type == AVIA)
//     {
//         sub_pcl = node_ptr->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//             lid_topic, 200000, livox_pcl_cbk);
//     }
//     else
//     {
//         sub_pcl = node_ptr->create_subscription<sensor_msgs::msg::PointCloud2>(
//             lid_topic, 200000, standard_pcl_cbk);
//     }
//     auto sub_imu = node_ptr->create_subscription<sensor_msgs::msg::Imu>(
//         imu_topic, 200000, imu_cbk);
//     auto pubLaserCloudFull = node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>
//             ("/cloud_registered", 100000);
//     auto pubLaserCloudFull_body = node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>
//             ("/cloud_registered_body", 100000);
//     auto pubLaserCloudEffect = node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>
//             ("/cloud_effected", 100000);
//     auto pubLaserCloudMap = node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>
//             ("/Laser_map", 100000);
//     auto pubOdomAftMapped = node_ptr->create_publisher<nav_msgs::msg::Odometry> 
//             ("/Odometry", 100000);
//     auto pubPath          = node_ptr->create_publisher<nav_msgs::msg::Path> 
//             ("/path", 100000);
// //------------------------------------------------------------------------------------------------------
//     signal(SIGINT, SigHandle);
//     rclcpp::Rate rate(5000);
//     bool status = rclcpp::ok();
//     while (status)
//     {
//         if (flg_exit) break;
//         rclcpp::spin_some(node_ptr);
//         if(sync_packages(Measures)) 
//         {
//             if (flg_first_scan)
//             {
//                 first_lidar_time = Measures.lidar_beg_time;
//                 p_imu->first_lidar_time = first_lidar_time;
//                 flg_first_scan = false;
//                 continue;
//             }
//             double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;
//             match_time = 0;
//             kdtree_search_time = 0.0;
//             solve_time = 0;
//             solve_const_H_time = 0;
//             svd_time   = 0;
//             t0 = omp_get_wtime();
//             p_imu->Process(Measures, kf, feats_undistort);
//             state_point = kf.get_x();
//             pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
//             if (feats_undistort->empty() || (feats_undistort == NULL))
//             {
//                 RCLCPP_WARN(node_ptr->get_logger(), "No point, skip this scan!\n");
//                 continue;
//             }
//             flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
//                             false : true;
//             /*** Segment the map in lidar FOV ***/
//             lasermap_fov_segment();
//             /*** downsample the feature points in a scan ***/
//             downSizeFilterSurf.setInputCloud(feats_undistort);
//             downSizeFilterSurf.filter(*feats_down_body);
//             t1 = omp_get_wtime();
//             feats_down_size = feats_down_body->points.size();
//             /*** initialize the map kdtree ***/
//             if(ikdtree.Root_Node == nullptr)
//             {
//                 if(feats_down_size > 5)
//                 {
//                     ikdtree.set_downsample_param(filter_size_map_min);
//                     feats_down_world->resize(feats_down_size);
//                     for(int i = 0; i < feats_down_size; i++)
//                     {
//                         pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
//                     }
//                     ikdtree.Build(feats_down_world->points);
//                 }
//                 continue;
//             }
//             int featsFromMapNum = ikdtree.validnum();
//             kdtree_size_st = ikdtree.size();
//             // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;
//             /*** ICP and iterated Kalman filter update ***/
//             if (feats_down_size < 5)
//             {
//                 RCLCPP_WARN(node_ptr->get_logger(), "No point, skip this scan!\n");
//                 continue;
//             }
//             normvec->resize(feats_down_size);
//             feats_down_world->resize(feats_down_size);
//             V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
//             fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
//             <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;
//             if(0) // If you need to see map point, change to "if(1)"
//             {
//                 PointVector ().swap(ikdtree.PCL_Storage);
//                 ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
//                 featsFromMap->clear();
//                 featsFromMap->points = ikdtree.PCL_Storage;
//             }
//             pointSearchInd_surf.resize(feats_down_size);
//             Nearest_Points.resize(feats_down_size);
//             int  rematch_num = 0;
//             bool nearest_search_en = true; //
//             t2 = omp_get_wtime();
//             /*** iterated state estimation ***/
//             double t_update_start = omp_get_wtime();
//             double solve_H_time = 0;
//             kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
//             state_point = kf.get_x();
//             euler_cur = SO3ToEuler(state_point.rot);
//             pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
//             geoQuat.x = state_point.rot.coeffs()[0];
//             geoQuat.y = state_point.rot.coeffs()[1];
//             geoQuat.z = state_point.rot.coeffs()[2];
//             geoQuat.w = state_point.rot.coeffs()[3];
//             double t_update_end = omp_get_wtime();
//             /******* Publish odometry *******/
//             publish_odometry(pubOdomAftMapped);
//             /*** add the feature points to map kdtree ***/
//             t3 = omp_get_wtime();
//             map_incremental();
//             t5 = omp_get_wtime();
//             /******* Publish points *******/
//             if (path_en)                         publish_path(pubPath);
//             if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
//             if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
//             // publish_effect_world(pubLaserCloudEffect);
//             // publish_map(pubLaserCloudMap);
//             /*** Debug variables ***/
//             if (runtime_pos_log)
//             {
//                 frame_num ++;
//                 kdtree_size_end = ikdtree.size();
//                 aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
//                 aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
//                 aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
//                 aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
//                 aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
//                 aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
//                 T1[time_log_counter] = Measures.lidar_beg_time;
//                 s_plot[time_log_counter] = t5 - t0;
//                 s_plot2[time_log_counter] = feats_undistort->points.size();
//                 s_plot3[time_log_counter] = kdtree_incremental_time;
//                 s_plot4[time_log_counter] = kdtree_search_time;
//                 s_plot5[time_log_counter] = kdtree_delete_counter;
//                 s_plot6[time_log_counter] = kdtree_delete_time;
//                 s_plot7[time_log_counter] = kdtree_size_st;
//                 s_plot8[time_log_counter] = kdtree_size_end;
//                 s_plot9[time_log_counter] = aver_time_consu;
//                 s_plot10[time_log_counter] = add_point_size;
//                 time_log_counter ++;
//                 printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
//                 ext_euler = SO3ToEuler(state_point.offset_R_L_I);
//                 fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
//                 <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
//                 dump_lio_state_to_log(fp);
//             }
//         }
//         status = rclcpp::ok();
//         rate.sleep();
//     }
//     /**************** save map ****************/
//     /* 1. make sure you have enough memories
//     /* 2. pcd save will largely influence the real-time performences **/
//     if (pcl_wait_save->size() > 0 && pcd_save_en)
//     {
//         string file_name = string("scans.pcd");
//         string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
//         pcl::PCDWriter pcd_writer;
//         cout << "current scan saved to /PCD/" << file_name<<endl;
//         pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
//     }
//     fout_out.close();
//     fout_pre.close();
//     if (runtime_pos_log)
//     {
//         vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
//         FILE *fp2;
//         string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
//         fp2 = fopen(log_dir.c_str(),"w");
//         fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
//         for (int i = 0;i<time_log_counter; i++){
//             fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
//             t.push_back(T1[i]);
//             s_vec.push_back(s_plot9[i]);
//             s_vec2.push_back(s_plot3[i] + s_plot6[i]);
//             s_vec3.push_back(s_plot4[i]);
//             s_vec5.push_back(s_plot[i]);
//         }
//         fclose(fp2);
//     }
//     rclcpp::shutdown();
//     return 0;
// }
