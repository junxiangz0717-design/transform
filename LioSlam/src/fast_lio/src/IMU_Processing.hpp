#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <cassert> // Added for assert
#include <rclcpp/rclcpp.hpp> // Replaced ros/ros.h
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp> // Updated header
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_broadcaster.h> // Updated header
#include <sensor_msgs/msg/imu.hpp> // Updated header
#include <sensor_msgs/msg/point_cloud2.hpp> // Updated header
#include <geometry_msgs/msg/vector3.hpp> // Updated header
#include <pcl_conversions/pcl_conversions.h>
#include "use-ikfom.hpp"
#include "imu_fitter.hpp"
#include "DebugLogger.hpp"
#include "UniversalLogger.hpp"
/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

M3D Exp_SO3_Local(const V3D& w) {
    double theta = w.norm();
    if (theta < 1e-6) return M3D::Identity();
    
    V3D a = w / theta;
    M3D a_x;
    a_x << 0, -a(2), a(1),
           a(2), 0, -a(0),
           -a(1), a(0), 0;
           
    return M3D::Identity() + sin(theta) * a_x + (1 - cos(theta)) * (a * a.transpose());
}

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  // Changed ConstPtr to ConstSharedPtr for ROS 2
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);
  void set_init_state(bool state) { imu_need_init_ = state; }
  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;

 private:
  double last_shake_time_ = -1.0;
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);
  RobustImuFitter imu_fitter_;//数据清洗和鲁棒拟合，不直接使用IMU数据进行卡尔曼滤波，而是使用数学方法进行分析（内部看不懂不影响）
  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_; // Changed to ROS 2 SharedPtr
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_; // Changed to ROS 2 SharedPtr
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);
  RCLCPP_WARN(rclcpp::get_logger("ImuProcess"), "Init Grav: %.2f %.2f %.2f", 
    init_state.grav[0], init_state.grav[1], init_state.grav[2]);
// [修正后的代码]
    
    // 1. 获取状态
    state_ikfom check = kf_state.get_x();
    
    // 2. [关键] 先转成 Eigen 矩阵
    Eigen::Matrix3d R_check = check.rot.toRotationMatrix();
    
    RCLCPP_WARN(rclcpp::get_logger("ImuProcess"), "=== DEBUG IMU INIT RESULT ===");
    
    // 3. 使用 R_check 来求行列式
    RCLCPP_WARN(rclcpp::get_logger("ImuProcess"), "Init Rot Determinant: %.4f (Should be 1.0)", R_check.determinant());
    
    // 4. [这里是报错点] 使用 R_check 来访问元素，而不是 check.rot
    RCLCPP_WARN(rclcpp::get_logger("ImuProcess"), "Init Rot Row0: %.2f %.2f %.2f", 
        R_check(0,0), R_check(0,1), R_check(0,2));
    
    // 检查逻辑
    if (std::abs(R_check.determinant()) < 1e-6) {
         RCLCPP_ERROR(rclcpp::get_logger("ImuProcess"), "[FATAL] Rotation Matrix is ZERO/Singular! Determinant is 0!");
         RCLCPP_ERROR(rclcpp::get_logger("ImuProcess"), "YOU MUST UNCOMMENT: 'init_state.rot = Eye3d;'");
    }
  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}

// ================== [核心函数 1] IMU 数据清洗与预测流程 ==================
void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  // 1. 更新全局时间戳 (基于雷达起始时间)
  // LOG_UPDATE_TIME(meas.lidar_beg_time); // 确保你的宏有效
  
  if(meas.imu.empty()) {return;};
  assert(meas.lidar != nullptr);

  // ==================== [Step 0: 过滤参数定义] ====================
  // 哨兵机器人参数调优 (Hard Clamping + Median + Split LPF)
  const double MAX_ACC_NORM = 25.0; // 硬限幅：2.5g (应对撞击)
  const int MEDIAN_WIN_SIZE = 3;    // 中值滤波窗口 (应对减速带尖峰)
  const double ALPHA_ACC = 0.3;     // Acc强滤：消除路面震动
  const double ALPHA_GYR = 0.6;     // Gyr弱滤：保留快速旋转动态

  // 静态滤波器状态
  static std::deque<V3D> acc_buffer; 
  static V3D last_acc_lpf = V3D::Zero();
  static V3D last_gyr_lpf = V3D::Zero();
  static bool lpf_init = false;
  
  MeasureGroup meas_clean = meas; // 创建深拷贝，用于存储清洗后的数据

  // ==================== [Step 0.5: 三级流水线过滤] ====================
  for (auto &imu : meas_clean.imu) {
      auto imu_new = std::make_shared<sensor_msgs::msg::Imu>(*imu);
        // 【唯一正确的修改】：强制将 G 转换为 m/s^2
    imu_new->linear_acceleration.x *= 9.81;
    imu_new->linear_acceleration.y *= 9.81;
    imu_new->linear_acceleration.z *= 9.81;

    // 接下来再取 raw_acc，此时 raw_acc.norm() 应该是 9.8 左右
    //V3D raw_acc(imu_new->linear_acceleration.x, imu_new->linear_acceleration.y, imu_new->linear_acceleration.z);
      // [日志关键 1] 更新当前微观时间 (IMU 频率)
      double imu_time = rclcpp::Time(imu->header.stamp).seconds();
      ULOG_SET_TIME(imu_time);

      // 计算合加速度模长 (入站原始状态)
      double acc_norm = std::sqrt(
          imu_new->linear_acceleration.x * imu_new->linear_acceleration.x +
          imu_new->linear_acceleration.y * imu_new->linear_acceleration.y +
          imu_new->linear_acceleration.z * imu_new->linear_acceleration.z
      );

      // 【新增探针 A1】：记录进入单位判断前的最原始模长
      ULOG_PLOT("IMU_Unit_Test", "Norm_Before_Scale", acc_norm);

      //// 逻辑：如果模长接近 1.0 (比如 < 3.0)，说明单位是 G，需要乘 9.81
    //   if (acc_norm < 3.0) { 
    //       imu_new->linear_acceleration.x *= 9.81;
    //       imu_new->linear_acceleration.y *= 9.81;
    //       imu_new->linear_acceleration.z *= 9.81;
          
    //       static bool first_print = true;
    //       if (first_print) {
    //           RCLCPP_WARN(rclcpp::get_logger("ImuProcess"), "IMU unit detected as G, converting to m/s^2");
    //           first_print = false;
    //       }
    //   }

      // 获取当前处理完的值
      V3D raw_acc(imu_new->linear_acceleration.x, imu_new->linear_acceleration.y, imu_new->linear_acceleration.z);
      V3D raw_gyr(imu_new->angular_velocity.x, imu_new->angular_velocity.y, imu_new->angular_velocity.z);

      // 【新增探针 A2 & A3】：记录处理逻辑后的真实模长和实际放大的倍数
      double norm_after = raw_acc.norm();
      ULOG_PLOT("IMU_Unit_Test", "Norm_After_Scale", norm_after);
      
      // 就算没被 if 捕获，这里也能算出比例是 1.0 还是 9.81
      double scale_applied = norm_after / (acc_norm + 1e-6); 
      ULOG_PLOT("IMU_Unit_Test", "Scale_Factor_Applied", scale_applied);

      // [日志关键 2] Layer 1: 原始数据监控
      ULOG_PLOT("IMU_Raw", "acc", raw_acc);
      ULOG_PLOT("IMU_Raw", "gyr", raw_gyr);
      ULOG_PLOT("IMU_Raw", "acc_norm", norm_after);

      // 核心防御：NaN 检查
      if (std::isnan(raw_acc.norm()) || std::isnan(raw_gyr.norm())) {
          raw_acc = last_acc_lpf; 
          raw_gyr = last_gyr_lpf;
          ULOG_PLOT("IMU_Event", "NaN_Detected", 1.0);
      } else {
          ULOG_PLOT("IMU_Event", "NaN_Detected", 0.0);
      }
      
      V3D cur_acc = raw_acc;
      V3D cur_gyr = raw_gyr;
      
      // ---------------- [Level 2: 中值滤波 (Median Filter)] ----------------
      acc_buffer.push_back(cur_acc);
      if (acc_buffer.size() > MEDIAN_WIN_SIZE) acc_buffer.pop_front();
      
      V3D median_acc = cur_acc; 
      if (acc_buffer.size() == MEDIAN_WIN_SIZE) {
          std::vector<double> xs, ys, zs;
          for(auto& v : acc_buffer) { xs.push_back(v.x()); ys.push_back(v.y()); zs.push_back(v.z()); }
          std::sort(xs.begin(), xs.end());
          std::sort(ys.begin(), ys.end());
          std::sort(zs.begin(), zs.end());
          median_acc = V3D(xs[1], ys[1], zs[1]); 
      }
      ULOG_PLOT("IMU_Median", "acc", median_acc);

      // ---------------- [Level 3: 分离式低通滤波 (Split LPF)] ----------------
      if (!lpf_init) {
          last_acc_lpf = median_acc;
          last_gyr_lpf = cur_gyr;
          lpf_init = true;
      } else {
          last_acc_lpf = last_acc_lpf * (1.0 - ALPHA_ACC) + median_acc * ALPHA_ACC;
          last_gyr_lpf = last_gyr_lpf * (1.0 - ALPHA_GYR) + cur_gyr * ALPHA_GYR;
      }

      // [日志关键 4] Layer 3: 最终输出 (EKF Input)
      ULOG_PLOT("IMU_Final", "acc", last_acc_lpf);
      ULOG_PLOT("IMU_Final", "gyr", last_gyr_lpf);
      
      // 填回数据
      imu_new->linear_acceleration.x = last_acc_lpf.x();
      imu_new->linear_acceleration.y = last_acc_lpf.y();
      imu_new->linear_acceleration.z = last_acc_lpf.z();
      
      imu_new->angular_velocity.x = last_gyr_lpf.x();
      imu_new->angular_velocity.y = last_gyr_lpf.y();
      imu_new->angular_velocity.z = last_gyr_lpf.z();
      
      imu = imu_new;
  }
  
  // 恢复全局时间到雷达帧开始，防止影响后续逻辑
  ULOG_SET_TIME(meas.lidar_beg_time);
  
  // [新增检查点 1] 检查进入去畸变前的 IMU 数据是否已经坏了
  if (!meas_clean.imu.empty()) {
    auto& last_imu = meas_clean.imu.back();
    if (std::isnan(last_imu->linear_acceleration.x) || 
        std::abs(last_imu->linear_acceleration.x) > 50.0) { 
        RCLCPP_ERROR(rclcpp::get_logger("ImuProcess"), 
            "[NAN DETECTED] Step 1: IMU Preprocess Output is NaN/Huge! Val: %.2f", 
            last_imu->linear_acceleration.x);
        ULOG_PLOT("NaN_Debug", "1_IMU_Preprocess_Fail", 1.0);
    }
  }
  
  // ==================== [Step 1: 物理限幅防御] ====================
  state_ikfom cur_state = kf_state.get_x();
  double vel_norm = cur_state.vel.norm();
  
//   if (vel_norm > 5.0) {
//       static double last_clamp_warn_time = 0.0;
//       if (meas.lidar_beg_time - last_clamp_warn_time > 1.0) {
//           RCLCPP_WARN(rclcpp::get_logger("ImuProcess"), 
//               "\033[1;31m[PHYSICS] Speed %.2f m/s exceeds Sentry limit! Clamping.\033[0m", vel_norm);
//           last_clamp_warn_time = meas.lidar_beg_time;
//       }
      
//       // 强制软衰减
//       cur_state.vel = cur_state.vel.normalized() * 4.0;
//       kf_state.change_x(cur_state);
      
//       Eigen::Matrix<double, 23, 23> P = kf_state.get_P();
//       P.block<3,3>(12,12) += Eigen::Matrix3d::Identity() * 1.0; 
//       P.block<3,3>(0,0) += Eigen::Matrix3d::Identity() * 0.5; 
//       kf_state.change_P(P);
//   }

  // ==================== [Step 1.5: 初始零偏保护] ====================
  if (cur_state.ba.norm() > 2.0) {
       cur_state.ba.setZero();
       kf_state.change_x(cur_state);
  }

  // ==================== [Step 2: 初始化逻辑] ====================
  if (imu_need_init_) {
    V3D cur_gyr_mean = V3D::Zero();
    double max_acc_norm = 0.0;
    for (const auto &imu : meas_clean.imu) {
        V3D acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
        if(acc.norm() > max_acc_norm) max_acc_norm = acc.norm();
        cur_gyr_mean += V3D(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    }
    cur_gyr_mean /= meas_clean.imu.size();

    if (max_acc_norm > 12.0 || cur_gyr_mean.norm() > 0.5) return; 
    
    IMU_init(meas_clean, kf_state, init_iter_num); 
    imu_need_init_ = true;
    last_imu_ = meas_clean.imu.back();
    
    if (init_iter_num > MAX_INI_COUNT) {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;
      cov_acc = cov_acc_scale; cov_gyr = cov_gyr_scale;
      RCLCPP_INFO(rclcpp::get_logger("ImuProcess"), "IMU Initialized Done!");
    }
    return;
  }
  
  // ==================== [Step 3: 核心处理] ====================
  UndistortPcl(meas_clean, kf_state, *cur_pcl_un_);
  
  // 检查去畸变后的点云是否含有 NaN
  ULOG_CHECK_PCL(cur_pcl_un_, "1_After_Undistort");
  
  // (注：由于你当前上下文中可能没有 LOG_STATE 的定义，如果编译报错请注释下一行)
  // state_ikfom state_pred = kf_state.get_x();
  // LOG_STATE("PREDICT", state_pred);
}
// ================== [核心函数 2] 去畸变与协方差自适应 ==================
void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  LOG_UPDATE_TIME(meas.lidar_beg_time);
  
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  
  const double pcl_beg_time = meas.lidar_beg_time;
  const double pcl_end_time = meas.lidar_end_time;
  
  // >>>>>>>>>>>>>>>>> [模块 A: 鲁棒多项式拟合] >>>>>>>>>>>>>>>>>
  std::vector<double> fit_times;
  std::vector<Eigen::Vector3d> fit_gyrs, fit_accs;
  
  size_t imu_sz = v_imu.size();
  fit_times.reserve(imu_sz);
  fit_gyrs.reserve(imu_sz);
  fit_accs.reserve(imu_sz);

  for(const auto& imu : v_imu) {
      fit_times.push_back(rclcpp::Time(imu->header.stamp).seconds());
      fit_accs.emplace_back(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
      fit_gyrs.emplace_back(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
  }

  MotionCoeffs coeffs = imu_fitter_.fit(fit_times, fit_gyrs, fit_accs);
  ULOG_SET_TIME(meas.lidar_beg_time);
  // [新增检查点 2] 检查拟合系数是否为 NaN
  bool coeffs_nan = false;
  if (std::isnan(coeffs.w_c0.norm()) || std::isnan(coeffs.a_c0.norm())) coeffs_nan = true;

  if (coeffs_nan) {
      RCLCPP_ERROR(rclcpp::get_logger("ImuProcess"), "[NAN DETECTED] Step 2: IMU Fitting Produced NaN!");
      ULOG_PLOT("NaN_Debug", "2_Fit_Coeffs_NaN", 1.0);
      // 强制把 coefficients 设为 0，防止后面爆炸 (调试用)
      coeffs.valid = false; 
  }
  // 记录拟合质量
  ULOG_PLOT("IMU_Fit_Analysis", "rmse_acc", coeffs.fit_error_acc);
  ULOG_PLOT("IMU_Fit_Analysis", "rmse_gyr", coeffs.fit_error_gyr);
  ULOG_PLOT("IMU_Fit_Analysis", "is_valid", coeffs.valid ? 1.0 : 0.0);

  if (!coeffs.valid) {
      ULOG_PLOT("IMU_Fit_Analysis", "FIT_FAILURE_TRIGGER", 1.0);
  } else {
      ULOG_PLOT("IMU_Fit_Analysis", "FIT_FAILURE_TRIGGER", 0.0);
  }

  // [关键调整] 协方差自适应逻辑
  double k_cov_acc = 1.0;
  double k_cov_gyr = 1.0;
  
  if (coeffs.valid) {
      // 阈值回调: 0.35 (兼容3m/s正常震动，拦截冲击)
      const double BASE_RMSE_ACC = 0.35;
      const double BASE_RMSE_GYR = 0.1;
      
      // Acc 膨胀逻辑 (指数级惩罚)
      if (coeffs.fit_error_acc > BASE_RMSE_ACC) {
          double ratio = (coeffs.fit_error_acc - BASE_RMSE_ACC);
          double raw_k = 1.0 + std::exp(ratio * 2.0); 
          k_cov_acc = std::min(raw_k, 1000.0);
      }
      
      if (coeffs.fit_error_gyr > BASE_RMSE_GYR) {
          double ratio = (coeffs.fit_error_gyr - BASE_RMSE_GYR);
          double raw_k = 1.0 + std::exp(ratio * 3.0);
          k_cov_gyr = std::min(raw_k, 1000.0);
      }

      LOG_VAL("IMU_FIT", "K_COV_ACC", k_cov_acc);
      LOG_VAL("IMU_FIT", "K_COV_GYR", k_cov_gyr);

  } else {
      // 拟合失败 -> 极大膨胀
      LOG_VAL("IMU_FIT", "K_COV_ACC", 100.0);
      if (!coeffs.valid) {
          RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ImuProcess"), steady_clock, 1000, "IMU Fit Failed!");
      }
  }
 
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

  IMUpose.clear();
  state_ikfom cur_state = kf_state.get_x(); 

  // ==================== [Step 3: 硬限幅熔断] ====================
  bool state_reset = false;

  // 1. 速度熔断 (放宽到 15.0 防止误触，主要靠 Process 里的 5.0 限制)
  if (cur_state.vel.norm() > 15.0) { 
      RCLCPP_WARN(rclcpp::get_logger("ImuProcess"), 
          "\033[1;31m[SAFETY] Velocity saturated (%.1f). Resetting.\033[0m", cur_state.vel.norm());
      LOG_VAL("SAFETY", "RESET_VEL", 1.0);
      cur_state.vel.setZero(); 
      state_reset = true;
  } else {
      LOG_VAL("SAFETY", "RESET_VEL", 0.0);
  }

  // 2. Bias 熔断
  if (cur_state.ba.norm() > 2.5) {
      RCLCPP_WARN(rclcpp::get_logger("ImuProcess"), 
           "\033[1;31m[SAFETY] Bias Acc diverged (%.1f). Resetting.\033[0m", cur_state.ba.norm());
      LOG_VAL("SAFETY", "RESET_BIAS_ACC", 1.0);
      cur_state.ba.setZero();
      state_reset = true;
  } else {
      LOG_VAL("SAFETY", "RESET_BIAS_ACC", 0.0);
  }

  if (cur_state.bg.norm() > 2.0) {
      cur_state.bg.setZero();
      state_reset = true;
  }

  if (state_reset) {
      kf_state.change_x(cur_state);
  }
  
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, cur_state.vel, cur_state.pos, cur_state.rot.toRotationMatrix()));

  // 4. 状态预测主循环
  double t_curr = fit_times.front(); 
  input_ikfom in;
  V3D angvel_avr, acc_avr;

  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    
    double t_next = rclcpp::Time(tail->header.stamp).seconds();
    

    double dt = t_next - t_curr;
    if (dt < 1e-7) continue; 

    // >>>>>>>>>>>>>>>>> [模块 B: 状态预测分支] >>>>>>>>>>>>>>>>>
    if (coeffs.valid) 
    {
        // ------------- 分支 1: Magnus 解析预测 (高精度) -------------
        double t_rel = t_curr - coeffs.t_start;

        V3D w_body_curr = coeffs.w_c0 + coeffs.w_c1 * t_rel;
        V3D w_unb = w_body_curr - cur_state.bg; 
        V3D psi_linear = w_unb * dt + 0.5 * coeffs.w_c1 * dt * dt;
        V3D psi_conic = (pow(dt, 3) / 12.0) * w_unb.cross(coeffs.w_c1);
        
        M3D R_next = cur_state.rot * Exp_SO3_Local(psi_linear + psi_conic);
        V3D gravity_vec = cur_state.grav;

        V3D a_body_curr = coeffs.a_c0 + coeffs.a_c1 * t_rel;
        V3D a_body_next = coeffs.a_c0 + coeffs.a_c1 * (t_rel + dt);
        
        V3D a_world_curr = cur_state.rot * (a_body_curr - cur_state.ba) + gravity_vec; 
        V3D a_world_next = R_next * (a_body_next - cur_state.ba) + gravity_vec;
        V3D acc_avg_world = 0.5 * (a_world_curr + a_world_next);
        
        V3D vel_next = cur_state.vel + acc_avg_world * dt;
        V3D pos_next = cur_state.pos + cur_state.vel * dt + 0.5 * acc_avg_world * dt * dt;

        double t_mid_rel = t_rel + 0.5 * dt;
        in.acc  = coeffs.a_c0 + coeffs.a_c1 * t_mid_rel; 
        in.gyro = coeffs.w_c0 + coeffs.w_c1 * t_mid_rel;
        in.acc = in.acc;
        if (std::isnan(in.acc.norm()) || in.acc.norm() > 30.0) {
            RCLCPP_ERROR(rclcpp::get_logger("ImuProcess"), 
                "[NAN DETECTED] Step 3.1: Bad EKF Input! Acc: %.2f, Gyr: %.2f", 
                in.acc.norm(), in.gyro.norm());
            ULOG_PLOT("NaN_Debug", "3_EKF_Input_Bad", 1.0);
        }
        // 应用自适应协方差膨胀
        Q.block<3, 3>(0, 0).diagonal() = cov_gyr * k_cov_gyr;
        Q.block<3, 3>(3, 3).diagonal() = cov_acc * k_cov_acc;
        Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
        Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
        // [新增] ================= 基于测度论的 Q 阵自适应 (分支1) =================
        // 1. 获取当前预测使用的加速度模长 (经过拟合的)
        double acc_mag = in.acc.norm();

        // 2. 计算原始数据的加加速度 (Raw Jerk)
        // 拟合后的 Jerk 会被平滑掉，必须用原始两帧数据 head 和 tail 来判断震动
        double raw_jerk = 0.0;
        V3D raw_acc_head(head->linear_acceleration.x, head->linear_acceleration.y, head->linear_acceleration.z);
        V3D raw_acc_tail(tail->linear_acceleration.x, tail->linear_acceleration.y, tail->linear_acceleration.z);
        if (dt > 1e-6) {
            raw_jerk = (raw_acc_tail - raw_acc_head).norm() / dt;
        }

        // 3. 计算膨胀因子
        double inflation_factor = 1.0;

        // [判据 A: 撞击 (Collision)] -> 有效的大值
        // 特征: Mag 大，但 Jerk 相对小 (力是连续传递的)
        if (acc_mag > 20) {
            double excess = acc_mag - 4.0;
            // 2的幂次增长：每超 2.0 m/s^2，不信任度翻倍
            inflation_factor += std::pow(2.0, excess / 2.0);
        }

        if (raw_jerk > 2000.0) {
            inflation_factor += 50.0; // 膨胀 50 倍足够了，不要加 1000
        }

        // 4. 应用膨胀
        if (inflation_factor > 1.0) {
            // 4. 应用膨胀 (最高 50 倍，绝不允许 1000 倍)
            if (inflation_factor > 50.0) inflation_factor = 50.0;
            
            ULOG_PLOT("EKF_Weight", "IMU_Q_Inflation", inflation_factor);
            Q.block<3, 3>(3, 3) *= inflation_factor;
        }

        ULOG_PLOT("EKF_Weight", "IMU_Q_Inflation", inflation_factor);
        kf_state.predict(dt, Q, in);
        // [探针 1] 监控 Magnus 覆盖前，EKF 自身的预测状态
        state_ikfom s_ekf = kf_state.get_x();
        ULOG_PLOT("Magnus_Debug", "EKF_Pred_Vel_Norm", V3D(s_ekf.vel).norm());
        
        // [探针 2] 监控 Magnus 算出的状态与 EKF 原生状态的偏差
        // 如果这个偏差巨大（比如 > 0.1m），说明 Magnus 积分逻辑或输入参数有误
        double pos_diff = (pos_next - V3D(s_ekf.pos)).norm();
        ULOG_PLOT("Magnus_Debug", "Magnus_EKF_Pos_Diff", pos_diff);
        state_ikfom check_state = kf_state.get_x();
        if (std::isnan(check_state.vel.norm()) || std::isnan(check_state.pos.norm()) || std::isnan(check_state.rot.w())) {
            RCLCPP_ERROR(rclcpp::get_logger("ImuProcess"), 
                "[NAN DETECTED] Step 3.2: EKF State Exploded AFTER predict! Vel: %.2f", check_state.vel.norm());
            ULOG_PLOT("NaN_Debug", "3_EKF_Output_NaN", 1.0);
            // 此时系统已死，建议直接 return 防止污染点云
            // return; 
        }
        cur_state.rot = R_next;
        cur_state.pos = pos_next;
        cur_state.vel = vel_next;
        kf_state.change_x(cur_state);
        // [探针 3] 监控回写后的全量协方差迹
        // 如果这里出现负数或天文数字，说明 change_x 破坏了矩阵正定性
        auto P_post = kf_state.get_P();
        ULOG_PLOT("Magnus_Debug", "Post_Change_P_Trace", P_post.trace());
        
        // [探针 4] 监控 Magnus 强制修正的速度大小
        ULOG_PLOT("Magnus_Debug", "Magnus_Final_Vel", vel_next.norm());
        angvel_avr = in.gyro; 
        acc_avr    = in.acc;
        angvel_last = w_unb; 
        acc_s_last  = a_world_curr; 
    }
    else 
    {
        // ------------- 分支 2: 降级处理 (原始 Fast-LIO) -------------
        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                      0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                      0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr    << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                      0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                      0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
        double mean_g_norm = mean_acc.norm();
        if (mean_g_norm > 1e-3) {
            acc_avr = acc_avr * G_m_s2 / mean_g_norm;
        } else {
            // 如果 mean_acc 还没初始化好，就别除以它了，直接用原始值或者乘 9.8
            // 避免产生 Inf/NaN
            acc_avr = acc_avr * (G_m_s2 / 9.81); 
        }
        
        in.acc = acc_avr;
        in.gyro = angvel_avr;
        
        // 紧急膨胀
        double k_emergency = 100.0; 
        Q.block<3, 3>(0, 0).diagonal() = cov_gyr * k_emergency;
        Q.block<3, 3>(3, 3).diagonal() = cov_acc * k_emergency;
        Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr; 
        Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
// [新增] ================= 基于测度论的 Q 阵自适应 (分支2) =================
        // 逻辑完全同上，确保两个分支行为一致
        double acc_mag = in.acc.norm();
        
        // 计算 Raw Jerk
        double raw_jerk = 0.0;
        V3D raw_acc_head(head->linear_acceleration.x, head->linear_acceleration.y, head->linear_acceleration.z);
        V3D raw_acc_tail(tail->linear_acceleration.x, tail->linear_acceleration.y, tail->linear_acceleration.z);
        if (dt > 1e-6) {
            raw_jerk = (raw_acc_tail - raw_acc_head).norm() / dt;
        }

        double inflation_factor = 1.0;

        // [判据 A: 撞击]
        if (acc_mag > 20) {
            double excess = acc_mag - 4.0;
            // 2的幂次增长：每超 2.0 m/s^2，不信任度翻倍
            inflation_factor += std::pow(2.0, excess / 2.0);
        }

        if (raw_jerk > 2000.0) {
            inflation_factor += 50.0; // 膨胀 50 倍足够了，不要加 1000
        }

        // 应用膨胀
        if (inflation_factor > 1.0) {
    // 4. 应用膨胀 (最高 50 倍，绝不允许 1000 倍)
            if (inflation_factor > 50.0) inflation_factor = 50.0;
            
            ULOG_PLOT("EKF_Weight", "IMU_Q_Inflation", inflation_factor);
            Q.block<3, 3>(3, 3) *= inflation_factor;
        }

        kf_state.predict(dt, Q, in);

        cur_state = kf_state.get_x();
        V3D gravity_vec = cur_state.grav;
        angvel_last = angvel_avr - cur_state.bg;
        acc_s_last  = cur_state.rot * (acc_avr - cur_state.ba);
        acc_s_last += gravity_vec;
    }

    cur_state.rot.normalize(); 

    // 保存位姿
    double offs_t = t_next - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, cur_state.vel, cur_state.pos, cur_state.rot.toRotationMatrix()));
    
    t_curr = t_next;
  }

  // 5. 补齐最后一段
  double dt_tail = pcl_end_time - t_curr;
  if (dt_tail > 0) {
      kf_state.predict(dt_tail, Q, in);
      state_ikfom s_tail = kf_state.get_x();
        ULOG_PLOT("Magnus_Debug", "Tail_Vel_Norm", V3D(s_tail.vel).norm());
        ULOG_PLOT("Magnus_Debug", "Tail_dt", dt_tail);
      last_imu_ = meas.imu.back();
      last_lidar_end_time_ = pcl_end_time;
  } else {
      last_imu_ = meas.imu.back();
      last_lidar_end_time_ = pcl_end_time;
  }

  LOG_VEC("PREDICT_INPUT", "ACC_AVR", acc_avr);

  // 6. 去畸变
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  // [探针 6] 监控去畸变时作为减数的“绝对终点”
    // 如果这个值和 IMUpose.back() 的位置对不上，雷达就会断档
    ULOG_PLOT("Magnus_Debug", "Undistort_Ref_Pos_Z", cur_state.pos.z());
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    
    M3D R_imu_head;
    R_imu_head << MAT_FROM_ARRAY(head->rot);
    
    V3D vel_imu_head(VEC_FROM_ARRAY(head->vel));
    V3D pos_imu_head(VEC_FROM_ARRAY(head->pos));
    V3D acc_imu_tail(VEC_FROM_ARRAY(tail->acc));
    V3D gyr_imu_tail(VEC_FROM_ARRAY(tail->gyr));

    for(; it_pcl->curvature / double(1000) > head->offset_time; )
    {
      double dt_p = it_pcl->curvature / double(1000) - head->offset_time;

      M3D R_i(R_imu_head * Exp(gyr_imu_tail, dt_p));
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu_head + vel_imu_head * dt_p + 0.5 * acc_imu_tail * dt_p * dt_p - cur_state.pos);
      
      V3D P_compensate = cur_state.offset_R_L_I.conjugate() * (cur_state.rot.conjugate() * (R_i * (cur_state.offset_R_L_I * P_i + cur_state.offset_T_L_I) + T_ei) - cur_state.offset_T_L_I);
      // [新增检查点 4] 检查单个点的计算结果
      if (std::isnan(P_compensate.x()) || std::isnan(P_compensate.y()) || std::isnan(P_compensate.z())) {
          // 为了防止日志刷屏，只打印第一次
          static bool printed = false;
          if (!printed) {
              RCLCPP_ERROR(rclcpp::get_logger("ImuProcess"), 
                  "[NAN DETECTED] Step 4: Undistort Math Failed! Point became NaN.");
              std::cout << "Debug Info: Pos " << cur_state.pos.transpose() << std::endl;
              std::cout << "Debug Info: Rot " << cur_state.rot.coeffs().transpose() << std::endl;
              printed = true;
          }
          ULOG_PLOT("NaN_Debug", "4_Undistort_Point_NaN", 1.0);
      }
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
      it_pcl--;
    }
  }
}


