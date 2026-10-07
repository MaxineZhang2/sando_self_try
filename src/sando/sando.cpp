/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "sando/sando.hpp"
#include <chrono>
#include <fstream>

using namespace sando;
using namespace termcolor;

typedef timer::Timer MyTimer;

// ----------------------------------------------------------------------------
// SANDO 启动时，把后面重规划要用的工具、参数、求解器、状态、地图尺寸全部准备好。
SANDO::SANDO(Parameters par) : par_(par) {    // 把外面传进来的 Parameters par 保存到成员变量 par_
  // Set up hgp_manager
  hgp_manager_.setParameters(par_);    // 初始化 HGP 全局规划器，先把规划器所需的参数 par_ 传进去

  // Compute factors_ for time allocation
  if (par_.use_dynamic_factor) {
    // Dynamic factor search
    num_dynamic_factors_ =
        static_cast<int>((2 * par_.dynamic_factor_k_radius) / par_.factor_constant_step_size) + 1;
    factors_.reserve(num_dynamic_factors_);
    for (int i = 0; i < num_dynamic_factors_; i++) {
      double factor = par_.dynamic_factor_initial_mean - par_.dynamic_factor_k_radius +
                      i * par_.factor_constant_step_size;
      if (factor >= par_.factor_initial && factor <= par_.factor_final) factors_.push_back(factor);
    }
  } else {
    // Constant factor search
    num_dynamic_factors_ =
        static_cast<int>(
            (par_.factor_final - par_.factor_initial) / par_.factor_constant_step_size) +
        1;
    factors_.reserve(num_dynamic_factors_);
    for (int i = 0; i < num_dynamic_factors_; i++) {
      double factor = par_.factor_initial + i * par_.factor_constant_step_size;
      factors_.push_back(factor);
    }
  }

  // Set up optimization solver for whole trajectory.
  // Hybrid threading: distribute CPU cores across external factor threads
  // so each Gurobi instance gets multiple internal threads rather than
  // all threads competing for all cores.
  
  // 给每个 Gurobi solver 分配线程数,机器总 CPU 核数 / factor 数量 = 每个 Gurobi solver 可用线程数,比如有 16 核，要并行试 4 个 factor，每个 Gurobi solver 用 4 个线程
  const int num_cores = static_cast<int>(std::thread::hardware_concurrency());
  const int grb_threads_per_solver = std::max(1, num_cores / std::max(1, num_dynamic_factors_));
  whole_traj_solver_ptrs_.reserve(num_dynamic_factors_);
  // 每个 factor 配一个 SolverGurobi，因为后面 planLocalTrajectory() 会并行尝试多个 factor，谁先求出可行轨迹就用谁，这能提高实时性
  for (int i = 0; i < num_dynamic_factors_; i++) {
    auto solver = std::make_shared<SolverGurobi>();
    solver->setGurobiThreads(grb_threads_per_solver);
    solver->initializeSolver(par_);
    whole_traj_solver_ptrs_.push_back(solver);    // 这段代码的作用是初始化多个求解器（SolverGurobi），并将它们存储到 whole_traj_solver_ptrs_ 容器中，此时并没有绑定任何任务
  }

  // Set up decomp ellip workers for each thread
  // 准备安全走廊分解 worker,ellip_workers_ 用于凸分解，也就是安全走廊生成，因为后面每个 factor 可能并行生成时空安全走廊，所以这里也准备多个 worker
  ellip_workers_.resize(whole_traj_solver_ptrs_.size());    

  // Pre-compute the worst initial_dt * par_.num_N，用 getInitialDt() 估计每段时间，再乘以轨迹段数 num_N
  // (this is the worst case time allocated for the whole trajectory)
  auto tmp_traj_solver_ptr = std::make_shared<SolverGurobi>();
  tmp_traj_solver_ptr->initializeSolver(par_);
  tmp_traj_solver_ptr->resetToNominalState();
  RobotState tmp_start_state, tmp_end_state;
  tmp_end_state.setPos(par_.num_P * par_.max_dist_vertexes, 0.0, 0.0);
  tmp_traj_solver_ptr->setX0(tmp_start_state);
  tmp_traj_solver_ptr->setXf(tmp_end_state);
  worst_traj_time_ = tmp_traj_solver_ptr->getInitialDt() * par_.num_N;    // 这个值后面可以用于动态障碍膨胀、未知空间截断、安全判断等

  // Set up basis converter，准备 MINVO 基变换矩阵，后面检查轨迹是否在安全走廊内会用
  BasisConverter basis_converter;
  A_rest_pos_basis_ = basis_converter.getArestMinvo();  // Use Minvo basis
  A_rest_pos_basis_inverse_ = A_rest_pos_basis_.inverse();

  // Parameters，保存动力学限制
  v_max_3d_ = Eigen::Vector3d(par_.v_max, par_.v_max, par_.v_max);
  v_max_ = par_.v_max;
  a_max_3d_ = Eigen::Vector3d(par_.a_max, par_.a_max, par_.a_max);
  j_max_3d_ = Eigen::Vector3d(par_.j_max, par_.j_max, par_.j_max);

  // Initialize the state
  changeDroneStatus(DroneStatus::GOAL_REACHED);

  // Initialize the map size
  wdx_ = par_.initial_wdx;
  wdy_ = par_.initial_wdy;
  wdz_ = par_.initial_wdz;

  // Map resolution
  map_res_ = par_.res;
}

// ----------------------------------------------------------------------------

void SANDO::startAdaptKValue() {
  // Compute the average computation time
  const size_t num_samples = store_computation_times_.size();
  for (const auto& comp_time : store_computation_times_) {
    est_comp_time_ += comp_time;
  }
  est_comp_time_ = est_comp_time_ / num_samples;

  // Start k_value adaptation
  use_adapt_k_value_ = true;
}

// ----------------------------------------------------------------------------
// 计算一个新的目标状态 G，并将其设置为机器人当前的目标状态。计算的目标状态 G 是基于当前机器人状态 A 和终端目标状态 G_term，并且受限于一个指定的预测范围（horizon）
void SANDO::computeG(const RobotState& A, const RobotState& G_term, double horizon) {
  // Initialize the result
  RobotState local_G;

  // Compute pos for G
  local_G.pos = sando_utils::projectPointToSphere(A.pos, G_term.pos, horizon);

  // Compute yaw for G
  Eigen::Vector3d dir = (G_term.pos - local_G.pos).normalized();
  local_G.yaw = atan2(dir[1], dir[0]);

  // Set G
  setG(local_G);
}

// ----------------------------------------------------------------------------
bool SANDO::needReplan(
    const RobotState& local_state,
    const RobotState& local_G_term,
    const RobotState& last_plan_state) {
  // Compute the distance to the terminal goal
  double dist_to_term_G = (local_state.pos - local_G_term.pos).norm();
  double dist_from_last_plan_state_to_term_G = (last_plan_state.pos - local_G_term.pos).norm();

  // Check velocity magnitude to ensure drone is moving slowly enough
  double vel_magnitude = local_state.vel.norm();
  const double max_goal_velocity = 0.1;  // [m/s] Maximum velocity when reaching goal

  // Hover avoidance: allow replanning when GOAL_REACHED（目标已经到达） or HOVER_AVOIDING（悬停避障）
  // Must be checked before the GOAL_REACHED early return so the drone can
  // detect nearby obstacles while hovering at the goal.
  if (par_.hover_avoidance_enabled && (drone_status_ == DroneStatus::GOAL_REACHED ||    // 规划到下一个目标点
                                       drone_status_ == DroneStatus::HOVER_AVOIDING)) {    // 规划一条避障路径，确保安全
    return true;
  }

  if (dist_to_term_G < par_.goal_radius && vel_magnitude < max_goal_velocity) {   // 这是判断无人机是否真正到达目标，只看位置距离是不够的，如果无人机高速掠过目标点，这是不算到达目标的
    if (par_.hover_avoidance_enabled) {
      p_hover_ = local_G_term.pos;
      changeDroneStatus(DroneStatus::HOVER_AVOIDING);
      return true;  // allow replan loop to run checkHoverAvoidance
    } else {
      changeDroneStatus(DroneStatus::GOAL_REACHED);
      p_hover_ = local_G_term.pos;
      return false;
    }
  }

  // Don't plan if drone is not traveling
  // IMPORTANT: this must come BEFORE the GOAL_SEEN check below, otherwise
  // the GOAL_SEEN distance check would overwrite YAWING status and trigger
  // replanning (causing the drone to move instead of yawing in place).
  if (drone_status_ == DroneStatus::GOAL_REACHED || drone_status_ == DroneStatus::YAWING)
    return false;

  if (dist_to_term_G < par_.goal_seen_radius) {
    changeDroneStatus(
        DroneStatus::GOAL_SEEN);  // This triggers to use the hard final state constraint
  }

  if (drone_status_ == DroneStatus::GOAL_SEEN &&
      dist_from_last_plan_state_to_term_G < par_.goal_radius)
    return false;

  return true;
}

// ----------------------------------------------------------------------------
// 主要功能是根据当前时间和规划路径，找到一个对接的状态 A 及其对应的时间 A_time，true 表示成功找到，false 表示失败。
bool SANDO::findAandAtime(    // 输入参数：current_time：当前时间；last_replaning_computation_time：上一次重新规划的计算时间。
    RobotState& A, double& A_time, double current_time, double last_replaning_computation_time) {    // 输出参数：A：目标状态（RobotState 类型），从规划路径中选取；A_time：目标状态对应的时间。
  mtx_plan_.lock();    // 使用互斥锁 mtx_plan_ 来保护对 plan_ 的访问，确保在多线程环境下读取路径时不会发生数据竞争
  int plan_size = plan_.size();
  mtx_plan_.unlock();

  if (plan_size == 0) {
    std::cout << bold << red << "plan_size == 0" << reset << std::endl;
    return false;
  }

  if (par_.use_state_update) {
    // Change k_value dynamically
    // To get stable results, we will use a default value of k_value until we have enough
    // computation time
    if (!use_adapt_k_value_) {
      // Use default k_value
      k_value_ = std::max((int)plan_size - par_.default_k_value, 0);

      // Store computation times
      if (num_replanning_ != 1)  // 如果不是第一次重新规划，则将上一次的计算时间存储到 store_computation_times_ 中
        store_computation_times_.push_back(last_replaning_computation_time);
    } else {
      // Computation time filtering,启用了动态调整（use_adapt_k_value_ == true），则根据上一次的计算时间动态调整 k_value_。
      est_comp_time_ = par_.alpha_k_value_filtering * last_replaning_computation_time +
                       (1 - par_.alpha_k_value_filtering) * est_comp_time_;    // 使用指数平滑公式计算估计的计算时间

      // Get state number based on est_comp_time_ and dc
      k_value_ =
          std::max((int)plan_size - (int)(par_.k_value_factor * est_comp_time_ / par_.dc), 0);
    }

    // Check if k_value_ is valid
    if (plan_size - 1 - k_value_ < 0 || plan_size - 1 - k_value_ >= plan_size) {
      k_value_ =
          plan_size - 1;  // If k_value_ is larger than the plan size, we set it to the last state
    }

    // Get A
    mtx_plan_.lock();
    A = plan_[plan_size - 1 - k_value_];
    mtx_plan_.unlock();

    // Get A_time
    A_time = current_time +
             (plan_size - 1 - k_value_) *
                 par_.dc;  // time to A from current_pos is (plan_size - 1 - k_value_) * par_.dc;
  } else  // If we don't update state - this is for global planner benchmarking purposes
  {
    // Get state
    getState(A);
    A_time = current_time;
  }

  // Check if A is within the map (especially for z)
  if ((A.pos[2] < par_.z_min || A.pos[2] > par_.z_max) ||
      (A.pos[0] < par_.x_min || A.pos[0] > par_.x_max) ||
      (A.pos[1] < par_.y_min || A.pos[1] > par_.y_max)) {
    printf("A (%f, %f, %f) is out of the map\n", A.pos[0], A.pos[1], A.pos[2]);
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------
// 检查某个点是否被占据（即是否在地图中被标记为障碍物或不可通行区域）
bool SANDO::checkIfPointOccupied(const Vec3f& point) {
  // Check if the point is free
  return hgp_manager_.checkIfPointOccupied(point);
}

// ----------------------------------------------------------------------------

bool SANDO::checkIfPointFree(const Vec3f& point) {
  // Check if the point is free
  return hgp_manager_.checkIfPointFree(point);
}

// ----------------------------------------------------------------------------
// HGP（全局规划器）输出的路径是几何上从 A 到 G 的一条折线。但这条折线可能穿过未知空间（传感器没看到的地方）
// 从路径起点开始，沿路径往前走。一旦碰到未知空间，就沿路径向后倒退，退到一个"已经充分远离未知空间"的安全点，然后截断——后面所有点全部丢弃
void SANDO::findSafeSubGoal(vec_Vecf<3>& global_path) {
  // Keep the original global path
  vec_Vecf<3> original_global_path = global_path;

  // Reset goal path
  global_path.clear();

  if (original_global_path.empty()) return;

  // Initialize it with the start point
  global_path.push_back(original_global_path[0]);

  // Kd-tree search parameters
  const int k = 1;  // nearest neighbor
  std::vector<int> pointIdxNKNSearch(k);
  std::vector<float> pointNKNSquaredDistance(k);

  // Sampling parameters (TODO: make these parameters configurable)
  const double sample_dist = 0.1;  // [m] distance between two samples along the trajectory

  // Inflation radius for unknown space (max extent)
  const double r_inflate = par_.obst_max_vel * traj_max_time_;  // [m]
  const double thr_orig = par_.drone_radius;                    // [m]
  const double thr_infl = par_.drone_radius + r_inflate;        // [m]
  const double thr_orig2 = thr_orig * thr_orig;
  const double thr_infl2 = thr_infl * thr_infl;

  // Mutex lock (KD-tree shared)
  std::lock_guard<std::mutex> lk(mtx_kdtree_unk_);

  // Helper: returns true if pt is within (unknown KD-tree distance) <= threshold^2.
  auto isWithinUnknown = [&](const Eigen::Vector3d& pt, double thr2) -> bool {
    pcl::PointXYZ searchPoint(pt(0), pt(1), pt(2));
    if (kdtree_unk_.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) >
        0) {
      return static_cast<double>(pointNKNSquaredDistance[0]) < thr2;
    }
    return false;
  };

  // Helper: backtrack from a hit location (segment i, arc-length s_hit along that segment)
  // until outside inflated unknown. Returns the backtracked safe point.
  // 命中点所在的段索引 seg_i 和 命中点在该段上的弧长 s_hit。输出：从命中点沿路径向后倒退、直到走出膨胀未知空间的那个点
  auto backtrackToOutsideInflated = [&](int seg_i, double s_hit) -> Eigen::Vector3d {
    // We will walk backward in steps of sample_dist along the polyline.
    int i = seg_i;
    if (i < 0) i = 0;
    if (i >= static_cast<int>(original_global_path.size()) - 1)
      i = static_cast<int>(original_global_path.size()) - 2;

    Eigen::Vector3d A = original_global_path[i];
    Eigen::Vector3d B = original_global_path[i + 1];

    Eigen::Vector3d d = B - A;
    double L = d.norm();
    if (L < 1e-9) return A;  // degenerate segment

    Eigen::Vector3d dir = d / L;

    // Clamp s to [0, L]
    double s = std::min(std::max(0.0, s_hit), L);

    // Start from the hit point
    Eigen::Vector3d pt = A + dir * s;

    // If we're already outside inflated unknown, keep it (shouldn't happen in your described flow)
    if (!isWithinUnknown(pt, thr_infl2)) return pt;

    // Walk backward until outside inflated unknown or we reach the start.
    // This can cross segment boundaries if inflation is large.
    while (true) {
      // Step backward on current segment
      s -= sample_dist;

      if (s >= 0.0) {
        // 还在当前段内，直接更新 pt
        pt = A + dir * s;
      } else {
        // Need to go to previous segment
        i -= 1;
        if (i < 0) {
          // 退到路径起点了，返回起点
          // We reached the very beginning; return the start point (best we can do)
          return original_global_path.front();
        }

        // New segment [i, i+1]
        // 取前一段 [i, i+1]
        A = original_global_path[i];
        B = original_global_path[i + 1];
        d = B - A;
        L = d.norm();
        if (L < 1e-9) {
          // Skip degenerate segment
          s = 0.0;
          pt = A;
          continue;
        }
        dir = d / L;

        // We crossed into previous segment: set s at its end (B) plus leftover negative s
        // Example: if s was -0.03, we start at L - 0.03 on the previous segment.
        // 关键：把负的 s 换算到前一段上的位置
        s = L + s;  // s is negative here,s 是负数，L + s 就是"从 B 往 A 退 |s| 米"
        if (s < 0.0) s = 0.0;
        if (s > L) s = L;

        pt = A + dir * s;
      }

      // Check inflated condition
      if (!isWithinUnknown(pt, thr_infl2)) return pt;
    }
  };

  // Loop through the global path and check for intersection with original unknown (NOT inflated)
  const int M = static_cast<int>(original_global_path.size());
  for (int i = 0; i < M - 1; i++) {
    Eigen::Vector3d current_gp = original_global_path[i];
    Eigen::Vector3d next_gp = original_global_path[i + 1];

    Eigen::Vector3d dir = next_gp - current_gp;
    double dist = dir.norm();
    if (dist < 1e-9) {
      // Degenerate; just continue
      continue;
    }
    dir /= dist;

    // Sample points along the line segment
    const int num_samples = static_cast<int>(dist / sample_dist);

    for (int j = 0; j <= num_samples; j++) {
      Eigen::Vector3d sample_point = current_gp + dir * (sample_dist * j);

      // Detect intersection with original unknown (same as before, but squared distance)
      if (isWithinUnknown(sample_point, thr_orig2)) {
        // Found first contact with original unknown -> now backtrack until outside inflated unknown
        const double s_hit = sample_dist * j;
        Eigen::Vector3d safe_pt = backtrackToOutsideInflated(i, s_hit);

        // Ensure we don't add duplicates
        if ((safe_pt - global_path.back()).norm() > 1e-6) global_path.push_back(safe_pt);

        return;  // Stop: this is the new last global path point
      }
    }

    // No unknown intersection on this segment; keep the next waypoint
    global_path.push_back(next_gp);
  }
}

// ----------------------------------------------------------------------------
// 输入min_pos和max_pos，对应当前无人机位置和局部目标 G
// 地图窗口覆盖“当前点到目标点”这一段区域，
// 左右上下再加 buffer，
// 同时不能小于最小地图尺寸。
void SANDO::computeMapSize(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos) {
  // Get local_A
  RobotState local_A;
  getA(local_A);

  // Increase the effective buffer size based on the number of HGP failures.
  double dynamic_buffer = par_.map_buffer;

  // Increase the effective buffer size based on velocity.
  double dynamic_buffer_x = dynamic_buffer;
  double dynamic_buffer_y = dynamic_buffer;
  double dynamic_buffer_z = dynamic_buffer;

  // Compute the distance to the terminal goal for each axis.
  double dist_x = std::abs(min_pos[0] - max_pos[0]);
  double dist_y = std::abs(min_pos[1] - max_pos[1]);
  double dist_z = std::abs(min_pos[2] - max_pos[2]);

  // Update the map size based on the min and max positions.
  wdx_ = std::max(dist_x + 2 * dynamic_buffer_x, par_.min_wdx);
  wdy_ = std::max(dist_y + 2 * dynamic_buffer_y, par_.min_wdy);
  wdz_ = std::max(dist_z + 2 * dynamic_buffer_z, par_.min_wdz);

  // Compute the base map center as the midpoint between the min and max positions.
  map_center_ = (min_pos + max_pos) / 2.0;
}

// ----------------------------------------------------------------------------
// 判断一个点是否在当前局部地图窗口里
// 主要用途是在：
// computeObstPosAndTrajMaxTimeForMapUpdate()
// 里筛选动态障碍物。
// 如果动态障碍物当前位置不在当前地图窗口里，就不用参与本次地图更新/heat map/局部规划。
bool SANDO::checkPointWithinMap(const Eigen::Vector3d& point) const {
  // Check if the point is within the map boundaries for each axis
  return (std::abs(point[0] - map_center_[0]) <= wdx_ / 2.0) &&
         (std::abs(point[1] - map_center_[1]) <= wdy_ / 2.0) &&
         (std::abs(point[2] - map_center_[2]) <= wdz_ / 2.0);
}

// ----------------------------------------------------------------------------

void SANDO::getStaticPushPoints(vec_Vecf<3>& static_push_points) {
  static_push_points = static_push_points_;
}

// ----------------------------------------------------------------------------

void SANDO::getLocalGlobalPath(
    vec_Vecf<3>& local_global_path, vec_Vecf<3>& local_global_path_after_push) {
  local_global_path = local_global_path_;
  local_global_path_after_push = local_global_path_after_push_;
}

// ----------------------------------------------------------------------------

void SANDO::getGlobalPath(vec_Vecf<3>& global_path) {
  mtx_global_path_.lock();
  global_path = global_path_;
  mtx_global_path_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getOriginalGlobalPath(vec_Vecf<3>& original_global_path) {
  mtx_original_global_path_.lock();
  original_global_path = original_global_path_;
  mtx_original_global_path_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getFreeGlobalPath(vec_Vecf<3>& free_global_path) {
  free_global_path = free_global_path_;
}

// ----------------------------------------------------------------------------

void SANDO::resetData() {
  final_g_ = 0.0;
  global_planning_time_ = 0.0;
  hgp_static_jps_time_ = 0.0;
  hgp_check_path_time_ = 0.0;
  hgp_dynamic_astar_time_ = 0.0;
  hgp_recover_path_time_ = 0.0;
  cvx_decomp_time_ = 0.0;
  local_traj_computation_time_ = 0.0;
  safe_paths_time_ = 0.0;
  safety_check_time_ = 0.0;
  yaw_sequence_time_ = 0.0;
  yaw_fitting_time_ = 0.0;

  poly_out_whole_.clear();
  poly_out_safe_.clear();
  goal_setpoints_.clear();
  pwp_to_share_.clear();
  optimal_yaw_sequence_.clear();
  yaw_control_points_.clear();
  yaw_knots_.clear();
  cps_.clear();
}

// ----------------------------------------------------------------------------

void SANDO::retrieveData(
    double& final_g,
    double& global_planning_time,
    double& hgp_static_jps_time,
    double& hgp_check_path_time,
    double& hgp_dynamic_astar_time,
    double& hgp_recover_path_time,
    double& cvx_decomp_time,
    double& local_traj_computatoin_time,
    double& safety_check_time,
    double& safe_paths_time,
    double& yaw_sequence_time,
    double& yaw_fitting_time,
    double& successful_factor) {
  final_g = final_g_;
  global_planning_time = global_planning_time_;
  hgp_static_jps_time = hgp_static_jps_time_;
  hgp_check_path_time = hgp_check_path_time_;
  hgp_dynamic_astar_time = hgp_dynamic_astar_time_;
  hgp_recover_path_time = hgp_recover_path_time_;
  cvx_decomp_time = cvx_decomp_time_;
  local_traj_computatoin_time = local_traj_computation_time_;
  safe_paths_time = safe_paths_time_;
  safety_check_time = safety_check_time_;
  yaw_sequence_time = yaw_sequence_time_;
  yaw_fitting_time = yaw_fitting_time_;
  successful_factor = successful_factor_;
}

// ----------------------------------------------------------------------------

void SANDO::retrievePolytopes(
    vec_E<Polyhedron<3>>& poly_out_whole, vec_E<Polyhedron<3>>& poly_out_safe) {
  poly_out_whole = poly_out_whole_;
  poly_out_safe = poly_out_safe_;
}

// ----------------------------------------------------------------------------

void SANDO::retrieveGoalSetpoints(std::vector<RobotState>& goal_setpoints) {
  goal_setpoints = goal_setpoints_;
}

// ----------------------------------------------------------------------------

void SANDO::retrieveListSubOptGoalSetpoints(
    std::vector<std::vector<RobotState>>& list_subopt_goal_setpoints) {
  list_subopt_goal_setpoints = list_subopt_goal_setpoints_;
}

// ----------------------------------------------------------------------------

void SANDO::retrieveCPs(std::vector<Eigen::Matrix<double, 3, 4>>& cps) { cps = cps_; }

// ----------------------------------------------------------------------------

std::tuple<bool, bool> SANDO::replan(double last_replaning_computation_time, double current_time) {
  /* -------------------- Housekeeping -------------------- */

  MyTimer timer_housekeeping(true);

  // Reset Data
  resetData();

  // Check if we need to replan
  if (!checkReadyToReplan()) {
    std::cout << bold << red << "Planner is not ready to replan" << reset << std::endl;
    return std::make_tuple(false, false);
  }

  // Get states we need
  RobotState local_state, local_G_term, last_plan_state;
  getState(local_state);
  getGterm(local_G_term);
  getLastPlanState(last_plan_state);

  // Check if we need to replan based on the distance to the terminal goal
  if (!needReplan(local_state, local_G_term, last_plan_state)) return std::make_tuple(false, false);

  // Hover avoidance: check obstacles and potentially set evasion goal
  if (par_.hover_avoidance_enabled && (drone_status_ == DroneStatus::GOAL_REACHED ||
                                       drone_status_ == DroneStatus::HOVER_AVOIDING)) {
    if (!checkHoverAvoidance(current_time))
      return std::make_tuple(false, false);  // no avoidance needed, stay hovering
  }

  if (par_.debug_verbose)
    std::cout << "Housekeeping: " << timer_housekeeping.getElapsedMicros() / 1000.0 << " ms"
              << std::endl;

  /* -------------------- Global Planning -------------------- */

  MyTimer timer_global(true);
  vec_Vecf<3> global_path;
  if (!generateGlobalPath(global_path, current_time, last_replaning_computation_time)) {
    if (par_.debug_verbose)
      std::cout << "Global Planning: " << timer_global.getElapsedMicros() / 1000.0 << " ms"
                << std::endl;
    return std::make_tuple(false, false);
  }
  if (par_.debug_verbose)
    std::cout << "Global Planning: " << timer_global.getElapsedMicros() / 1000.0 << " ms"
              << std::endl;

  /* -------------------- Local Trajectory Optimization -------------------- */

  MyTimer timer_local(true);
  if (!planLocalTrajectory(global_path, last_replaning_computation_time)) {
    if (par_.debug_verbose)
      std::cout << "Local Trajectory Optimization: " << timer_local.getElapsedMicros() / 1000.0
                << " ms" << std::endl;
    return std::make_tuple(false, true);
  }
  if (par_.debug_verbose)
    std::cout << "Local Trajectory Optimization: " << timer_local.getElapsedMicros() / 1000.0
              << " ms" << std::endl;

  /* -------------------- Append to Plan -------------------- */

  MyTimer timer_append(true);
  if (!appendToPlan()) {
    if (par_.debug_verbose)
      std::cout << "Append to Plan: " << timer_append.getElapsedMicros() / 1000.0 << " ms"
                << std::endl;
    return std::make_tuple(false, true);
  }
  if (par_.debug_verbose)
    std::cout << "Append to Plan: " << timer_append.getElapsedMicros() / 1000.0 << " ms"
              << std::endl;

  /* -------------------- Final Housekeeping -------------------- */

  MyTimer timer_final(true);

  if (par_.debug_verbose)
    printf("\033[1;32mReplanning succeeded (factor=%.2f)\033[0m\n", successful_factor_);

  // Reset the replanning failure count
  replanning_failure_count_ = 0;
  if (par_.debug_verbose)
    std::cout << "Final Housekeeping: " << timer_final.getElapsedMicros() / 1000.0 << " ms"
              << std::endl;

  return std::make_tuple(true, true);
}

// ----------------------------------------------------------------------------
// 从当前状态 A 到局部目标 G，生成一条粗糙但安全的全局路径
// 获取当前机器人状态（A）和目标状态（G）。
// 更新地图信息。
// 调用高阶全局规划器（HGP，Hierarchical Global Planner）生成路径。
// 对生成的路径进行后处理（如修剪、保存、可视化等）。
bool SANDO::generateGlobalPath(
    vec_Vecf<3>& global_path, double current_time, double last_replaning_computation_time) {    // global_path：引用类型，存储生成的全局路径
  // Get G and G_term
  RobotState local_G, local_G_term;
  getG(local_G);    // 当前目标状态，表示机器人在当前规划周期内的目标位置
  getGterm(local_G_term);    // 终端目标状态，表示机器人最终需要到达的位置

  // Declare local variables
  RobotState local_A;     // 当前机器人状态，包括位置和朝向
  double A_time;    // 当前机器人状态对应的时间

  // Find A and A_time，找到起点 A 及其对应时间 A_time
  if (!findAandAtime(local_A, A_time, current_time, last_replaning_computation_time)) {
    replanning_failure_count_++;
    return false;
  }

  // Set A and A_time，写入成员变量 A_ 和 A_time_，后面 planLocalTrajectory 会通过 getA / getA_time 读出来用
  setA(local_A);
  setA_time(A_time);

  // Compute G
  computeG(local_A, local_G_term, par_.horizon);

  // Update Map
  // fake_sim / rviz_only	updateOccupancyMap	只有占据栅格，没有未知空间 KD-tree
  // gazebo / hardware	updateMap	同时更新占据栅格 + 未知空间 KD-tree + 动态障碍
  // update内部地图更新都包括：
  // 调 computeMapSize 计算地图尺寸和中心；
  // 调 computeObstPosAndTrajMaxTimeForMapUpdate 采样动态障碍；
  // 调 hgp_manager_.updateMap(...) 把点云、障碍、预测样本喂给 HGP。
  if (par_.sim_env == "fake_sim" || par_.sim_env == "rviz_only") {
    updateOccupancyMap(current_time);
  } else {
    updateMap(current_time);
  }

  // Set up the HGP planner 初始化 HGP 规划器(since updateVmax() needs to be called after setupHGPPlanner, we use
  // v_max_ from the last replan)
  // 一大堆参数传给 HGP：
  // map_res_：地图分辨率
  // v_max_、a_max、j_max：动力学限制（HGP 用它估计启发式/时间）
  // hgp_timeout_duration_ms：HGP 搜索超时
  // max_num_expansion：最大扩展节点数
  // w_unknown：未知空间惩罚权重（让路径尽量避开未知区域）
  // w_align、decay_len_cells、w_side、los_cells：路径平滑/对齐相关
  // min_len、min_turn：最小段长、最小转弯
  hgp_manager_.setupHGPPlanner(   
      par_.global_planner, par_.global_planner_verbose, map_res_, v_max_, par_.a_max, par_.j_max,
      par_.hgp_timeout_duration_ms, par_.max_num_expansion, par_.w_unknown, par_.w_align,
      par_.decay_len_cells, par_.w_side, par_.los_cells, par_.min_len, par_.min_turn);

  // Free start and goal if necessary
  // 如果起点或终点恰好落在障碍物/未知空间的膨胀区域内，HGP 会直接失败。freeStart / freeGoal 会在起点/终点附近临时清除一定半径内的障碍，保证 HGP 能起搜。
  if (par_.use_free_start) hgp_manager_.freeStart(local_A.pos, par_.free_start_factor);
  if (par_.use_free_goal) hgp_manager_.freeGoal(local_G.pos, par_.free_goal_factor);

  // Debug
  if (par_.debug_verbose) std::cout << "Solving HGP" << std::endl;

  // if using ground robot, we fix the z，地面机器人只在平面运动，强制起点和终点 z=1.0
  if (par_.vehicle_type != "uav") {
    local_A.pos[2] = 1.0;
    local_G.pos[2] = 1.0;
  }

  // 1) Build a direction hint from the *previous* global path
  // HGP 是基于采样的规划器，如果没有方向先验，可能在开阔空间里搜出一条"绕来绕去"的路径。用上次路径的第一段方向做提示，能让新路径和旧路径尽量对齐，减少抖动。
  vec_Vecf<3> prev_global;
  getGlobalPath(prev_global);  // last successful global path

  Eigen::Vector3d dir_hint = (local_G.pos - local_A.pos).normalized();
  if (prev_global.size() >= 2) {
    Eigen::Vector3d s0 = prev_global[0];
    Eigen::Vector3d s1 = prev_global[1];
    Eigen::Vector3d seg = s1 - s0;
    if (seg.norm() > 1e-8) {
      dir_hint = seg.normalized();
    }
  } else {
    dir_hint = local_G.pos - local_A.pos;
    if (dir_hint.norm() > 1e-8) dir_hint.normalize();
  }

  // Keep ground robots planar
  if (par_.vehicle_type != "uav") dir_hint[2] = 0.0;

  // 2) Use this as the "start_vel" argument (magnitude doesn't matter; we use the direction)
  Vec3f start_dir_hint(dir_hint.x(), dir_hint.y(), dir_hint.z());

  // Solve HGP
  // 参数：
  // local_A.pos：起点位置
  // start_dir_hint：方向提示
  // local_G.pos：终点位置
  // final_g_：输出，最终代价（cost）
  // par_.global_planner_heuristic_weight：启发式权重（A* 的 ε）
  // A_time：起始时间（动态障碍用）
  // global_path：输出，处理后的路径（可能被 HGP 内部平滑/裁剪）
  // raw_global_path：输出，原始路径（未处理，用于可视化）
  // 失败就计数并返回 false。
  vec_Vecf<3> raw_global_path;
  if (!hgp_manager_.solveHGP(
          local_A.pos, start_dir_hint, local_G.pos, final_g_, par_.global_planner_heuristic_weight,
          A_time, global_path, raw_global_path)) {
    if (par_.debug_verbose)
      std::cout << bold << red << "HGP did not find a solution" << reset << std::endl;
    hgp_failure_count_++;
    replanning_failure_count_++;
    return false;
  }

  // Log replan details to file (only when debug_verbose is enabled)
  // 把 A、G、G_term、dir_hint、路径前 5 个点写到 /tmp/sando_goal_log.txt，方便离线分析。
  if (par_.debug_verbose) {
    static const std::string path = "/tmp/sando_goal_log.txt";
    std::ofstream f(path, std::ios::app);
    if (f.is_open()) {
      static auto t0 = std::chrono::steady_clock::now();
      double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      f << std::fixed << std::setprecision(3) << "[" << t << "s] REPLAN #" << num_replanning_
        << "\n"
        << "  A:          (" << local_A.pos.x() << ", " << local_A.pos.y() << ", "
        << local_A.pos.z() << ")\n"
        << "  G:          (" << local_G.pos.x() << ", " << local_G.pos.y() << ", "
        << local_G.pos.z() << ")\n"
        << "  G_term:     (" << local_G_term.pos.x() << ", " << local_G_term.pos.y() << ", "
        << local_G_term.pos.z() << ")\n"
        << "  dir_hint:   (" << dir_hint.x() << ", " << dir_hint.y() << ", " << dir_hint.z()
        << ")\n"
        << "  global_path (" << global_path.size() << " pts):";
      for (size_t i = 0; i < std::min(global_path.size(), (size_t)5); i++)
        f << " (" << global_path[i][0] << "," << global_path[i][1] << "," << global_path[i][2]
          << ")";
      if (global_path.size() > 5) f << " ...";
      f << "\n\n";
      f.close();
    }
  }

  // 保存全局路径（供可视化和其他模块用）
  // use this for map resizing
  // global_path_：处理后的路径（后续 planLocalTrajectory 用）
  // original_global_path_：原始路径（可视化用，能看到 HGP 的原始输出）
  // 用 mutex 保护，因为 getGlobalPath / getOriginalGlobalPath 可能在别的线程被调用。
  mtx_global_path_.lock();
  global_path_ = global_path;
  mtx_global_path_.unlock();

  // For visualization
  mtx_original_global_path_.lock();
  original_global_path_ = raw_global_path;
  mtx_original_global_path_.unlock();

  // Make sure global path does not exceed (num_P + 1)
  // 因为后面凸分解和 Gurobi 优化的规模直接和路径段数挂钩：const size_t P = (global_path.size() >= 2) ? (global_path.size() - 1) : 0;
  // P 是安全走廊段数，P 越大，cvxEllipsoidDecomp 和 Gurobi 求解越慢。为了保证实时性，把路径限制在 num_P + 1 个点（即 num_P 段）以内
  // 裁掉的是尾部，也就是远端部分。近端（起点附近）保留，因为那是本次真正要飞的。
  if (global_path.size() > par_.num_P + 1) {
    // Trim the global path
    global_path.resize(par_.num_P + 1);
  }

  // Find global path with safe sub goal
  // 从路径起点开始，沿路径以 0.1 m 采样
  // 一旦某个采样点落在未知空间（kdtree_unk_ 查询距离 < drone_radius），说明路径穿过了传感器没看到的地方
  // 从这个命中点沿路径向后倒退，直到退到"膨胀未知空间"之外（膨胀半径 = obst_max_vel * traj_max_time_）
  // 把退到的安全点加入 global_path，丢弃后面所有点
  // 结果：global_path.back() 从"HGP的原始末端"变成"安全截断点"。
  // 为什么需要？ 未知空间不能假设是空的。如果直接拿穿过未知空间的路径做安全走廊，走廊可能横跨未知区域，轨迹就危险了。
  findSafeSubGoal(global_path);

  // Debug
  if (par_.debug_verbose) std::cout << "global_path.size(): " << global_path.size() << std::endl;

  // Get computation time
  // HGP 内部有多个阶段（静态 JPS、路径检查、动态 A*、路径恢复），分别计时，写到成员变量，供 retrieveData 取出分析。
  hgp_manager_.getComputationTime(
      global_planning_time_, hgp_static_jps_time_, hgp_check_path_time_, hgp_dynamic_astar_time_,
      hgp_recover_path_time_);

  return true;
}

// ----------------------------------------------------------------------------
// 输入：HGP 给出的 global_path
// 输出：一条真正可执行的局部轨迹 goal_setpoints_
// 过程：围绕 global_path 生成安全走廊，然后用 Gurobi 优化一条满足动力学约束的轨迹
bool SANDO::planLocalTrajectory(vec_Vecf<3>& global_path, double last_replaning_computation_time) {
  // Get local_A, local_G and A_time
  RobotState local_A, local_G, local_E;
  double A_time;
  getA(local_A);    // 新轨迹从哪里开始
  getG(local_G);    // 当前 horizon 内的局部目标
  getA_time(A_time);    // 新轨迹起点 A 对应的未来时间

  // If the global path has exactly 2 points, subdivide the longest segment
  // by inserting midpoints until we reach at least 3 points.
  // 如果全局路径只有A → G，就插入中点A → mid → G
  // 原因是后面的安全走廊和局部轨迹分段需要至少两个空间段。否则局部优化空间太弱，约束分配也容易退化。
  // 改进：
  // 现在只是简单插中点。
  // 可以按路径长度、自适应曲率、障碍物密度插点，而不是只在 size==2 时插一个中点。
  // 比如：长直线段可以按 max_dist_vertexes 多次细分；靠近障碍物处可以更密集；空旷区域可以更稀疏。
  while (global_path.size() == 2) {
    // Find the midpoint of the single segment
    Eigen::Vector3d mid = (global_path[0] + global_path[1]) / 2.0;
    global_path.insert(global_path.begin() + 1, mid);
  }

  // If the global path's size is still < 3, we cannot proceed
  if (global_path.empty() || global_path.size() < 3) {
    replanning_failure_count_++;
    return false;
  }

  // Initialize flag
  bool optimization_succeeded = false;

  // Set local_E
  // 这里 local_E 是 Gurobi 优化的末端状态
  // 改进：local_E 目前主要只设置 pos，正常飞行下可以更精细地设置 E 的速度方向,否则局部轨迹末端速度/姿态可能不够自然，尤其高速飞行时会影响轨迹平滑性
  if (drone_status_ == DroneStatus::GOAL_REACHED || drone_status_ == DroneStatus::GOAL_SEEN ||
      drone_status_ == DroneStatus::HOVER_AVOIDING)
    local_E = local_G;
  else
    local_E.pos = global_path.back();    // 沿着 HGP 路径推进

  // if using ground robot, we fix the z
  if (par_.vehicle_type != "uav") {
    local_A.pos[2] = 1.0;
    local_E.pos[2] = 1.0;
  }

  /*
   * Parallelized Local Trajectory Optimization
   */

  // Reset whole trajectory planners to nominal state
  for (auto& solver : whole_traj_solver_ptrs_) solver->resetToNominalState();

  // Get the base map vector
  vec_Vec3f base_map;
  if (par_.sim_env == "gazebo" || par_.sim_env == "hardware") {
    hgp_manager_.getVecUnknownOccupied(base_map);    // base_map = unknown + occupied
  } else if (par_.sim_env == "fake_sim") {
    hgp_manager_.getVecOccupied(base_map);    // base_map = occupied
  }

  // Filter out floor voxels at z_min from base_map before corridor decomposition.
  // The floor is already enforced by the solver's z_min constraint and the
  // ellipsoid decomp's set_z_min_and_max(). Including floor voxels in the obstacle
  // set causes the ellipsoid to shrink in all axes (ellipsoidal coupling), producing
  // unnecessarily skinny corridors.
  // 过滤地面点
  // 原因是地面已经由 solver 的 z 约束处理，如果把地面点也送进椭球分解，会把安全走廊压得很窄。
  // 改进：
  // 地面过滤阈值现在比较简单：z_min + res。
  // 可以根据机器人尺寸、地形高度、局部平面估计做更稳健的地面剔除。
  // 尤其如果环境不是平地，固定 z 阈值可能不够。
  {
    const float z_floor_thresh = static_cast<float>(par_.z_min) + static_cast<float>(par_.res);
    const size_t before = base_map.size();
    base_map.erase(
        std::remove_if(
            base_map.begin(), base_map.end(),
            [z_floor_thresh](const Vec3f& pt) { return pt.z() <= z_floor_thresh; }),
        base_map.end());
    if (par_.debug_verbose && base_map.size() != before)
      std::cout << "[replan] Filtered " << (before - base_map.size())
                << " floor voxels (z<=" << z_floor_thresh << ") from base_map" << std::endl;
  }

  // Get obst_pos and obst_bbox
  // 这里复制一份动态障碍物位置和尺寸，保证本次优化过程中数据一致
  // 改进：
  // 目前这里拿的是 obst_pos_ 和 obst_bbox_，时间层里的动态预测主要依赖 cvxEllipsoidDecompTimeLayered 里的膨胀逻辑。
  // 如果要更精确，可以直接把每个时间层的预测位置传入走廊生成，而不是只用当前位置 + 膨胀。
  vec_Vecf<3> obst_pos;
  vec_Vecf<3> obst_bbox;
  {
    std::lock_guard<std::mutex> lk(mtx_obst_pos_);
    obst_pos = obst_pos_;
    obst_bbox = obst_bbox_;
  }
  // Compute an initial dt for the local trajectory optimization
  // initial_dt 是根据距离和动力学约束估计出来的基础时间
  // initial_dt 目前主要由 A-E 端点估计，可以结合 global_path 长度，而不是只看直线距离 A 到 E。
  // 因为绕障路径可能很长，但 A-E 直线距离短，这会低估所需时间，导致小 factor 下 Gurobi 更容易失败。
  whole_traj_solver_ptrs_[0]->setX0(local_A);
  whole_traj_solver_ptrs_[0]->setXf(local_E);
  double initial_dt = whole_traj_solver_ptrs_[0]->getInitialDt();

  // Compute sub goal vector once
  std::vector<double> sub_goal;
  sub_goal.push_back(local_G.pos[0]);
  sub_goal.push_back(local_G.pos[1]);
  sub_goal.push_back(local_G.pos[2]);

  // Pre-compute convex decomposition if environment is static or dynamic_worst_case
  // 预计算静态安全走廊
  // 如果环境静态，安全走廊和时间 factor 关系不大，可以提前算一次，所有线程复用。
  // 如果是 dynamic_worst_case，它把动态障碍物按最大时间范围膨胀，也可以提前算一套保守空间走廊。
  // 改进：
  // dynamic_worst_case 很保守，可能导致走廊过窄。
  // 可以考虑风险分级膨胀、概率膨胀，或者只对靠近路径的障碍做 worst-case。
  std::vector<LinearConstraint3D> shared_spatial_constraints;    // 凸分解产出的线性约束（每个多面体的 A*x <= b），后面喂给 Gurobi
  vec_E<Polyhedron<3>> shared_spatial_poly_out;    // 凸分解产出的多面体几何，用于可视化
  // 判断当前环境假设是否是"静态"或"动态最坏情况",这两种情况下，所有 factor 线程可以共用同一套空间走廊，因为走廊不依赖 factor（时间）
  // 动态环境则不行——每个 factor 对应不同的 dt，障碍膨胀量不同，走廊也不同，必须每个线程自己算
  bool use_precomputed_constraints =    
      (par_.environment_assumption == "static" ||
       par_.environment_assumption == "dynamic_worst_case");
  
  if (use_precomputed_constraints) {
    const size_t P = (global_path.size() >= 2) ? (global_path.size() - 1) : 0;    //P = 全局路径的段数（点数 - 1）,每段对应一个凸多面体
    
    // 动态最坏情况：所有段末端时间设为最大,凸分解用它决定动态障碍膨胀多少
    std::vector<double> seg_end_times;
    if (par_.environment_assumption == "dynamic_worst_case") {
      // Worst-case inflation: set ALL segment end times to the maximum possible time horizon
      // across all factor threads. This inflates every obstacle by obst_max_vel * max_time,
      // producing the most conservative corridors (ablation baseline).
      const double max_time_horizon =
          static_cast<double>(par_.num_N) * initial_dt * factors_.back();
      seg_end_times.assign(P, max_time_horizon);
    } else {
      // Static environment: compute seg_end_times based on worst-case trajectory time per spatial
      // segment
      seg_end_times = computeWorstSegEndTimesPoly(initial_dt, factors_[0], P);
    }

    // Run spatial convex decomposition once before threading
    // ellip_workers_[0]：用第 0 个凸分解 worker（因为只算一次，不需要多 worker）。
    // global_path：截断后的全局路径。
    // base_map：静态障碍体素（已过滤地面）。
    // obst_pos / obst_bbox：动态障碍位置和包围盒。
    // seg_end_times：每段末端时间。
    // 输出：
    // shared_spatial_constraints：线性约束集
    // shared_spatial_poly_out：多面体几何
    if (!hgp_manager_.cvxEllipsoidDecomp(
            ellip_workers_[0], global_path, base_map, obst_pos, obst_bbox, seg_end_times,
            shared_spatial_constraints, shared_spatial_poly_out)) {
      std::cout << bold << red
                << "Precomputed spatial convex decomposition failed for static environment" << reset
                << std::endl;
      return false;
    }

    // Save the whole polytopes for visualization (available even if local traj optimization fails)
    // poly_out_whole_：成员变量，保存整条路径的所有多面体。
    // 即使后面 Gurobi 优化失败，这个可视化数据也是可用的。
    // 通过 retrievePolytopes 取出。
    poly_out_whole_ = shared_spatial_poly_out;
  }

  // Shared flag: when one thread succeeds, all others abort early
  // std::atomic<bool>：原子布尔，多线程读写安全,普通 bool 如果 4 个线程同时写，可能：线程 A 写 true，线程 B 也在写 true，结果读到一半是乱的值或者编译器/CPU 优化导致一个线程看不到另一个线程的修改
  // (atomic 保证读和写是"一步完成"的，不会读到一个"半真半假"的值，多个人同时读写这个 bool，不会出乱子)
  // std::make_shared：make_shared 是创建一个"共享指针"，多个地方同时持有它，它就不会被销毁；所有持有者都释放了，才销毁。
  // 为什么需要 shared_ptr？ 因为 lambda（线程函数）捕获这个变量时，是拷贝捕获的，如果不用，4 个 lambda 各自拷贝一份，各改各的，互不影响 → 完全失效
  // 用 shared_ptr 后，4 个 lambda 拷贝的是同一个指针，指向同一个 atomic<bool>，所以一个改了，其他人能看到
  // 作用：任何一个线程成功，把它设为 true，其它线程看到就提前退出，省 CPU
  auto any_thread_succeeded = std::make_shared<std::atomic<bool>>(false); 

  // future 的返回类型是 tuple<bool, double, double, double, vec_E<Polyhedron<3>>>：
  // bool：是否成功
  // double：Gurobi 计算时间
  // double：凸分解时间
  // double：该线程的 factor
  // vec_E<Polyhedron<3>>：该线程的安全走廊多面体（可视化用）
  // reserve 预分配，避免 push_back 时反复扩容。
  std::vector<std::future<std::tuple<bool, double, double, double, vec_E<Polyhedron<3>>>>> futures;
  futures.reserve(factors_.size());

  // Time the parallel optimization section
  // std::chrono = C++ 的时间库，steady_clock = "稳定时钟"
  // system_clock，它跟着系统时间走。如果系统时间被改了（比如 NTP 同步、用户手动调），它可能倒退。
  // steady_clock 是单调递增的，只会往前走，永远不会倒退，适合测时间间隔。
  // 类比：system_clock 像墙上的挂钟（可以被调），steady_clock 像秒表（只会往前走）。
  auto parallel_opt_start = std::chrono::steady_clock::now();
  
  // 启动并行任务
  // 遍历每个 factor，factor 是时间缩放因子，factor 越大，轨迹时间越长，越容易可行，但飞得越慢
  for (size_t i = 0; i < factors_.size(); ++i) {
    const double factor = factors_[i];  // corresponding factor for solver i
    
    // std::async(std::launch::async, lambda):std::launch::async 告诉 C++ 这个 lambda 要在新线程上跑。std::async 立即返回一个 future，主线程不等 lambda 跑完，继续下一轮 for 循环。每个 factor 一个线程，真正并行。
    futures.push_back(std::async(
        std::launch::async,
        [this, i, factor, &global_path, local_A, local_E, sub_goal, A_time, initial_dt, &obst_pos,    // [……] 捕获列表，this表示访问成员变量（whole_traj_solver_ptrs_、ellip_workers_），i为线程索引，每个线程不同
         &obst_bbox, &base_map, use_precomputed_constraints, &shared_spatial_constraints,
         &shared_spatial_poly_out,
         any_thread_succeeded]() -> std::tuple<bool, double, double, double, vec_E<Polyhedron<3>>> {    // () -> std::tuple<bool, double, double, double, vec_E<Polyhedron<3>>>显式指定返回类型，和 future 的模板参数一致
          try {
            // Early exit: another thread already found a solution,提前退出检查
            // std::memory_order_relaxed：只保证原子性，不保证顺序。这里只需要"看到最新值"，relaxed 足够，性能最好
            // any_thread_succeeded->load(...)：原子读
            if (any_thread_succeeded->load(std::memory_order_relaxed))    
              return {false, 0.0, 0.0, factor, vec_E<Polyhedron<3>>{}};   

            double thread_gurobi_time = 0.0;
            double thread_convx_decomp_time = 0.0;
            vec_E<Polyhedron<3>> thread_poly_out_safe;

            // Per-worker decomp util (no sharing across worker index)
            // ellip_workers_ 里的每个元素是一个 EllipsoidDecomp3D 对象(凸分解工具对象)，每个 factor 一个
            EllipsoidDecomp3D& ellip = this->ellip_workers_[i];

            const bool result = generateLocalTrajectory(
                ellip, global_path, local_A, local_E, sub_goal, A_time, 
                thread_gurobi_time,    // 输出：Gurobi 耗时；
                thread_convx_decomp_time,     // 输出：凸分解耗时；
                whole_traj_solver_ptrs_[i],     // 本线程的 Gurobi solver
                factor, initial_dt, obst_pos,
                obst_bbox,
                base_map,  // base_uo snapshot，静态障碍体素
                thread_poly_out_safe,    // 输出：本线程的安全走廊多面体（可视化用）
                use_precomputed_constraints ? &shared_spatial_constraints : nullptr,    // 静态/最坏情况：传预计算约束的指针，generateLocalTrajectory 内部用 use_spatial_only 分支，不重复做凸分解
                use_precomputed_constraints ? &shared_spatial_poly_out : nullptr);    // 动态环境：传 nullptr，generateLocalTrajectory 内部走 cvxEllipsoidDecompTimeLayered 分支，每个线程自己算时间分层走廊

            // Signal other threads to stop
            // 如果本线程成功，设置全局标志,其它线程下次检查时会提前退出
            if (result) any_thread_succeeded->store(true, std::memory_order_relaxed);

            return {
                result, thread_gurobi_time, thread_convx_decomp_time, factor, thread_poly_out_safe};
          } catch (const std::exception& ex) {    // 任何异常都捕获，返回 false，不让线程崩掉整个程序
            std::cerr << "Exception in async task with factor " << factor << ": " << ex.what()    // 打印 factor 和异常信息，方便调试
                      << std::endl;
            return {false, 0.0, 0.0, factor, vec_E<Polyhedron<3>>{}};
          }
        }));
  }
  
  // 结果容器
  // Poll futures for first success instead of blocking sequentially.
  // This ensures we react immediately when ANY thread finishes successfully,
  // rather than waiting for earlier (by index) threads to complete first.
  std::vector<bool> vec_optimization_succeeded;
  std::vector<std::vector<RobotState>> vec_goal_setpoints;
  std::vector<PieceWisePol> vec_pwp_to_share;
  std::vector<std::vector<Eigen::Matrix<double, 3, 4>>> vec_cps;
  std::vector<double> vec_gurobi_times;
  std::vector<double> vec_convx_decomp_times;
  std::vector<vec_E<Polyhedron<3>>> vec_poly_out_safe;

  const size_t num_factors = factors_.size();
  vec_optimization_succeeded.resize(num_factors, false);
  vec_goal_setpoints.resize(num_factors);
  vec_pwp_to_share.resize(num_factors);
  vec_cps.resize(num_factors);
  vec_gurobi_times.resize(num_factors, 0.0);
  vec_convx_decomp_times.resize(num_factors, 0.0);
  vec_poly_out_safe.resize(num_factors);

  std::vector<bool> collected(num_factors, false);    // collected[i]:第 i 个 future 是否已取
  size_t num_collected = 0;    // 已收集的 future 数量
  int winner_index = -1;    // 第一个成功的索引，-1 表示还没找到
  
  // Poll until we find a winner or all futures are collected
  // 整个 planLocalTrajectory 一开始是在主线程（重规划线程）里跑的
  // for (size_t i = 0; i < factors_.size(); ++i) { ... } 这个for 循环本身在主线程里跑，但它内部为每个 factor 开了一个新线程
  // while (num_collected < num_factors) { ... } 主线程进入 while 轮询，看谁先完成
  // 主线程:  [启动线程0] [启动线程1] [启动线程2] [启动线程3] [轮询 futures...]
  // 线程0:               [====跑 Gurobi====]                    
  // 线程1:               [====跑 Gurobi====]                    
  // 线程2:               [====跑 Gurobi====]                    
  // 线程3:               [====跑 Gurobi====]                    
  // 伪代码：
  // while 还有任务:
  //   如果某个任务成功:
  //     保存它的解
  //     请求其他任务停止
  //     等其他任务安全退出
  //     break
  //   如果某些任务失败:
  //     收集失败结果，继续看其他任务
  // 如果还不清楚，可以看word文档实例说明。“while主线程实例说明”
  while (num_collected < num_factors) {
    // 内层 for 是在轮询每个 future，每个 factor 对应一个 future，最后都要收尾(stopExecution(),让其他线程停止)，结果可能是失败、成功或异常
    // 会有一点收尾成本，但不是等它们完整求解，而是等它们响应停止请求。这个成本换来线程安全。
    // 为什么不能直接强制杀线程？在 C++ 里，如果一个线程正在执行：Gurobi 求解、内存分配、写 solver 内部状态、更新模型变量、你突然强制杀掉它，
    // 可能会造成：Gurobi 环境损坏、mutex 没释放、内存泄漏、对象状态半写入、程序崩溃。
    // 所以常见做法是：主线程发出停止请求，工作线程自己检查停止信号，然后主动退出，也就是这里的：whole_traj_solver_ptrs_[j]->stopExecution();和前面线程里的any_thread_succeeded
    for (size_t i = 0; i < num_factors; ++i) {
      // 已处理过就跳过
      if (collected[i]) continue;    

      // Non-blocking check: is this future ready?
      // 为什么不能直接 futures[i].get()?
      // futures[0] 很慢，要 1s
      // futures[1] 很快，10ms 成功
      // 如果代码先执行：
      // futures[0].get()
      // 它会卡住等 1 秒。即使 futures[1] 早就成功了，主线程也不知道。
      // 用wait_for(0)来非阻塞检查：这个线程算完了吗？
      if (futures[i].wait_for(std::chrono::microseconds(0)) != std::future_status::ready) continue;       // 没算完就跳过，看下一个
      
      // 算完就取结果
      auto
          [result, thread_gurobi_time, thread_convx_decomp_time, thread_factor,
           thread_poly_out_safe] = futures[i].get();
      // futures[i].get()这个 future 里的结果就被“取走”了，不能再取第二次。
      // 所以要记录：
      collected[i] = true;
      num_collected++;

      // Save polytopes for visualization even if the optimizer failed
      if (poly_out_safe_.empty() && !thread_poly_out_safe.empty())
        poly_out_safe_ = thread_poly_out_safe;

      if (!result) continue;    // 如果 result == false，就继续看其他任务

      // First success — immediately stop all other solvers
      // 当第 i 个成功了，代码立刻通知其他 Gurobi solver：不用继续算了
      for (size_t j = 0; j < num_factors; ++j) {
        if (j == i) continue;
        try {
          whole_traj_solver_ptrs_[j]->stopExecution();     // stopExecution() 通常会设置一个终止标志，让 Gurobi callback 尽快中断求解
        } catch (const std::exception& e) {
          std::cout << "it's likely that the solver has gurobi error and already released the "
                       "gurobi environment"
                    << std::endl;
          std::cerr << e.what() << '\n';
        }
      }

      // Get results from the successful solver成功后从 solver 里取真正轨迹
      // future 返回的 tuple 里没有直接带轨迹，轨迹结果存放在whole_traj_solver_ptrs_[i]
      // 所以成功后要从对应 solver 里取:
      whole_traj_solver_ptrs_[i]->fillGoalSetPoints();    // // goal_setpoints：后面控制器要跟踪的离散目标点
      whole_traj_solver_ptrs_[i]->getGoalSetpoints(vec_goal_setpoints[i]);    
      whole_traj_solver_ptrs_[i]->getPieceWisePol(vec_pwp_to_share[i]);    // pwp_to_share：分段多项式轨迹
      whole_traj_solver_ptrs_[i]->getControlPoints(vec_cps[i]);  // Bezier control points,cps：控制点，可视化用
      vec_gurobi_times[i] = thread_gurobi_time;
      vec_convx_decomp_times[i] = thread_convx_decomp_time;
      vec_poly_out_safe[i] = thread_poly_out_safe;
      vec_optimization_succeeded[i] = true;    // 第 i 个 factor 成功了
      winner_index = static_cast<int>(i);
    }

    // If we found a winner, still drain remaining futures (they should exit fast
    // due to stopExecution + any_thread_succeeded flag)
    // 虽然已经找到成功轨迹了，但其他后台线程可能还没完全退出,C++ 里不能直接不管这些 futures。你需要把它们收尾，也就是调用 get() 等它们结束
    if (winner_index >= 0 && num_collected < num_factors) {
      for (size_t i = 0; i < num_factors; ++i) {
        if (!collected[i]) {
          futures[i].get();  // These should return quickly since solvers were stopped，因为前面已经做了stopExecution()
          collected[i] = true;
          num_collected++;
        }
      }
      break;
    }

    // Brief yield to avoid busy-spin，我这个主线程暂时让出 CPU，让其他线程有机会继续运行。
    // 如果不 yield()，主线程可能一直占着 CPU 空转，yield() 可以减少忙等带来的 CPU 浪费
    std::this_thread::yield();
  }

  // Measure wall-clock time for the parallel optimization only
  auto parallel_opt_end = std::chrono::steady_clock::now();
  double parallel_opt_ms =
      std::chrono::duration<double, std::milli>(parallel_opt_end - parallel_opt_start).count();

  // Find the first successful optimization
  // 可能还有别的线程也成功了,前面的while是轮询阶段，接下来是写回阶段
  // 轮询阶段在找到第一个成功后就通知其他线程停止了，然后用futures[i].get();收尾其他线程（收尾可能是成功的失败的），但是轮询阶段也仅仅是收尾，并没有查询其他future的结果
  // 所以这里要再遍历一次 vec_optimization_succeeded，找到第一个成功的索引，写回结果
  int successful_index = -1;
  for (size_t i = 0; i < vec_optimization_succeeded.size(); ++i) {
    if (vec_optimization_succeeded[i]) {
      optimization_succeeded = true;
      goal_setpoints_ = vec_goal_setpoints[i];
      pwp_to_share_ = vec_pwp_to_share[i];
      cps_ = vec_cps[i];
      local_traj_computation_time_ = parallel_opt_ms;
      cvx_decomp_time_ = vec_convx_decomp_times[i];
      successful_factor_ = factors_[i];
      poly_out_safe_ = vec_poly_out_safe[i];
      successful_index = i;
      break;  // Exit the loop after the first success
    }
  }

  if (optimization_succeeded) {
    // update list_subopt_goal_setpoints_ 次优轨迹列表（其它 factor 也可能成功了）(vec_goal_setpoints without the successful one)
    // 用途：可能用于分析、fallback、或者多轨迹选择
    list_subopt_goal_setpoints_.clear();
    list_subopt_goal_setpoints_.reserve(vec_goal_setpoints.size() - 1);
    for (size_t i = 0; i < vec_goal_setpoints.size(); ++i) {
      if (i != successful_index && !vec_goal_setpoints[i].empty()) {
        list_subopt_goal_setpoints_.push_back(std::move(vec_goal_setpoints[i]));
      }
    }

    // update the factors_ vector
    // 成功时更新 factors_（动态 factor 模式）
    if (par_.use_dynamic_factor) {
      // Save the successful factor BEFORE clearing
      double successful_factor = factors_[successful_index];

      // clear factors_ first
      factors_.clear();
      factors_.reserve(num_dynamic_factors_);

      // Set the successful factor to be the mean of k-radius factors
      // 下次重规划时，factor 窗口平移到成功 factor 附近，形成自适应搜索。
      for (int i = 0; i < num_dynamic_factors_; i++) {
        double factor =
            successful_factor - par_.dynamic_factor_k_radius + i * par_.factor_constant_step_size;
        if (factor >= par_.factor_initial && factor <= par_.factor_final)
          factors_.push_back(factor);
      }

      if (!dynamic_factor_inital_sucess_) dynamic_factor_inital_sucess_ = true;
    }
  // 失败时更新 factors_
  } else {
    // if the optimization failed, we increase the factors_ for next replanning
    if (par_.use_dynamic_factor) {
      // compute current mean of the factor window
      double current_mean = 0.0;
      for (size_t i = 0; i < factors_.size(); i++) current_mean += factors_[i];
      current_mean /= static_cast<double>(factors_.size());

      if (current_mean + par_.factor_constant_step_size > par_.factor_final) {
        // reset factors back to the initial window
        factors_.clear();
        factors_.reserve(num_dynamic_factors_);
        for (int i = 0; i < num_dynamic_factors_; i++) {
          double factor = par_.dynamic_factor_initial_mean - par_.dynamic_factor_k_radius +
                          i * par_.factor_constant_step_size;
          if (factor >= par_.factor_initial && factor <= par_.factor_final)
            factors_.push_back(factor);
        }
      } else {
        // shift all the factors in factors_ by factor_constant_step_size
        for (size_t i = 0; i < factors_.size(); i++) {
          factors_[i] = factors_[i] + par_.factor_constant_step_size;
        }
        // remove any factors that exceed factor_final
        factors_.erase(
            std::remove_if(
                factors_.begin(), factors_.end(),
                [this](double f) { return f > par_.factor_final; }),
            factors_.end());
      }
    }
  }

  return optimization_succeeded;
}

// ----------------------------------------------------------------------------

void SANDO::getPieceWisePol(PieceWisePol& pwp) { pwp = pwp_to_share_; }

// ----------------------------------------------------------------------------
// 给定空间段数 P、时间层总数 num_seg、每层时长 dt = initial_dt * factor，算出一组"每个空间段末端时间" seg_end_times，用于后续动态障碍物膨胀
// 局部轨迹有 N(由 par_.num_N 决定) 个时间段，由 Gurobi 一次性整体求解，不是每段单独求解。
// 安全走廊有 P 个空间 polytope，来自 global_path 的 P 个空间段。
// Gurobi 会把整条 N 段轨迹约束在这些 polytope / time-layered polytope 里面。整个局部轨迹作为一个优化问题，一次性求解。
// 改进：当前对于dt数量的分配更多是基于工程经验，未来可以考虑更智能的分配策略，比如根据障碍物密度、曲率、速度限制等因素动态调整每段时间长度。
std::vector<double> SANDO::computeWorstSegEndTimesPoly(
    double initial_dt, double factor, size_t num_seg) {    // 初始时间步（每段的基准时长）
  std::vector<double> seg_end_times;
  seg_end_times.reserve(num_seg);

  if (num_seg == 0) return seg_end_times;
  
  // P = par_.num_P：参数里配置的空间段数上限
  const int P = std::max(0, par_.num_P);
  if (P <= 0) {
    // Fallback: still produce valid per-segment times
    const double dt = initial_dt * factor;
    double t_acc = 0.0;
    for (size_t i = 0; i < num_seg; ++i) {
      t_acc += dt;
      seg_end_times.push_back(t_acc);
    }
    return seg_end_times;
  }

  // Assign segments to polytopes under your rule, but using num_seg (NOT par_.num_N-1).
  // Number of last polytopes that can be guaranteed 1 segment:
  const int max_last_ones = P - 1;
  const int min_one = std::min<int>(max_last_ones, static_cast<int>(num_seg));

  std::vector<int> segments_per_poly(P, 0);

  // Last min_one polytopes get 1 segment each
  // 后面 min_one 个空间段，每个空间段至少分配 1 个时间段
  for (int k = 0; k < min_one; ++k) {
    const int p = (P - 1) - k;
    segments_per_poly[p] = 1;
  }

  // First polytope gets the remainder
  // 第一段空间段分配剩余的时间段
  const int assigned_to_last = min_one;
  const int first_segments = static_cast<int>(num_seg) - assigned_to_last;
  if (first_segments > 0) segments_per_poly[0] += first_segments;

  // Convert to per-segment cumulative end times
  // 每个空间段末端时间 = 前面所有时间段的总和，比如第一段空间段有 3 个时间段，第二段空间段有 2 个时间段，第三段空间段有 1 个时间段，
  // dt = 0.5s，那么 seg_end_times = [1.5, 2.5, 3.0]，表示第一段空间段末端时间是 1.5s，第二段空间段末端时间是 1.5+1s，第三段空间段末端时间是 3.0s。
  const double dt = initial_dt * factor;
  double t_acc = 0.0;

  size_t produced = 0;
  for (int p = 0; p < P && produced < num_seg; ++p) {
    const int k = segments_per_poly[p];
    for (int s = 0; s < k && produced < num_seg; ++s) {
      t_acc += dt;
      seg_end_times.push_back(t_acc);
      ++produced;
    }
  }

  // Safety fallback: if anything went odd, pad to length num_seg
  while (seg_end_times.size() < num_seg) {
    t_acc += dt;
    seg_end_times.push_back(t_acc);
  }

  return seg_end_times;
}

// ----------------------------------------------------------------------------
// 给定一条全局路径、起点 A、终点 E，先生成安全走廊（凸分解），再把走廊和动力学约束喂给 Gurobi，解出一条局部轨迹
// whole_traj_solver_ptr是输入输出，可能被重置
bool SANDO::generateLocalTrajectory(
    EllipsoidDecomp3D& ellip,
    const vec_Vecf<3>& global_path,
    const RobotState& local_A,
    const RobotState& local_E,
    const std::vector<double>& sub_goal,
    double A_time,
    double& gurobi_computation_time,
    double& cvx_decomp_time,
    std::shared_ptr<SolverGurobi>& whole_traj_solver_ptr,
    double factor,
    double initial_dt,
    const vec_Vecf<3>& obst_pos,
    const vec_Vecf<3>& obst_bbox,
    const vec_Vec3f& base_uo,
    vec_E<Polyhedron<3>>& poly_out_safe,
    const std::vector<LinearConstraint3D>* precomputed_spatial_constraints,
    const vec_E<Polyhedron<3>>* precomputed_spatial_poly_out) {
  // P: spatial corridor pieces (global segments)
  const size_t P = (global_path.size() >= 2) ? (global_path.size() - 1) : 0;
  if (P == 0) return false;

  // N: local trajectory segments (time layers)
  const size_t N = static_cast<size_t>(par_.num_N);
  if (N == 0) return false;

  // Local time layers: end time of local segment n
  // NOTE: this matches your solver's uniform dt assumption (dt = initial_dt * factor).
  const double dt_layer = initial_dt * factor;
  std::vector<double> time_end_times;
  time_end_times.reserve(N);
  // 代码已经实现了动态障碍物时间分层安全走廊，是否使用时间分层完全取决于 environment_assumption 参数。
  for (size_t n = 0; n < N; ++n)
    time_end_times.push_back((static_cast<double>(n) + 1.0) * dt_layer);

  // Timer for computing the safe corridor
  // MyTimer 是 timer::Timer 的 typedef（文件开头），构造参数 true 表示立即启动，后面用 getElapsedMicros() 取耗时，只对动态环境的凸分解计时，静态环境用预计算，耗时为 0。
  MyTimer cvx_decomp_timer(true);

  // Check if we have precomputed spatial constraints (static environment)
  bool use_spatial_only =
      (precomputed_spatial_constraints != nullptr && precomputed_spatial_poly_out != nullptr);

  // Declare constraints outside if block so they're available for solver setup
  std::vector<std::vector<LinearConstraint3D>> l_constraints_by_time;    // l_constraints_by_time：[N][P] 的二维数组，每个时间层每个空间段一个线性约束集
  std::vector<vec_E<Polyhedron<3>>> poly_out_by_time;  // oly_out_by_time：[N][P] 的二维数组，每个时间层每个空间段一个多面体几何（可视化用）

  if (use_spatial_only) {
    // Static environment: use precomputed spatial-only constraints
    // Copy the spatial polytopes for visualization
    poly_out_safe = *precomputed_spatial_poly_out;
    cvx_decomp_time = 0.0;  // No decomposition time since we're using precomputed
  } else {
    // Dynamic environment: compute time-layered constraints for this thread
    if (!hgp_manager_.cvxEllipsoidDecompTimeLayered(
            ellip, global_path, base_uo, obst_pos, obst_bbox, time_end_times, l_constraints_by_time,
            poly_out_by_time)) {
      poly_out_safe.clear();
      return false;
    }

    cvx_decomp_time = cvx_decomp_timer.getElapsedMicros() / 1000.0;

    // Build poly_out_safe from all time layers for visualization
    poly_out_safe.clear();
    poly_out_safe.reserve(N * P);
    for (size_t n = 0; n < N; ++n) {
      for (size_t p = 0; p < P; ++p) {
        poly_out_safe.emplace_back(poly_out_by_time[n][p]);
      }
    }
  }

  // Initialize the solver.
  whole_traj_solver_ptr->setX0(local_A);  // Initial condition
  whole_traj_solver_ptr->setXf(local_E);  // Final condition

  // Set polytopes based on environment type
  if (use_spatial_only) {
    // Static environment: use spatial-only polytopes
    whole_traj_solver_ptr->setPolytopes(*precomputed_spatial_constraints);
  } else {
    // Dynamic environment: use time-layered polytopes
    whole_traj_solver_ptr->setPolytopesTimeLayered(l_constraints_by_time);
  }
  whole_traj_solver_ptr->setT0(A_time);             // Initial time (kept as-is)
  whole_traj_solver_ptr->setInitialDt(initial_dt);  // Initial dt

  // Solve the optimization problem.
  bool gurobi_error_detected = false;    // 输出，是否发生 Gurobi 错误（比如环境崩了）
  bool gurobi_result = whole_traj_solver_ptr->generateNewTrajectory(
      gurobi_error_detected, gurobi_computation_time, factor);

  // If a Gurobi error occurred, reset the solver and return.
  if (gurobi_error_detected) {
    whole_traj_solver_ptr = std::make_shared<SolverGurobi>();
    whole_traj_solver_ptr->initializeSolver(par_);
    return false;
  }

  // If no solution is found, return.
  if (!gurobi_result) {
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

bool SANDO::appendToPlan() {
  if (par_.debug_verbose)
    std::cout << "goal_setpoints_.size(): " << goal_setpoints_.size() << std::endl;

  // mutex lock
  mtx_plan_.lock();

  // get the size of the plan and plan_safe_paths
  int plan_size = plan_.size();

  // If the plan size is less than k_value_, which means we already passed point A, we cannot use
  // this plan
  if (plan_size < k_value_) {
    if (par_.debug_verbose)
      std::cout << bold << red << "(plan_size - k_value_) = " << (plan_size - k_value_) << " < 0"
                << reset << std::endl;
    k_value_ = std::max(1, plan_size - 1);  // Decrease k_value_ to plan_size - 1 but at least 1
  } else  // If the plan size is greater than k_value_, which means we haven't passed point A yet,
          // we can use this plan
  {
    plan_.erase(plan_.end() - k_value_, plan_.end());
    plan_.insert(plan_.end(), goal_setpoints_.begin(), goal_setpoints_.end());
  }

  // mutex unlock
  mtx_plan_.unlock();

  // k_value adaptation initialization
  if (!got_enough_replanning_) {
    if (store_computation_times_.size() < par_.num_replanning_before_adapt) {
      num_replanning_++;
    } else {
      startAdaptKValue();
      got_enough_replanning_ = true;
    }
  }

  return true;
}

// ----------------------------------------------------------------------------

void SANDO::getGterm(RobotState& G_term) {
  mtx_G_term_.lock();
  G_term = G_term_;
  mtx_G_term_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::setGterm(const RobotState& G_term) {
  mtx_G_term_.lock();
  G_term_ = G_term;
  mtx_G_term_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getG(RobotState& G) {
  mtx_G_.lock();
  G = G_;
  mtx_G_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getE(RobotState& E) {
  mtx_E_.lock();
  E = E_;
  mtx_E_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::setG(const RobotState& G) {
  mtx_G_.lock();
  G_ = G;
  mtx_G_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getA(RobotState& A) {
  mtx_A_.lock();
  A = A_;
  mtx_A_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::setA(const RobotState& A) {
  mtx_A_.lock();
  A_ = A;
  mtx_A_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getA_time(double& A_time) {
  mtx_A_time_.lock();
  A_time = A_time_;
  mtx_A_time_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::setA_time(double A_time) {
  mtx_A_time_.lock();
  A_time_ = A_time;
  mtx_A_time_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getState(RobotState& state) {
  mtx_state_.lock();
  state = state_;
  mtx_state_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getLastPlanState(RobotState& state) {
  mtx_plan_.lock();
  if (plan_.empty()) {
    mtx_plan_.unlock();
    getState(state);  // fallback to current state when no plan exists
    return;
  }
  state = plan_.back();
  mtx_plan_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getTrajs(std::vector<std::shared_ptr<DynTraj>>& out) {
  std::lock_guard<std::mutex> lock(mtx_trajs_);
  out = trajs_;  // copies shared_ptr only, not expressions
}

// ----------------------------------------------------------------------------

void SANDO::cleanUpOldTrajs(double current_time) {
  std::lock_guard<std::mutex> lock(mtx_trajs_);

  // remove_if moves all “expired” to the end, then erase() chops them off
  trajs_.erase(
      std::remove_if(
          trajs_.begin(), trajs_.end(),
          [&](const std::shared_ptr<DynTraj>& t) {
            return (current_time - t->time_received) > par_.traj_lifetime;
          }),
      trajs_.end());
}

// ----------------------------------------------------------------------------

void SANDO::addTraj(std::shared_ptr<DynTraj> new_traj, double current_time) {
  // Always update existing trajectories (to keep time_received fresh and data current).
  // Only apply map/horizon filtering when adding a brand-new trajectory.
  {
    std::lock_guard<std::mutex> lock(mtx_trajs_);
    auto it = std::find_if(trajs_.begin(), trajs_.end(), [&](const std::shared_ptr<DynTraj>& t) {
      return t && t->id == new_traj->id;
    });

    if (it != trajs_.end()) {
      *it = new_traj;  // always update existing trajectory
      return;
    }
  }

  // New trajectory: only add if currently within map and horizon
  Eigen::Vector3d p = new_traj->eval(current_time);
  if (!checkPointWithinMap(p)) return;
  if ((p - state_.pos).norm() > par_.horizon) return;

  {
    std::lock_guard<std::mutex> lock(mtx_trajs_);
    trajs_.push_back(new_traj);
  }
}

// ----------------------------------------------------------------------------

void SANDO::updateState(RobotState data) {
  // If we are doing hardware and provide goal in global frame (e.g. vicon), we need to transform
  // the goal to the local frame

  if (par_.use_hardware && par_.provide_goal_in_global_frame &&
      !par_.state_already_in_global_frame) {
    // Apply transformation to position
    Eigen::Vector4d homo_pos(data.pos[0], data.pos[1], data.pos[2], 1.0);
    Eigen::Vector4d global_pos = init_pose_transform_ * homo_pos;
    data.pos = Eigen::Vector3d(global_pos[0], global_pos[1], global_pos[2]);

    // Apply rotation to velocity
    data.vel = init_pose_transform_rotation_ * data.vel;

    // Apply rotation to accel
    data.accel = init_pose_transform_rotation_ * data.accel;

    // Apply rotation to jerk
    data.jerk = init_pose_transform_rotation_ * data.jerk;

    // Apply yaw
    data.yaw += yaw_init_offset_;
  }

  mtx_state_.lock();
  state_ = data;
  mtx_state_.unlock();

  if (state_initialized_ == false || drone_status_ == DroneStatus::YAWING) {
    // create temporary state
    RobotState tmp;
    if (drone_status_ == DroneStatus::YAWING) {
      // During YAWING, command the FIXED start position so PX4 actively
      // pulls the drone back if it drifts. Using the drifting current
      // position would let the drone wander and change desired_yaw.
      tmp.pos = yaw_start_pos_;
    } else {
      tmp.pos = data.pos;
    }
    tmp.yaw = data.yaw;

    // Only seed previous_yaw_ on first-ever state.
    // During YAWING, previous_yaw_ is seeded once by setTerminalGoal() and
    // then driven forward by yaw() each goal tick.  Resetting it here every
    // state callback would collapse the step to a tiny w_max*dc delta that
    // PX4 ignores, so the drone never starts rotating.
    if (!state_initialized_) previous_yaw_ = data.yaw;

    // Push the state to the plan
    mtx_plan_.lock();
    plan_.clear();
    plan_.push_back(tmp);
    mtx_plan_.unlock();

    // Update Point A
    setA(tmp);

    // Update Point G
    setG(tmp);

    // Update the flag
    state_initialized_ = true;
  }
}

// ----------------------------------------------------------------------------
// 从 plan_ 队列里弹出下一个 setpoint，计算对应的 yaw，必要时做全局→局部坐标变换，返回给控制器。
// 它是每个控制周期调用一次的函数（比如 100 Hz），是 SANDO 和底层控制器（PX4/MAVROS）之间的接口
bool SANDO::getNextGoal(RobotState& next_goal) {
  // Check if the planner is initialized.
  // During YAWING we only need state + terminal goal (no map required — just rotating in place).因为YAWING 时，无人机是原地旋转，不需要地图、不需要规划
  if (drone_status_ == DroneStatus::YAWING) {
    if (!state_initialized_ || !terminal_goal_initialized_) return false;
  } else if (!checkReadyToReplan()) {    // 其他时候就需要严格检查
    return false;
  }

  // Pop the front of the plan，RobotState::setZero()：把 next_goal 的所有字段（pos、vel、accel、jerk、yaw、dyaw）设为 0
  next_goal.setZero();

  // If the plan is empty, return false
  // plan_ 是共享资源，appendToPlan（重规划线程）可能同时修改它。用 mtx_plan_ 锁保护
  mtx_plan_.lock();  // Lock the mutex
  auto local_plan = plan_;    // plan_ 是一个 std::deque<RobotState>，里面存的是离散的 setpoint 序列，按时间顺序排列。每次调用 getNextGoal 就弹出一个
  mtx_plan_.unlock();  // Unlock the mutex

  // Get the next goal,local_plan.front()：取队首元素（最早要执行的 setpoint）。
  next_goal = local_plan.front();

  // If there's more than one goal setpoint, pop the front
  if (local_plan.size() > 1) {    // 让 plan_ 永不为空，getNextGoal 永远能返回一个有效 setpoint（即使已经到达终点，也返回终点位置）
    mtx_plan_.lock();
    plan_.pop_front();    // std::deque<RobotState>（双端队列）。pop_front() 的行为：删除队首元素，队列长度减 1，后面的元素整体前移
    mtx_plan_.unlock();
  }

  // ---- Yaw computation (BEFORE frame transform — everything in global frame) ----
  if (!(drone_status_ == DroneStatus::GOAL_REACHED)) {
    // Get the desired yaw
    // If the planner keeps failing, just keep spinning
    if (replanning_failure_count_ > par_.yaw_spinning_threshold &&
        drone_status_ != DroneStatus::HOVER_AVOIDING) {
      next_goal.yaw = previous_yaw_ + par_.yaw_spinning_dyaw * par_.dc;
      next_goal.dyaw = par_.yaw_spinning_dyaw;
      previous_yaw_ = next_goal.yaw;
    } else {
      // If the local_plan is small just use the previous yaw with no dyaw
      // Exception: during YAWING we always need to call getDesiredYaw (plan is
      // intentionally kept at 1 entry by updateState)
      // plan_ 快空了，说明重规划没跟上，此时不根据速度算 yaw（因为速度可能不可靠），保持当前朝向
      if (local_plan.size() < 5 && drone_status_ != DroneStatus::YAWING &&
          drone_status_ != DroneStatus::HOVER_AVOIDING) {
        next_goal.yaw = previous_yaw_;
        next_goal.dyaw = 0.0;
      // 正常情况：调 getDesiredYaw
      } else {
        // next_goal.vel is still in GLOBAL frame here, matching previous_yaw_
        getDesiredYaw(next_goal);
      }
    }

    next_goal.dyaw = std::clamp(next_goal.dyaw, -par_.w_max, par_.w_max);
  // GOAL_REACHED：保持 yaw 不变
  } else {
    next_goal.yaw = previous_yaw_;
    next_goal.dyaw = 0.0;
  }

  // ---- Frame transform (global → local for MAVROS) ----
  // par_.use_hardware硬件模式；par_.provide_goal_in_global_frame目标在全局坐标系给出
  // init_pose_set_初始位姿已设置，三个条件都满足才做变换。仿真模式下 use_hardware = false，跳过。
  if (par_.use_hardware && par_.provide_goal_in_global_frame && init_pose_set_) {
    // Convert position from global to local frame
    Eigen::Vector4d homo_pos(next_goal.pos[0], next_goal.pos[1], next_goal.pos[2], 1.0);    // homo_pos：齐次坐标 (x, y, z, 1)
    Eigen::Vector4d local_pos = init_pose_transform_inv_ * homo_pos;    // init_pose_transform_inv_：全局→局部的 SE(3) 变换矩阵（4×4）

    // Apply inverse rotation to velocity, accel, jerk,只用旋转部分，不用平移
    Eigen::Vector3d local_vel = init_pose_transform_rotation_inv_ * next_goal.vel;
    Eigen::Vector3d local_accel = init_pose_transform_rotation_inv_ * next_goal.accel;
    Eigen::Vector3d local_jerk = init_pose_transform_rotation_inv_ * next_goal.jerk;

    next_goal.pos = Eigen::Vector3d(local_pos[0], local_pos[1], local_pos[2]);
    next_goal.vel = local_vel;
    next_goal.accel = local_accel;
    next_goal.jerk = local_jerk;

    // Convert yaw from global to local frame and wrap to [-pi, pi]
    next_goal.yaw -= yaw_init_offset_;
    sando_utils::angle_wrap(next_goal.yaw);    // sando_utils::angle_wrap：把角度包到 [-π, π]
  }

  return true;
}

// ----------------------------------------------------------------------------

void SANDO::getDesiredYaw(RobotState& next_goal) {
  // YAWING/HOVER_AVOIDING: closed-loop diff from actual drone yaw (state_.yaw).
  // TRAVELING/GOAL_SEEN: open-loop diff from commanded reference (previous_yaw_).

  double desired_yaw = 0.0;

  switch (drone_status_) {
    case DroneStatus::YAWING: {
      mtx_G_term_.lock();
      RobotState G_term = G_term_;
      mtx_G_term_.unlock();
      // Use the fixed yaw-start position (not the potentially drifted
      // next_goal.pos) so that desired_yaw stays stable during rotation.
      desired_yaw = atan2(G_term.pos[1] - yaw_start_pos_[1], G_term.pos[0] - yaw_start_pos_[0]);
      break;
    }
    case DroneStatus::HOVER_AVOIDING: {
      // Face toward the hover position (p_hover_), unless too close (atan2 unstable)
      double dx = p_hover_.x() - next_goal.pos[0];
      double dy = p_hover_.y() - next_goal.pos[1];
      double dist_xy = std::sqrt(dx * dx + dy * dy);
      if (dist_xy < 0.3) {
        // Too close — hold current yaw
        next_goal.yaw = previous_yaw_;
        next_goal.dyaw = 0.0;
        return;
      }
      desired_yaw = atan2(dy, dx);
      break;
    }
    // case DroneStatus::TRAVELING: 后面没有单独代码，是因为它和下面的 GOAL_SEEN 共享同一段处理逻辑。这里用的是 C/C++ 里的 switch fall-through 写法。
    case DroneStatus::TRAVELING:
    case DroneStatus::GOAL_SEEN: {
      double speed_xy =
          std::sqrt(next_goal.vel[0] * next_goal.vel[0] + next_goal.vel[1] * next_goal.vel[1]);
      if (speed_xy < 0.01) {
        next_goal.yaw = previous_yaw_;
        next_goal.dyaw = 0.0;
        return;
      }
      desired_yaw = atan2(next_goal.vel[1], next_goal.vel[0]);
      break;
    }
    case DroneStatus::GOAL_REACHED:
      next_goal.dyaw = 0.0;
      next_goal.yaw = previous_yaw_;
      return;
  }

  if (drone_status_ == DroneStatus::YAWING) {
    // Closed-loop yaw from actual drone heading.
    RobotState local_state;
    getState(local_state);
    double diff = desired_yaw - local_state.yaw;
    sando_utils::angle_wrap(diff);

    // Convergence check: transition when within ~17 deg of target
    if (std::fabs(diff) < 0.3) {
      changeDroneStatus(DroneStatus::TRAVELING);
    }

    // Timeout: if yawing for > 5 seconds and roughly facing the right way, transition
    double yaw_elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - yaw_start_time_).count();
    if (yaw_elapsed > 10.0 && std::fabs(diff) < 1.0) {
      changeDroneStatus(DroneStatus::TRAVELING);
    }

    // Step from previous_yaw_ (commanded reference) toward desired_yaw.
    // Using previous_yaw_ instead of local_state.yaw ensures the quaternion
    // marches steadily toward the target — PX4 tracks the quaternion position,
    // not the dyaw feedforward, so the commanded yaw must lead the drone.
    double diff_cmd = desired_yaw - previous_yaw_;
    sando_utils::angle_wrap(diff_cmd);
    double max_step = par_.w_max_yawing * par_.dc;
    double step = std::clamp(diff_cmd, -max_step, max_step);
    next_goal.yaw = previous_yaw_ + step;
    next_goal.dyaw = step / par_.dc;
    previous_yaw_ = next_goal.yaw;
  } else if (drone_status_ == DroneStatus::HOVER_AVOIDING) {
    // Open-loop yaw toward hover position (same pattern as YAWING).
    double diff_cmd = desired_yaw - previous_yaw_;
    sando_utils::angle_wrap(diff_cmd);
    double max_step = par_.w_max_yawing * par_.dc;
    double step = std::clamp(diff_cmd, -max_step, max_step);
    next_goal.yaw = previous_yaw_ + step;
    next_goal.dyaw = step / par_.dc;
    previous_yaw_ = next_goal.yaw;
  } else {
    // TRAVELING/GOAL_SEEN: open-loop from commanded reference (previous_yaw_)
    // for smooth convergence without overshoot.
    double diff = desired_yaw - previous_yaw_;
    sando_utils::angle_wrap(diff);
    yaw(diff, next_goal);
  }
}

// ----------------------------------------------------------------------------
//  yaw 的输入是"还差多少角度"，输出是"这一步走多少、最终 yaw 是多少"
void SANDO::yaw(double diff, RobotState& next_goal) {
  // Filter the yaw ANGLE directly instead of the rate.
  // Each step covers a fraction of the remaining error → exponential
  // convergence with zero overshoot. PX4's attitude controller handles
  // the actual rate tracking (just like RC controller yaw).
  // step：这一步要走的角度,par_.alpha_filter_dyaw：滤波系数，[0, 1] 之间,alpha_filter_dyaw = 0.7,每次只走剩余误差的 30%，走完这一步后还剩 70%
  double step = (1.0 - par_.alpha_filter_dyaw) * diff;    

  // Clamp step size to enforce max yaw rate
  double max_step = par_.w_max * par_.dc;
  step = std::clamp(step, -max_step, max_step);

  next_goal.yaw = previous_yaw_ + step;
  next_goal.dyaw = step / par_.dc;  // feedforward rate for PX4；dyaw：给 PX4 的前馈角速度
  // PX4 的姿态控制器主要跟踪 quaternion（位置），但也接受 dyaw 前馈。给前馈可以让 PX4 更精确地跟踪
  previous_yaw_ = next_goal.yaw;
}

// ----------------------------------------------------------------------------

void SANDO::logGoalEvent(
    const std::string& event,
    const RobotState& drone,
    const RobotState& goal,
    const Eigen::Vector3d& G_projected) {
  if (!par_.debug_verbose) return;
  static const std::string path = "/tmp/sando_goal_log.txt";
  std::ofstream f(path, std::ios::app);
  if (!f.is_open()) return;

  auto now = std::chrono::steady_clock::now();
  static auto t0 = now;
  double t = std::chrono::duration<double>(now - t0).count();

  f << std::fixed << std::setprecision(3) << "[" << t << "s] " << event << "\n"
    << "  drone_pos:  (" << drone.pos.x() << ", " << drone.pos.y() << ", " << drone.pos.z() << ")\n"
    << "  drone_vel:  (" << drone.vel.x() << ", " << drone.vel.y() << ", " << drone.vel.z() << ")\n"
    << "  drone_yaw:  " << drone.yaw << "\n"
    << "  term_goal:  (" << goal.pos.x() << ", " << goal.pos.y() << ", " << goal.pos.z() << ")\n"
    << "  G_project:  (" << G_projected.x() << ", " << G_projected.y() << ", " << G_projected.z()
    << ")\n"
    << "  status:     " << static_cast<int>(drone_status_)
    << " (0=YAWING,1=TRAVELING,2=GOAL_SEEN,3=GOAL_REACHED)\n"
    << "\n";
  f.close();
}

// ----------------------------------------------------------------------------
// 接收一个新的终端目标 G_term，根据无人机当前状态决定是"平滑切换"还是"完全重新初始化"，并更新相关状态。
// 它是上层（ROS 回调 / 用户输入!!!）设置新目标!!!的入口，调用方（在 ROS 回调里）:void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
// 无返回值，但会修改 G_term_、G_、p_hover_、plan_、previous_yaw_、replanning_failure_count_、yaw_start_pos_、yaw_start_time_、drone_status_、terminal_goal_initialized_ 等一堆成员变量
void SANDO::setTerminalGoal(const RobotState& term_goal) {
  // Ignore duplicate goals — the goal_sender re-publishes every 2s for reliability,上层每 2 秒重发一次目标（为了可靠性，防止消息丢失）
  // but re-triggering YAWING clears the plan and stops the drone mid-flight.
  // 如果不忽略重复，每次收到都重新初始化，会：清空 plan_，无人机停在半路，重新进入 YAWING，原地转，反复触发，无人机永远飞不到目标
  if (terminal_goal_initialized_) {
    RobotState current_gterm;
    getGterm(current_gterm);
    if ((current_gterm.pos - term_goal.pos).norm() < 0.1) return;  // same goal, skip
  }

  // Get the state
  RobotState local_state;
  getState(local_state);

  // If the drone is already in TRAVELING or GOAL_SEEN state (i.e. mid-flight),
  // smoothly update the terminal goal without stopping.  The replanning timer
  // will pick up the new goal on the next cycle and replan toward it.
  // terminal_goal_initialized_：不是第一次设置目标，drone_status_ == TRAVELING || GOAL_SEEN：无人机正在飞，两个条件都满足，走"平滑更新"分支。
  if (terminal_goal_initialized_ &&
      (drone_status_ == DroneStatus::TRAVELING || drone_status_ == DroneStatus::GOAL_SEEN)) {
    setGterm(term_goal);    // setGterm(term_goal)：更新终端目标 G_term_
    p_hover_ = term_goal.pos;    // p_hover_ = term_goal.pos：更新悬停点（到达目标后悬停的位置）

    // Project the terminal goal to the sphere for the next replan
    // 从当前位置 local_state.pos 向新目标方向，取 horizon 距离处的点，作为新的局部目标 G_，下次重规划会用这个 G_
    mtx_G_.lock();
    G_.pos = sando_utils::projectPointToSphere(local_state.pos, term_goal.pos, par_.horizon);
    mtx_G_.unlock();
    
    // 写日志：记录一次"平滑更新"
    logGoalEvent("SMOOTH_UPDATE (mid-flight)", local_state, term_goal, G_.pos);

    // Go back to TRAVELING if we were in GOAL_SEEN (since we have a new goal now)
    // 如果原来已经"看见目标"（接近目标），但目标变了，说明还要继续飞，切回 TRAVELING
    if (drone_status_ == DroneStatus::GOAL_SEEN) changeDroneStatus(DroneStatus::TRAVELING);

    return;
  }
  
  // 如果没走平滑更新分支（第一次设目标，或状态是 YAWING/GOAL_REACHED/HOVER_AVOIDING），执行完全初始化
  // First goal or drone is in YAWING/GOAL_REACHED: full initialization
  // Re-initialize plan from current state so that the first replan's
  // A point reflects the actual drone position (not the stale position
  // from when state_initialized_ was first set, e.g. z=0 on the ground).
  {
    RobotState tmp;
    tmp.pos = local_state.pos;
    tmp.vel = local_state.vel;
    tmp.accel = local_state.accel;
    tmp.yaw = local_state.yaw;

    mtx_plan_.lock();
    plan_.clear();
    plan_.push_back(tmp);
    mtx_plan_.unlock();

    setA(tmp);
    setG(tmp);
  }

  // Set the terminal goal
  setGterm(term_goal);
  p_hover_ = term_goal.pos;

  // Reset previous_yaw_ to actual drone yaw so the first getDesiredYaw
  // during YAWING computes diff from the correct reference
  previous_yaw_ = local_state.yaw;

  // Reset replanning failure count so YAWING uses getDesiredYaw (not spinning mode)
  replanning_failure_count_ = 0;

  // Project the terminal goal to the sphere
  mtx_G_.lock();
  G_.pos = sando_utils::projectPointToSphere(local_state.pos, term_goal.pos, par_.horizon);
  mtx_G_.unlock();

  // Store the position and time where yawing starts so the drone holds this
  // point (not the continuously-drifting state) during rotation.
  yaw_start_pos_ = local_state.pos;
  yaw_start_time_ = std::chrono::steady_clock::now();

  // Start with YAWING: rotate to face terminal goal before planning
  // In interactive/sim mode, skip YAWING and go directly to TRAVELING
  if (par_.skip_initial_yawing) {
    changeDroneStatus(DroneStatus::TRAVELING);
    logGoalEvent("FULL_INIT (TRAVELING, skip_yaw)", local_state, term_goal, G_.pos);
  } else {
    changeDroneStatus(DroneStatus::YAWING);
    logGoalEvent("FULL_INIT (YAWING)", local_state, term_goal, G_.pos);
  }

  if (!terminal_goal_initialized_) terminal_goal_initialized_ = true;
}

// ----------------------------------------------------------------------------

void SANDO::changeDroneStatus(int new_status) {
  if (new_status == drone_status_) return;

  std::cout << "Changing DroneStatus from ";

  switch (drone_status_) {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset;
      break;
    case DroneStatus::HOVER_AVOIDING:
      std::cout << bold << "status_=HOVER_AVOIDING" << reset;
      break;
  }

  std::cout << " to ";

  switch (new_status) {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset;
      break;
    case DroneStatus::HOVER_AVOIDING:
      std::cout << bold << "status_=HOVER_AVOIDING" << reset;
      break;
  }

  std::cout << std::endl;

  drone_status_ = new_status;
}

// ----------------------------------------------------------------------------

bool SANDO::checkReadyToReplan() {
  bool map_init = hgp_manager_.isMapInitialized();
  bool kdtree_ok = !par_.use_hardware || kdtree_map_initialized_;

  return state_initialized_ && terminal_goal_initialized_ && map_init && kdtree_ok;
}

// ----------------------------------------------------------------------------

void SANDO::updateMapPtr(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_map,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_unk) {
  // 1) Atomically store the incoming clouds，把最新地图缓存到 SANDO 内部
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);    // 为什么要锁？因为 ROS 回调可能在一个线程更新点云，而规划线程同时读取点云。锁是为了防止读写冲突。
    pclptr_map_ = pclptr_map;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_unk_);
    pclptr_unk_ = pclptr_unk;
  }

  // 初始化 KD-tree, KD-tree 用于快速查最近障碍点
  // Build the KD-tree from the incoming occupied cloud so that
  // kdtree_map_initialized_ becomes true even before the first replan.
  // Without this, checkReadyToReplan() would block forever on hardware
  // because updateMap() (which also builds the kdtree) only runs inside
  // the replan loop — a chicken-and-egg problem.
  if (pclptr_map && !pclptr_map->points.empty() && !kdtree_map_initialized_) {
    std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
    kdtree_map_.setInputCloud(pclptr_map);
    kdtree_map_initialized_ = true;
  }
  // 第一次收到地图时立刻初始化 HGP map
  if (!hgp_manager_.isMapInitialized()) {
    updateMap(0.0);
  }
}

// ----------------------------------------------------------------------------
// 是 gazebo / hardware 主地图更新函数
// 输入current_time
// 输出不是返回值，而是更新内部状态，包括：
// hgp_manager_ 里的 voxel map
// kdtree_map_
// kdtree_unk_
// vec_o_
// vec_uo_
// traj_max_time_
// obst_pos_
// obst_bbox_
void SANDO::updateMap(double current_time) {
  // Update the map size，地图窗口应该覆盖 当前无人机位置 → 当前局部目标 G，不是全世界都建 voxel map（降低计算量）
  RobotState local_state, local_G;
  getState(local_state);
  getG(local_G);
  computeMapSize(local_state.pos, local_G.pos);

  // Get dynamic obstacles' positions, bboxes, and traj_max_time
  vec_Vecf<3> obst_pos;
  vec_Vecf<3> obst_bbox;
  std::vector<vec_Vecf<3>> pred_samples;
  std::vector<float> pred_times;
  
  // computeObstPosAndTrajMaxTimeForMapUpdate函数的作用：
  // 筛选地图窗口内、规划 horizon 内的动态障碍物
  // 提取障碍物当前位置 obst_pos
  // 提取障碍物膨胀 bbox
  // 采样未来预测轨迹 pred_samples
  // 计算最大预测时间 traj_max_time_
  traj_max_time_ = computeObstPosAndTrajMaxTimeForMapUpdate(
      obst_pos, obst_bbox, pred_samples, pred_times, current_time);
  
  // 把预测样本交给 HGP
  // 全局路径搜索时，不只是看静态障碍，还会根据动态障碍未来轨迹生成 heat cost。
  hgp_manager_.setDynamicPredictedSamples(pred_samples, pred_times);

  // time the map update
  MyTimer timer_map(true);

  // 2) map update
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);
    std::lock_guard<std::mutex> lk2(mtx_pclptr_unk_);
    
    // 真正构建 voxel map 的地方在 hgp_manager.cpp，最终是 VoxelMapUtil::readMap() 把点云离散成 voxel grid
    hgp_manager_.updateMap(
        wdx_, wdy_, wdz_, map_center_, pclptr_map_, pclptr_unk_, obst_pos, obst_bbox,
        traj_max_time_);

    if (par_.debug_verbose)
      std::cout << "Map update time: " << timer_map.getElapsedMicros() / 1000.0 << " ms"
                << std::endl;

    // 3) Known‐space KD‐tree，更新已知占据 KD-tree 和 vec_o
    if (pclptr_map_ && !pclptr_map_->points.empty()) {
      std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
      kdtree_map_.setInputCloud(pclptr_map_);
      kdtree_map_initialized_ = true;
      hgp_manager_.updateVecOccupied(pclptr_to_vec(pclptr_map_));    // vec_o_ 是把 PCL 点云转成 Eigen 点数组vec_Vec3f， 这个后面会给安全走廊生成用
    } else {
      RCLCPP_WARN(
          rclcpp::get_logger("sando"),
          "updateMap: member pclptr_map_ was null or empty; skipping KD-tree update");
    }
  }

  // 4) Unknown‐space KD‐tree，更新未知空间 KD-tree 和 vec_uo
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_unk_);
    if (pclptr_unk_ && !pclptr_unk_->points.empty()) {
      std::lock_guard<std::mutex> lk(mtx_kdtree_unk_);
      kdtree_unk_.setInputCloud(pclptr_unk_);
      kdtree_unk_initialized_ = true;
      // merge known into unknown vector，vec_uo_ = unknown + occupied
      // 因为安全走廊生成时，在 gazebo/hardware 这种带未知空间的模式下，通常要把未知空间 + 已知障碍物都当成不能穿越的点集
      // 所以 vec_uo_ 实际含义更像non-free points，而不只是 unknown
      hgp_manager_.updateVecUnknownOccupied(pclptr_to_vec(pclptr_unk_));
      hgp_manager_.insertVecOccupiedToVecUnknownOccupied();
    } else {
      RCLCPP_WARN(
          rclcpp::get_logger("sando"),
          "updateMap: member pclptr_unk_ was null or empty; skipping KD‐tree update");
    }
  }
}

// ----------------------------------------------------------------------------
// 简化版 updateMapPtr()，输入只有pclptr_map占据点云，没有pclptr_unk未知空间点云，对应场景fake_sim和rviz_only
void SANDO::updateOccupancyMapPtr(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_map) {
  // store the incoming clouds
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);
    pclptr_map_ = pclptr_map;
  }

  if (!hgp_manager_.isMapInitialized()) {
    updateOccupancyMap(0.0);
  }
}

// ----------------------------------------------------------------------------
// updateMap() 的简化版，没有unknown：
// updateMap()
//   输入 occupied + unknown
//   更新 known KD-tree
//   更新 unknown KD-tree
//   更新 vec_o_
//   更新 vec_uo_ = unknown + occupied
// updateOccupancyMap()
//   输入 occupied
//   unknown 为空
//   更新 known KD-tree
//   更新 vec_o_
//   不更新 unknown KD-tree / vec_uo_
// gazebo/hardware：安全走廊避开 unknown + occupied
// fake_sim：安全走廊只避开 occupied
void SANDO::updateOccupancyMap(double current_time) {
  // Update the map size
  RobotState local_state, local_G;
  getState(local_state);
  getG(local_G);
  computeMapSize(local_state.pos, local_G.pos);

  // Get dynamic obstacles' positions, bboxes, and traj_max_time
  vec_Vecf<3> obst_pos;
  vec_Vecf<3> obst_bbox;
  std::vector<vec_Vecf<3>> pred_samples;
  std::vector<float> pred_times;

  traj_max_time_ = computeObstPosAndTrajMaxTimeForMapUpdate(
      obst_pos, obst_bbox, pred_samples, pred_times, current_time);

  hgp_manager_.setDynamicPredictedSamples(pred_samples, pred_times);

  // 2) map update (unlocked)
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);
    // 没有 unknown map，就创建一个空 unknown map
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>());

    hgp_manager_.updateMap(
        wdx_, wdy_, wdz_, map_center_, pclptr_map_, empty_pclptr_unk, obst_pos, obst_bbox,
        traj_max_time_);

    // 3) Known‐space KD‐tree
    if (pclptr_map_ && !pclptr_map_->points.empty()) {
      std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
      kdtree_map_.setInputCloud(pclptr_map_);
      kdtree_map_initialized_ = true;
      hgp_manager_.updateVecOccupied(pclptr_to_vec(pclptr_map_));
    } else {
      RCLCPP_WARN(
          rclcpp::get_logger("sando"),
          "updateMap: member pclptr_map_ was null or empty; skipping KD-tree update");
    }
  }
}

// ----------------------------------------------------------------------------
// 从当前所有动态障碍轨迹中，筛选出"在当前地图范围内、且在预测时域内"的障碍，提取它们的位置和包围盒，
// 并对每条轨迹做时间采样，供 HGP 构建时空热力图（heat-map）使用。
double SANDO::computeObstPosAndTrajMaxTimeForMapUpdate(
    vec_Vecf<3>& obst_pos,
    vec_Vecf<3>& obst_bbox,                  // bbox of each obstacle
    std::vector<vec_Vecf<3>>& pred_samples,  // [K][M]
    std::vector<float>& pred_times,          // [M], relative times from now
    double current_time) {
  obst_pos.clear();
  obst_bbox.clear();
  pred_samples.clear();
  pred_times.clear();

  std::vector<std::shared_ptr<DynTraj>> local_trajs;
  getTrajs(local_trajs);

  // 1) Filter obstacles and build obst_pos and obst_bbox in a consistent order
  // 障碍物当前位置在当前地图窗口内
  // 并且距离无人机不超过 planning horizon
  std::vector<std::shared_ptr<DynTraj>> selected_trajs;
  selected_trajs.reserve(local_trajs.size());

  for (const auto& traj : local_trajs) {
    Eigen::Vector3d p = traj->eval(current_time);
    bool in_map = checkPointWithinMap(p);
    double dist = (p - state_.pos).norm();
    bool in_horizon = dist <= par_.horizon;
    if (!in_map || !in_horizon) {
      continue;
    }

    obst_pos.push_back(p);
    obst_bbox.push_back(traj->bbox);  // Extract bbox from DynTraj
    selected_trajs.push_back(traj);
  }

  // Update obst_pos_ and obst_bbox_
  {
    std::lock_guard<std::mutex> lock(mtx_obst_pos_);
    obst_pos_ = obst_pos;
    obst_bbox_ = obst_bbox;
  }

  // 2) Horizon for map update (your existing “worst possible”)
  // 如果最慢那条候选轨迹都要飞 Th 秒，
  // 那动态障碍物至少要预测到 Th 秒后。
  const double Th = worst_traj_time_ * factors_.back();  // [s]
  if (!(Th > 0.0) || selected_trajs.empty()) return Th;

  // 3) Build time samples between [0, Th]
  const double dt = 0.5;  // [s]
  int M = static_cast<int>(std::ceil(Th / dt)) + 1;
  // keep it bounded for cost (heat-map build is O(#voxels * K * M))
  M = std::max(5, std::min(M, 10));

  pred_times.resize(M);
  for (int j = 0; j < M; ++j) {
    const double a = (M == 1) ? 0.0 : (double)j / (double)(M - 1);
    pred_times[j] = static_cast<float>(a * Th);  // relative time from now
  }

  // 4) Sample each obstacle trajectory at (current_time + pred_times[j])
  pred_samples.resize(selected_trajs.size());
  for (size_t k = 0; k < selected_trajs.size(); ++k) {
    pred_samples[k].resize(M);
    for (int j = 0; j < M; ++j) {
      const double t_abs = current_time + (double)pred_times[j];
      Eigen::Vector3d pk = selected_trajs[k]->eval(t_abs);

      // NOTE: We do NOT drop samples outside the map, because the heat map
      // will simply have no effect there. But we must avoid NaNs.
      if (!std::isfinite(pk.x()) || !std::isfinite(pk.y()) || !std::isfinite(pk.z())) {
        // fallback: use current position (safe default)
        pk = selected_trajs[k]->eval(current_time);
      }

      pred_samples[k][j] = pk;
    }
  }

  return Th;
}

// ----------------------------------------------------------------------------

std::shared_ptr<sando::VoxelMapUtil> SANDO::getMapUtilSharedPtr() {
  return hgp_manager_.getMapUtilSharedPtr();
}

// ----------------------------------------------------------------------------

void SANDO::setInitialPose(const geometry_msgs::msg::TransformStamped& init_pose) {
  init_pose_ = init_pose;

  // Extract and normalize quaternion
  Eigen::Quaterniond q(
      init_pose_.transform.rotation.w, init_pose_.transform.rotation.x,
      init_pose_.transform.rotation.y, init_pose_.transform.rotation.z);
  q.normalize();

  Eigen::Vector3d t(
      init_pose_.transform.translation.x, init_pose_.transform.translation.y,
      init_pose_.transform.translation.z);

  // Rotation matrix and its transpose (= inverse for orthogonal matrices)
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Matrix3d R_T = R.transpose();

  // Forward transform: local → global  [R | t; 0 0 0 1]
  init_pose_transform_ = Eigen::Matrix4d::Identity();
  init_pose_transform_.block<3, 3>(0, 0) = R;
  init_pose_transform_.block<3, 1>(0, 3) = t;

  // Inverse transform: global → local  [R^T | -R^T * t; 0 0 0 1]
  // Using the analytic SE(3) inverse instead of general matrix inverse
  init_pose_transform_inv_ = Eigen::Matrix4d::Identity();
  init_pose_transform_inv_.block<3, 3>(0, 0) = R_T;
  init_pose_transform_inv_.block<3, 1>(0, 3) = -R_T * t;

  // Store rotation matrices
  init_pose_transform_rotation_ = R;
  init_pose_transform_rotation_inv_ = R_T;

  // Yaw offset
  yaw_init_offset_ = std::atan2(R(1, 0), R(0, 0));

  // Sanity check: M_inv * init_pos should give (0,0,0) in local frame
  Eigen::Vector4d t_homo(t.x(), t.y(), t.z(), 1.0);
  Eigen::Vector4d local_origin = init_pose_transform_inv_ * t_homo;
  double sanity_err = local_origin.head<3>().norm();
  if (sanity_err < 0.1) {
    std::cout << bold << green << "****** [SANDO] READY TO FLY ******" << reset << std::endl;
  } else {
    std::cout << "\033[1;31m"
              << "****** [SANDO] TRANSFORM SANITY CHECK FAILED ******"
              << "\033[0m" << std::endl;
    std::cout << "\033[1;31m"
              << "inv * init_pos = (" << local_origin[0] << ", " << local_origin[1] << ", "
              << local_origin[2] << ") [should be ~(0,0,0), err=" << sanity_err << "]"
              << "\033[0m" << std::endl;
  }

  init_pose_set_ = true;
}

// ----------------------------------------------------------------------------

// Apply the initial pose transformation to the pwp
void SANDO::applyInitiPoseTransform(PieceWisePol& pwp) {
  // Loop thru the intervals
  for (int i = 0; i < pwp.coeff_x.size(); i++) {
    // Loop thru a, b, c, and d
    for (int j = 0; j < 4; j++) {
      Eigen::Vector4d coeff;
      coeff[0] = pwp.coeff_x[i][j];
      coeff[1] = pwp.coeff_y[i][j];
      coeff[2] = pwp.coeff_z[i][j];
      coeff[3] = 1.0;

      // Apply multiplication
      coeff = init_pose_transform_ * coeff;

      // cout agent frame pose
      pwp.coeff_x[i][j] = coeff[0];
      pwp.coeff_y[i][j] = coeff[1];
      pwp.coeff_z[i][j] = coeff[2];
    }
  }
}

// ----------------------------------------------------------------------------

// Apply the inverse of initial pose transformation to the pwp
void SANDO::applyInitiPoseInverseTransform(PieceWisePol& pwp) {
  // Loop thru the intervals
  for (int i = 0; i < pwp.coeff_x.size(); i++) {
    // Loop thru a, b, c, and d
    for (int j = 0; j < 4; j++) {
      Eigen::Vector4d coeff;
      coeff[0] = pwp.coeff_x[i][j];
      coeff[1] = pwp.coeff_y[i][j];
      coeff[2] = pwp.coeff_z[i][j];
      coeff[3] = 1.0;

      // Apply multiplication
      coeff = init_pose_transform_inv_ * coeff;

      pwp.coeff_x[i][j] = coeff[0];
      pwp.coeff_y[i][j] = coeff[1];
      pwp.coeff_z[i][j] = coeff[2];
    }
  }
}

// ----------------------------------------------------------------------------

bool SANDO::goalReachedCheck() {
  if (checkReadyToReplan() && (drone_status_ == DroneStatus::GOAL_REACHED ||
                               drone_status_ == DroneStatus::HOVER_AVOIDING)) {
    return true;
  }
  return false;
}

// ----------------------------------------------------------------------------

bool SANDO::checkHoverAvoidance(double current_time) {
  RobotState local_state;
  getState(local_state);

  // Get current obstacle positions from trajs_
  std::vector<std::shared_ptr<DynTraj>> local_trajs;
  getTrajs(local_trajs);

  // Helper: check if a point is within d_trigger of any obstacle.
  // When lookahead=true, samples over a future time window to catch periodic orbits
  // (used for evasion goal validation). When lookahead=false, only checks the
  // obstacle's current position (used for return-to-hover decisions so the drone
  // returns as soon as the obstacle is visually clear).
  const double lookahead_window = 15.0;  // seconds into the future
  const double lookahead_step = 0.5;     // sampling interval
  auto isPointThreatened = [&](const Eigen::Vector3d& pt, bool lookahead = true) -> bool {
    for (size_t i = 0; i < local_trajs.size(); ++i) {
      // Always check the agent's actual reported position first
      if ((pt - local_trajs[i]->current_pos).norm() < par_.hover_avoidance_d_trigger) return true;

      // If lookahead enabled, also sample the predicted trajectory.
      // For PWP trajectories (agents), limit to the trajectory's valid time range
      // to avoid using stale endpoint extrapolation as a "future prediction".
      // For analytic trajectories (obstacles), use the full lookahead window.
      if (lookahead) {
        double t_end_lookahead = current_time + lookahead_window;
        if (local_trajs[i]->mode == DynTraj::Mode::Piecewise &&
            !local_trajs[i]->pwp.times.empty()) {
          t_end_lookahead = std::min(t_end_lookahead, local_trajs[i]->pwp.times.back());
        }
        for (double t = current_time; t <= t_end_lookahead; t += lookahead_step) {
          Eigen::Vector3d p_obs = local_trajs[i]->eval(t);
          if ((pt - p_obs).norm() < par_.hover_avoidance_d_trigger) return true;
        }
      }
    }
    return false;
  };

  // Compute repulsion vector from drone's current position
  Eigen::Vector3d n_total = Eigen::Vector3d::Zero();
  double closest_dist = 1e9;
  for (const auto& traj : local_trajs) {
    // Use the agent's actual reported position for repulsion
    Eigen::Vector3d p_obs = traj->current_pos;
    Eigen::Vector3d r_i = local_state.pos - p_obs;
    double dist_i = r_i.norm();
    closest_dist = std::min(closest_dist, dist_i);

    if (dist_i < par_.hover_avoidance_d_trigger && dist_i > 1e-6) {
      double w_i = 1.0 / (dist_i * dist_i);
      n_total += w_i * (r_i / dist_i);
    }
  }

  if (n_total.norm() > par_.hover_avoidance_min_repulsion_norm) {
    // Store hover position once when first entering avoidance
    if (drone_status_ != DroneStatus::HOVER_AVOIDING) {
      RobotState temp_gterm;
      getGterm(temp_gterm);
      p_hover_ = temp_gterm.pos;
      changeDroneStatus(DroneStatus::HOVER_AVOIDING);
    }

    // Compute evasion goal — push away from current position (not p_hover_)
    Eigen::Vector3d direction = n_total.normalized();

    // 2D mode: zero out vertical component so avoidance stays at current altitude
    if (par_.hover_avoidance_2d) direction.z() = 0.0;

    // Re-normalize after zeroing z (guard against degenerate case)
    if (direction.norm() < 1e-6)
      direction = Eigen::Vector3d(1.0, 0.0, 0.0);
    else
      direction.normalize();

    Eigen::Vector3d p_evasion = local_state.pos + par_.hover_avoidance_h * direction;

    // In 2D mode keep the drone's current altitude; otherwise clamp to safe range
    if (par_.hover_avoidance_2d)
      p_evasion.z() = local_state.pos.z();
    else
      p_evasion.z() = std::max(par_.z_min + 0.5, std::min(p_evasion.z(), par_.z_max - 0.5));

    // Reject evasion goal if it's still inside an obstacle's d_trigger
    if (isPointThreatened(p_evasion, true)) {
      // Try rotated directions to escape
      const std::vector<double> angles = {M_PI / 6, -M_PI / 6, M_PI / 3, -M_PI / 3,
                                          M_PI / 2, -M_PI / 2, M_PI};
      bool found_safe = false;
      for (double angle : angles) {
        double cos_a = std::cos(angle), sin_a = std::sin(angle);
        Eigen::Vector3d rotated_dir(
            direction.x() * cos_a - direction.y() * sin_a,
            direction.x() * sin_a + direction.y() * cos_a, direction.z());
        Eigen::Vector3d candidate =
            local_state.pos + par_.hover_avoidance_h * rotated_dir.normalized();
        if (par_.hover_avoidance_2d)
          candidate.z() = local_state.pos.z();
        else
          candidate.z() = std::max(par_.z_min + 0.5, std::min(candidate.z(), par_.z_max - 0.5));
        if (!isPointThreatened(candidate, true) && !checkIfPointOccupied(Vec3f(candidate))) {
          p_evasion = candidate;
          found_safe = true;
          break;
        }
      }
      if (!found_safe) return false;  // can't find safe evasion point, stay put
    }

    // Also reject if evasion point hits a static obstacle
    if (checkIfPointOccupied(Vec3f(p_evasion))) {
      const std::vector<double> angles = {M_PI / 6,  -M_PI / 6, M_PI / 3,
                                          -M_PI / 3, M_PI / 2,  -M_PI / 2};
      bool found_free = false;
      for (double angle : angles) {
        double cos_a = std::cos(angle), sin_a = std::sin(angle);
        Eigen::Vector3d rotated_dir(
            direction.x() * cos_a - direction.y() * sin_a,
            direction.x() * sin_a + direction.y() * cos_a, direction.z());
        Eigen::Vector3d candidate =
            local_state.pos + par_.hover_avoidance_h * rotated_dir.normalized();
        if (par_.hover_avoidance_2d)
          candidate.z() = local_state.pos.z();
        else
          candidate.z() = std::max(par_.z_min + 0.5, std::min(candidate.z(), par_.z_max - 0.5));
        if (!checkIfPointOccupied(Vec3f(candidate)) && !isPointThreatened(candidate, true)) {
          p_evasion = candidate;
          found_free = true;
          break;
        }
      }
      if (!found_free) return false;
    }

    // Set evasion goal
    RobotState evasion_goal;
    evasion_goal.setPos(p_evasion.x(), p_evasion.y(), p_evasion.z());
    setGterm(evasion_goal);

    mtx_G_.lock();
    G_.pos = sando_utils::projectPointToSphere(local_state.pos, p_evasion, par_.horizon);
    mtx_G_.unlock();

    return true;  // continue with replanning
  } else if (drone_status_ == DroneStatus::HOVER_AVOIDING) {
    // Obstacles cleared from drone's current position.
    // Only return to p_hover_ if it's also clear of all obstacles.
    if (isPointThreatened(p_hover_, false)) {
      // p_hover_ is still unsafe — stay at current position and keep waiting.
      // Do NOT overwrite p_hover_; the obstacle will eventually move away.
      return false;
    }

    // Safe to return to original hover position
    RobotState hover_goal;
    hover_goal.setPos(p_hover_.x(), p_hover_.y(), p_hover_.z());
    setGterm(hover_goal);

    mtx_G_.lock();
    G_.pos = sando_utils::projectPointToSphere(local_state.pos, p_hover_, par_.horizon);
    mtx_G_.unlock();

    // Stay in HOVER_AVOIDING — drone is back at hover position but keeps monitoring
    return true;
  }

  // No obstacles nearby — stay in HOVER_AVOIDING (ready to dodge)
  return false;
}