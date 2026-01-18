#include "ego_planner/yaw_optimizer.h"
#include <ros/ros.h>

namespace ego_planner
{

// Helper to normalize angle to [-PI, PI]
static double normalize_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// B-spline Basis Functions (Uniform, k=3 for cubic)
// Evaluate value at u in [0, 1] relative to span
// Control points influence: P0, P1, P2, P3
// For uniform cubic B-spline, basis functions are fixed.
// u = (t - t_i) / dt
static void getBasisValues(double u, Eigen::Vector4d& val, Eigen::Vector4d& grad)
{
  double u2 = u * u;
  double u3 = u2 * u;

  // Matrix form for Cubic B-spline (Uniform)
  // [ 1  4  1  0 ] / 6
  // [-3  0  3  0 ] / 6
  // [ 3 -6  3  0 ] / 6
  // [-1  3 -3  1 ] / 6
  // Wrong. Standard basis:
  // N0 = (1-u)^3 / 6
  // N1 = (3u^3 - 6u^2 + 4) / 6
  // N2 = (-3u^3 + 3u^2 + 3u + 1) / 6
  // N3 = u^3 / 6
  
  // Implementation
  val(0) = (1.0 - 3.0*u + 3.0*u2 - u3) / 6.0;
  val(1) = (4.0 - 6.0*u2 + 3.0*u3) / 6.0;
  val(2) = (1.0 + 3.0*u + 3.0*u2 - 3.0*u3) / 6.0;
  val(3) = u3 / 6.0;

  // Derivatives wrt u (multiply by 1/dt for wrt t)
  grad(0) = (-3.0 + 6.0*u - 3.0*u2) / 6.0;
  grad(1) = (-12.0*u + 9.0*u2) / 6.0;
  grad(2) = (3.0 + 6.0*u - 9.0*u2) / 6.0;
  grad(3) = 3.0*u2 / 6.0;
}

bool YawOptimizer::optimize(const poly_traj::Trajectory& xyz_traj, 
                            double start_yaw, 
                            double end_yaw, 
                            std::vector<double>& yaw_ctrl_pts,
                            double dt)
{
  double total_duration = xyz_traj.getTotalDuration();
  if (total_duration < 0.1) return false;

  // 1. Setup B-spline structure
  // Uniform knot spacing dt
  // Number of spans M = ceil(T / dt)
  int M = std::ceil(total_duration / dt);
  if (M < 3) M = 3; // Minimum spans
  dt = total_duration / M; // Adjust dt slightly to fit perfectly
  
  // Number of control points N = M + k. Cubic B-spline k=3 (degree), p=k+1=4 points per span?
  // Standard B-spline: N = M + Order - 1 = M + 3 (if clamping?)
  // For uniform unclamped: N = M + 3.
  // P0, P1, P2 are for start (if using specific boundary conditions)
  // Let's use N = M + 3 for Cubic.
  int N = M + 3;
  
  // 2. Sample Angle References (Unrolling)
  std::vector<double> ts;
  std::vector<double> yaw_refs;
  
  // Store start yaw as first reference
  // We will fix first few Control Points to enforce start state
  
  // Sample samples per span for integration
  int samples_per_span = 3;
  double d_sample = dt / samples_per_span;

  // Unrolling Loop
  // Check trajectory velocity to determine "forward" yaw
  // If velocity is too small, hold previous yaw
  double last_valid_yaw = start_yaw;

  // Prepare QP
  // min x'Hx + f'x
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(N, N);
  Eigen::VectorXd f = Eigen::VectorXd::Zero(N);

  double T_align = 2.0; // Time window for alignment

  for (int i = 0; i < M; ++i) // For each span
  {
    for (int j = 0; j < samples_per_span; ++j)
    {
      double u = (double)j / samples_per_span;
      double t = (i + u) * dt;
      if (t > total_duration) t = total_duration;

      // Current basis influence
      // Span i affects Control Points i, i+1, i+2, i+3 ?
      // Yes, for uniform B-spline defined on [0, M], span [k, k+1] uses P_k, P_{k+1}, P_{k+2}, P_{k+3}
      // Index offset = i
      Eigen::Vector4d basis_val;
      Eigen::Vector4d basis_grad;
      getBasisValues(u, basis_val, basis_grad); // grad wrt u
      
      // Calculate Reference Yaws
      // 1. Velocity Yaw (Psi_vel)
      Eigen::Vector3d vel = xyz_traj.getVel(t);
      double vel_yaw = last_valid_yaw;
      if (vel.head(2).norm() > 0.1) {
        vel_yaw = std::atan2(vel.y(), vel.x());
      }
      
      // Unroll vel_yaw
      double diff = vel_yaw - last_valid_yaw;
      while (diff > M_PI) diff -= 2*M_PI;
      while (diff < -M_PI) diff += 2*M_PI;
      vel_yaw = last_valid_yaw + diff;
      last_valid_yaw = vel_yaw;

      // 2. Target Yaw (Psi_req)
      // Unroll end_yaw relative to current vel_yaw for smooth blending
      double req_yaw = end_yaw;
      diff = req_yaw - vel_yaw;
      while (diff > M_PI) diff -= 2*M_PI;
      while (diff < -M_PI) diff += 2*M_PI;
      req_yaw = vel_yaw + diff;

      // Weights
      double time_to_go = total_duration - t;
      double w_v = 0.0;
      double w_t = 0.0;

      if (time_to_go > T_align) {
        // Cruising: strong velocity coupling
        w_v = w_vel_;
        w_t = 0.0;
      } else {
        // Approach: blend to target
        double ratio = (T_align - time_to_go) / T_align; // 0 -> 1
        if (ratio > 1.0) ratio = 1.0;
        if (ratio < 0.0) ratio = 0.0;
        
        // Sigmoid or Linear blend
        w_v = w_vel_ * (1.0 - ratio);
        w_t = w_target_ * ratio;
      }

      // Add to H and f
      // J = w_v * (Pos - vel_yaw)^2 + w_t * (Pos - req_yaw)^2
      // Pos = sum(basis_k * P_{i+k})
      // Local interaction
      
      for (int r = 0; r < 4; ++r) {
        int idx_r = i + r;
        if (idx_r >= N) continue;

        for (int c = 0; c < 4; ++c) {
          int idx_c = i + c;
          if (idx_c >= N) continue;
          
          double val = d_sample * (w_v + w_t) * basis_val(r) * basis_val(c);
          H(idx_r, idx_c) += val;
        }
        
        double weighted_ref = w_v * vel_yaw + w_t * req_yaw;
        f(idx_r) += -2.0 * d_sample * weighted_ref * basis_val(r);
      }
    }
  }

  // Smoothness Cost (Min Acceleration)
  // Approximate with control point differences (finite difference)
  // Acc ~ (x_{k+2} - 2x_{k+1} + x_k) / dt^2
  // Integral Sum (Acc^2) * dt
  
  // Actually, for B-spline, integral of squared derivative can be computed exactly.
  // But finite difference on control points is a standard robust proxy.
  // Penalty: w_smooth * sum || P_{k+2} - 2P_{k+1} + P_k ||^2
  // Iterate valid triples.
  for (int k = 0; k < N - 2; ++k)
  {
    // term = P_k - 2P_{k+1} + P_{k+2}
    // coeffs: 1, -2, 1 at indices k, k+1, k+2
    // square it:
    // P_k^2 + 4P_{k+1}^2 + P_{k+2}^2 - 4P_kP_{k+1} + 2P_kP_{k+2} - 4P_{k+1}P_{k+2}
    
    double w = w_smooth_; // * dt ? Weights usually relative.

    H(k, k) += w; 
    H(k+1, k+1) += 4*w;
    H(k+2, k+2) += w;
    
    H(k, k+1) -= 2*w; H(k+1, k) -= 2*w;
    H(k, k+2) += w;   H(k+2, k) += w;
    H(k+1, k+2) -= 2*w; H(k+2, k+1) -= 2*w;
  }

  // 3. Constraints (Fix Start/End)
  // Fix P0 to start_yaw
  // Fix P1 to start_yaw + start_yaw_dot * dt / 3 (for velocity continuity?)
  // Let's assume start velocity 0 or continuous with previous?
  // User manual says: start_yaw is given.
  // We fix P0 = start_yaw.
  // For P1, if we want to enforce start yaw_dot, we need previous state.
  // If not provided, we can leave P1 optimized (smooth transition).
  // But strictly, P0 is position at t=0 (for uniform BS, P(0) depends on P0, P1, P2?)
  // Wait, for standard uniform B-spline, curve starts at t=0 inside span 0?
  // Actually, standard uniform B-spline is defined for u in range.
  // Usually, to interpolate endpoints, we repeat knots or extend control points.
  // Here we use N = M+3.
  // Curve defined on [0, M].
  // P(0) uses basis functions at u=0.
  // Basis(0) = [1/6, 4/6, 1/6, 0].
  // So Pos(0) = (P0 + 4P1 + P2) / 6.
  // This means P0 is NOT the start position directly.
  
  // FIX: To make it easier, we can treat the "First Control Point P0" as the value *interpolating* the start?
  // Constraints:
  // 1. Pos(0) = start_yaw
  //    (P0 + 4P1 + P2)/6 = start_yaw
  // 2. Vel(0) = start_yaw_dot (assume 0 or aligned)
  //    Vel(0) = (- P0 + P2) / (2*dt) = start_yaw_dot
  //
  // Solving for P0, P1, P2 given Pos(0) and Vel(0).
  // We have 2 eq, 3 vars. Need 3rd? Acc(0)=0?
  // Acc(0) = (P0 - 2P1 + P2)/dt^2 = 0 => P0-2P1+P2 = 0.
  
  // It's easier to use large weights to enforce Boundary Conditions in H/f
  // Penalty: W_fix * (Pos(0) - start_yaw)^2
  // Penalty: W_fix * (Vel(0) - 0)^2
  // Penalty: W_fix * (Pos(T) - end_yaw)^2
  
  // Use Hard Constraints by removing variables? No, simpler to use Soft Constraints with High Weights.
  // [关键修复] 增大终点约束权重，确保轨迹终点精确收敛到目标 Yaw
  // 原来 w_fix = 10000 太弱，被平滑性约束压制
  double w_fix = 1000000.0;
  
  // Start Pos: (P0 + 4P1 + P2)/6 = start
  // 1/6 * P0 + 4/6 * P1 + 1/6 * P2 - start = 0
  Eigen::Vector3d vec_s(1.0/6.0, 4.0/6.0, 1.0/6.0);
  int idx_s[3] = {0, 1, 2};
  for(int r=0; r<3; ++r) {
    for(int c=0; c<3; ++c) {
      H(idx_s[r], idx_s[c]) += w_fix * vec_s(r) * vec_s(c);
    }
    f(idx_s[r]) += -2.0 * w_fix * start_yaw * vec_s(r);
  }
  
  // End Pos: t=T corresponds to span M-1, u=1? No, span indices 0..M-1.
  // Span M-1, basis at u=1.
  // Basis(1) = [0, 1/6, 4/6, 1/6]
  // Affected by P_{M-1}, P_M, P_{M+1}, P_{M+2}
  // Indices: i+(0,1,2,3). i=M-1. -> M-1, M, M+1, M+2.
  // Since N = M+3. Indices 0..N-1.
  // Max index is N-1 = M+2.
  // So P_{M-1}, P_M, P_{M+1}, P_{M+2} are valid indices.
  // Equation: (P_{M-1} + 4P_M + P_{M+1})/6 = end_yaw? No.
  // Basis at u=1 is reversed of u=0.
  // [0, 1/6, 4/6, 1/6]
  // Vector: P_{M-1}, P_M, P_{M+1}, P_{M+2}
  // Coeffs: 0, 1/6, 4/6, 1/6.
  // So: (P_M + 4P_{M+1} + P_{M+2}) / 6 = end_yaw.
  
  // [关键修复] 正确展开终点 Yaw
  // 问题：之前使用 last_valid_yaw（速度方向累积展开值）作为参考
  // 但 last_valid_yaw 可能已经偏离了 start_yaw 很远（如 start=0, last=3.0）
  // 导致 end_yaw=-3.03 被错误展开为 final_target=3.25（绕远路）
  // 
  // 修复：以 start_yaw 为基准展开 end_yaw，确保整体旋转方向正确
  // 这样可以保证从 start_yaw 到 end_yaw 走最短路径
  double final_target = end_yaw;
  double d = final_target - start_yaw;
  while(d > M_PI) d -= 2*M_PI;
  while(d < -M_PI) d += 2*M_PI;
  final_target = start_yaw + d;
  
  Eigen::Vector3d vec_e(1.0/6.0, 4.0/6.0, 1.0/6.0);
  int idx_e[3] = {N-3, N-2, N-1};
  for(int r=0; r<3; ++r) {
    for(int c=0; c<3; ++c) {
      H(idx_e[r], idx_e[c]) += w_fix * vec_e(r) * vec_e(c);
    }
    f(idx_e[r]) += -2.0 * w_fix * final_target * vec_e(r);
  }
  
  // End Vel: (-P_M + P_{M+2}) / (2dt) = 0 (stop rotating)
  // Indices: N-3, N-2, N-1 (corresponds to M, M+1, M+2 in previous block?)
  // Actually Basis Grad at u=1:
  // [-0.5, 0, 0.5, 0] ?
  // Grad(1):
  // N0'(1) = (-3+6-3)/6 = 0
  // N1'(1) = (-12+9)/6 = -0.5
  // N2'(1) = (3+6-9)/6 = 0
  // N3'(1) = 3/6 = 0.5
  // Correct.
  // So Vel = (-0.5*P_M + 0.5*P_{M+2}) / dt.
  // Constraint: P_{M+2} - P_M = 0.
  // Weight it.
  Eigen::Vector2d vec_ev(-1.0, 1.0); // scaled
  int idx_ev[2] = {N-3, N-1};
  for(int r=0; r<2; ++r) {
    for(int c=0; c<2; ++c) {
      H(idx_ev[r], idx_ev[c]) += w_fix * vec_ev(r) * vec_ev(c);
    }
    // Target 0
  }

  // 4. Solve QP
  Eigen::VectorXd x;
  if (solveQP(H, f, x)) {
    yaw_ctrl_pts.resize(N);
    for(int i=0; i<N; ++i) yaw_ctrl_pts[i] = x(i);
    
    // [Debug] 验证优化结果
    // double yaw_at_start = (x(0) + 4*x(1) + x(2)) / 6.0;
    // double yaw_at_end = (x(N-3) + 4*x(N-2) + x(N-1)) / 6.0;
    // ROS_INFO("[YawOpt] start_yaw=%.2f, end_yaw=%.2f, final_target=%.2f", 
    //          start_yaw, end_yaw, final_target);
    // ROS_INFO("[YawOpt] Result: yaw(0)=%.2f, yaw(T)=%.2f, error=%.2f deg", 
    //          yaw_at_start, yaw_at_end, (end_yaw - yaw_at_end) * 180.0 / M_PI);
    
    return true;
  }
  
  return false;
}

bool YawOptimizer::solveQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& f, Eigen::VectorXd& x)
{
  // Dense solver: LDLT
  Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
  if (ldlt.info() != Eigen::Success) {
    // Try adding regularization
    Eigen::MatrixXd H_reg = H + 1e-4 * Eigen::MatrixXd::Identity(H.rows(), H.cols());
    ldlt.compute(H_reg);
    if (ldlt.info() != Eigen::Success) {
      std::cerr << "YawOptimizer: QP Solver Failed!" << std::endl;
      return false;
    }
  }
  // [关键修复] 修正 QP 求解公式
  // 代价函数：J = x^T H x + f^T x
  // 梯度：dJ/dx = 2Hx + f = 0
  // 解：x = -H^{-1} * f / 2
  // 之前代码用的是 x = -H^{-1} * f，误差因子正好是 2
  x = ldlt.solve(-f / 2.0);
  return true;
}

double YawOptimizer::evaluateYaw(const std::vector<double>& ctrl_pts, double t, double total_dur)
{
  if (ctrl_pts.empty()) return 0.0;
  
  int M = ctrl_pts.size() - 3;
  double dt = total_dur / M;
  
  int i = (int)(t / dt);
  if (i < 0) i = 0;
  if (i >= M) i = M - 1;
  
  double u = (t - i * dt) / dt;
  
  Eigen::Vector4d val, grad;
  getBasisValues(u, val, grad);
  
  double yaw = 0.0;
  for(int k=0; k<4; ++k) {
    yaw += val(k) * ctrl_pts[i+k];
  }
  
  // [关键修复] 不在此处归一化！
  // B-spline 控制点在展开的角度空间中优化，
  // 归一化会在跨越 ±π 边界时产生突变。
  // 调用者 (calculate_yaw / publishCmd) 负责最终归一化。
  return yaw;
}

double YawOptimizer::evaluateYawDot(const std::vector<double>& ctrl_pts, double t, double total_dur)
{
  if (ctrl_pts.empty()) return 0.0;
  
  int M = ctrl_pts.size() - 3;
  double dt = total_dur / M;
  
  int i = (int)(t / dt);
  if (i < 0) i = 0;
  if (i >= M) i = M - 1;
  
  double u = (t - i * dt) / dt;
  
  Eigen::Vector4d val, grad;
  getBasisValues(u, val, grad);
  
  double yaw_dot = 0.0;
  for(int k=0; k<4; ++k) {
    yaw_dot += grad(k) * ctrl_pts[i+k];
  }
  
  return yaw_dot / dt; // Chain rule
}

} // namespace ego_planner
