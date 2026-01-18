#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include "ego_planner/rog_astar.h"
#include "ego_planner/plan_container.hpp"
#include "ego_planner/optimizer/lbfgs.hpp"
#include "ego_planner/optimizer/poly_traj_utils.hpp"
// 使用新的 STAR-MVF 地图系统
#include "star_mvf/star_mvf.h"

namespace ego_planner
{

  class ConstraintPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the start of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.

    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();

      points.resize(3, size_set);
      base_point.resize(cp_size);
      direction.resize(cp_size);
      flag_temp.resize(cp_size);
    }

    void segment(ConstraintPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= cp_size || points.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize_cp(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.cp_size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];
      }
    }

    static inline int two_thirds_id(Eigen::MatrixXd &points, const bool touch_goal)
    {
      return touch_goal ? points.cols() - 1 : points.cols() - 1 - (points.cols() - 2) / 3;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {

  private:
    star_mvf::MapManager::Ptr map_manager_;
    AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;
    ConstraintPoints cps_;

    int cps_num_prePiece_;   // number of distinctive constraint points each piece
    int variable_num_;       // optimization variables
    int piece_num_;          // poly traj piece numbers
    int iter_num_;           // iteration of the solver
    bool touch_goal_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_, wei_obs_soft_;      // obstacle weight
    double wei_feas_;                    // feasibility weight
    double wei_sqrvar_;                  // squared variance weight
    double wei_time_;                    // time weight
    double obs_clearance_, obs_clearance_soft_; // safe distance
    double max_vel_, max_acc_;           // dynamic limits

    /* height keeping parameters */
    bool enable_height_keeping_;         // 是否启用高度保持
    double wei_height_;                  // 高度保持权重
    double desired_height_;              // 期望高度 [m]
    double height_tolerance_;            // 高度容差 [m]

    double t_now_;
    
    // Map resolution for path checking
    double resolution_;

  public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    enum CHK_RET
    {
      OBS_FREE,
      ERR,
      FINISH
    };

    /* set variables */
    void setParam(ros::NodeHandle &nh);
    void setEnvironment(const star_mvf::MapManager::Ptr &map);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setIfTouchGoal(const bool touch_goal);
    void setConstraintPoints(ConstraintPoints cps);
    void setDesiredHeight(double height);  // 设置期望高度（用于高度保持）
    void setEnableHeightKeeping(bool enable);  // 启用/禁用高度保持

    /* helper functions */
    inline const ConstraintPoints &getControlPoints(void) { return cps_; }
    inline const poly_traj::MinJerkOpt &getMinJerkOpt(void) { return jerkOpt_; }
    inline int get_cps_num_prePiece_(void) { return cps_num_prePiece_; }
    inline double getResolution() const { return resolution_; }

    /* main planning API */
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            Eigen::MatrixXd &optimal_points, double &final_cost);

    bool computePointsToCheck(poly_traj::Trajectory &traj, int id_end, PtsChk_t &pts_check);

    /* check collision and set {p,v} pairs to constraint points */
    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              const poly_traj::MinJerkOpt &pt_data,
                                              const bool flag_first_init /*= true*/);

    bool roughlyCheckConstraintPoints(void);

    /* multi-topo support */
    std::vector<ConstraintPoints> distinctiveTrajs(std::vector<std::pair<int, int>> segments);
    
    /* occupancy check using map_manager */
    inline bool getInflateOccupancy(const Eigen::Vector3d &pos) const
    {
      // 转换为 float 类型以兼容 star_mvf::Position
      star_mvf::Position pos_f(static_cast<float>(pos.x()), 
                               static_cast<float>(pos.y()), 
                               static_cast<float>(pos.z()));
      return map_manager_->isOccupiedInflate(pos_f);
    }

  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector3d &p,
                           Eigen::Vector3d &gradp,
                           double &costp);

    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    bool heightKeepingGradCostP(const Eigen::Vector3d &p,
                                Eigen::Vector3d &gradp,
                                double &costp);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

  public:
    typedef std::unique_ptr<PolyTrajOptimizer> Ptr;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

} // namespace ego_planner
#endif
