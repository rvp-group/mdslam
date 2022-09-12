#pragma once
#include "factor_stack.h"
#include <md_slam/pyramid_message.h>
#include <srrg_solver/solver_core/solver.h>

namespace md_slam {

  class MDPairwiseAligner : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(PropertyConfigurable_<srrg2_solver::Solver>,
          solver,
          "solver running photometric alignment",
          srrg2_solver::SolverPtr(new srrg2_solver::Solver),
          nullptr);

    PARAM(PropertyFloat, omega_depth, "omega for the depth cue", 1, nullptr);

    PARAM(PropertyFloat, omega_normal, "omega for the normal cue", 1, nullptr);

    PARAM(PropertyFloat, omega_intensity, "omega for the intensity cue", 1, nullptr);

    PARAM(PropertyFloat,
          depth_rejection_threshold,
          "points with a depth error higher than this are rejected",
          0.25,
          nullptr);

    PARAM(PropertyFloat, kernel_chi_threshold, "above this chi2, kernel acts", 1, nullptr);

    MDPairwiseAligner();

    inline void setMoving(MDImagePyramid* moving_) {
      _moving = moving_;
    }

    inline void setFixed(MDImagePyramid* fixed_) {
      _fixed = fixed_;
    }

    inline void setEstimate(const Isometry3f estimate_) {
      _estimate = estimate_;
    }

    void compute();

    void initialize();

    inline const Isometry3f& estimate() const {
      return _estimate;
    }

    inline const Matrix6f& informationMatrix() const {
      return _information;
    }

  protected:
    MDFactorStack _md_factors;
    srrg2_solver::FactorGraph _pairwise_graph;
    std::shared_ptr<srrg2_solver::VariableSE3QuaternionRight> _v;
    Eigen::Isometry3f _estimate       = Eigen::Isometry3f::Identity(); // initial guess and estimate
    srrg2_core::Matrix6f _information = srrg2_core::Matrix6f::Identity();
    MDImagePyramid* _moving           = nullptr;
    MDImagePyramid* _fixed            = nullptr;
    bool is_initialized               = false;
  };

  using MDPairwiseAlignerPtr = std::shared_ptr<MDPairwiseAligner>;
} // namespace md_slam
