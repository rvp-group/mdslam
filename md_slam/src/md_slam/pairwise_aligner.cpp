#include "pairwise_aligner.h"
namespace md_slam {

  using namespace srrg2_core;
  using namespace srrg2_solver;

  // prev fixed
  // curr moving

  MDPairwiseAligner::MDPairwiseAligner() {
    param_solver.setValue(SolverPtr(new Solver));
    auto term_crit = std::dynamic_pointer_cast<SimpleTerminationCriteria>(
      param_solver->param_termination_criteria.value());
    term_crit->param_epsilon.setValue(1e-5);
    param_solver->param_max_iterations.value() = vector<int>{10, 20, 50};

    _v.reset(new VariableSE3QuaternionRight);
    _v->setGraphId(0);
    _v->setEstimate(Eigen::Isometry3f::Identity());
    _pairwise_graph.addVariable(_v);
  }

  void MDPairwiseAligner::initialize() {
    for (auto& action : param_solver->param_actions.value()) {
      MDFactorShowActionPtr show_action = std::dynamic_pointer_cast<MDFactorShowAction>(action);
      if (show_action)
        show_action->setFactors(_md_factors);
    }
    _md_factors.setFixed(*_fixed); // previous in tracking
    _md_factors.makeFactors();
    _md_factors.setVariableId(_v->graphId());
    _md_factors.addFactors(_pairwise_graph);
    for (auto& f : _md_factors) {
      f->setOmegaDepth(param_omega_depth.value());
      f->setOmegaIntensity(param_omega_intensity.value());
      f->setOmegaNormals(param_omega_normal.value());
      f->setDepthRejectionThreshold(param_depth_rejection_threshold.value());
      f->setKernelChiThreshold(param_kernel_chi_threshold.value());
    }
  }

  void MDPairwiseAligner::compute() {
    _v->setEstimate(_estimate);
    _md_factors.setFixed(*_fixed);
    _md_factors.setMoving(*_moving);
    _md_factors.assignPyramids();
    param_solver->setGraph(_pairwise_graph);
    param_solver->compute();
    // get data off
    _estimate    = _v->estimate();
    _information = param_solver->extractFisherInformationBlock(*_v);
  }
} // namespace md_slam
