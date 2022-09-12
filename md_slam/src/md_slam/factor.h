#pragma once
#include "factor_common.h"
#include "image_pyramid.h"
#include <srrg_solver/solver_core/factor.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3.h>
#include <srrg_system_utils/chrono.h>
#include <srrg_viewer/active_drawable.h>
namespace md_slam {
  using namespace srrg2_core;

  class MDFactor : public MDFactorCommon,
                   public srrg2_solver::Factor_<
                     srrg2_solver::VariablePtrTuple_<srrg2_solver::VariableSE3QuaternionRight>> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! populate H and b with measurments contributions
    void compute(bool chi_only = false, bool force = false) override;

    //! return false if the computation has encountered some problem
    bool isValid() const override;

    int measurementDim() const override;

    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;

  protected:
    //! compute the pieces needed for the minimization
    inline void linearize(bool chi_only = false);
    //! error and Jacobian computation
    inline PointStatusFlag errorAndJacobian(srrg2_core::Vector5f& e_,
                                            srrg2_core::Matrix5_6f& J_,
                                            WorkspaceEntry& entry_,
                                            bool chi_only);
    Matrix3_6f _J_icp = Matrix3_6f::Zero();
  };

  using MDFactorPtr = std::shared_ptr<MDFactor>;

} // namespace md_slam
