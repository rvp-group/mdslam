#pragma once
#include "image_pyramid.h"
#include <srrg_solver/variables_and_factors/types_3d/variable_se3.h>

namespace md_slam {
  class MDVariableSE3 : public srrg2_solver::VariableSE3QuaternionRight {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MDImagePyramid* pyramid() {
      return _pyramid.fetch();
    }
    void setPyramid(MDImagePyramid* pyramid_) {
      _pyramid.set(pyramid_);
    }
    void serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context);
    void deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context);
    const double& timestamp() const {
      return _timestamp;
    }

  protected:
    double _timestamp;
    MDImagePyramidReference _pyramid;
  };

  using MDVariableSE3Ptr = std::shared_ptr<MDVariableSE3>;
} // namespace md_slam
