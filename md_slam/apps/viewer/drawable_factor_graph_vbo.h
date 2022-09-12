#pragma once
#include "camera_pose_vbo.h"
#include "cloud_vbo.h"
#include "drawable_pyramid_variable_se3.h"
#include "factors_binary_vbo.h"

namespace srrg2_core {

  using namespace srrg2_solver;
  using namespace md_slam;
  using namespace std;

  struct DrawableFactorGraphVBO : public DrawableVBO_<FactorGraph*> {
    DrawableFactorGraphVBO(FactorGraph* instance_);
    void init() override;

    void update() override;

    void draw(const Eigen::Matrix4f& projection,
              const Eigen::Matrix4f& model_pose,
              const Eigen::Matrix4f& object_pose,
              const Eigen::Vector3f& light_direction) override;

    std::shared_ptr<FactorsBinaryVBO> _factors_binary_vbo;
    std::map<VariableBase::Id, std::shared_ptr<DrawablePyramidVariableSE3VBO>> _variables_vbo;
  };

} // namespace srrg2_core
