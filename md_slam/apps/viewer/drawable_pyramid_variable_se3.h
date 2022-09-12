#pragma once
#include "camera_pose_vbo.h"
#include "cloud_vbo.h"
#include "drawable_base_vbo.h"
#include <md_slam/pyramid_variable_se3.h>
#include <md_slam/utils.h>

namespace srrg2_core {
  using namespace srrg2_core;
  using namespace md_slam;
  using namespace std;

  struct DrawablePyramidVariableSE3VBO : public DrawableVBO_<MDVariableSE3*> {
    DrawablePyramidVariableSE3VBO(MDVariableSE3* instance_);
    void init() override;
    void update() override;
    void draw(const Eigen::Matrix4f& projection,
              const Eigen::Matrix4f& model_pose,
              const Eigen::Matrix4f& object_pose,
              const Eigen::Vector3f& light_direction) override;

    std::shared_ptr<PointNormalIntensity3fVectorCloudVBO> _cloud_vbo;
    static std::shared_ptr<CameraPoseVBO> _camera_pose_vbo;
    PointNormalIntensity3fVectorCloud cloud;
  };
} // namespace srrg2_core
