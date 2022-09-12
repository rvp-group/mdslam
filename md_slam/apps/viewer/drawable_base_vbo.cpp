#include "camera_pose_vbo.h"
#include "cloud_vbo.h"
#include "md_slam/pyramid_variable_se3.h"
#include <Eigen/Core>

namespace srrg2_core {
  struct DrawablePyramidVariableSE3VBO : public DrawableVBO_<MDVariableSE3*> {
    DrawablePyramidVariableSE3VBO(MDVariableSE3* instance_) :
      DrawableVBO_<MDVariableSE3*>(instance_) {
      if (!_instance)
        return;

      if (_cloud_vbo)
        return;

      auto pyramid = _instance->pyramid();
      if (!pyramid) {
        std::cerr << "unable to load pyramid, aborting" << std::endl;
        return;
      }
      // get the highest level
      const auto& l = pyramid->front();
      MDMatrixCloud matrix_cloud;
      l->toCloud(matrix_cloud);

      std::back_insert_iterator<PointNormalIntensity3fVectorCloud> dest(cloud);
      matrix_cloud.copyTo(dest);
      _instance->setPyramid(0);
      _cloud_vbo.reset(new PointNormalIntensity3fVectorCloudVBO(cloud));
      // cerr << "variable_id: " << _instance->graphId()
      //      << " cloud_vbo_id: " << _cloud_vbo->_gl_vertex_buffer
      //      << " cloud_vbo_aid: " << _cloud_vbo->_gl_vertex_array << endl;
      // cerr << "matrix_cloud.size() =" << cloud.size() << endl;
      if (!_camera_pose_vbo) {
        _camera_pose_vbo.reset(new CameraPoseVBO(0.1, 0.1));
      }
    }

    void init() override {
    }
    void update() override {
    }
    void draw(const Eigen::Matrix4f& projection,
              const Eigen::Matrix4f& model,
              const Eigen::Vector3f& light_direction) override {
      if (!(_cloud_vbo && _camera_pose_vbo))
        return;
      Eigen::Matrix4f m = model * _instance->estimate().matrix();
      _cloud_vbo->draw(projection, m, light_direction);
      _camera_pose_vbo->draw(projection, m, light_direction);
    }
    std::shared_ptr<PointNormalIntensity3fVectorCloudVBO> _cloud_vbo;
    static std::shared_ptr<CameraPoseVBO> _camera_pose_vbo;
    PointNormalIntensity3fVectorCloud cloud;
  };
