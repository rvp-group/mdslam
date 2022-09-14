#include "drawable_pyramid_variable_se3.h"

namespace srrg2_core {
  DrawablePyramidVariableSE3VBO::DrawablePyramidVariableSE3VBO(MDVariableSE3* instance_) :
    DrawableVBO_<MDVariableSE3*>(instance_) {
    if (!_instance)
      return;

    if (_cloud_vbo)
      return;

    auto pyramid = _instance->pyramid();
    if (!pyramid) {
      cerr << "unable to load pyramid, aborting" << endl;
      return;
    }
    // get the highest level
    const auto& l = pyramid->front();
    MDMatrixCloud matrix_cloud;
    l->toCloud(matrix_cloud);

    std::back_insert_iterator<PointNormalIntensity3fVectorCloud> dest(cloud);
    matrix_cloud.copyTo(dest);
    //_instance->setPyramid(0);
    _cloud_vbo.reset(new PointNormalIntensity3fVectorCloudVBO(cloud));
    // cerr << "variable_id: " << _instance->graphId() << " cloud_vbo_id: " <<
    // _cloud_vbo->_gl_vertex_buffer << " cloud_vbo_aid: " << _cloud_vbo->_gl_vertex_array << endl;
    cerr << "current cloud size: " << cloud.size() << endl;
    if (!_camera_pose_vbo) {
      _camera_pose_vbo.reset(new CameraPoseVBO(0.1, 0.1));
    }
  }

  void DrawablePyramidVariableSE3VBO::init() {
  }
  void DrawablePyramidVariableSE3VBO::update() {
  }
  void DrawablePyramidVariableSE3VBO::draw(const Eigen::Matrix4f& projection,
                                           const Eigen::Matrix4f& model_pose,
                                           const Eigen::Matrix4f& object_pose,
                                           const Eigen::Vector3f& light_direction,
                                           const bool draw_cloud) {
    if (!(_cloud_vbo && _camera_pose_vbo))
      return;

    Eigen::Matrix4f this_object_pose = object_pose * _instance->estimate().matrix();
    if (draw_cloud)
      _cloud_vbo->draw(projection, model_pose, this_object_pose, light_direction);
    _camera_pose_vbo->draw(projection, model_pose, this_object_pose, light_direction);
  }

  std::shared_ptr<CameraPoseVBO> DrawablePyramidVariableSE3VBO::_camera_pose_vbo;

} // namespace srrg2_core
