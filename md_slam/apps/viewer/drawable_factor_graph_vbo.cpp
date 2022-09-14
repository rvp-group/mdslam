#include "drawable_factor_graph_vbo.h"

namespace srrg2_core {

  DrawableFactorGraphVBO::DrawableFactorGraphVBO(FactorGraph* instance_) :
    DrawableVBO_<FactorGraph*>(instance_) {
    if (!_instance)
      return;
    _factors_binary_vbo.reset(new FactorsBinaryVBO(*_instance));
  }

  void DrawableFactorGraphVBO::init() {
  }

  void DrawableFactorGraphVBO::update() {
    if (!_instance && _variables_vbo.size()) {
      _variables_vbo.clear();
      return;
    }
    if (_factors_binary_vbo)
      _factors_binary_vbo->update();
    // scan the variables in viewer, and remove the ones that are not anymore in the game
    for (auto v_it = _variables_vbo.begin(); v_it != _variables_vbo.end(); ++v_it) {
      if (!_instance->variable(v_it->first)) {
        std::cerr << "erasing VBO" << std::endl;
        auto v_erased = v_it;
        ++v_it;
        _variables_vbo.erase(v_erased);
      }
    }
    // scan the variables in the graph, and add those that are new
    for (auto v_it : _instance->variables()) {
      VariableBase* v_base = v_it.second;
      MDVariableSE3* v     = dynamic_cast<MDVariableSE3*>(v_base);
      if (!v)
        continue;
      if (!_variables_vbo.count(v_it.first)) {
        _variables_vbo.insert(std::make_pair(
          v_it.first,
          std::shared_ptr<DrawablePyramidVariableSE3VBO>(new DrawablePyramidVariableSE3VBO(v))));
      }
    }
  }

  void DrawableFactorGraphVBO::draw(const Eigen::Matrix4f& projection,
                                    const Eigen::Matrix4f& model_pose,
                                    const Eigen::Matrix4f& object_pose,
                                    const Eigen::Vector3f& light_direction,
                                    const bool draw_cloud) {
    for (auto v_it : _variables_vbo) {
      v_it.second->draw(projection, model_pose, object_pose, light_direction, draw_cloud);
    }
    if (_factors_binary_vbo)
      _factors_binary_vbo->draw(projection, model_pose, object_pose, light_direction);
  }

} // namespace srrg2_core
