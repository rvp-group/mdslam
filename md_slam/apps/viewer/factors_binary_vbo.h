#pragma once
#include "vbo_base.h"
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <iostream>
#include <srrg_pcl/point_types.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include <vector>

namespace srrg2_core {
  struct FactorsBinaryVBO : public VBOBase {
    using Vector3fVector = std::vector<Vector3f, Eigen::aligned_allocator<Vector3f>>;

    FactorsBinaryVBO(srrg2_solver::FactorGraph& graph);
    ~FactorsBinaryVBO();

    void draw(const Eigen::Matrix4f& projection,
              const Eigen::Matrix4f& model_pose,
              const Eigen::Matrix4f& object_pose,
              const Eigen::Vector3f& light_direction);
    void update();

  protected:
    static const char* vertex_shader_source;
    static const char* fragment_shader_source;
    static ShaderBasePtr _my_shader;
    Vector3fVector _line_endpoints;
    srrg2_solver::FactorGraph& _graph;
    static ShaderBasePtr getShader();
    unsigned int _gl_vertex_buffer = 0, _gl_vertex_array = 0;
  };

} // namespace srrg2_core
