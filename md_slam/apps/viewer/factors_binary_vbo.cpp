#include "factors_binary_vbo.h"
#include <iostream>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3.h>

namespace srrg2_core {
  using namespace srrg2_solver;
  using namespace std;
  ShaderBasePtr FactorsBinaryVBO::getShader() {
    if (!_my_shader) {
      _my_shader.reset(new ShaderBase(vertex_shader_source, fragment_shader_source));
    }
    return _my_shader;
  }

  FactorsBinaryVBO::FactorsBinaryVBO(FactorGraph& graph_) : VBOBase(getShader()), _graph(graph_) {
    update();
  }

  void FactorsBinaryVBO::update() {
    if (_gl_vertex_array)
      glDeleteVertexArrays(1, &_gl_vertex_array);
    if (_gl_vertex_buffer)
      glDeleteBuffers(1, &_gl_vertex_buffer);
    // resize the number of lines;
    _line_endpoints.reserve(_graph.factors().size());
    _line_endpoints.clear();
    for (auto f_it : _graph.factors()) {
      FactorBase* f = f_it.second;
      if (f->numVariables() == 2) {
        VariableSE3Base* v0 = dynamic_cast<VariableSE3Base*>(f->variable(0));
        if (!v0)
          continue;
        VariableSE3Base* v1 = dynamic_cast<VariableSE3Base*>(f->variable(1));
        if (!v1)
          continue;
        _line_endpoints.push_back(v0->estimate().translation());
        _line_endpoints.push_back(v1->estimate().translation());
      }
    }
    glGenVertexArrays(1, &_gl_vertex_array);
    glGenBuffers(1, &_gl_vertex_buffer);
    glBindVertexArray(_gl_vertex_array);
    glBindBuffer(GL_ARRAY_BUFFER, _gl_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(Vector3f) * _line_endpoints.size(),
                 &_line_endpoints[0](0),
                 GL_STATIC_DRAW);
    glBindVertexArray(_gl_vertex_array);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3f), 0);
  }

  FactorsBinaryVBO::~FactorsBinaryVBO() {
    cerr << __PRETTY_FUNCTION__ << this << " dtor" << endl;
    glDeleteVertexArrays(1, &_gl_vertex_array);
    glDeleteBuffers(1, &_gl_vertex_buffer);
  }

  void FactorsBinaryVBO::draw(const Eigen::Matrix4f& projection,
                              const Eigen::Matrix4f& model_pose,
                              const Eigen::Matrix4f& object_pose,
                              const Eigen::Vector3f& light_direction) {
    callShader(projection, model_pose, object_pose, light_direction);
    glBindVertexArray(_gl_vertex_array);
    glEnableVertexAttribArray(0);
    glDrawArrays(GL_LINE_STRIP, 0, _line_endpoints.size());
  }

  ShaderBasePtr FactorsBinaryVBO::_my_shader;

  const char* FactorsBinaryVBO::vertex_shader_source =
    "#version 330 core\n"
    "#extension GL_ARB_separate_shader_objects : enable\n"
    "layout (location = 0) in vec3 coords;\n"
    "uniform mat4 model_pose;\n"
    "uniform mat4 object_pose;\n"
    "uniform mat4 projection;\n"
    "uniform vec3 light_direction;\n"
    "void main()\n"
    "{\n"
    "   gl_Position = projection*model_pose*object_pose*vec4(coords.x, coords.y, coords.z, 1);\n"
    "}\0";

  const char* FactorsBinaryVBO::fragment_shader_source =
    "#version 330 core\n"
    "#extension GL_ARB_separate_shader_objects : enable\n"
    "out vec4 FragColor;\n"
    "void main()\n"
    "{\n"
    "   FragColor = vec4(0,1,0, 1);\n"
    "}\n\0";

} // namespace srrg2_core
