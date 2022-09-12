#include "vbo_base.h"
#include <iostream>
#include <string>

using namespace std;

namespace srrg2_core {

  ShaderBase::ShaderBase(const char* vertex_shader_source, const char* fragment_shader_source) {
    // compile shaders
    // cerr << "ShaderBase::ctor " << this << endl;
    unsigned int vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    // cerr << "_vertex_shader_id: " << vertex_shader << endl;

    glShaderSource(vertex_shader, 1, &vertex_shader_source, NULL);
    glCompileShader(vertex_shader);
    int success;
    char info_log[512];
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      glGetShaderInfoLog(vertex_shader, 512, NULL, info_log);
      // std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << info_log << std::endl;
    }

    unsigned int fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    // cerr << "_fragment_shader_id: " << fragment_shader << endl;

    glShaderSource(fragment_shader, 1, &fragment_shader_source, NULL);
    glCompileShader(fragment_shader);
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      glGetShaderInfoLog(fragment_shader, 512, NULL, info_log);
      // std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << info_log << std::endl;
    }

    _shader_program = glCreateProgram();
    glAttachShader(_shader_program, vertex_shader);
    glAttachShader(_shader_program, fragment_shader);
    glLinkProgram(_shader_program);
    glGetProgramiv(_shader_program, GL_LINK_STATUS, &success);
    if (!success) {
      glGetProgramInfoLog(_shader_program, 512, NULL, info_log);
      // std::cout << "ERROR::SHADER::PROGRAM::LINK_FAILED\n" << info_log << std::endl;
    }
    _model_pose_location      = glGetUniformLocation(_shader_program, "model_pose");
    _object_pose_location     = glGetUniformLocation(_shader_program, "object_pose");
    _projection_location      = glGetUniformLocation(_shader_program, "projection");
    _light_direction_location = glGetUniformLocation(_shader_program, "light_direction");
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
  }

  ShaderBase::~ShaderBase() {
    // cerr << "shader " << this << " dtor" << endl;
  }

  void VBOBase::callShader(const Eigen::Matrix4f& projection,
                           const Eigen::Matrix4f& model_pose,
                           const Eigen::Matrix4f& object_pose,
                           const Eigen::Vector3f& light_direction) {
    auto ld = light_direction.normalized();
    glUseProgram(_shader->_shader_program);
    glUniformMatrix4fv(_shader->_model_pose_location, 1, GL_FALSE, model_pose.data());
    glUniformMatrix4fv(_shader->_object_pose_location, 1, GL_FALSE, object_pose.data());
    glUniformMatrix4fv(_shader->_projection_location, 1, GL_FALSE, projection.data());
    glUniform3fv(_shader->_light_direction_location, 1, ld.data());
  }

} // namespace srrg2_core
