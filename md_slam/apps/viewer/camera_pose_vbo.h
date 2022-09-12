#pragma once
#include "vbo_base.h"
#include <iostream>
#include <Eigen/Core>

namespace srrg2_core {

  struct CameraPoseVBO: public VBOBase {
    CameraPoseVBO(float width, float height);
    ~CameraPoseVBO();
  
    void draw(const Eigen::Matrix4f& projection,
              const Eigen::Matrix4f& model_pose,
              const Eigen::Matrix4f& object_pose,
              const Eigen::Vector3f& light_direction) override;

  protected:
    static const char* vertex_shader_source;
    static const char* fragment_shader_source;
    static ShaderBasePtr _my_shader;
    static constexpr float _unscaled_vertices[]={
      0,  0, -1,
      -1, -1, 0,
      1, -1, 0,
    
      0,  0, -1,
      1, -1, 0,
      1,  1, 0,

      0, 0, -1,
      1, 1, 0,
      -1, 1, 0,

      0,  0, -1,
      -1,  1, 0,
      -1, -1, 0
    };
    static constexpr int num_coordinates=sizeof(_unscaled_vertices)/(sizeof(float));
    static constexpr int num_vertices=num_coordinates/3;
    static  float _vertices[num_coordinates];
    unsigned int _gl_vertex_buffer=0, _gl_vertex_array=0;
    static ShaderBasePtr getShader();  

  };

}
