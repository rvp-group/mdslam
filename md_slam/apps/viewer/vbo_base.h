#pragma once
#define GL_GLEXT_PROTOTYPES 1
#include <Eigen/Core>
#include <GL/gl.h>
#include <memory>
using namespace std;

namespace srrg2_core {

  struct ShaderBase {
    unsigned int _shader_program=0;
    unsigned int _model_pose_location=0;
    unsigned int _object_pose_location=0;
    unsigned int _projection_location=0;
    unsigned int _light_direction_location=0;
    ShaderBase(const char* vertex_shader_source,
               const char* fragment_shader_source);
    virtual ~ShaderBase();
  };
  
  using ShaderBasePtr=std::shared_ptr<ShaderBase> ;
  
  struct VBOBase {
    ShaderBasePtr _shader;
    VBOBase(ShaderBasePtr shader_):
      _shader(shader_){}

    void callShader(const Eigen::Matrix4f& projection,
                    const Eigen::Matrix4f& model_pose,
                    const Eigen::Matrix4f& object_pose,
                    const Eigen::Vector3f& light_direction);
    
    virtual void draw(const Eigen::Matrix4f& projection,
                      const Eigen::Matrix4f& model_pose,
                      const Eigen::Matrix4f& object_pose,                      
                      const Eigen::Vector3f& light_direction) = 0;

  };

 
}
