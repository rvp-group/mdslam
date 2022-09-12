#pragma once
#include "vbo_base.h"
#include "get_pcl_offset.h"
#include <iostream>
#include <srrg_pcl/point_types.h>

namespace srrg2_core {
  using namespace std;
  template <typename PointCloudType_>
  struct CloudVBO_: public VBOBase {
    unsigned int _gl_vertex_buffer=0, _gl_vertex_array=0;
    CloudVBO_(ShaderBasePtr shader_,
              const PointCloudType_& cloud_):
      VBOBase(shader_),
      _cloud(cloud_){
      glGenVertexArrays(1, &_gl_vertex_array);
      glGenBuffers(1,&_gl_vertex_buffer);
      glBindVertexArray(_gl_vertex_array);
      glBindBuffer(GL_ARRAY_BUFFER, _gl_vertex_buffer);
      glBufferData(GL_ARRAY_BUFFER, sizeof(typename PointCloudType_::PointType)*_cloud.size(), &(_cloud)[0], GL_STATIC_DRAW);
    }
    virtual ~CloudVBO_() {
      cerr << __PRETTY_FUNCTION__ << this << " dtor" << endl;
      glDeleteVertexArrays(1, &_gl_vertex_array);
      glDeleteBuffers(1, &_gl_vertex_buffer);
    }

    const PointCloudType_& _cloud;
  };

  struct PointNormalIntensity3fVectorCloudVBO:
    public CloudVBO_<PointNormalIntensity3fVectorCloud>
  {
    PointNormalIntensity3fVectorCloudVBO(const PointNormalIntensity3fVectorCloud& cloud_);
    void draw(const Eigen::Matrix4f& projection,
              const Eigen::Matrix4f& model_pose,
              const Eigen::Matrix4f& object_pose,
              const Eigen::Vector3f& light_direction) override;
  protected:
    static ShaderBasePtr _my_shader;
    static const char *_vertex_shader_source;
    static const char *_fragment_shader_source;
    static ShaderBasePtr getShader();
  };
}
