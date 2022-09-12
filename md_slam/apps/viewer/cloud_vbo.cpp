#include "cloud_vbo.h"
namespace srrg2_core {
  using namespace std;

  ShaderBasePtr PointNormalIntensity3fVectorCloudVBO::_my_shader;

  const char* PointNormalIntensity3fVectorCloudVBO::_vertex_shader_source =
    "#version 330 core\n"
    "#extension GL_ARB_separate_shader_objects : enable\n"
    "layout (location = 0) in vec3 coords;\n"
    "layout (location = 1) in vec3 normal;\n"
    "layout (location = 2) in float intensity;\n"
    "uniform mat4 model_pose;\n"
    "uniform mat4 object_pose;\n"
    "uniform mat4 projection;\n"
    "uniform vec3 light_direction;\n"
    "out vec3 my_color;\n"
    "void main()\n"
    "{\n"
    "   gl_Position = projection*model_pose*object_pose*vec4(coords.x, coords.y, coords.z, 1);\n"
    "   mat3 R = mat3(object_pose);\n"
    "   vec3 n = R*normal;\n"
    " float i = intensity; \n" // RGBD
    //"   float i = clamp(dot(n, light_direction), 0.1, 0.7); \n" // lidar
    "   my_color = vec3(i,i,i);\n"
    "   //my_color = n;\n"
    "}\0";

  //    "   float i = intensity;\n"
  //    "   float i = intensity*clamp(dot(normal, light_direction), 0.1, 1);\n"

  const char* PointNormalIntensity3fVectorCloudVBO::_fragment_shader_source =
    "#version 330 core\n"
    "#extension GL_ARB_separate_shader_objects : enable\n"
    "out vec4 FragColor;\n"
    "in vec3  my_color;\n"
    "void main()\n"
    "{\n"
    "   FragColor = vec4(my_color, 0.5);\n" // RGBD
    //"   FragColor = vec4(my_color, 0.3);\n" // lidar
    "}\n\0";

  ShaderBasePtr PointNormalIntensity3fVectorCloudVBO::getShader() {
    if (!_my_shader)
      _my_shader.reset(new ShaderBase(_vertex_shader_source, _fragment_shader_source));
    return _my_shader;
  }

  PointNormalIntensity3fVectorCloudVBO::PointNormalIntensity3fVectorCloudVBO(
    const PointNormalIntensity3fVectorCloud& cloud_) :
    CloudVBO_<PointNormalIntensity3fVectorCloud>(getShader(), cloud_) {
    glBindVertexArray(_gl_vertex_array);
    glVertexAttribPointer(0,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          sizeof(PointNormalIntensity3fVectorCloud::PointType),
                          (const void*) field_offset_<PointNormalIntensity3f, 0>);

    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          sizeof(PointNormalIntensity3fVectorCloud::PointType),
                          (const void*) field_offset_<PointNormalIntensity3f, 1>);

    glVertexAttribPointer(2,
                          1,
                          GL_FLOAT,
                          GL_FALSE,
                          sizeof(PointNormalIntensity3fVectorCloud::PointType),
                          (const void*) field_offset_<PointNormalIntensity3f, 2>);
    // glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

  void PointNormalIntensity3fVectorCloudVBO::draw(const Eigen::Matrix4f& projection,
                                                  const Eigen::Matrix4f& model_pose,
                                                  const Eigen::Matrix4f& object_pose,
                                                  const Eigen::Vector3f& light_direction) {
    callShader(projection, model_pose, object_pose, light_direction);
    glBindVertexArray(_gl_vertex_array);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    glDrawArrays(GL_POINTS, 0, _cloud.size());
  }
} // namespace srrg2_core
