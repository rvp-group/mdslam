#include <iostream>
#include "camera_pose_vbo.h"
namespace srrg2_core {
  using namespace std;

  ShaderBasePtr CameraPoseVBO::getShader() {
    if (! _my_shader) {
      _my_shader.reset(new ShaderBase(vertex_shader_source, fragment_shader_source));
    }
    return _my_shader;
  }
  
  CameraPoseVBO::CameraPoseVBO(float width, float height):
    VBOBase(getShader())
  {
    cerr << "num_coordinates: " << num_coordinates << endl;
    cerr << "num_vertices: " << num_vertices << endl;
    
    for (int i=0; i<num_coordinates; i+=3) {
      _vertices[i] = _unscaled_vertices[i]*width/2;
      _vertices[i+1] = _unscaled_vertices[i+1]*width/2;
      _vertices[i+2] = _unscaled_vertices[i+2]*height;
    }
    glGenVertexArrays(1, &_gl_vertex_array);
    glGenBuffers(1,&_gl_vertex_buffer);
    glBindVertexArray(_gl_vertex_array);
    glBindBuffer(GL_ARRAY_BUFFER, _gl_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_vertices), _vertices, GL_STATIC_DRAW);
    glBindVertexArray(_gl_vertex_array);
    glVertexAttribPointer(0,3,GL_FLOAT, GL_FALSE,
                          3*sizeof(float),
                          0);
    
  }
  CameraPoseVBO::~CameraPoseVBO() {
    cerr << __PRETTY_FUNCTION__ << this << " dtor" << endl;
    glDeleteVertexArrays(1, &_gl_vertex_array);
    glDeleteBuffers(1, &_gl_vertex_buffer);
  }
  
  void CameraPoseVBO::draw(const Eigen::Matrix4f& projection,
            const Eigen::Matrix4f& model_pose,
            const Eigen::Matrix4f& object_pose,
            const Eigen::Vector3f& light_direction)   {
    callShader(projection, model_pose, object_pose, light_direction);
    glBindVertexArray(_gl_vertex_array);
    glEnableVertexAttribArray(0);
    glLineWidth(3);
    glDrawArrays(GL_LINE_STRIP, 0, num_vertices);
  }

  ShaderBasePtr CameraPoseVBO::_my_shader;

  float  CameraPoseVBO::_vertices[CameraPoseVBO::num_coordinates];

  const char* CameraPoseVBO::vertex_shader_source=
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

  const char* CameraPoseVBO::fragment_shader_source =
    "#version 330 core\n"
    "#extension GL_ARB_separate_shader_objects : enable\n"
    "out vec4 FragColor;\n"
    "void main()\n"
    "{\n"
    "   FragColor = vec4(1,0,0, 1);\n"
    "}\n\0";

}
