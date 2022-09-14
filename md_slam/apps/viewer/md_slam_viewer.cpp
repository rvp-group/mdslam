#include "md_slam_viewer.h"
#include <QGLViewer/camera.h>

namespace srrg2_core {

  using namespace srrg2_solver;
  using namespace md_slam;
  using namespace std;

  MDViewer::MDViewer(FactorGraphPtr graph_, std::mutex& proc_mutex_) :
    _graph(graph_),
    _proc_mutex(proc_mutex_) {
  }

  void MDViewer::init() {
    QGLViewer::init();

    // set background color white
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // set some default settings
    // glEnable(GL_LINE_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_FLAT);
    setSceneRadius(100);
    glEnable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // don't save state
    setStateFileName(QString::null);

    cerr << "instantiating required VBO" << endl;
    _proc_mutex.lock();
    _graph_vbo.reset(new DrawableFactorGraphVBO(_graph.get()));
    _proc_mutex.unlock();
    // mouse bindings
    setMouseBinding(Qt::NoModifier, Qt::RightButton, CAMERA, ZOOM);
    setMouseBinding(Qt::NoModifier, Qt::MidButton, CAMERA, TRANSLATE);
    setMouseBinding(Qt::ControlModifier, Qt::LeftButton, RAP_FROM_PIXEL);
    setKeyDescription(Qt::Key_C, "Draw/undraw pointcloud");
  }

  void MDViewer::keyPressEvent(QKeyEvent* e) {
    // defines the showing cloud shortcut
    if (e->key() == Qt::Key_C) {
      _draw_cloud = !_draw_cloud;
    } else
      QGLViewer::keyPressEvent(e);
  }

  void MDViewer::draw() {
    QGLViewer::draw();
    _proc_mutex.lock();
    _graph_vbo->update();
    Eigen::Matrix4f model;
    Eigen::Matrix4f projection;
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    Eigen::Vector3f light_direction(0.5, 0.5, -0.5);
    light_direction.normalize();
    glGetFloatv(GL_MODELVIEW_MATRIX, model.data());
    glGetFloatv(GL_PROJECTION_MATRIX, projection.data());
    Eigen::Matrix4f temp   = Eigen::Matrix4f::Identity();
    temp.block<3, 1>(0, 3) = _camera_pose.matrix().inverse().block<3, 1>(0, 3);
    _camera_pose(2, 3)     = 0;
    model                  = model * _camera_pose.matrix().inverse();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    _graph_vbo->draw(projection, model, mat, light_direction, _draw_cloud);
    _proc_mutex.unlock();
  }

  std::shared_ptr<DrawableFactorGraphVBO> _graph_vbo;
  MDViewer::~MDViewer() {
    cerr << "viewer Dtor" << endl;
  }

} // namespace srrg2_core
