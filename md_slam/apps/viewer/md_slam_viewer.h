#pragma once
#include "drawable_factor_graph_vbo.h"
#include <QGLViewer/qglviewer.h>
#include <mutex>

namespace srrg2_core {

  using namespace srrg2_solver;
  using namespace md_slam;
  using namespace std;

  class MDViewer : public QGLViewer {
  public:
    MDViewer(FactorGraphPtr graph_, std::mutex& proc_mutex_);
    void init() override;
    void draw() override;
    virtual ~MDViewer();
    void setCamera(const Eigen::Isometry3f& camera_pose_) {
      _camera_pose = camera_pose_;
    };
    FactorGraphPtr _graph;

  protected:
    std::shared_ptr<DrawableFactorGraphVBO> _graph_vbo;
    std::mutex& _proc_mutex;
    Eigen::Isometry3f _camera_pose = Eigen::Isometry3f::Identity();
  };

  using MDViewerPtr = std::shared_ptr<MDViewer>;

} // namespace srrg2_core
