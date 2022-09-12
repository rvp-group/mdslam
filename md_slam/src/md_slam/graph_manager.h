#pragma once
#include "loop_closer.h"
#include "pairwise_aligner.h"
#include "pyramid_message.h"
#include "pyramid_variable_se3.h"
#include "tracker_status_message.h"
#include <mutex>
#include <srrg_messages/message_handlers/message_sink_base.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_viewer/active_drawable.h>
#include <thread>

namespace md_slam {

  class MDGraphManager : public srrg2_core::MessageSinkBase, public srrg2_core::ActiveDrawable {
  public:
    MDGraphManager();
    ~MDGraphManager();

    PARAM(PropertyConfigurable_<MDPairwiseAligner>,
          pairwise_aligner,
          "photometric pairwise aligner, takes 2 pyramids gives you a pose",
          nullptr,
          nullptr);

    PARAM(srrg2_core::PropertyConfigurable_<srrg2_solver::Solver>,
          solver,
          "pose-graph solver",
          nullptr,
          nullptr);

    PARAM(srrg2_core::PropertyConfigurable_<MDCloser>,
          closer,
          "closer manager for md slam",
          nullptr,
          nullptr);

    PARAM(srrg2_core::PropertyString,
          status_topic,
          "tracker status",
          "/md_tracker_status",
          nullptr);

    PARAM(PropertyString,
          tf_dyn_topic,
          "topic where to push the dynamic transforms",
          "/tf",
          nullptr);

    PARAM(PropertyString, map_frame_id, "map frame id of the graph", "/md_map", nullptr);

    PARAM(srrg2_core::PropertyBool, enable_closures, "enable closures", true, nullptr);

    PARAM(PropertyFloat, angle_check, "loop clousure neighboroud check on angle", 7e-2, nullptr);
    PARAM(PropertyFloat,
          translation_check,
          "loop clousure neighboroud check on norm of translation",
          7e-2,
          nullptr);

    void reset() override;
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;
    void closureCallback();
    void publishTransform();

    inline void quitClosureThread() {
      _quit_closure_thread = true;
    }

    inline std::thread& closureThread() {
      return _closure_thread;
    }
    Matrix6f
    photometricClosure(Isometry3f& estimate_, MDImagePyramid* fixed_, MDImagePyramid* moving_);
    inline srrg2_solver::FactorGraphPtr graph() {
      return _graph;
    }
    inline std::mutex& graphMutex() {
      return _mutex;
    }

  protected:
    bool cmdSaveGraph(std::string& response, const std::string& filename);
    void _drawImpl(srrg2_core::ViewerCanvasPtr gl_canvas_) const override;
    mutable srrg2_solver::FactorGraphPtr _graph;

    MDTrackerStatusMessagePtr _status_msg;
    MDImagePyramidMessagePtr _pyramid_msg;
    MDVariableSE3Ptr _previous_variable;
    std::string _pyramid_topic  = "";
    mutable bool _lists_created = false;
    bool _initial_var           = true;
    int _max_id                 = 0;
    // closure thread stuff
    mutable bool _quit_closure_thread = false;
    std::thread _closure_thread;
    mutable bool _is_closure_valid = false;
    std::mutex _mutex;
    std::unique_ptr<std::queue<MDVariableSE3Ptr>> _variable_buffer_queue;
  };

  //   extern std::atomic<bool> QUIT_REQUESTED;
} // namespace md_slam
