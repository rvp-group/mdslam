#include "instances.h"
#include "factor.h"
#include "graph_manager.h"
#include "orientation_estimator.h"
#include "pairwise_aligner.h"
#include "tracker.h"
#include "tracker_status_message.h"
#include "tracker_viewer.h"

#include "pyramid_generator.h"
#include "pyramid_message.h"
#include "pyramid_variable_se3.h"

#include "loop_closer.h"
#include "loop_detector_base.h"
#include "loop_detector_hbst.h"
#include "loop_validator.h"


// sick we need to register this for dataset manip
#include "../../apps/dataset_manipulators/lidar_config_object.h"

namespace md_slam {

  void md_registerTypes() {
    // basic stuff
    BOSS_REGISTER_CLASS(MDPyramidGenerator);
    BOSS_REGISTER_CLASS(MDNormalComputator2DCrossProduct);
    BOSS_REGISTER_CLASS(MDImagePyramidReference);
    BOSS_REGISTER_CLASS(MDImagePyramidMessage);
    BOSS_REGISTER_CLASS(MDVariableSE3);
    // tracker stuff
    BOSS_REGISTER_CLASS(MDFactor);
    BOSS_REGISTER_CLASS(MDFactorShowAction);
    BOSS_REGISTER_CLASS(MDPairwiseAligner);
    BOSS_REGISTER_CLASS(MDTrackerStandalone);
    BOSS_REGISTER_CLASS(MDTrackerStatusMessage);
    BOSS_REGISTER_CLASS(MDTrackerViewer);
    BOSS_REGISTER_CLASS(MDGraphManager);
    BOSS_REGISTER_CLASS(MDOrientationEstimator);
    // loop closure stuff
    BOSS_REGISTER_CLASS(LoopDetectorBase);
    BOSS_REGISTER_CLASS(LoopDetectorHBST);
    BOSS_REGISTER_CLASS(LoopValidator);
    BOSS_REGISTER_CLASS(MDCloser);
    // dataset manip
    BOSS_REGISTER_CLASS(MDLidarConfiguration);
  }
} // namespace md_slam
