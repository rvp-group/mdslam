#pragma once
#include "factor_stack.h"
#include "orientation_estimator.h"
#include "pairwise_aligner.h"
#include "pyramid_generator.h"
#include "pyramid_message.h"
#include <srrg_messages/message_handlers/message_sink_base.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_property/property_eigen.h>
#include <srrg_solver/solver_core/solver.h>

namespace md_slam {

  class MDTrackerStandalone : public MessageSinkBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(PropertyConfigurable_<MDPairwiseAligner>,
          pairwise_aligner,
          "photometric pairwise aligner, takes 2 pyramids gives you a pose",
          MDPairwiseAlignerPtr(new MDPairwiseAligner),
          nullptr);

    PARAM(PropertyString, pyramid_topic, "topic of the pyramid to listen", "/md_pyramid", nullptr);

    PARAM(PropertyString, base_frame_id, "base_link_of_the_robot", "/base_link", nullptr);

    PARAM(PropertyString,
          keyframe_frame_id,
          "pose of current keyframe in md slam map origin",
          "/md_keyframe",
          nullptr);

    PARAM(PropertyString, origin_frame_id, "md_origin_frame_id", "/md_origin", nullptr);

    PARAM(PropertyString,
          local_frame_id,
          "pose of current frame in keyframe",
          "/md_local",
          nullptr);

    PARAM(PropertyString,
          tf_dyn_topic,
          "topic where to push the dynamic transforms",
          "/tf",
          nullptr);

    PARAM(PropertyInt, keyframe_steps, "how many frames between keyframes", 10, nullptr);

    PARAM(PropertyFloat,
          keyframe_translation,
          "translations above this, change keyframe",
          0.1,
          nullptr);

    PARAM(PropertyFloat, keyframe_rotation, "rotations above this, change keyframe", 0.1, nullptr);

    PARAM(PropertyBool, enable_imu, "if set to true enable imu in tracking", true, nullptr);

    MDTrackerStandalone();
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;
    void publishTransform();

    // working vars
    int _elapsed_steps = 0;
    int _num_updates   = 0;
    Eigen::Isometry3f _keyframe_t; // pose of the last keyframe in world
    Eigen::Isometry3f _local_t;    // pose of the last frame in keyframe
    Eigen::Isometry3f _estimate;   // global_pose
    MDImagePyramidMessagePtr _prev_pyr_msg;
    Chrono::ChronoMap _timings;
    double _last_update_time;

  protected:
    //     imu stuff
    bool _processImu(srrg2_core::IMUMessagePtr imu_msg_);
    std::unique_ptr<MDOrientationEstimator> _rotation_estimator_imu;
    double _prev_timestamp    = 0.0;
    bool _is_init_imu         = true; // if init first one lock timestamp for integration
    Isometry3f _prev_imu_pose = Isometry3f::Identity();
    Isometry3f _curr_imu_pose = Isometry3f::Identity();
  };

  using MDTrackerStandalonePtr = std::shared_ptr<MDTrackerStandalone>;
} // namespace md_slam
