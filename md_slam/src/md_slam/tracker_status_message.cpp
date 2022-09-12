#include "tracker_status_message.h"

namespace md_slam {
  using namespace srrg2_core;
  using namespace std;

  MDTrackerStatusMessage::MDTrackerStatusMessage(const std::string& topic_,
                                                 const std::string& frame_id_,
                                                 int seq_,
                                                 const double& timestamp_) :
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(global_pose, Isometry3f::Identity()),
    SETUP_PROPERTY(local_pose, Isometry3f::Identity()),
    SETUP_PROPERTY(information_matrix, Matrix6f::Identity()),
    SETUP_PROPERTY(is_keyframe, false),
    SETUP_PROPERTY(pyramid_topic, "/md_pyramid") {
  }
} // namespace md_slam
