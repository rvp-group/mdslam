#pragma once
#include <srrg_messages/messages/base_sensor_message.h>
#include <srrg_property/property_eigen.h>

namespace md_slam {

  class MDTrackerStatusMessage : public srrg2_core::BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MDTrackerStatusMessage(const std::string& topic_    = "",
                           const std::string& frame_id_ = "",
                           int seq_                     = -1,
                           const double& timestamp_     = -1);
    srrg2_core::PropertyEigen_<srrg2_core::Isometry3f> global_pose;

    // local pose is the offset w.r.t. the last keyframe, before self (even if keyframe)
    srrg2_core::PropertyEigen_<srrg2_core::Isometry3f> local_pose;
    srrg2_core::PropertyEigen_<srrg2_core::Matrix6f> information_matrix;
    srrg2_core::PropertyBool is_keyframe;
    srrg2_core::PropertyString pyramid_topic;
  };
  using MDTrackerStatusMessagePtr = std::shared_ptr<MDTrackerStatusMessage>;

} // namespace md_slam
