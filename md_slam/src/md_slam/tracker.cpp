#include "tracker.h"
#include "pyramid_message.h"
#include "tracker_status_message.h"
#include "utils.h"
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <unistd.h>

namespace md_slam {
  using namespace srrg2_core;
  using namespace std;

  MDTrackerStandalone::MDTrackerStandalone() {
    // imu stuff
    _rotation_estimator_imu = std::unique_ptr<MDOrientationEstimator>(new MDOrientationEstimator);
  }

  bool MDTrackerStandalone::_processImu(srrg2_core::IMUMessagePtr imu_msg_) {
    const Vector3d omega = imu_msg_->angular_velocity.value().template cast<double>();
    const Vector3d acc   = imu_msg_->linear_acceleration.value().template cast<double>();

    if (_is_init_imu) {
      _rotation_estimator_imu->update(acc, omega, 0.0); // initialize quaternion
      _prev_timestamp = imu_msg_->timestamp.value();
      _is_init_imu    = false;
      return false;
    }

    const double dt = imu_msg_->timestamp.value() - _prev_timestamp;
    _rotation_estimator_imu->update(acc, omega, dt);
    _prev_timestamp = imu_msg_->timestamp.value(); // override last abs time for dt calculation
    return true;
  }

  bool MDTrackerStandalone::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {
    // if imu integrate batch of imu msgs and calculate abs orientation
    if (param_enable_imu.value()) {
      IMUMessagePtr imu_msg = std::dynamic_pointer_cast<IMUMessage>(msg_);
      if (imu_msg) {
        _processImu(imu_msg);
      }
    }

    // get current pyr msg
    std::shared_ptr<MDImagePyramidMessage> curr_pyr_msg =
      std::dynamic_pointer_cast<MDImagePyramidMessage>(msg_);
    if (!curr_pyr_msg) {
      return false;
    }

    _last_update_time = curr_pyr_msg->timestamp.value();

    MDImagePyramid* curr_img_pyr = curr_pyr_msg->get();
    if (!curr_img_pyr) {
      std::cerr << "MDTrackerStandalone::putMessage|current img pyr invalid!" << std::endl;
      return false;
    }

    MDTrackerStatusMessagePtr status_msg(
      new MDTrackerStatusMessage("/md_tracker_status",
                                 curr_pyr_msg->frame_id.value(),
                                 curr_pyr_msg->seq.value(),
                                 curr_pyr_msg->timestamp.value()));
    status_msg->pyramid_topic.setValue(curr_pyr_msg->topic.value());

    // if is init or we changed keyframe
    if (!_prev_pyr_msg) {
      // one shot initialization
      _keyframe_t.setIdentity();
      _local_t.setIdentity();
      _elapsed_steps = 0;
      std::cerr << "MDTrackerStandalone::putMessage|init done" << std::endl;

      _prev_pyr_msg = curr_pyr_msg;

      // initialize factor stack
      param_pairwise_aligner.value()->setFixed(_prev_pyr_msg->get()); // fixed mapped to prev pyr
      param_pairwise_aligner.value()->initialize();

      _estimate    = _keyframe_t;
      _num_updates = 0;
      status_msg->is_keyframe.setValue(false);
      status_msg->local_pose.setValue(_local_t);
    } else {
      // process imu initial guess if enabled
      if (param_enable_imu.value()) {
        _curr_imu_pose.setIdentity();
        _curr_imu_pose.linear() =
          _rotation_estimator_imu->rotation().template cast<float>().transpose();
        // obtain relative chunk of imu odom
        const Isometry3f delta_imu = _prev_imu_pose.inverse() * _curr_imu_pose;
        _local_t                   = _local_t * delta_imu; // use imu to complete initial guess
        _prev_imu_pose             = _curr_imu_pose;
      }
      // setting up for pairwise alignment
      param_pairwise_aligner.value()->setFixed(_prev_pyr_msg->get());
      param_pairwise_aligner.value()->setMoving(curr_img_pyr);
      param_pairwise_aligner.value()->setEstimate(_local_t);
      {
        Chrono solve_time("solve: ", &_timings, false);
        param_pairwise_aligner.value()->compute();
        _local_t = param_pairwise_aligner.value()->estimate();
        status_msg->local_pose.setValue(_local_t);
        // propagate information mat
        status_msg->information_matrix.setValue(
          param_pairwise_aligner.value()->informationMatrix());
        _estimate = _keyframe_t * _local_t;
        ++_elapsed_steps;
      }
      Eigen::AngleAxisf aa(_local_t.linear());
      if ((param_keyframe_steps.value() > 0 && _elapsed_steps > param_keyframe_steps.value()) ||
          fabs(aa.angle()) > param_keyframe_rotation.value() ||
          _local_t.translation().norm() > param_keyframe_rotation.value()) {
        _prev_pyr_msg = curr_pyr_msg;
        _keyframe_t   = _keyframe_t * _local_t;
        _local_t.setIdentity();
        _elapsed_steps = 0;
        status_msg->is_keyframe.setValue(true);
        // cerr << "kf_prev: " << geometry3d::t2v(previous_kf).transpose() << endl;
        // cerr << "kf curr: " << geometry3d::t2v(_keyframe_t).transpose() << endl;
      }
      status_msg->global_pose.setValue(_estimate);
      publishTransform();
    }

    _prev_imu_pose = _curr_imu_pose;
    Chrono::printReport(_timings);
    ++_num_updates;
    propagateMessage(status_msg);
    return true;
  }

  void MDTrackerStandalone::publishTransform() {
    if (!platform())
      return;
    // publish open loop estimate, not SLAM
    TransformEventsMessagePtr transform_event(new TransformEventsMessage);
    transform_event->seq.setValue(_num_updates);
    transform_event->timestamp.setValue(_last_update_time);
    transform_event->frame_id.setValue(param_origin_frame_id.value());
    transform_event->topic.setValue(param_tf_dyn_topic.value());
    transform_event->events.resize(2);
    TransformEvent global_to_keyframe(_last_update_time,
                                      param_keyframe_frame_id.value(),
                                      _keyframe_t,
                                      param_origin_frame_id.value());
    transform_event->events.setValue(0, global_to_keyframe);
    TransformEvent keyframe_to_local(
      _last_update_time, param_local_frame_id.value(), _local_t, param_keyframe_frame_id.value());
    transform_event->events.setValue(1, keyframe_to_local);
    // TODO debug imu stuff
    // Isometry3f imu_pose  = Isometry3f::Identity();
    // Matrix3f orientation = _rotation_estimator_imu->rotation().template cast<float>();
    // imu_pose.linear()    = orientation.transpose();
    // TransformEvent global_to_imu(
    //   _last_update_time, "/imu_orientation", imu_pose, param_origin_frame_id.value());
    // transform_event->events.setValue(2, global_to_imu);
    // Isometry3f key_frame_rot = Isometry3f::Identity();
    // key_frame_rot.linear()   = _keyframe_t.linear();
    // TransformEvent global_to_keyframe_r(
    //   _last_update_time, "/md_orientation", key_frame_rot, param_origin_frame_id.value());
    // transform_event->events.setValue(3, global_to_keyframe_r);
    propagateMessage(transform_event);
  }

} // namespace md_slam
