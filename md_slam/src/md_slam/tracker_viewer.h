#pragma once
#include "pyramid_message.h"
#include "tracker_status_message.h"
#include <srrg_messages/message_handlers/message_sink_base.h>
#include <srrg_viewer/active_drawable.h>

namespace md_slam {
  class MDTrackerViewer : public srrg2_core::MessageSinkBase, public srrg2_core::ActiveDrawable {
  public:
    PARAM(srrg2_core::PropertyFloat, voxelize_coord_res, "coordinates resolution", 0.05, nullptr);
    PARAM(srrg2_core::PropertyFloat, voxelize_normal_res, "normal resolution", 0.f, nullptr);
    PARAM(srrg2_core::PropertyInt, voxelize_interval, "when to voxelize", 10, nullptr);
    PARAM(srrg2_core::PropertyString,
          status_topic,
          "tracker status",
          "/md_tracker_status",
          nullptr);
    void reset() override;
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;

  protected:
    inline MDVectorCloud& globalCloud() {
      return _global_cloud[_g_idx & 0x1];
    }
    inline MDVectorCloud& otherCloud() {
      return _global_cloud[(_g_idx + 1) & 0x1];
    }
    inline const MDVectorCloud& globalCloud() const {
      return _global_cloud[_g_idx & 0x1];
    }
    inline const MDVectorCloud& otherCloud() const {
      return _global_cloud[(_g_idx + 1) & 0x1];
    }
    inline void voxelize();
    void addCloud(const MDMatrixCloud& cloud, const Isometry3f& isometry, bool is_keyframe = false);
    void _drawImpl(srrg2_core::ViewerCanvasPtr gl_canvas_) const override;

    MDVectorCloud _global_cloud[2];
    uint8_t _g_idx = 0;
    MDTrackerStatusMessagePtr _status_msg;
    MDImagePyramidMessagePtr _pyramid_msg;
    MDVectorCloud _current_cloud;
    int _last_time_voxelize  = 0;
    Isometry3f _current_pose = Isometry3f::Identity();
    std::list<Isometry3f, Eigen::aligned_allocator<Isometry3f>> _trajectory;
    std::string _pyramid_topic  = "";
    mutable bool _lists_created = false;
  };
} // namespace md_slam
