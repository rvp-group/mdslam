#pragma once
#include "image_pyramid.h"
#include <srrg_messages/messages/base_sensor_message.h>

namespace md_slam {
  class MDImagePyramidMessage : public srrg2_core::BaseSensorMessage {
  public:
    MDImagePyramidMessage(const std::string& topic_    = "",
                          const std::string& frame_id_ = "",
                          int seq_                     = -1,
                          const double& timestamp_     = -1) :
      BaseSensorMessage(topic_, frame_id_, seq_, timestamp_) {
    }
    MDImagePyramid* get() {
      return _pyramid.get();
    }
    void set(MDImagePyramid* pyr) {
      _pyramid.set(pyr);
    }
    void serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context);
    void deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context);

  protected:
    MDImagePyramidReference _pyramid;
  };
  using MDImagePyramidMessagePtr = std::shared_ptr<MDImagePyramidMessage>;

} // namespace md_slam
