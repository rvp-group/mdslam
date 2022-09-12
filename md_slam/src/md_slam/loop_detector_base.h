#pragma once

#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_image/image.h>
#include <srrg_messages/messages/base_sensor_message.h>
#include <srrg_pcl/point_types.h>

#include "frame.h"

namespace md_slam_closures {
  // generic interface for loop detector, dbow can be easily inserted
  class LoopDetectorBase : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // merd
    static constexpr size_t DESCRIPTOR_SIZE_ORB = 256;

    PARAM(srrg2_core::PropertyUnsignedInt,
          frame_interspace,
          "frame interspace to consider a match",
          50,
          nullptr);

    // some usings
    using ThisType   = LoopDetectorBase;
    using BaseType   = srrg2_core::Configurable;
    using IdFloatMap = std::map<size_t, float>;

    LoopDetectorBase();
    virtual ~LoopDetectorBase() = default;

    //! @brief inline const accessor
    inline const FramePointAssociationVector& associations() const noexcept {
      return _associations;
    }

    //! @brief inline accessor to the best matching frame
    inline Frame* match() const noexcept {
      return _matched_frame;
    }

    //! @brief inline accessor to all candidate matching frames
    inline const FrameVector& matches() const noexcept {
      return _matched_frame_vector;
    }

    //! @brief inline accessor to the score of the best matching frame
    inline const float& score() const noexcept {
      return _best_score;
    }

    //! @brief inline accessor to the scores container of all matching frames
    inline const IdFloatMap& scores() const noexcept {
      return _scores;
    }

    //! @brief set the current frame (query)
    inline void setCurrentFrame(Frame* frame_) noexcept {
      _query_frame = frame_;
    }

    //! @brief do the job, train relocalizer and
    //!  - caches the best matched frame (if any)
    //!  - populates the _matched_frame_vector with all possible matching frames
    virtual void compute();

  protected:
    //! @brief current frame
    Frame* _query_frame = nullptr;

    //! @brief (eventually) bast matched frame (that would be used in slam)
    Frame* _matched_frame = nullptr;

    //! @brief all possible matched frames for the current query (unfiltered)
    FrameVector _matched_frame_vector;

    //! @brief score of the best matching frame
    float _best_score = -1.0;

    //! @brief vector of the scores associated with each reference (indexed by frame_id)
    IdFloatMap _scores;

    //! @brief associations for the current frame w/ its matching frame
    FramePointAssociationVector _associations;
  };

} /* namespace md_slam_closures */
