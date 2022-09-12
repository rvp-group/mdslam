#pragma once
#include "image_pyramid.h"
#include <srrg_image/image.h>
#include <srrg_messages/messages/base_sensor_message.h>
#include <srrg_pcl/point_types.h>

namespace md_slam_closures {

  using namespace md_slam;

  class Frame;

  struct FramePoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    cv::Mat descriptor;
    cv::KeyPoint kp;
    float depth_value = 0.f;
    float intensity   = 0.f;
    // the frame that created this framepoint
    Frame* parent_frame = nullptr;

  protected:
    FramePoint(Frame* parent_) {
      parent_frame = parent_;
    }
    ~FramePoint() {
      descriptor.release();
    }

  public:
    friend class Frame;
  };

  // still don't understand when to allocate or not, just saving my ass
  using FramePointVector = std::vector<FramePoint*, Eigen::aligned_allocator<FramePoint*>>;
  using FramePointSet =
    std::set<FramePoint*, std::less<FramePoint*>, Eigen::aligned_allocator<FramePoint*>>;

  struct FramePointAssociation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FramePointAssociation(FramePoint* fixed_, FramePoint* moving_) {
      fixed  = fixed_;
      moving = moving_;
    }
    FramePoint* fixed  = nullptr;
    FramePoint* moving = nullptr;
  };

  using FramePointAssociationVector =
    std::vector<FramePointAssociation, Eigen::aligned_allocator<FramePointAssociation>>;

  class Frame {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline const FramePointVector* framepoints() const {
      return _framepoints;
    }

    inline MDImagePyramid* pyramid() {
      return _pyramid;
    }

    inline const size_t& numFramePoints() const {
      return _num_framepoints;
    }

    inline const cv::Mat& cvImage() const {
      return _cv_intensity_image;
    }

    inline const srrg2_core::Isometry3f& pose() const {
      return _pose;
    }

    inline void setPose(const srrg2_core::Isometry3f& pose_) {
      _pose = pose_;
    }

    inline const double& timestamp() const {
      return _timestamp;
    }

    inline const size_t& id() const {
      return _id;
    }

    FramePoint* createFramePoint() {
      FramePoint* fp = new FramePoint(this);
      _framepoints->emplace_back(fp);
      _num_framepoints = _framepoints->size();
      return fp;
    }

  protected:
    Frame() = delete;
    Frame(const size_t& id_,
          const double& timestamp_,
          const size_t& num_framepoints_,
          const cv::Mat& cv_intensity_image_);
    Frame(const size_t& id_,
          const double& timestamp_,
          const size_t& num_framepoints_,
          const cv::Mat& cv_intensity_image_,
          MDImagePyramid* pyramid_);

    virtual ~Frame();

    const size_t _id;
    double _timestamp              = -1.f;
    FramePointVector* _framepoints = nullptr;
    MDImagePyramid* _pyramid       = nullptr;
    size_t _num_framepoints        = 0;
    cv::Mat _cv_intensity_image;
    srrg2_core::Isometry3f _pose = srrg2_core::Isometry3f::Identity();

  public:
    friend class FramesContainer;
  };

  using FrameVector = std::vector<Frame*, Eigen::aligned_allocator<Frame*>>;
  using IdFrameMap  = std::
    map<size_t, Frame*, std::less<size_t>, Eigen::aligned_allocator<std::pair<size_t, Frame*>>>;

  class FramesContainer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FramesContainer();
    virtual ~FramesContainer();

    Frame* createFrame(const double& timestamp_,
                       const size_t& num_framepoints_,
                       const cv::Mat& cv_intensity_image_);

    Frame* createFrame(const double& timestamp_,
                       const size_t& num_framepoints_,
                       const cv::Mat& cv_intensity_image_,
                       MDImagePyramid* pyramid_);

    inline const FrameVector& frames() const {
      return _frames_pull;
    }

    inline Frame* frame(const size_t& id_) const {
      return _frame_map.at(id_);
    }

  protected:
    FrameVector _frames_pull;
    IdFrameMap _frame_map;

  private:
    static size_t _frame_id_generator;
    static void resetIDGenerator();
  };

  using FramesContainerPtr = std::shared_ptr<FramesContainer>;

  struct Closure {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Closure(Frame* query_, Frame* reference_) : query(query_), reference(reference_) {
    }
    Frame* query      = nullptr;
    Frame* reference  = nullptr;
    float score       = 0.f; // TODO decide how to perform final score
    bool is_validated = false;
  };

} // namespace md_slam_closures
