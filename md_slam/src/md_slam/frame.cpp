#include "frame.h"

namespace md_slam_closures {

  Frame::Frame(const size_t& id_,
               const double& timestamp_,
               const size_t& num_framepoints_,
               const cv::Mat& cv_intensity_image_) :
    _id(id_) {
    if (_framepoints) {
      throw std::runtime_error("Frame::Frame|ERROR, framepoints already there");
    }
    _timestamp   = timestamp_;
    _framepoints = new FramePointVector();
    _framepoints->reserve(num_framepoints_);
    _cv_intensity_image = cv_intensity_image_;
  }

  Frame::Frame(const size_t& id_,
               const double& timestamp_,
               const size_t& num_framepoints_,
               const cv::Mat& cv_intensity_image_,
               MDImagePyramid* pyramid_) :
    _id(id_) {
    if (_framepoints) {
      throw std::runtime_error("Frame::Frame|ERROR, framepoints already there");
    }
    _timestamp   = timestamp_;
    _framepoints = new FramePointVector();
    _framepoints->reserve(num_framepoints_);
    _cv_intensity_image = cv_intensity_image_;
    _pyramid            = pyramid_;
  }

  Frame::~Frame() {
    if (_framepoints) {
      for (FramePoint* fp : *_framepoints) {
        delete fp;
      }
      _framepoints->clear();
      delete _framepoints;
    }
  }

  size_t FramesContainer::_frame_id_generator = 0;
  void FramesContainer::resetIDGenerator() {
    _frame_id_generator = 0;
  }

  FramesContainer::FramesContainer() {
  }

  FramesContainer::~FramesContainer() {
    if (_frames_pull.size()) {
      for (Frame* f : _frames_pull) {
        delete f;
      }
      _frames_pull.clear();
    }
    FramesContainer::resetIDGenerator();
  }

  Frame* FramesContainer::createFrame(const double& timestamp_,
                                      const size_t& num_framepoints_,
                                      const cv::Mat& cv_intensity_image_) {
    Frame* f = new Frame(_frame_id_generator++, timestamp_, num_framepoints_, cv_intensity_image_);
    _frames_pull.push_back(f);
    _frame_map.insert(std::make_pair(f->id(), f));
    return f;
  }

  Frame* FramesContainer::createFrame(const double& timestamp_,
                                      const size_t& num_framepoints_,
                                      const cv::Mat& cv_intensity_image_,
                                      MDImagePyramid* pyramid_) {
    Frame* f =
      new Frame(_frame_id_generator++, timestamp_, num_framepoints_, cv_intensity_image_, pyramid_);
    _frames_pull.push_back(f);
    _frame_map.insert(std::make_pair(f->id(), f));
    return f;
  }

} // namespace md_slam_closures
