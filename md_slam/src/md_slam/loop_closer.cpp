#include "loop_closer.h"

namespace md_slam {
  using namespace srrg2_core;
  using namespace md_slam_closures;

  MDCloser::MDCloser() {
    // only fucking containers storing info for closures
    _fcontainer = FramesContainerPtr(new FramesContainer);
  }

  MDCloser::~MDCloser() {
    std::cerr << "MDCloser::total number of closures " << _total_num_closures << std::endl;
  }

  bool MDCloser::compute() {
    // init stuff
    _reference_closures_idx = -1;
    _estimate.setIdentity();
    assert((param_loop_det.value() || param_loop_validator.value()) &&
           "MDCloser::compute|invalid config, no loop detector or validator");

    // generate current frame points
    _generateFrame(_fcontainer, _tracker_pose);

    // set last frame to detector
    param_loop_det.value()->setCurrentFrame(_fcontainer->frames().back());
    param_loop_det.value()->compute();
    if (!param_loop_det.value()->match()) {
      // features not "closed" enough
      return false;
    }

    std::unique_ptr<Closure> closure(
      new Closure(_fcontainer->frames().back(), param_loop_det.value()->match()));
    param_loop_validator.value()->setClosure(std::move(closure));
    // param_loop_validator.value()->setCameraMatrix(_cam_matrix);
    param_loop_validator.value()->setAssociations(param_loop_det.value()->associations());
    param_loop_validator.value()->compute();

    if (!param_loop_validator.value()->isLoopValid()) {
      // alignment not possible
      return false;
    }

    // store idx to variable and relative estimate for loop closing
    _estimate               = param_loop_validator.value()->estimate();
    _reference_closures_idx = param_loop_det.value()->match()->id(); // reference, from past
    _total_num_closures++;
    std::cerr << BG_GREEN("MDCloser::compute|closure accepted ") << _total_num_closures
              << std::endl;
    return true;
  }

  // thanks to leo brizi and his cool work with superpointssssss
  std::vector<cv::KeyPoint> MDCloser::_nonMaxSuppression(std::vector<cv::KeyPoint>& keypoints_,
                                                         const size_t& rows_,
                                                         const size_t& cols_,
                                                         const int& nms_dist_) {
    Eigen::ArrayXXi grid(rows_ + 2 * nms_dist_, cols_ + 2 * nms_dist_);
    grid.setZero();
    Eigen::ArrayXXi response(rows_ + 2 * nms_dist_, cols_ + 2 * nms_dist_);
    Eigen::ArrayXXi dim(rows_ + 2 * nms_dist_, cols_ + 2 * nms_dist_);

    // sort by confidence decreasing mode
    std::sort(keypoints_.begin(),
              keypoints_.end(),
              [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });

    // initialize the grid
    for (const auto& kp : keypoints_) {
      grid(kp.pt.y + nms_dist_, kp.pt.x + nms_dist_)     = 1;
      response(kp.pt.y + nms_dist_, kp.pt.x + nms_dist_) = kp.response;
      dim(kp.pt.y + nms_dist_, kp.pt.x + nms_dist_)      = kp.size;
    }

    // iterate through points, highest to lowest conf, suppress neighborhood
    for (const auto& kp : keypoints_) {
      const int x = kp.pt.x + nms_dist_;
      const int y = kp.pt.y + nms_dist_;
      if (grid(y, x) == 1) {
        grid.block(y - nms_dist_, x - nms_dist_, 2 * nms_dist_ + 1, 2 * nms_dist_ + 1) = 0;
        grid(y, x)                                                                     = -1;
      }
    }

    std::vector<cv::KeyPoint> result;
    result.reserve(keypoints_.size());
    // get all surviving -1's
    for (size_t r = 0; r < rows_; ++r) {
      for (size_t c = 0; c < cols_; ++c) {
        if (grid(r, c) == -1)
          result.emplace_back(
            cv::KeyPoint(c - nms_dist_, r - nms_dist_, dim(r, c), -1, response(r, c)));
      }
    }
    result.shrink_to_fit();
    return result;
  }

  void MDCloser::_generateFrame(FramesContainerPtr frames, const Isometry3f& curr_pose_) {
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(30); // TODO tune
    cv::Ptr<cv::DescriptorExtractor> extractor;
    const auto& cam_type = param_loop_validator.value()->param_camera_type.value();
    switch (cam_type) {
      case md_slam::CameraType::Pinhole:
        // TODO find a good configuration for normal camera
        extractor = cv::ORB::create(500);
      case md_slam::CameraType::Spherical:
        // extractor = cv::ORB::create(2500, 1.2f, 8, 1); // a tromba
        // good for os128
        extractor = cv::ORB::create(300,                   // ia nfeatures
                                    1.2f,                  // ia scale factor
                                    8,                     // ia nlevels
                                    15,                    // ia edge thresh
                                    0,                     // ia first level
                                    2,                     // ia wta_k
                                    cv::ORB::HARRIS_SCORE, // ia score type
                                    15,                    // ia patch size
                                    20);                   // ia fast threshold
        break;
      default:
        throw std::runtime_error("LoopValidator::compute|unknown camera model");
    }

    // get intensity img and extract features
    std::vector<cv::KeyPoint> kp_vec;
    cv::Mat desc;
    cv::Mat intensity_img;
    _intensity.toCv(intensity_img);
    intensity_img.convertTo(intensity_img, CV_8UC1, 255);
    detector->detect(intensity_img, kp_vec);
    auto kp_vec_nms =
      _nonMaxSuppression(kp_vec, _intensity.rows(), _intensity.cols(), param_nms_radius.value());
    extractor->compute(intensity_img, kp_vec_nms, desc);
    const size_t container_size = kp_vec_nms.size();
    Frame* current_frame        = nullptr;
    if (_pyramid) {
      current_frame = frames->createFrame(_timestamp, container_size, intensity_img, _pyramid);
    } else {
      current_frame = frames->createFrame(_timestamp, container_size, intensity_img);
    }
    current_frame->setPose(curr_pose_);

    // populate the frame with framepoints
    for (size_t k = 0; k < container_size; ++k) {
      const auto& keypoint = kp_vec_nms[k];
      const float depth    = (float) _depth.at(keypoint.pt.y, keypoint.pt.x); // already scaled
      // depth map we have only positive value, 0 corresponds to NaN
      if (depth == 0.f) {
        continue;
      }

      const float intensity = (float) _intensity.at(keypoint.pt.y, keypoint.pt.x);

      FramePoint* f_pt  = current_frame->createFramePoint();
      f_pt->descriptor  = desc.row(k);
      f_pt->intensity   = intensity;
      f_pt->depth_value = depth;
      f_pt->kp          = keypoint;
    }

    this->_need_redraw = true;
    this->draw();
  }

  void MDCloser::_drawImpl(ViewerCanvasPtr gl_canvas_) const {
    if (!gl_canvas_)
      return;
    // viz feature extraction
    cv::Mat img_display = _fcontainer->frames().back()->cvImage() + 0;
    cv::cvtColor(img_display, img_display, CV_GRAY2RGB);
    auto curr_frame = _fcontainer->frames().back()->framepoints();
    for (auto* f : *curr_frame) {
      cv::circle(img_display, f->kp.pt, 2, cv::Scalar(0, 0, 255));
    }
    gl_canvas_->putImage(img_display);
    gl_canvas_->flush();
  }

} // namespace md_slam
