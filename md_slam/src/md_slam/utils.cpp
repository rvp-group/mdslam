#include "utils.h"
#include <srrg_system_utils/shell_colors.h>

namespace md_slam {
  using namespace std; // evvaffanculo
  using namespace srrg2_core;

  void prepareImage(ImageFloat& dest,
                    const ImageFloat& src,
                    int max_scale,
                    int row_scale,
                    int col_scale,
                    bool suppress_zero) {
    int d_rows = src.rows() / row_scale;
    int d_cols = src.cols() / col_scale;
    d_rows     = (d_rows / max_scale) * max_scale;
    d_cols     = (d_cols / max_scale) * max_scale;

    int s_rows = d_rows * row_scale;
    int s_cols = d_cols * col_scale;
    // cerr << "d_rows: " << d_rows << " d_cols: " << d_cols << endl;
    // cerr << "s_rows: " << s_rows << " s_cols: " << s_cols << endl;

    dest.resize(d_rows, d_cols);
    dest.fill(0);
    ImageInt counts(d_rows, d_cols);
    counts.fill(0);
    for (int r = 0; r < s_rows; ++r) {
      int dr = r / row_scale;
      for (int c = 0; c < s_cols; ++c) {
        int dc  = c / col_scale;
        float v = src.at(r, c);
        if (suppress_zero && !v)
          continue;
        dest.at(dr, dc) += src.at(r, c);
        ++counts.at(dr, dc);
      }
    }
    for (int r = 0; r < d_rows; ++r) {
      for (int c = 0; c < d_cols; ++c) {
        int cnt = counts.at(r, c);
        if (cnt)
          dest.at(r, c) *= (1. / cnt);
      }
    }
  }

  bool loadImage(BaseImage& img, const std::string filename) {
    // try for depth
    ImageUInt16* depth = dynamic_cast<ImageUInt16*>(&img);
    if (depth) {
      // cerr << "loading image from file [" << filename << "] (depth, uint16)" << endl;
      cv::Mat depth_cv = cv::imread(filename, CV_LOAD_IMAGE_ANYDEPTH);
      assert(!depth_cv.empty() &&
             std::string("utils::loadImage|unable to load depth [ " + filename + " ]").c_str());
      depth->fromCv(depth_cv);
      size_t accumulator = 0;
      size_t num_pixels  = 0;
      for (size_t r = 0; r < depth->rows(); ++r)
        for (size_t c = 0; c < depth->cols(); ++c) {
          uint16_t v = depth->at(r, c);
          if (v) {
            accumulator += v;
            ++num_pixels;
          }
        }
      // cerr << "loaded, rows: " << depth->rows()
      //      << " cols: " << depth->cols()
      //      << " pix: " << depth->cols() * depth->rows()
      //      << " avg: " << (float)accumulator/float(num_pixels)
      //      << " null: " << depth->cols() * depth->rows() - num_pixels << endl;

      return true;
    }

    // try for rgb
    ImageVector3uc* rgb = dynamic_cast<ImageVector3uc*>(&img);
    if (rgb) {
      // cerr << "loading image from file [" << filename << "] (intensity, rgb)" << endl;
      cv::Mat rgb_cv = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
      assert(!rgb_cv.empty() &&
             std::string("utils::loadImage|unable to load rgb [ " + filename + " ]").c_str());
      rgb->fromCv(rgb_cv);
      return true;
    }

    // try for monochrome
    ImageUInt8* intensity = dynamic_cast<ImageUInt8*>(&img);
    if (intensity) {
      // cerr << "loading image from file [" << filename << "] (intensity, uint8)" << endl;
      cv::Mat intensity_cv = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
      assert(
        !intensity_cv.empty() &&
        std::string("utils::loadImage | unable to load intensity [ " + filename + " ]").c_str());
      intensity->fromCv(intensity_cv);
      return true;
    }
    return false;
  }

  void showImage(const BaseImage& img, const std::string& title, int wait_time) {
    cv::Mat m;
    img.toCv(m);
    cv::imshow(title, m);
    if (wait_time > -1)
      cv::waitKey(wait_time);
  }

  bool project(Vector2f& image_point_,
               Vector3f& camera_point_,
               float& depth_,
               const Vector3f& point_,
               const CameraType& camera_type_,
               const Matrix3f& camera_mat_,
               const float& min_depth_,
               const float& max_depth_) {
    switch (camera_type_) {
      case CameraType::Pinhole:
        depth_ = point_.z();
        if (depth_ < min_depth_ || depth_ > max_depth_) {
          return false;
        }
        camera_point_ = camera_mat_ * point_;
        image_point_  = camera_point_.head<2>() * 1.f / depth_;
        break;
      case CameraType::Spherical:
        depth_ = point_.norm();
        if (depth_ < min_depth_ || depth_ > max_depth_) {
          return false;
        }
        camera_point_.x() = atan2(point_.y(), point_.x());
        camera_point_.y() = atan2(point_.z(), point_.head<2>().norm());
        camera_point_.z() = depth_;

        image_point_.x() = camera_mat_(0, 0) * camera_point_.x() + camera_mat_(0, 2);
        image_point_.y() = camera_mat_(1, 1) * camera_point_.y() + camera_mat_(1, 2);
        break;
      default:
        throw std::runtime_error("utils::project | unknown camera type");
    }
    return true;
  }

  void sparseProjection(Point2fVectorCloud& dest_,
                        const Point3fVectorCloud& source_,
                        const Matrix3f& cam_matrix_,
                        const float& min_depth_,
                        const float& max_depth_,
                        const int& n_rows_,
                        const int& n_cols_,
                        const CameraType& camera_type_) {
    assert((n_rows_ != 0 || n_cols_ != 0) && "sparseProjection|num rows and cols of "
                                             "image not set");
    assert(camera_type_ != CameraType::Unknown && "sparseProjection|camera type not set");

    size_t i = 0;
    dest_.resize(source_.size());
    for (auto it = source_.begin(); it != source_.end(); ++it, ++i) {
      const auto& point_full = *it;
      const Vector3f& point  = point_full.coordinates();

      float depth           = 0.f;
      Vector3f camera_point = Vector3f::Zero();
      // initialize
      dest_[i].status        = POINT_STATUS::Invalid;
      dest_[i].coordinates() = Vector2f::Zero();
      Vector2f image_point   = Vector2f::Zero();

      const bool is_good = project(
        image_point, camera_point, depth, point, camera_type_, cam_matrix_, min_depth_, max_depth_);
      if (!is_good)
        continue;

      const int irow = cvRound(image_point.y());
      const int icol = cvRound(image_point.x());

      if (irow > n_rows_ || icol > n_cols_)
        continue;

      dest_[i].status        = POINT_STATUS::Valid;
      dest_[i].coordinates() = image_point;

      // size_t i = 0;
      // dest_.resize(source_.size());
      // for (auto it = source_.begin(); it != source_.end(); ++it, ++i) {
      //   const auto& point_full = *it;
      //   const Vector3f& point  = point_full.coordinates();

      //   float depth           = 0.f;
      //   float inverse_depth   = 1.f;
      //   Vector3f camera_point = Vector3f::Zero();
      //   // initialize
      //   dest_[i].status        = POINT_STATUS::Invalid;
      //   dest_[i].coordinates() = Vector2f::Zero();
      //   Vector2f image_point   = Vector2f::Zero();

      //   switch (camera_type_) {
      //     case CameraType::Pinhole:
      //       depth = point.z();
      //       if (depth < min_depth_ || depth > max_depth_) {
      //         continue;
      //       }
      //       inverse_depth = 1.f / depth;
      //       camera_point  = cam_matrix_ * point;
      //       image_point   = camera_point.head<2>() * inverse_depth;
      //       break;
      //     case CameraType::Spherical:
      //       depth = point.norm();
      //       if (depth < min_depth_ || depth > max_depth_) {
      //         continue;
      //       }
      //       camera_point.x() = atan2(point.y(), point.x());
      //       camera_point.y() = atan2(point.z(), point.head<2>().norm());
      //       camera_point.z() = depth;

      //       image_point.x() = cam_matrix_(0, 0) * camera_point.x() + cam_matrix_(0, 2);
      //       image_point.y() = cam_matrix_(1, 1) * camera_point.y() + cam_matrix_(1, 2);
      //       break;
      //     default:
      //       throw std::runtime_error("unknown camera type");
      //   }

      //   const int irow = cvRound(image_point.y());
      //   const int icol = cvRound(image_point.x());

      //   if (irow > n_rows_ || icol > n_cols_)
      //     continue;

      //   dest_[i].status        = POINT_STATUS::Valid;
      //   dest_[i].coordinates() = image_point;
    }
  }

  // given a depth img, finds an img of kernels value based on internal lambda function
  ImageUInt8 calculateKernelImg(const ImageFloat& depth_img_,
                                const float min_depth_,
                                const int num_bins_,
                                const float max_binning_depth_) {
    const int& rows = depth_img_.rows();
    const int& cols = depth_img_.cols();
    // find thresholds value to discretize depth map
    const int step = (int) (max_binning_depth_ - min_depth_) / num_bins_;
    std::vector<float> thresholds(num_bins_ + 1);
    for (int i = 0; i < num_bins_ + 1; ++i) {
      thresholds[i] = min_depth_ + step * i;
    }

    // relu like function to map depth to number of bin
    int min_relu_value = 3;
    auto map_kernels   = [&](const int& x) {
      return x > 0 ? std::max(-2 * x + (num_bins_ * 2 + 1), min_relu_value) : 0;
    };

    // initialize discretize depth with ones, since after we avoid division by 0
    ImageUInt8 kernels_img(rows, cols);
    kernels_img.fill(0.f);
    for (int i = 0; i < num_bins_; ++i) {
      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          const float& depth_v = depth_img_(r, c);
          if (depth_v >= thresholds.at(i) && depth_v <= thresholds.at(i + 1)) {
            kernels_img(r, c) = static_cast<uint8_t>(map_kernels(i + 1));
          } else if (depth_v > thresholds.at(num_bins_)) {
            kernels_img(r, c) = static_cast<uint8_t>(map_kernels(num_bins_ + 1));
          }
        }
      }
    }

    return kernels_img;
  }

  ImageVector3f adaptiveNormalsSmoothing(const ImageVector3f& normals_,
                                         const ImageUInt8& kernel_img_) {
    const int& rows = normals_.rows();
    const int& cols = normals_.cols();
    ImageVector3f smoothed_normals(rows, cols);
    smoothed_normals.fill(Vector3f::Zero());
    ImageVector3f integral;
    normals_.integralImage(integral);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        // odd value reflecting the whole kernel window size
        const size_t& ks = kernel_img_(r, c);
        if (ks <= 0)
          continue;
        // scaling for rows since image lidar resolution is very unbalanced
        const size_t ks_r = ks > 5 ? 5 : 3;
        // std::cerr << ks << " " << ks_r << std::endl; // obtaining windows to apply blurring
        const int window_c     = ks / 2;
        const int window_r     = ks_r / 2;
        const size_t win_left  = c - window_c <= 0 ? c : window_c;
        const size_t win_up    = r - window_r <= 0 ? r : window_r;
        const size_t win_right = c + window_c >= cols ? cols - c - 1 : window_c;
        const size_t win_down  = r + window_r >= rows ? rows - r - 1 : window_r;

        // blurring using integral img stuff
        const Vector3f& m00  = integral.at(r - win_up, c - win_left);
        const Vector3f& m01  = integral.at(r - win_up, c + win_right);
        const Vector3f& m11  = integral.at(r + win_down, c + win_right);
        const Vector3f& m10  = integral.at(r + win_down, c - win_left);
        const Vector3f n_sum = m11 + m00 - m01 - m10;
        if (n_sum.dot(n_sum) > 0.2)
          smoothed_normals.at(r, c) = n_sum.normalized();
      }
    }
    return smoothed_normals;
  }

  void bilateralFiltering(ImageUInt16& out_img_,
                          const ImageUInt16& in_img_,
                          const float& radius_factor_,
                          const float& sigma_pixel_,
                          const float& sigma_inv_depth_,
                          const float& max_depth_,
                          const float& scale_) {
    const int& rows                 = in_img_.rows();
    const int& cols                 = in_img_.cols();
    const uint16_t max_depth_scaled = max_depth_ / scale_;

    // radius of bilateral filter
    const int radius         = radius_factor_ * sigma_pixel_ + 0.5f;
    const int radius_squared = radius * radius;

    const float denom_pixel = 2.f * sigma_pixel_ * sigma_pixel_;

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        const uint16_t center_value = in_img_.at(r, c);
        if (center_value == 0 || center_value > max_depth_scaled) {
          continue;
        }
        const float inv_center_value = 1.f / (scale_ * center_value);

        // bilateral filtering
        float sum    = 0.f;
        float weight = 0.f;

        const int min_r = std::max(0, r - radius);
        const int max_r = std::min(rows - 1, r + radius);
        for (int sample_r = min_r; sample_r <= max_r; ++sample_r) {
          const int dr    = sample_r - r;
          const int min_c = std::max(0, c - radius);
          const int max_x = std::min(cols - 1, c + radius);

          for (int sample_c = min_c; sample_c <= max_x; ++sample_c) {
            const int dc                    = sample_c - c;
            const int grid_distance_squared = dr * dr + dc * dc;

            if (grid_distance_squared > radius_squared) {
              continue;
            }

            // discard invalid pixels from calculation
            const uint16_t& sample = in_img_.at(sample_r, sample_c);
            if (sample == 0) {
              continue;
            }

            const float inv_sample = 1.f / (scale_ * sample);

            float value_distance_squared = inv_center_value - inv_sample;
            value_distance_squared *= value_distance_squared;

            const float w = std::exp(-grid_distance_squared / denom_pixel +
                                     -value_distance_squared / sigma_inv_depth_);
            sum += w * inv_sample;
            weight += w;
          }
        }
        // std::cerr << "od: " << in_img_.at(r, c) * scale_;
        out_img_.at(r, c) = (weight == 0.f) ? 0 : (1.f / (scale_ * sum / weight));
        // std::cerr << " nd: " << out_img_.at(r, c) * scale_ << std::endl;
      }
    }
  }

} // namespace md_slam
