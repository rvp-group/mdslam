#pragma once
#include "md_slam/utils.h"
#include <srrg_geometry/geometry_defs.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_pcl/instances.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// chrono stuff TBR
#include <chrono>
#include <ctime>
#include <ratio>

namespace md_slam {

  const tf::tfMessage tfFromEigMatrix(size_t& seq_,
                                      const srrg2_core::Isometry3f& pose_,
                                      const ros::Time& time_,
                                      const std::string& frame_id_,
                                      const std::string& child_frame_id_) {
    geometry_msgs::TransformStamped pose_offset;
    pose_offset.header.stamp            = time_;
    pose_offset.header.seq              = seq_++;
    pose_offset.header.frame_id         = frame_id_;
    pose_offset.child_frame_id          = child_frame_id_;
    pose_offset.transform.translation.x = pose_.translation().x();
    pose_offset.transform.translation.y = pose_.translation().y();
    pose_offset.transform.translation.z = pose_.translation().z();
    const srrg2_core::Quaternionf quat(pose_.linear());
    pose_offset.transform.rotation.x = quat.x();
    pose_offset.transform.rotation.y = quat.y();
    pose_offset.transform.rotation.z = quat.z();
    pose_offset.transform.rotation.w = quat.w();
    std::cerr << child_frame_id_ << " in " << frame_id_ << "\n" << pose_.matrix() << std::endl;
    tf::tfMessage tfmsg;
    tfmsg.transforms = {pose_offset};
    return tfmsg;
  }

  // ia checks whether file exists
  inline bool checkFile(const std::string& name_) {
    struct stat buffer;
    return (stat(name_.c_str(), &buffer) == 0);
  }

  template <typename DestMessageType_>
  std::shared_ptr<DestMessageType_> extractMessage(srrg2_core::BaseSensorMessagePtr measurement,
                                                   const std::string& topic_) {
    srrg2_core::MessagePackPtr message_pack =
      std::dynamic_pointer_cast<srrg2_core::MessagePack>(measurement);

    // ia if it's not a pack, try to process a single laser message
    if (!message_pack) {
      if (measurement->topic.value() == topic_) {
        return std::dynamic_pointer_cast<DestMessageType_>(measurement);
      }
      return std::shared_ptr<DestMessageType_>();
    }
    // is a pack
    for (srrg2_core::BaseSensorMessagePtr& message : message_pack->messages) {
      if (message->topic.value() != topic_) {
        continue;
      }
      std::shared_ptr<DestMessageType_> result =
        std::dynamic_pointer_cast<DestMessageType_>(message);
      if (result) {
        return result;
      }
    }
    return nullptr;
  }

  // projection utils part

  using namespace srrg2_core;
  using namespace srrg2_core_ros;

  struct ProjectionEntry {
    bool valid      = false;
    int source_idx  = -1;
    float depth     = 0.f;
    float intensity = 0.f;
    float azimuth   = 0.f;
    float elevation = 0.f;
    uint32_t row    = 0;
    uint32_t col    = 0;
  };
  using ProjectionMatrixType =
    srrg2_core::Matrix_<ProjectionEntry, Eigen::aligned_allocator<ProjectionEntry>>;
  inline int computeProjections(ProjectionMatrixType& dest_,
                                const PointIntensity3fVectorCloud& source_,
                                const Matrix3f& cam_matrix_,
                                const float min_depth_,
                                const float max_depth_,
                                const int n_rows_,
                                const int n_cols_,
                                const CameraType camera_type_) {
    assert(n_rows_ != 0 && n_cols_ != 0 &&
           "computeProjections|num rows and cols of "
           "image not set");

    size_t num_valid = 0;
    size_t i         = 0;
    dest_.clear();
    dest_.resize(n_rows_, n_cols_);

    for (auto it = source_.begin(); it != source_.end(); ++it, ++i) {
      const auto& point_full = *it;
      const Vector3f& point  = point_full.coordinates();
      // ia discard invalid points
      if (point_full.status != srrg2_core::Valid) {
        continue;
      }

      float depth                 = 0.f;
      Vector3f camera_point       = Vector3f::Zero();
      Vector2f img_point_subpixel = Vector2f::Zero();
      const bool is_good          = project(img_point_subpixel,
                                   camera_point,
                                   depth,
                                   point,
                                   camera_type_,
                                   cam_matrix_,
                                   min_depth_,
                                   max_depth_);
      if (!is_good)
        continue;

      const int irow = cvRound(img_point_subpixel.y());
      const int icol = cvRound(img_point_subpixel.x());
      if (irow < 0 || icol < 0 || irow >= n_rows_ || icol >= n_cols_)
        continue;

      auto& p_entry = dest_.at(img_point_subpixel.y(), img_point_subpixel.x());

      // check if this entry is already occupied
      if (p_entry.valid) {
        // depth buffer
        if (depth > p_entry.depth) {
          continue;
        } else {
          p_entry.source_idx = i;
          p_entry.depth      = depth;
          p_entry.intensity  = point_full.intensity();
          p_entry.row        = irow;
          p_entry.col        = icol;
          p_entry.azimuth    = camera_point.x();
          p_entry.elevation  = camera_point.y();
        }
      } else {
        p_entry.valid      = true;
        p_entry.source_idx = i;
        p_entry.depth      = depth;
        p_entry.intensity  = point_full.intensity();
        p_entry.row        = irow;
        p_entry.col        = icol;
        p_entry.azimuth    = camera_point.x();
        p_entry.elevation  = camera_point.y();
        ++num_valid; // increment only here
      }
    }
    return num_valid;
  }

  void computeLidarImages(cv::Mat& cv_intensity_,
                          cv::Mat& cv_depth_,
                          const PointIntensity3fVectorCloud& raw_cloud_,
                          const Matrix3f& K_,
                          const size_t& image_rows_,
                          const size_t& image_cols_,
                          const float& min_depth_,
                          const float& max_depth_,
                          const float& max_intensity_normalizer_) {
    ProjectionMatrixType projection;
    const size_t valid_projected_points = computeProjections(projection,
                                                             raw_cloud_,
                                                             K_,
                                                             min_depth_,
                                                             max_depth_,
                                                             image_rows_,
                                                             image_cols_,
                                                             CameraType::Spherical);

    const float projected_points_ratio = (float) valid_projected_points / (float) raw_cloud_.size();
    if (projected_points_ratio < 0.8f) {
      std::cerr << "computeIntensityImage|WARNING, projected [ " << valid_projected_points
                << " ] over [ " << raw_cloud_.size() << " ] cloud points\n ";
    }

    srrg2_core::ImageFloat depth_image(image_rows_, image_cols_);
    srrg2_core::ImageFloat intensity_image(image_rows_, image_cols_);
    srrg2_core::ImageFloat temp_intensity_image(image_rows_, image_cols_);
    float min_intensity = std::numeric_limits<float>::max();
    float max_intensity = std::numeric_limits<float>::min();
    for (size_t r = 0; r < image_rows_; ++r) {
      for (size_t c = 0; c < image_cols_; ++c) {
        if (!projection.at(r, c).valid) {
          continue;
        }
        temp_intensity_image.at(r, c) = projection.at(r, c).intensity;
        depth_image.at(r, c)          = projection.at(r, c).depth;

        if (projection.at(r, c).intensity < min_intensity) {
          min_intensity = projection.at(r, c).intensity;
        }
        if (projection.at(r, c).intensity > max_intensity) {
          max_intensity = projection.at(r, c).intensity;
        }
      }
    }
    std::cerr << "max intensity: " << max_intensity << " min intensity: " << min_intensity
              << std::endl;

    // go to cv
    depth_image.toCv(cv_depth_);
    temp_intensity_image.toCv(cv_intensity_);

    // normalize and convert intensity
    cv_intensity_ = cv_intensity_ / max_intensity_normalizer_;
    cv_intensity_.convertTo(cv_intensity_, CV_8UC1, 255);

    // apply bilateral filter
    // cv::Mat temp;
    // cv::bilateralFilter(BaseType::_cv_intensity__image, temp, 5, 10.f, 2.5f,
    // cv::BORDER_DEFAULT);
    // BaseType::_cv_intensity__image = temp.clone();

    // intensity_image.clear();
    // intensity_image.fromCv(cv_intensity_);
  }

} // namespace md_slam
