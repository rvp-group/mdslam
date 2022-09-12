#pragma once

#include <srrg_geometry/geometry_defs.h>
#include <srrg_image/image.h>
#include <srrg_pcl/point_types.h>

namespace md_slam {
  using namespace srrg2_core;

  enum CameraType { Pinhole = 0, Spherical = 1, Unknown = -1 };

  // loads an image, depending on the type of img
  bool loadImage(srrg2_core::BaseImage& img, const std::string filename);

  void showImage(const srrg2_core::BaseImage& img, const std::string& title, int wait_time = 0);

  // handles damn images with odd rows/cols
  void prepareImage(ImageFloat& dest,
                    const ImageFloat& src,
                    int max_scale,
                    int row_scale      = 1,
                    int col_scale      = 1,
                    bool suppress_zero = false);

  bool project(Vector2f& image_point_,
               Vector3f& camera_point_,
               float& depth_,
               const Vector3f& point_,
               const CameraType& camera_type_,
               const Matrix3f& camera_mat_,
               const float& min_depth_,
               const float& max_depth_);

  void sparseProjection(Point2fVectorCloud& dest_,
                        const Point3fVectorCloud& source_,
                        const Matrix3f& cam_matrix_,
                        const float& min_depth_,
                        const float& max_depth_,
                        const int& n_rows_,
                        const int& n_cols_,
                        const CameraType& camera_type_);

  ImageUInt8 calculateKernelImg(const ImageFloat& depth_img_,
                                const float min_depth_,
                                const int num_bins_            = 5,
                                const float max_binning_depth_ = 20.f);

  ImageFloat adaptiveDepthSmoothing(const ImageFloat& depth_,
                                    const ImageUInt8& discretized_depth_,
                                    const float min_depth_,
                                    const float max_depth_);

  ImageVector3f adaptiveNormalsSmoothing(const ImageVector3f& normals_,
                                         const ImageUInt8& kernel_img_);

  void bilateralFiltering(ImageUInt16& out_img_,
                          const ImageUInt16& in_img_,
                          const float& radius_factor_,
                          const float& sigma_pixel_,
                          const float& inv_depth_sigma_,
                          const float& max_depth_,
                          const float& scale_);

} // namespace md_slam
