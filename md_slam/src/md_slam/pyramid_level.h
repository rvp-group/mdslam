#pragma once
#include "utils.h"
#include <iostream>
#include <srrg_boss/blob.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_image/image.h>
#include <srrg_pcl/point_types.h>
#include <vector>

namespace md_slam {

  using srrg2_core::ImageFloat;
  using srrg2_core::ImageUInt8;
  using srrg2_core::ImageVector3f;
  using srrg2_core::Isometry3f;
  using srrg2_core::Matrix3f;
  using srrg2_core::Matrix5_2f;
  using srrg2_core::PointNormalIntensity3f;
  using srrg2_core::Vector2f;
  using srrg2_core::Vector5f;
  using MDMatrixCloud =
    srrg2_core::PointCloud_<srrg2_core::Matrix_<srrg2_core::PointNormalIntensity3f,
                                                Eigen::aligned_allocator<PointNormalIntensity3f>>>;

  using MDVectorCloud = srrg2_core::PointNormalIntensity3fVectorCloud;

  enum ChannelType { Intensity = 0x0, Depth = 0x1, Normal = 0x2 };
  enum FilterPolicy { Ignore = 0, Suppress = 1, Clamp = 2 };
  enum PointStatusFlag {
    Good            = 0x00,
    Outside         = 0x1,
    DepthOutOfRange = 0x2,
    Masked          = 0x3,
    Occluded        = 0x4,
    DepthError      = 0x5,
    Invalid         = 0x7
  };

  // MDPyramidMatrixEntry: represents an entry of a MD Image,
  // in terms of both point and derivatives
  struct MDPyramidMatrixEntry {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector5f value;         // [i, d, nx, ny, nz]: intensity, depth, normal
    Matrix5_2f derivatives; // col and row derivatives
#ifdef _MD_ENABLE_SUPERRES_
    float r = 0, c = 0; // subpixel coordinates
#endif
    bool _masked = true;
    MDPyramidMatrixEntry() {
      value.setZero();
      derivatives.setZero();
    }
    inline float intensity() const {
      return value(0);
    }
    inline void setIntensity(float intensity_) {
      value(0) = intensity_;
    }
    inline float depth() const {
      return value(1);
    }
    inline void setDepth(float depth_) {
      value(1) = depth_;
    }
    inline Eigen::Vector3f normal() const {
      return value.block<3, 1>(2, 0);
    }
    inline void setNormal(const Eigen::Vector3f& n) {
      value.block<3, 1>(2, 0) = n;
    }
    inline bool masked() const {
      return _masked;
    }
    inline void setMasked(bool masked_) {
      _masked = masked_;
    }
  };

  using MDPyramidMatrix =
    srrg2_core::Matrix_<MDPyramidMatrixEntry, Eigen::aligned_allocator<MDPyramidMatrixEntry>>;

  // MDPyramidLevel: contains all the data of a specific pyramid level, i.e.
  //  - the image as a MDPyramidMatrix
  //  - a mask of valid points
  //  - camera matrix at this level of pyramid
  //  - image size
  //  - the corresponding cloud
  // the MDPyramidGenerator takes care of initialize Levels
  class MDPyramidLevel : public srrg2_core::BLOB {
    friend class MDImagePyramid;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief PyramidLevel c'tor
    MDPyramidLevel(size_t rows_ = 0, size_t cols_ = 0);

    //! @brief resizes image and mask
    void resize(const size_t& rows_, const size_t& cols_);

    //! @brief get the MDPyramidMatrixEntry obtained with bilinear interpolation. False if outside
    inline bool getSubPixel(Vector5f& v, Matrix5_2f& d, const Vector2f& image_point) const;

    //! @brief rows of a level
    size_t rows() const {
      return matrix.rows();
    }
    //! @brief cols of a level
    size_t cols() const {
      return matrix.cols();
    }

    //! @brief generates a cloud based on the pyramid
    void toCloud(MDMatrixCloud& target) const; // puts a cloud rendered from self in target

    //! @brief generates the level based on the cloud
    //! the matrix should be resized before calling the method
    //! uses the embedded parameters
    void fromCloud(MDMatrixCloud& src_cloud); // fills own values from src_cloud

    //! @brief produces a 3x3  tiled image of the pyramid for debug
    void toTiledImage(ImageVector3f& canvas);

    //! @brief: these are to get a cue stripped from the pyramid level
    void getIntensity(ImageFloat& intensity) const;
    void getDepth(ImageFloat& depth) const;
    void getNormals(ImageVector3f& normals) const;

    //! generates a scaled pyramid from this one. Scale should be an int
    void scaleTo(MDPyramidLevel& dest, int scale) const; // constructs a scaled version of self

    void write(std::ostream& os) const override;
    bool read(std::istream& is) override;

    float min_depth          = 0.3f;                   // minimum depth of the cloud
    float max_depth          = 50.f;                   // max depth of the cloud
    Matrix3f camera_matrix   = Matrix3f::Identity();   // camera matrix for this level
    Isometry3f sensor_offset = Isometry3f::Identity(); // sensor offset
    CameraType camera_type   = Pinhole;                // pinhole or speherical
    MDPyramidMatrix matrix;                            // image with all stuff;
    // parameters used to compute the derivatives and the mask
    float thresholds[3]      = {10.f, 0.5f, 0.5f};
    FilterPolicy policies[3] = {Ignore, Suppress, Clamp};

    // float intensity_threshold=10.f, depth_threshold=0.5f, normal_threshold=0.5f;
    // FilterPolicy intensity_policy=Ignore, depth_policy=Suppress, normal_policy=Clamp;
    int mask_grow_radius = 3;

    //! generates a scaled pyramid from this one. Scale should be an int
    void writeToCharArray(char*& dest, size_t& size) const;

    //! reads a pyramid from a byte array2
    void readFromCharArray(const char*& src, size_t& size);

  protected:
    void growMask();
    void updateDerivatives();
  };

  using MDPyramidLevelPtr = std::shared_ptr<MDPyramidLevel>;

} // namespace md_slam

// here is the definition of the inline function
#include "pyramid_level.hpp"
