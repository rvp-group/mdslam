#pragma once
#include "pyramid_level.h"

namespace md_slam {

  // MDPyramidLevelVector: Vector to store a pyramid of images
  class MDImagePyramid : public srrg2_core::BLOB {
    friend class MDPyramidGenerator;

  public:
    using MDLevelVector = std::vector<MDPyramidLevelPtr>;
    // vector interface
    inline bool empty() const {
      return _levels.empty();
    }
    const double& timestamp() const {
      return _timestamp;
    }
    inline size_t numLevels() const {
      return _levels.size();
    }
    inline void resize(size_t new_size) {
      _levels.resize(new_size);
      _relative_scales.resize(new_size);
    }
    inline const MDPyramidLevelPtr& front() const {
      return _levels.front();
    }
    inline const MDPyramidLevelPtr& back() const {
      return _levels.back();
    }
    inline const MDPyramidLevelPtr& at(size_t pos) const {
      return _levels.at(pos);
    }
    inline MDPyramidLevelPtr& at(size_t pos) {
      return _levels.at(pos);
    }

    // TODO remove this redundant shit from here
    // full resolution required for loop closures
    inline void setFullIntensity(const ImageFloat& intensity_) {
      _full_intensity = intensity_;
    }
    inline void setFullDepth(const ImageFloat& depth_) {
      _full_depth = depth_;
    }

    inline const ImageFloat& fullIntensity() const {
      return _full_intensity;
    }
    inline const ImageFloat& fullDepth() const {
      return _full_depth;
    }

    inline void setSensorOffset(const Isometry3f& sensor_offset_) {
      _sensor_offset = sensor_offset_;
    }
    inline void setCameraMatrix(const Matrix3f& camera_matrix_) {
      _camera_matrix = camera_matrix_;
    }

    inline void setCameraType(CameraType camera_type_) {
      _camera_type = camera_type_;
    }

    inline const Isometry3f& sensorOffset() const {
      return _sensor_offset;
    }

    inline const Matrix3f& cameraMatrix() const {
      return _camera_matrix;
    }

    inline CameraType cameraType() const {
      return _camera_type;
    }

    void write(std::ostream& os) const override;
    bool read(std::istream& is) override;
    // protected:
    std::vector<int> _relative_scales;
    MDLevelVector _levels;

  protected:
    //! generates a scaled pyramid from this one. Scale should be an int
    void writeToCharArray(char*& dest, size_t& size) const;
    //! reads a pyramid from a byte array2
    void readFromCharArray(const char*& src, size_t& size);
    double _timestamp = 0.0;
    srrg2_core::ImageFloat _full_intensity; // used for feature based loop closures
    srrg2_core::ImageFloat _full_depth;
    Matrix3f _camera_matrix   = Matrix3f::Identity(); // camera matrix for this level
    CameraType _camera_type   = Pinhole;
    Isometry3f _sensor_offset = Isometry3f::Identity();
  };
  using MDImagePyramidPtr       = std::shared_ptr<MDImagePyramid>;
  using MDImagePyramidReference = srrg2_core::BLOBReference<MDImagePyramid>;

} // namespace md_slam
