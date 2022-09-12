#pragma once
#include "image_pyramid.h"
#include <srrg_geometry/geometry3d.h>
#include <srrg_system_utils/chrono.h>

namespace md_slam {
  using namespace srrg2_core;

  // common stuff for tracking and adjustment factors
  // defines workspace and functions for projecting pyramid

  class MDFactorCommon {
  public:
    //! c'tor for initializing factor vars
    MDFactorCommon();

    //! set the Pyramid for the current factor
    void setFixed(const MDPyramidLevel& pyramid_level_);

    //! set the Cloud for the current factor
    void setMoving(const MDPyramidLevel& pyramid_level_);

    // debug
    void toTiledImage(ImageVector3f& canvas) const;

    Chrono::ChronoMap timings;

    inline float omegaDepth() const {
      return _omega_depth;
    }
    inline float omegaIntensity() const {
      return _omega_intensity;
    }
    inline float omegaNormals() const {
      return _omega_normals;
    }
    inline float depthRejectionThreshold() const {
      return _depth_error_rejection_threshold;
    }
    inline float kernelChiThreshold() const {
      return _kernel_chi_threshold;
    }
    inline void setOmegaDepth(float v) {
      _omega_depth = v;
    }
    inline void setOmegaIntensity(float v) {
      _omega_intensity = v;
    }
    inline void setOmegaNormals(float v) {
      _omega_normals = v;
    }
    inline void setDepthRejectionThreshold(float v) {
      _depth_error_rejection_threshold = v;
    }
    inline void setKernelChiThreshold(float v) {
      _kernel_chi_threshold = v;
    }

  protected:
    //! performs the occlusion check
    void computeProjections();

    void setMovingInFixedEstimate(const Isometry3f&);

    //! An entry of the Solver is represented in this way TODO move outside?
    struct WorkspaceEntry {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      WorkspaceEntry() {
        _prediction[1] = std::numeric_limits<float>::max();
      }
      // prediction the point values on  the fixed frame
      Vector5f _prediction = Vector5f::Zero();

      // point in the moving frame, before applying transform
      Vector3f _point = Vector3f::Zero();

      // normal in the moving frame, before applying transform
      Vector3f _normal = Vector3f::Zero();

      // point transformed on the fixed frame
      Vector3f _transformed_point = Vector3f::Zero(); // point after applying transform

      // transformed point after applying the camera matrix/in polar for spherical

      Vector3f _camera_point = Vector3f::Zero(); // camera point in camera coords

      // point in the image frame
      Vector2f _image_point   = Vector2f::Zero(); // point in image coords
      int _index              = -1;               // index of projected point
      float _chi              = 0.f;
      PointStatusFlag _status = Good;

      const float& intensity() {
        return _prediction[0];
      }
      const float& depth() {
        return _prediction[1];
      }

      Vector5f _error = Vector5f::Zero(); // error
    };
    using Workspace = srrg2_core::Matrix_<WorkspaceEntry, Eigen::aligned_allocator<WorkspaceEntry>>;

    const MDPyramidLevel* _level_ptr = 0;

    // parameters:
    float _omega_intensity = 1.f; //

    // default parameters
    float _omega_depth                     = 5.f;
    float _omega_normals                   = 1.f;
    float _depth_error_rejection_threshold = .25f; // if depth error>that eliminate
    float _kernel_chi_threshold            = 1.f;  // if chi> this suppress/kernelize

    // moving cloud in robot frame
    MDMatrixCloud _cloud;
    Workspace _workspace;
    // static thread_local Workspace _workspace;

    int _rows;
    int _cols;
    float _min_depth;
    float _max_depth;
    int _max_level;
    srrg2_core::Matrix3f _sensor_offset_rotation_inverse;
    srrg2_core::Isometry3f _sensor_offset_inverse;
    srrg2_core::Matrix3f _camera_matrix;
    CameraType _camera_type;

    float _omega_intensity_sqrt;
    float _omega_depth_sqrt;
    float _omega_normals_sqrt;

    Isometry3f _sensor_in_robot_inverse;
    Isometry3f _SX;      // product between _sensor_in_robot_inverse and X
    Matrix3f _neg2rotSX; // two times the rotation of SX, used in jacobians
  };
} // namespace md_slam
