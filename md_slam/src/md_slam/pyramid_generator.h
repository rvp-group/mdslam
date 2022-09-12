#pragma once

#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_property/property_eigen.h>
#include <srrg_property/property_vector.h>

#include <srrg_data_structures/matrix.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_image/image.h>
#include <srrg_messages/message_handlers/message_sink_base.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_pcl/normal_computator.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_types.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_system_utils/chrono.h>
#include <srrg_viewer/active_drawable.h>

#include "pyramid_message.h"

#include <iostream>
#include <vector>

namespace md_slam {
  using namespace srrg2_core;

  using MDNormalComputator2DCrossProduct =
    srrg2_core::NormalComputator2DCrossProduct<MDMatrixCloud, 1>;
  // using MDNormalComputator2DDifferential =
  //   srrg2_core::NormalComputator2DDifferential<MDMatrixCloud, 1>;
  using MDNormalComputatorBase = srrg2_core::NormalComputatorBase<MDMatrixCloud, 1>;
  // MDPyramidGenerator: generate the pyramidLevels
  class MDPyramidGenerator : public srrg2_core::MessageSinkBase, public srrg2_core::ActiveDrawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PARAM(PropertyString,
          depth_topic,
          "topic for the depth image, registered to the intensity channel",
          "/camera/aligned_depth_to_infra/image_raw",
          nullptr);

    PARAM(PropertyString,
          intensity_topic,
          "topic for the depth image, registered to the intensity channel",
          "/camera/infra/image_raw",
          nullptr);

    PARAM(PropertyString,
          camera_info_topic,
          "topic where the camera info is broadcasted",
          "/camera/infra/camera_info",
          nullptr);

    PARAM(PropertyString,
          pyramid_output_topic,
          "topic of the pyramid to generate",
          "/md_pyramid",
          nullptr);

    PARAM(PropertyString, base_frame_id, "base_link_of_the_robot", "/base_link", nullptr);

    PARAM(PropertyBool, adaptive_blur, "set true blur adaptevely based on depth value", false, 0);

    PARAM(srrg2_core::PropertyFloat,
          min_depth,
          "min depth to consider while generating pyramids",
          0.3f,
          0);
    PARAM(srrg2_core::PropertyFloat,
          max_depth,
          "max depth to consider while generating pyramids",
          5.0f,
          0);
    PARAM(srrg2_core::PropertyInt, mask_grow_radius, "radius to grow invalid mask", 3, 0);
    PARAM(srrg2_core::PropertyUnsignedInt,
          normals_scaled_blur_multiplier,
          "normal blurring factor",
          1,
          0);
    PARAM(srrg2_core::PropertyUnsignedInt,
          normals_blur_region_size,
          "window size for normals blurring",
          3,
          0);
    PARAM(srrg2_core::PropertyFloat, intensity_derivative_threshold, "todo", 10.f, 0);
    PARAM(srrg2_core::PropertyFloat, depth_derivative_threshold, "todo", 0.5f, 0);
    PARAM(srrg2_core::PropertyFloat, normals_derivative_threshold, "todo", 0.3f, 0);
    PARAM(srrg2_core::PropertyUInt8,
          intensity_policy,
          "filter policy for Intensity {0 Ignore, 1 Suppress, 2 Clamp}",
          0,
          0);
    PARAM(srrg2_core::PropertyUInt8,
          depth_policy,
          "filter policy for Depth {0 Ignore, 1 Suppress, 2 Clamp}",
          1,
          0);
    PARAM(srrg2_core::PropertyUInt8,
          normals_policy,
          "filter policy for Normals {0 Ignore, 1 Suppress, 2 Clamp}",
          2,
          0);
    PARAM_VECTOR(srrg2_core::PropertyVector_<int>,
                 scales,
                 "Scales for Pyramids",
                 &_scales_changed_flag);
    PARAM(srrg2_core::PropertyConfigurable_<MDNormalComputatorBase>,
          normal_computator,
          "algorithm to compute the normals",
          std::shared_ptr<MDNormalComputatorBase>(new MDNormalComputator2DCrossProduct),
          nullptr);

    PARAM(PropertyInt, row_prescaling, "applies a pre-scaling to the image rows", 1, nullptr);

    PARAM(PropertyInt, col_prescaling, "applies a pre-scaling to the image cols", 1, nullptr);

    PARAM(PropertyInt,
          cam_type_override,
          "if not set to -1, overrides the type of the camera to selected value",
          CameraType::Unknown,
          &_scales_changed_flag);

    PARAM(PropertyFloat,
          depth_scale_override,
          "if not set to a negative value, overrides the depth scale in camera info",
          -1,
          &_scales_changed_flag);

    PARAM(PropertyFloat,
          radius_factor,
          "radius factor impacting bilinear filtering for depth smoothing",
          2.f,
          0);

    MDPyramidGenerator();

    inline const Isometry3f& sensorOffset() const {
      return _sensor_offset;
    }
    inline void setSensorOffset(const Isometry3f& sensor_offset_) {
      _sensor_offset = sensor_offset_;
    }

    inline const Matrix3f& cameraMatrix() const {
      return _camera_matrix_original;
    }
    inline void setCameraMatrix(const Matrix3f& camera_matrix_) {
      _camera_matrix_original = camera_matrix_;
      _camera_matrix_scaled   = camera_matrix_;
      _camera_matrix_scaled.row(0) *= (1. / param_col_prescaling.value());
      _camera_matrix_scaled.row(1) *= (1. / param_row_prescaling.value());
    }

    inline CameraType cameraType() const {
      return _camera_type;
    }
    inline void setCameraType(CameraType camera_type_) {
      _camera_type = camera_type_;
    }

    inline float depthScale() const {
      return _depth_scale;
    }
    inline void setDepthScale(float depth_scale_) {
      _depth_scale = depth_scale_;
    }

    inline size_t rows() const {
      return _depth.rows();
    }
    inline size_t cols() const {
      return _depth.cols();
    }

    //! @brief sets the images, and allocates the ws accordingly
    void setImages(const srrg2_core::ImageUInt16& raw_depth,
                   const srrg2_core::BaseImage& raw_intensity);

    //! @brief get the pyramid vector calculated after compute()
    inline MDImagePyramidMessagePtr& pyramidMessage() {
      return _pyramid_msg;
    }

    //! @brief calculates the pyramid (stored in _pyramid)
    void compute();

    virtual ~MDPyramidGenerator();

  protected:
    inline MDImagePyramid* pyramid() {
      if (_pyramid_msg)
        return _pyramid_msg->get();
    }

    using PointUnprojectorBase =
      srrg2_core::PointUnprojectorBase_<srrg2_core::PointNormalIntensity3fVectorCloud>;
    using PointUnprojectorPinhole =
      srrg2_core::PointUnprojectorPinhole_<srrg2_core::PointNormalIntensity3fVectorCloud>;
    using PointUnprojectorPolar =
      srrg2_core::PointUnprojectorPolar_<srrg2_core::PointNormalIntensity3fVectorCloud>;

    bool _is_setup                   = false;
    float _depth_scale               = 1e-3f;
    Matrix3f _camera_matrix_original = Matrix3f::Identity();
    Matrix3f _camera_matrix_scaled   = Matrix3f::Identity();
    Isometry3f _sensor_offset        = Isometry3f::Identity();
    CameraType _camera_type          = Pinhole;

    MDImagePyramidMessagePtr _pyramid_msg; // the generated Pyramid
    MDPyramidLevel _top_pyramid;           // top level of the pyramid

    srrg2_core::ImageFloat _depth;          // depth image [m]
    srrg2_core::ImageFloat _intensity;      // grayscale image [0:1]
    srrg2_core::ImageFloat _full_intensity; // for loop closures when original img is scaled [m]
    srrg2_core::ImageFloat _full_depth;     // for loop closures when original img is scaled [0:1]
    srrg2_core::ImageVector3f _normals;     // normals of input image
    srrg2_core::ImageVector3f _points;      // 2D points of input image
    srrg2_core::ImageUInt8 _mask;
    bool _scales_changed_flag = false;
    std::unique_ptr<PointUnprojectorBase> _unprojector;
    MDMatrixCloud _cloud;
    MDPyramidLevel _level_zero;
    size_t _seq = 0;

    // TODO maybe try with different sigma values
    // sigma in bilateral filtering (pixel)
    const float _sigma_pixel     = 1.5f;
    const float _sigma_inv_depth = 0.005f;

    //! @brief prepares the storage
    void allocatePyramids();
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg) override;
    void _setupProjector(CameraInfoMessagePtr camera_info_);
    void _drawImpl(ViewerCanvasPtr gl_canvas_) const override;
    Chrono::ChronoMap _timings;
  };

  using MDPyramidGeneratorPtr = std::shared_ptr<MDPyramidGenerator>;

} // namespace md_slam
