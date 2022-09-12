#pragma once
#include <srrg_config/configurable.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_pcl/point_intensity.h>
#include <srrg_property/property.h>

namespace md_slam {
  class MDLidarConfiguration : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // standard config for OS1-64
    PARAM(srrg2_core::PropertyFloat,
          max_intensity,
          "lidar maximum reflectivity-intensity valuse used for normalization",
          500.f,
          0);
    PARAM(srrg2_core::PropertyUnsignedInt,
          image_rows,
          "projection image rows (lidar model dependent, usually number of vertical beams)",
          64,
          0);
    PARAM(srrg2_core::PropertyUnsignedInt,
          image_cols,
          "projection image cols (can vary arbitrarely)",
          1024,
          0);
    PARAM(srrg2_core::PropertyFloat,
          min_depth,
          "minimum point depth to be considered in projection, lower values are discarded",
          0.3f,
          0);
    PARAM(srrg2_core::PropertyFloat,
          max_depth,
          "maximum point depth to be considered in projection, higher values are discarded",
          120.f,
          0);
    PARAM(srrg2_core::PropertyFloat,
          vfov_max,
          "maximum vertical fov (in DEG), usually upper - if left to zero are automatically "
          "calculated -> suggested!",
          0.f,
          0);
    PARAM(srrg2_core::PropertyFloat,
          vfov_min,
          "minimum vertical fov (in DEG), usually lower - if left to zero are automatically "
          "calculated -> suggested!",
          0.f,
          0);
    PARAM(
      srrg2_core::PropertyFloat,
      hfov_max,
      "maximum vertical fov (in DEG), usually left - if left to zero are automatically calculated",
      360.f,
      0);
    PARAM(
      srrg2_core::PropertyFloat,
      hfov_min,
      "minimu vertical fov (in DEG), usually right - if left to zero are automatically calculated",
      0.f,
      0);
    PARAM(srrg2_core::PropertyString,
          point_cloud_topic,
          "name of the point cloud lidar topic",
          "",
          0);

    MDLidarConfiguration() {
    }
    ~MDLidarConfiguration() = default;

    const float& vFOVmin() const {
      return _vfov_min;
    }
    const float& vFOVmax() const {
      if (_vfov_max == 0.f)
        throw std::runtime_error("MDLidarConfiguration|vFOVmax call calculateVerticalFOV() first");
      return _vfov_max;
    }
    const float& hFOVmin() const {
      return _hfov_min;
    }
    const float& hFOVmax() const {
      return _hfov_max;
    }

    inline void calculateVerticalFOV(const srrg2_core::PointIntensity3fVectorCloud& lidar_cloud_) {
      float min_elevation = std::numeric_limits<float>::max();
      float max_elevation = std::numeric_limits<float>::min();
      for (const auto& lidar_point : lidar_cloud_) {
        const srrg2_core::Vector3f& p = lidar_point.coordinates();
        const float range             = p.norm();
        if (range < 1e-8f) { // TODO use param min depth
          continue;
        }
        const float elevation = std::asin(p.z() / range);
        if (elevation < min_elevation) {
          min_elevation = elevation;
        }
        if (elevation > max_elevation) {
          max_elevation = elevation;
        }
      }
      _vfov_min = min_elevation;
      _vfov_max = max_elevation;
    }

    inline void
    calculateHorizontalFOV(const srrg2_core::PointIntensity3fVectorCloud& lidar_cloud_) {
      float min_azimuth = std::numeric_limits<float>::max();
      float max_azimuth = std::numeric_limits<float>::min();
      for (const auto& lidar_point : lidar_cloud_) {
        const srrg2_core::Vector3f& p = lidar_point.coordinates();
        if (p.norm() < 1e-8f) {
          continue;
        }
        const float azimuth = atan2(p.y(), p.x());
        if (azimuth < min_azimuth) {
          min_azimuth = azimuth;
        }
        if (azimuth > max_azimuth) {
          max_azimuth = azimuth;
        }
      }
      _hfov_min = min_azimuth;
      _hfov_max = max_azimuth;
    }

  protected:
    float _hfov_min = 0.f;
    float _hfov_max = 0.f;
    float _vfov_min = 0.f;
    float _vfov_max = 0.f;
  };
} // namespace md_slam
