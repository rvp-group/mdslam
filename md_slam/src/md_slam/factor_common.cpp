#include "factor.h"
#include <md_slam/utils.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_system_utils/chrono.h>

namespace md_slam {

  using namespace std;
  using namespace srrg2_core;

  // thread_local MDFactorCommon::Workspace MDFactorCommon::_workspace;

  MDFactorCommon::MDFactorCommon() {
    _rows                 = 0;
    _cols                 = 0;
    _min_depth            = 0.f;
    _max_depth            = 0.f;
    _omega_intensity_sqrt = 0.f;
    _omega_depth_sqrt     = 0.f;
    _omega_normals_sqrt   = 0.f;
    _camera_type          = CameraType::Pinhole;
    _sensor_offset_rotation_inverse.setIdentity();
    _sensor_offset_inverse.setIdentity();
    _camera_matrix.setIdentity();
  }

  void MDFactorCommon::setFixed(const MDPyramidLevel& pyramid_level_) {
    _level_ptr                      = &pyramid_level_;
    _rows                           = _level_ptr->rows();
    _cols                           = _level_ptr->cols();
    _min_depth                      = _level_ptr->min_depth;
    _max_depth                      = _level_ptr->max_depth;
    _camera_matrix                  = _level_ptr->camera_matrix;
    _camera_type                    = _level_ptr->camera_type;
    _sensor_offset_inverse          = _level_ptr->sensor_offset.inverse();
    _sensor_offset_rotation_inverse = _sensor_offset_inverse.linear();
    _workspace.resize(_level_ptr->rows(), _level_ptr->cols());
  }

  void MDFactorCommon::setMoving(const MDPyramidLevel& pyramid_level_) {
    pyramid_level_.toCloud(_cloud);
  }

  void MDFactorCommon::setMovingInFixedEstimate(const Isometry3f& X) {
    _SX        = _sensor_offset_inverse * X;
    _neg2rotSX = -2.f * _SX.linear();
  }

  void MDFactorCommon::computeProjections() {
    Chrono t_proj("projections", &timings, false);
    _workspace.fill(WorkspaceEntry());

    size_t i = 0;
    for (MDMatrixCloud::const_iterator it = _cloud.begin(); it != _cloud.end(); ++it, ++i) {
      const PointNormalIntensity3f& full_point = *it;
      const Vector3f& point                    = full_point.coordinates();
      const Vector3f& normal                   = full_point.normal();
      const float intensity                    = full_point.intensity();
      const Vector3f transformed_point         = _SX * point;

      float depth           = 0.f;
      Vector3f camera_point = Vector3f::Zero();
      Vector2f image_point  = Vector2f::Zero();

      const bool& is_good = project(image_point,
                                    camera_point,
                                    depth,
                                    transformed_point,
                                    _camera_type,
                                    _camera_matrix,
                                    _min_depth,
                                    _max_depth);
      if (!is_good)
        continue;

      const int irow = cvRound(image_point.y());
      const int icol = cvRound(image_point.x());

      if (!_workspace.inside(irow, icol)) {
        continue;
      }

      // check if masked
      if (_level_ptr->matrix.at(irow, icol).masked()) {
        continue;
      }

      WorkspaceEntry& entry = _workspace(irow, icol);

      if (depth > entry.depth())
        continue;

      const Vector3f rn = _SX.linear() * normal;
      entry._prediction << intensity, depth, rn.x(), rn.y(), rn.z();
      entry._point             = point;
      entry._normal            = normal;
      entry._transformed_point = transformed_point;
      entry._camera_point      = camera_point;
      entry._image_point       = image_point;
      entry._index             = i;
      entry._chi               = 0;
      entry._status            = Good;
    }
  }

  void MDFactorCommon::toTiledImage(ImageVector3f& canvas) const {
    const int rows = _workspace.rows();
    const int cols = _workspace.cols();
    canvas.resize(rows * 3, cols);
    canvas.fill(Vector3f::Zero());
    Vector3f* dest_intensity = &canvas.at(0, 0);
    Vector3f* dest_depth     = &canvas.at(rows, 0);
    Vector3f* dest_normal    = &canvas.at(2 * rows, 0);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        const WorkspaceEntry& src = _workspace.at(r, c);
        if (src._status == Good) {
          float intensity = src._error(0);
          float depth     = src._error(1);
          Vector3f normal = src._error.block<3, 1>(2, 0);
          *dest_intensity = Vector3f(intensity, intensity, intensity);
          *dest_depth     = Vector3f(depth, depth, depth);
          *dest_normal    = normal;
        }
        ++dest_intensity;
        ++dest_depth;
        ++dest_normal;
      }
    }
  }

} // namespace md_slam
