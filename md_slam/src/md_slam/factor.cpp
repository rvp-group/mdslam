#include "factor.h"
#include "utils.h"
#include <srrg_system_utils/chrono.h>

namespace md_slam {

  using namespace std;
  using namespace srrg2_core;

  int MDFactor::measurementDim() const {
    return 5;
  }

  PointStatusFlag MDFactor::errorAndJacobian(srrg2_core::Vector5f& e_,
                                             srrg2_core::Matrix5_6f& J_,
                                             WorkspaceEntry& entry_,
                                             bool chi_only) {
    PointStatusFlag status            = Good;
    const float z                     = entry_.depth();
    const float iz                    = 1.0f / z;
    const Vector3f& point             = entry_._point;
    const Vector3f& normal            = entry_._normal;
    const Vector3f& transformed_point = entry_._transformed_point;
    const Vector3f& camera_point      = entry_._camera_point;
    const Vector2f& image_point       = entry_._image_point;

    Vector5f measurement;
    Matrix5_2f image_derivatives;
    const bool ok = _level_ptr->getSubPixel(measurement, image_derivatives, image_point);
    if (!ok) {
      return Masked;
    }

    // in error put the difference between prediction and measurement
    e_            = entry_._prediction - measurement;
    entry_._error = e_;
    e_(0) *= _omega_intensity_sqrt;
    e_(1) *= _omega_depth_sqrt;
    e_.tail(3) *= _omega_normals_sqrt;

    // if depth error is to big we drop
    if (std::pow(e_(1), 2) > _depth_error_rejection_threshold)
      return DepthError;
    if (chi_only)
      return status;

    J_.setZero();
    // compute the pose jacobian, including sensor offset
    _J_icp.block<3, 3>(0, 3) = _neg2rotSX * geometry3d::skew((const Vector3f)(point));

    // extract values from hom for readability
    const float iz2 = iz * iz;

    // extract the values from camera matrix
    const float& fx = _camera_matrix(0, 0);
    const float& fy = _camera_matrix(1, 1);
    const float& cx = _camera_matrix(0, 2);
    const float& cy = _camera_matrix(1, 2);

    // computes J_hom*K explicitly to avoid matrix multiplication and stores it in J_proj
    Matrix2_3f J_proj = Matrix2_3f::Zero();

    switch (_camera_type) {
      case CameraType::Pinhole:
        // fill the left  and the right 2x3 blocks of J_proj with J_hom*K
        J_proj(0, 0) = fx * iz;
        J_proj(0, 2) = cx * iz - camera_point.x() * iz2;
        J_proj(1, 1) = fy * iz;
        J_proj(1, 2) = cy * iz - camera_point.y() * iz2;

        // add the jacobian of depth prediction to row 1.
        J_.row(1) = _J_icp.row(2);

        break;
      case CameraType::Spherical: {
        const float ir    = iz;
        const float ir2   = iz2;
        const float rxy2  = transformed_point.head(2).squaredNorm();
        const float irxy2 = 1. / rxy2;
        const float rxy   = sqrt(rxy2);
        const float irxy  = 1. / rxy;

        J_proj << -fx * transformed_point.y() * irxy2, // 1st row
          fx * transformed_point.x() * irxy2, 0,
          -fy * transformed_point.x() * transformed_point.z() * irxy * ir2, // 2nd row
          -fy * transformed_point.y() * transformed_point.z() * irxy * ir2, fy * rxy * ir2;

        Matrix1_3f J_depth; // jacobian of range(x,y,z)
        J_depth << transformed_point.x() * ir, transformed_point.y() * ir,
          transformed_point.z() * ir;

        // add the jacobian of range prediction to row 1.
        J_.row(1) = J_depth * _J_icp;

      } break;
      default:
        throw std::runtime_error("MDFactor::errorAndJacobian|unknown camera model");
    }
    // chain rule on projection + pose
    Matrix2_6f J_proj_icp = J_proj * _J_icp;

    // chain rule to get the jacobian
    J_.noalias() -= image_derivatives * J_proj_icp;

    // massage the jacobian by subtracting the derivatives of the projected quantities
    // J_normal/dPert += skew(-predicted normal)
    J_.block<3, 3>(2, 3).noalias() += _neg2rotSX * geometry3d::skew((const Vector3f) normal);

    // Omega is diagonal matrix
    // to avoid multiplications we premultiply the rows of J by sqrt of diag
    // elements
    J_.row(0) *= _omega_intensity_sqrt;
    J_.row(1) *= _omega_depth_sqrt;
    J_.block<3, 2>(2, 0) *= _omega_normals_sqrt;
    return status;
  }

  inline void computeAtxA(Matrix6d& dest, const Matrix5_6f& src, const float& lambda) {
    for (int c = 0; c < 6; ++c) {
      for (int r = 0; r <= c; ++r) {
        dest(r, c) += static_cast<double>(src.col(r).dot(src.col(c)) * lambda);
      }
    }
  }

  inline void copyLowerTriangleUp(Matrix6d& A) {
    for (int c = 0; c < 6; ++c) {
      for (int r = 0; r < c; ++r) {
        A(c, r) = A(r, c);
      }
    }
  }

  // https : // en.wikipedia.org/wiki/Kahan_summation_algorithm
  template <typename MatType>
  inline MatType kahanSum(const std::vector<MatType>& vec_) {
    MatType sum = vec_[0];
    MatType c   = MatType::Zero(); // a running compensation for lost low-order bits
    for (size_t i = 1; i < vec_.size(); ++i) {
      const MatType y = vec_[i] + c; // so far so good, c is 0
      const MatType t = sum + y;     // sum is big, y small, so low-order digits of y are lost
      c   = (t - sum) - y; // recovers the high-order part of y; subtrac y recovers -(low part of y)
      sum = t; // next time around, the lost low part will be added to y in a fresh attempt.
    }
    return sum;
  }

  void MDFactor::linearize(bool chi_only) {
    Chrono t_lin("linearize", &timings, false);
    _omega_intensity_sqrt    = std::sqrt(_omega_intensity);
    _omega_depth_sqrt        = std::sqrt(_omega_depth);
    _omega_normals_sqrt      = std::sqrt(_omega_normals);
    _neg2rotSX               = -2.f * _SX.linear();
    _J_icp.block<3, 3>(0, 0) = _SX.linear();

    float total_chi    = 0;
    size_t num_inliers = 0;

    std::vector<Matrix6d> H_container;
    std::vector<Vector6d> b_container;

    const int system_size = _workspace.rows() * _workspace.cols();
    const double scaling  = 1.0 / system_size;
    H_container.reserve(system_size);
    b_container.reserve(system_size);

    int num_good = 0;
    for (size_t r = 0; r < _workspace.rows(); ++r) {
      WorkspaceEntry* entry_ptr = _workspace.rowPtr(r);
      Vector5f e;
      Matrix5_6f J;
      for (size_t c = 0; c < _workspace.cols(); ++c, ++entry_ptr) {
        WorkspaceEntry& entry = *entry_ptr;
        const int idx         = entry._index;

        if (idx < 0) {
          continue;
        }

        PointStatusFlag& status = entry._status;
        if (status != Good) {
          continue;
        }
        status = errorAndJacobian(e, J, entry, chi_only);
        if (status != Good) {
          continue;
        }
        ++num_good;
        float chi    = e.dot(e);
        entry._chi   = chi;
        float lambda = 1;
        if (chi > _kernel_chi_threshold) {
          lambda = sqrt(_kernel_chi_threshold / chi);
        } else {
          ++num_inliers;
        }
        total_chi += chi;
        if (!chi_only) {
          // using double the precision for accumulation
          Matrix6d H_contrib = Matrix6d::Zero();
          computeAtxA(H_contrib, J, lambda); // TODO magic scaling number 0.0242742
          const Vector6d b_contrib = (J.transpose() * e * lambda).cast<double>();
          // summation will be evaluated later
          H_container.emplace_back(H_contrib * scaling);
          b_container.emplace_back(b_contrib * scaling);
        }
      }
    }

    _stats.chi = total_chi / num_good;
    if (!chi_only) {
      // initially container are empty
      if (b_container.empty()) {
        return;
      }

      H_container.shrink_to_fit();
      b_container.shrink_to_fit();

      Matrix6d H = kahanSum<Matrix6d>(H_container);
      Vector6d b = kahanSum<Vector6d>(b_container);

      copyLowerTriangleUp(H);

      Eigen::Map<Matrix6f> _H(this->_H_blocks[0]->storage());
      Eigen::Map<Vector6f> _b(this->_b_blocks[0]->storage());
      _H.noalias() += H.cast<float>();
      _b.noalias() -= b.cast<float>();
      return;
    }
  }

  void MDFactor::compute(bool chi_only, bool force) {
    Chrono time("compute", &timings, false);
    assert(level() != -1 && "MDFactor::compute|current factor active level unset");

    if (!this->isActive() && !force) {
      return;
    }

    if (level() != this->currentLevel()) {
      _stats.status = srrg2_solver::FactorStats::Suppressed;
      return;
    }

    // bdc this factor is seen by the solver as a single contribution for the solver
    _stats.status = srrg2_solver::FactorStats::Inlier;

    // bdc checks if needed variables are correctly set
    assert((_level_ptr->rows() != 0 || _level_ptr->cols()) != 0 &&
           "MDFactor::compute|level rows or columns set to zero");
    assert((_level_ptr->max_depth != 0.f || _level_ptr->min_depth != 0.f) &&
           "MDFactor::compute|level level max_depth or min_depth set to zero");

    setMovingInFixedEstimate(_variables.at<0>()->estimate());

    if (this->_variables.updated()) {
      // bdc computes cloud projections into workspace vector
      computeProjections();
    }

    // bdc performs linearization through jacobian computation
    linearize(chi_only);
  }

  bool MDFactor::isValid() const {
    // TODO
    return true;
  }

  void MDFactor::serialize(ObjectData& odata, IdContext& context) {
    Identifiable::serialize(odata, context);
    odata.setInt("graph_id", graphId());
    odata.setBool("enabled", enabled());
    odata.setInt("level", level());
    ArrayData* adata = new ArrayData;
    for (int pos = 0; pos < NumVariables; ++pos) {
      adata->add((int) variableId(pos));
    }
    odata.setField("variables", adata);
    odata.setFloat("omega_intensity", _omega_intensity);
    odata.setFloat("omega_depth", _omega_depth);
    odata.setFloat("omega_normals", _omega_normals);
    odata.setFloat("dept_rejection", _depth_error_rejection_threshold);
    odata.setFloat("kernel_chi", _kernel_chi_threshold);
  }

  void MDFactor::deserialize(ObjectData& odata, IdContext& context) {
    Identifiable::deserialize(odata, context);
    _graph_id = odata.getInt("graph_id");
    if (odata.getField("enabled")) {
      FactorBase::_enabled = odata.getBool("enabled");
    }
    if (odata.getField("level")) {
      FactorBase::setLevel(odata.getInt("level"));
    }
    ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField("variables"));
    int pos          = 0;
    for (auto it = adata->begin(); it != adata->end(); ++it) {
      ThisType::_variables.setGraphId(pos, (*it)->getInt());
      ++pos;
    }
    _omega_intensity                 = odata.getFloat("omega_intensity");
    _omega_depth                     = odata.getFloat("omega_depth");
    _omega_normals                   = odata.getFloat("omega_normals");
    _depth_error_rejection_threshold = odata.getFloat("dept_rejection");
    _kernel_chi_threshold            = odata.getFloat("kernel_chi");
  }

} // namespace md_slam
