#include "loop_validator.h"
#include <srrg_geometry/permutation_sampler.cpp>

using namespace srrg2_core;

namespace md_slam_closures {

  LoopValidator::LoopValidator() {
  }

  LoopValidator::~LoopValidator() {
  }

  int LoopValidator::_computeMatchesProjective(float& error_,
                                               CorrespondenceVector& tmp_correspondences_,
                                               const Point3fVectorCloud& tmp_ref_cloud_) {
    const int& num_rows = _associations.at(0).fixed->parent_frame->cvImage().rows;
    const int& num_cols = _associations.at(0).fixed->parent_frame->cvImage().cols;
    Point2fVectorCloud features_reprojected;
    const md_slam::CameraType cam_type =
      static_cast<md_slam::CameraType>(param_camera_type.value());
    md_slam::sparseProjection(features_reprojected,
                              tmp_ref_cloud_,
                              _cam_matrix,
                              _unprojector->param_range_min.value(),
                              _unprojector->param_range_max.value(),
                              num_rows,
                              num_cols,
                              cam_type);
    assert(_associations.size() == tmp_ref_cloud_.size() &&
           "LoopValidator::_computeMatchesProjective|clouds and associations size "
           "mismatch");
    tmp_correspondences_.clear();
    tmp_correspondences_.reserve(_associations.size());
    // TODO query as int from image, ref as float from reprojection function
    for (size_t i = 0; i < _associations.size(); ++i) {
      const auto& query_match = _associations[i].fixed;
      const Vector2f qp(query_match->kp.pt.x, query_match->kp.pt.y);

      const auto& ref_match = features_reprojected[i];
      if (ref_match.status == POINT_STATUS::Invalid) {
        continue;
      }

      // calculate reprojection error
      const float r_error = (qp - ref_match.coordinates()).squaredNorm();
      if (r_error < param_max_reprojection_error.value()) {
        tmp_correspondences_.emplace_back(Correspondence(i, i, r_error));
        error_ += r_error;
      }
    }
    tmp_correspondences_.shrink_to_fit();
    error_ /= tmp_correspondences_.size();
    return tmp_correspondences_.size();
  }

  Isometry3f LoopValidator::_solveLinear(const std::vector<Vector3f>& q_points,
                                         const std::vector<Vector3f>& r_points,
                                         const Vector3d& q_centroid_,
                                         const Vector3d& r_centroid_,
                                         const size_t& num_matches_) {
    MatrixXd zigma = MatrixXd::Zero(3, 3);
    // add contribution of each point and get a shitty 3x3
    for (size_t i = 0; i < num_matches_; ++i) {
      zigma += (q_points[i].cast<double>() - q_centroid_) *
               (r_points[i].cast<double>() - r_centroid_).transpose();
    }
    // enforce orthonormality for rotation matrix creation
    Matrix3d W = Matrix3d::Identity();
    // TODO this dynamic shit is pissing me off, in Release no problems with
    // Mat3
    Eigen::JacobiSVD<MatrixXd> svd(zigma, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0) {
      W(2, 2) = -1.0;
    }
    Isometry3f estimate = Isometry3f::Identity();
    estimate.linear()   = (svd.matrixU() * W * svd.matrixV().transpose()).cast<float>();
    // TODO maybe useful to add scale to  -> t = q - s * R * r;
    estimate.translation() =
      q_centroid_.cast<float>() - estimate.linear() * r_centroid_.cast<float>();
    return estimate;
  }

  int LoopValidator::_RANSACSVDICP(Isometry3f& estimate_,
                                   CorrespondenceVector& best_matches_,
                                   float& error_,
                                   const Point3fVectorCloud& query_cloud_,
                                   const Point3fVectorCloud& ref_cloud_,
                                   const int num_rounds_,
                                   const std::vector<double>& weights_) {
    assert(query_cloud_.size() == ref_cloud_.size() &&
           "LoopValidator::linearRelaxationRANSAC|clouds size mismatch");
    const int& min_num_matches = param_min_ransac_correspondences.value();
    const size_t cloud_size    = query_cloud_.size();
    best_matches_.clear();
    best_matches_.reserve(cloud_size);
    int best_num_inliers = 0;
    std::vector<Vector3f> query_current(min_num_matches);
    std::vector<Vector3f> ref_current(min_num_matches);
    const float inv_matches = 1.f / min_num_matches;

    Vector3d query_centroid = Vector3d::Zero();
    Vector3d ref_centroid   = Vector3d::Zero();
    std::vector<float> errors_current(cloud_size);
    std::vector<double> weights(cloud_size, 1.0);
    if (weights_.size()) {
      assert(weights.size() == cloud_size &&
             "LoopValidator::linearRelaxationRANSAC|weights size mismatch");
      weights = weights_;
    }
    PermutationSampler sampler(weights);
    // RANSACCO loop
    for (int round_num = 0; round_num < num_rounds_; ++round_num) {
      for (int i = 0; i < min_num_matches; ++i) {
        const double r   = drand48() * sampler.getSum();
        const int idx    = sampler.sampleWithRemoval(r);
        query_current[i] = query_cloud_[idx].coordinates();
        ref_current[i]   = ref_cloud_[idx].coordinates();
        query_centroid.noalias() += query_current[i].cast<double>();
        ref_centroid.noalias() += ref_current[i].cast<double>();
      }
      sampler.recoverWeights();
      query_centroid *= inv_matches;
      ref_centroid *= inv_matches;

      // solve with svd and get transformation mat
      const Isometry3f current_estimate =
        _solveLinear(query_current, ref_current, query_centroid, ref_centroid, min_num_matches);

      // transform reference cloud with estimate to find associations
      const Point3fVectorCloud ref_tmp = ref_cloud_.transform<Isometry>(current_estimate);

      // calculate inliers and check error
      float current_error = 0.f;
      CorrespondenceVector current_matches;
      const int current_inliers =
        _computeMatchesProjective(current_error, current_matches, ref_tmp);
      // update stats
      if (current_inliers > best_num_inliers) {
        estimate_        = current_estimate;
        error_           = current_error;
        best_matches_    = current_matches;
        best_num_inliers = current_inliers;
      }
    }
    std::cerr << "LoopValidator::linearRelaxationRANSAC|best obtained with "
                 "RANSAC and SVD ICP"
              << std::endl;
    std::cerr << "LoopValidator::linearRelaxationRANSAC|num inliers: " << best_num_inliers
              << " among tot matches: " << query_cloud_.size() << std::endl;
    std::cerr << "LoopValidator::linearRelaxationRANSAC|inliers error: " << error_ << std::endl;
    return best_num_inliers;
  }

  void LoopValidator::_refinementSVDICP(Isometry3f& estimate_,
                                        const Point3fVectorCloud& query_cloud_,
                                        const Point3fVectorCloud& ref_cloud_,
                                        const CorrespondenceVector& matches_) {
    const size_t tot_num_inliers = matches_.size();
    std::vector<Vector3f> query_current(tot_num_inliers);
    std::vector<Vector3f> ref_current(tot_num_inliers);
    const float inv_matches = 1.f / tot_num_inliers;
    Vector3d query_centroid = Vector3d::Zero();
    Vector3d ref_centroid   = Vector3d::Zero();
    for (size_t i = 0; i < tot_num_inliers; ++i) {
      const auto& m    = matches_[i];
      query_current[i] = query_cloud_[m.fixed_idx].coordinates();
      ref_current[i]   = ref_cloud_[m.moving_idx].coordinates();
      // std::cerr << query_current[i].transpose() << std::endl;
      // std::cerr << ref_current[i].transpose() << std::endl;
      query_centroid.noalias() += query_current[i].cast<double>();
      ref_centroid.noalias() += ref_current[i].cast<double>();
    }
    query_centroid *= inv_matches;
    ref_centroid *= inv_matches;

    // solve with svd and get transformation mat
    estimate_ =
      _solveLinear(query_current, ref_current, query_centroid, ref_centroid, tot_num_inliers);

    // // transform reference cloud with estimate to find associations
    // const Point3fVectorCloud ref_tmp = ref_cloud_.transform<Isometry>(estimate_);
    // // calculate inliers and check error
    // float current_error = 0.f;
    // CorrespondenceVector current_matches;
    // int current_inliers = _computeMatchesProjective(current_error, current_matches, ref_tmp);
    // std::cerr << "after inliers contribution - num_inliers: " << current_inliers
    //           << " reproj error: " << current_error << std::endl;
  }

  void LoopValidator::compute() {
    assert(_closure && "LoopValidator::compute|potential closure not set");
    assert((_query || _reference) && "LoopValidator::compute|query and reference not set");

    // initialize
    _is_loop_valid = false;

    std::cerr << "LoopValidator::compute|geometric verification between [ " << _query->id() << " - "
              << _reference->id() << " ]\n";

    const int num_associations = _associations.size();
    if (num_associations < param_min_ransac_correspondences.value()) {
      std::cerr << "LoopValidator::compute|not enough correspondences\n";
      return;
    }

    // use containers for 3d feautures, initialize weight speed up
    // RANSAC process with descriptors heuristics
    Point3fVectorCloud query_cloud, reference_cloud;
    query_cloud.resize(num_associations);
    reference_cloud.resize(num_associations);
    // generic lambda for fast sigmoid
    auto fast_sigmoid = [](const double x) -> double { return x / (1.0 + std::abs(x)); };
    std::vector<double> weights(num_associations);
    for (int i = 0; i < num_associations; ++i) {
      const auto& feature_match = _associations[i];
      const auto& qm            = feature_match.fixed;
      const auto& rm            = feature_match.moving;
      query_cloud[i].coordinates() << qm->kp.pt.x, qm->kp.pt.y, qm->depth_value;
      reference_cloud[i].coordinates() << rm->kp.pt.x, rm->kp.pt.y, rm->depth_value;
      // get appearance information to initialize RANSAC minimal set
      const double curr_distance = cv::norm(qm->descriptor, rm->descriptor, cv::NORM_HAMMING);
      const double curr_score    = std::abs(fast_sigmoid(curr_distance) - 1.0);
      weights[i]                 = curr_score;
    }

    // use right camera type
    switch (param_camera_type.value()) {
      case md_slam::CameraType::Pinhole:
        _unprojector.reset(new PointUnprojectorPinhole());
        break;
      case md_slam::CameraType::Spherical:
        _unprojector.reset(new PointUnprojectorPolar());
        break;
      default:
        throw std::runtime_error("LoopValidator::compute|unknown camera model");
    }

    // invalidate unprojector check on depth since we want to have exact sizes 2D-3D
    _unprojector->setCameraMatrix(_cam_matrix);
    _unprojector->param_range_min.setValue(-1.f);
    _unprojector->param_range_max.setValue(std::numeric_limits<float>::max());

    // get child unprojector and compute
    PointUnprojectorPinhole* pinhole = dynamic_cast<PointUnprojectorPinhole*>(_unprojector.get());
    PointUnprojectorPolar* polar     = dynamic_cast<PointUnprojectorPolar*>(_unprojector.get());
    if (!pinhole && !polar) {
      throw std::runtime_error("LoopValidator::compute|no unprojector set");
    }
    Point3fVectorCloud query_cloud_3d, reference_cloud_3d;
    if (pinhole) {
      pinhole->compute(query_cloud, query_cloud_3d);
      pinhole->compute(reference_cloud, reference_cloud_3d);
    } else {
      polar->compute(query_cloud, query_cloud_3d);
      polar->compute(reference_cloud, reference_cloud_3d);
    }

    // just to make sure, if sizes of 3d containers mismatch, unprojection fucked up
    if (reference_cloud_3d.size() != query_cloud_3d.size()) {
      std::cerr << "LoopValidator::compute|3d container sizes mismatch" << std::endl;
      _associations.clear();
      return;
    }

    // do SVD ICP
    float error = std::numeric_limits<float>::max();
    CorrespondenceVector matches;
    int num_inliers = _RANSACSVDICP(_estimate,
                                    matches,
                                    error,
                                    query_cloud_3d,
                                    reference_cloud_3d,
                                    param_ransac_rounds.value(),
                                    weights);

    // TODO lidar
    // tuning validator based on size of rotation
    // int num_inliers_preferred = param_min_num_inliers.value();
    // // TODO decide an auto-tuning threshold especially for lidar shit
    // // const Quaternionf quat = geometry3d::r2q((Matrix3f) _estimate.linear());
    // // std::cerr << "LoopValidator::compute|relative rotation: \n";
    // // std::cerr << quat.coeffs().transpose() << std::endl; // x y z w
    // // if (quat.w() < 0.2f) {
    // //   // num_inliers_preferred *= 0.3f;
    // //   num_inliers_preferred = 4;
    // // }

    if (num_inliers < param_min_num_inliers.value()) {
      std::cerr << "LoopValidator::compute|RANSAC not enough inliers, rejected!\n";
      return;
    }

    // TODO final refinement with all inliers does not improve
    std::cerr << "LoopValidator::compute|estimate before final ref:\n"
              << _estimate.matrix() << std::endl;

    _refinementSVDICP(_estimate, query_cloud_3d, reference_cloud_3d, matches);
    std::cerr << "LoopValidator::compute|estimate after final ref:\n"
              << _estimate.matrix() << std::endl;

    // TODO fix this shit
    FramePointAssociationVector curr;
    for (auto& t : matches) {
      curr.emplace_back(_associations.at(t.fixed_idx));
    }

    _associations.clear();
    _associations = curr;
    curr.clear();

    // if we survive closure is good (maybe)
    std::cerr << "LoopValidator::compute|final number of correspondences " << _associations.size()
              << std::endl;

    _is_loop_valid = true;
  }

} // namespace md_slam_closures
