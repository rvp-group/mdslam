#include <bits/stdc++.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_pcl/point_types.h>

#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

#include <srrg_image/image.h>

#include <iostream>
#include <vector>

#include <md_slam/factor.h>
#include <md_slam/factor_stack.h>
#include <md_slam/pyramid_generator.h>
#include <md_slam/utils.h>
#include <srrg_pcl/normal_computator.h>
#include <srrg_solver/solver_core/iteration_algorithm_gn.h>
#include <srrg_solver/solver_core/solver.h>

#include <gtest/gtest.h>
#include <srrg_test/test_helper.hpp>

#ifndef MD_TEST_DATA_FOLDER
#error "NO TEST DATA FOLDER"
#endif

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace md_slam;
using namespace std;

using PointVector = PointNormalIntensity3fVectorCloud;

using MDPyramidGeneratorPtr               = std::shared_ptr<MDPyramidGenerator>;
using MDNormalComputator2DCrossProductPtr = std::shared_ptr<MDNormalComputator2DCrossProduct>;
using MDNormalComputatorSRI               = srrg2_core::NormalComputatorSRI<MDMatrixCloud, 1>;
using MDNormalComputatorSRIPtr            = std::shared_ptr<MDNormalComputatorSRI>;

const std::string test_path          = MD_TEST_DATA_FOLDER;
const std::string depth_filename     = test_path + "os1_depth.png";
const std::string intensity_filename = test_path + "os1_intensity.png";

TEST(DUMMY, MDFactor) {
  // we load 2 imayges
  ImageUInt16 depth_fixed;
  ImageUInt8 intensity_fixed;
  loadImage(depth_fixed, depth_filename);
  loadImage(intensity_fixed, intensity_filename);

  // we set the same as moving, the aligner will start from a wrong initial
  // guess
  ImageUInt16 depth_moving    = depth_fixed;
  ImageUInt8 intensity_moving = intensity_fixed;

  Matrix3f lidar_camera_matrix;
  lidar_camera_matrix << -162.9746551513672f, 0.f, 512.f, 0.f, -81.4873275756836f, 32.f, 0.f, 0.f,
    1.f;

  float min_depth          = 0.30f;
  float max_depth          = 100.f;
  Isometry3f forced_offset = Isometry3f::Identity();
  // forced_offset.translation() = Vector3f(0.2, 0.3, 0.1);
  // forced_offset.linear()      = AngleAxisf(1, Vector3f(1, 0,
  // 0)).toRotationMatrix();

  MDNormalComputator2DCrossProductPtr normal_computator(new MDNormalComputator2DCrossProduct);
  normal_computator->param_col_gap.setValue(1.f);
  normal_computator->param_row_gap.setValue(1.f);
  normal_computator->param_squared_max_distance.setValue(10.f);
  // MDNormalComputatorSRIPtr normal_computator(new MDNormalComputatorSRI);

  // configure the pyrgen
  std::cerr << "instantiating pyramid generator" << std::endl;
  MDPyramidGeneratorPtr pyrgen(new MDPyramidGenerator);
  pyrgen->param_intensity_derivative_threshold.setValue(1000);
  pyrgen->param_depth_derivative_threshold.setValue(1000);
  pyrgen->param_depth_policy.setValue(1);
  pyrgen->param_col_prescaling.setValue(1); // 4
  pyrgen->param_row_prescaling.setValue(1);
  // pyrgen->param_depth_scale_override.setValue(0.001);
  pyrgen->setDepthScale(0.01); // 0.001
  // setDepthScale(camera_info->depth_scale.value());
  // if (param_depth_scale_override.value() > 0)
  //   setDepthScale(param_depth_scale_override.value());
  pyrgen->param_intensity_policy.setValue(0);
  pyrgen->param_mask_grow_radius.setValue(1);
  pyrgen->param_min_depth.setValue(min_depth);
  pyrgen->param_max_depth.setValue(max_depth);
  pyrgen->param_normals_blur_region_size.setValue(3.f);
  pyrgen->param_normals_derivative_threshold.setValue(0.5f);
  pyrgen->param_normals_policy.setValue(0.f);
  pyrgen->param_normals_scaled_blur_multiplier.setValue(1.f);
  pyrgen->setSensorOffset(forced_offset);
  pyrgen->setCameraMatrix(lidar_camera_matrix);
  pyrgen->param_scales.value() = vector<int>{1, 2, 4};
  pyrgen->param_normal_computator.setValue(normal_computator);
  pyrgen->setCameraType(CameraType::Spherical);

  // compute prev pyr
  pyrgen->setImages(depth_fixed, intensity_fixed);
  pyrgen->compute();
  auto pyramid_fixed = pyrgen->pyramidMessage()->get();
  // cv::Mat normals_img       = pyrgen->normalsImage();
  // cv::Mat cross_normals_img = pyrgen->crossNormalsImage();
  // cv::imshow("normals", normals_img);
  // cv::imshow("cross_normals", cross_normals_img);
  // cv::waitKey(0);

  // ImageVector3f pyr_img;
  // int level_num = 0;
  // for (const auto& pyr : pyramid_fixed->_levels) {
  //   pyr->toTiledImage(pyr_img);
  //   cv::Mat shown_img;
  //   pyr_img.toCv(shown_img);
  //   const std::string img_name = "normals_level_" +
  //   std::to_string(level_num); level_num++; cv::imshow(img_name, shown_img);
  //   cv::waitKey(1);
  // }
  // cv::waitKey(0);

  // compute curr pyr
  pyrgen->setImages(depth_moving, intensity_moving);
  pyrgen->compute();
  auto pyramid_moving = pyrgen->pyramidMessage()->get();
  std::cerr << "pyramids computed" << std::endl;

  // create graph
  std::cerr << "create graph" << std::endl;
  FactorGraph graph;

  std::cerr << "create and add variable" << std::endl;
  // add variable, with id 0
  std::shared_ptr<VariableSE3QuaternionRight> v(new VariableSE3QuaternionRight);
  v->setGraphId(0);
  Vector6f guess;
  guess << 0.01, 0.01, 0.01, 0.01, 0.01, 0.02;
  Isometry3f guess_T = geometry3d::ta2t(guess);
  v->setEstimate(guess_T);
  graph.addVariable(v);

  std::cerr << "create factors for this problem" << std::endl;
  // create the stack for the factors

  MDFactorStack factors;
  for (auto& f : factors) {
    f->setOmegaDepth(1);
    f->setOmegaIntensity(1);
    f->setOmegaNormals(1);
    f->setDepthRejectionThreshold(5);
    f->setKernelChiThreshold(5);
  }
  factors.setFixed(*pyramid_fixed);
  factors.makeFactors();
  factors.setVariableId(v->graphId());
  std::cerr << "add them to the graph" << std::endl;
  factors.addFactors(graph);

  factors.setMoving(*pyramid_moving);
  factors.assignPyramids();

  std::cerr << "create solver for this problem" << std::endl;
  // TEST SOLVER
  Solver solver;
  std::shared_ptr<IterationAlgorithmGN> algorithm(new IterationAlgorithmGN);
  algorithm->param_damping.setValue(0.1f);
  solver.param_algorithm.setValue(algorithm);
  solver.setGraph(graph); // 10, 20, 50
  solver.param_max_iterations.value() = vector<int>{10, 20, 50};
  solver.param_termination_criteria.setValue(0);
  {
    Chrono timings("total: ", nullptr, true);
    solver.compute();
  }
  std::cerr << "========= iteration stats =========" << std::endl;
  std::cerr << solver.iterationStats() << std::endl;
  std::cerr << "===================================" << std::endl;

  std::cerr << "vec [t|qn]" << std::endl;
  std::cerr << "guess   T: " << FG_GREEN(geometry3d::t2v(guess_T).transpose()) << std::endl;
  std::cerr << "GT      T: " << FG_GREEN(Vector6f::Zero().transpose()) << std::endl;
  std::cerr << "solver  T: " << FG_GREEN(geometry3d::t2v(v->estimate()).transpose()) << std::endl;

  for (auto& f : factors) {
    Chrono::printReport(f->timings);
  }

  const auto diff = geometry3d::t2v(v->estimate());

  std::cerr << "error [t|qn] : " << diff.transpose() << std::endl;
  std::cerr << "error norm: [t|qn] : [ " << diff.head(3).norm() << " | " << diff.tail(3).norm()
            << " ]\n";

  ASSERT_NEAR_EIGEN(diff.head(3), Vector3f::Zero(), 6e-3);
  ASSERT_NEAR_EIGEN(diff.tail(3), Vector3f::Zero(), 4e-3);
}

int main(int argc, char** argv) {
  return srrg2_test::runTests(argc, argv, true /*use test folder*/);
}
