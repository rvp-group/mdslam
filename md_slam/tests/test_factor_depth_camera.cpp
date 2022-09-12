#include <bits/stdc++.h>
#include <random>
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

using MDPyramidGeneratorPtr = std::shared_ptr<MDPyramidGenerator>;

const std::string test_path          = MD_TEST_DATA_FOLDER;
const std::string depth_filename     = test_path + "camera.depth.image_raw_00002102.pgm";
const std::string intensity_filename = test_path + "camera.rgb.image_raw_00002102.png";

TEST(DUMMY, MDFactor) {
  // we load 2 imayges
  ImageUInt16 depth_fixed;
  ImageVector3uc intensity_fixed;
  loadImage(depth_fixed, depth_filename);
  loadImage(intensity_fixed, intensity_filename);

  // we set the same as moving, the aligner will start from a wrong initial guess
  ImageUInt16 depth_moving        = depth_fixed;
  ImageVector3uc intensity_moving = intensity_fixed;


  // bdc camera matrix for Xtion test image, offset of the sensor, depth cropping parameters
  Matrix3f pinhole_camera_matrix;
  pinhole_camera_matrix << 481.2f, 0.f, 319.5f, 0.f, 481.f, 239.5f, 0.f, 0.f, 1.f;
  float min_depth = 0.3;
  float max_depth = 5.0;
  Isometry3f forced_offset;
  forced_offset.translation() = Vector3f(0.2, 0.3, 0.1);
  forced_offset.linear()      = AngleAxisf(1, Vector3f(1, 0, 0)).toRotationMatrix();

  // configure the pyrgen
  std::cerr << "instantiating pyramid generator" << std::endl;
  MDPyramidGeneratorPtr pyrgen(new MDPyramidGenerator);
  pyrgen->param_min_depth.setValue(min_depth);
  pyrgen->param_max_depth.setValue(max_depth);
  pyrgen->setSensorOffset(forced_offset);
  pyrgen->setDepthScale(0.001);
  pyrgen->param_normals_policy.setValue(0); //.pushBack(4);
  pyrgen->setCameraMatrix(pinhole_camera_matrix);
  pyrgen->param_scales.value() = vector<int>{1, 2, 4};

  // bdc compute previous pyramid
  pyrgen->setImages(depth_fixed, intensity_fixed);
  pyrgen->compute();
  auto pyramid_fixed = pyrgen->pyramidMessage()->get();

  // bdc compute current pyramid
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

  MDFactorStack md_factors;
  md_factors.setFixed(*pyramid_fixed);
  md_factors.makeFactors();
  md_factors.setVariableId(v->graphId());
  std::cerr << "add them to the graph" << std::endl;
  md_factors.addFactors(graph);

  md_factors.setMoving(*pyramid_moving);
  md_factors.assignPyramids();

  std::cerr << "create solver for this problem" << std::endl;
  // TEST SOLVER
  Solver solver;
  std::shared_ptr<IterationAlgorithmGN> algorithm(new IterationAlgorithmGN);
  algorithm->param_damping.setValue(0.1f);
  solver.param_algorithm.setValue(algorithm);
  solver.setGraph(graph);
  solver.param_max_iterations.value() = vector<int>{10, 20, 50};
  solver.param_termination_criteria.setValue(0);
  {
    Chrono timings("total: ", nullptr, true);
    solver.compute();
  }
  std::cerr << " ========= iteration stats =========" << std::endl;
  std::cerr << solver.iterationStats() << std::endl;
  std::cerr << " ===================================" << std::endl;

  std::cerr << "guess   T: " << FG_GREEN(geometry3d::t2v(guess_T).transpose()) << std::endl;
  std::cerr << "GT      T: " << FG_GREEN(Vector6f::Zero().transpose()) << std::endl;
  std::cerr << "Solver  T: " << FG_GREEN(geometry3d::t2v(v->estimate()).transpose()) << std::endl;

  for (auto& f : md_factors) {
    Chrono::printReport(f->timings);
  }
  Vector6f estimate = geometry3d::t2v(v->estimate());

  const float tolerance_ = 3e-6;
  ASSERT_NEAR_EIGEN(estimate.head<3>(), Vector3f::Zero(), tolerance_);
  ASSERT_NEAR_EIGEN(estimate.tail<3>(), Vector3f::Zero(), tolerance_);
}

int main(int argc, char** argv) {
  return srrg2_test::runTests(argc, argv, true /*use test folder*/);
}
