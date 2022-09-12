#include "graph_manager.h"
#include "factor_stack.h"
#include <srrg_config/configurable_command.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h>
#include <unistd.h>

namespace md_slam {
  using namespace srrg2_solver;
  using namespace srrg2_core;
  using namespace std;

  MDGraphManager::MDGraphManager() {
    addCommand(new ConfigurableCommand_<MDGraphManager,
                                        typeof(&MDGraphManager::cmdSaveGraph),
                                        std::string,
                                        std::string>(
      this, "save", "saves a graph to a json file", &MDGraphManager::cmdSaveGraph));
    _previous_variable.reset();
    _variable_buffer_queue = std::make_unique<std::queue<MDVariableSE3Ptr>>();
    _graph                 = std::make_shared<FactorGraph>();
  }

  MDGraphManager::~MDGraphManager() {
    std::cerr << "joining closure thread...\n";
    if (_closure_thread.joinable()) {
      _quit_closure_thread = true;
      sleep(1);
      _closure_thread.join();
      std::cerr << "closure thread joined!\n";
    }
  }

  bool MDGraphManager::cmdSaveGraph(std::string& response, const std::string& filename) {
    response = "saving graph to file [" + filename + "]";
    _graph->write(filename);
    return true;
  }

  bool MDGraphManager::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {
    MDImagePyramidMessagePtr pyr_msg     = std::dynamic_pointer_cast<MDImagePyramidMessage>(msg_);
    MDTrackerStatusMessagePtr status_msg = std::dynamic_pointer_cast<MDTrackerStatusMessage>(msg_);
    if (status_msg && status_msg->topic.value() == param_status_topic.value()) {
      _status_msg    = status_msg;
      _pyramid_topic = _status_msg->pyramid_topic.value();
    }
    if (pyr_msg && pyr_msg->topic.value() == _pyramid_topic) {
      _pyramid_msg = pyr_msg;
    }
    if (_pyramid_msg && _status_msg &&
        _pyramid_msg->timestamp.value() == _status_msg->timestamp.value()) {
      MDImagePyramid* current = _pyramid_msg->get();
      // current->setTimestamp(_pyramid_msg->timestamp.value());
      if (!current) {
        cerr << "! current" << endl;
        return false;
      }
      if (_status_msg->is_keyframe.value()) {
        // std::cerr << "is keyframe: " << _status_msg->is_keyframe.value() << std::endl;
        MDVariableSE3Ptr var(new MDVariableSE3);
        var->setGraphId(_max_id);
        // set initial var to fixed state
        if (_max_id == 0) {
          var->setStatus(VariableBase::Fixed);
          // handling thread creation at the beginning
          _closure_thread = std::thread(&MDGraphManager::closureCallback, this);
          // if first time, init photometric aligner
          if (param_pairwise_aligner.value()) {
            param_pairwise_aligner.value()->setFixed(current);
            param_pairwise_aligner.value()->initialize();
          }
        }
        _max_id++;
        var->setEstimate(_status_msg->global_pose.value());
        var->setPyramid(new MDImagePyramid(*current));

        _mutex.lock();
        _graph->addVariable(var);
        _mutex.unlock();

        if (_previous_variable) {
          std::shared_ptr<SE3PosePoseGeodesicErrorFactor> f(new SE3PosePoseGeodesicErrorFactor);
          f->setVariableId(0, _previous_variable->graphId());
          f->setVariableId(1, var->graphId());
          f->setMeasurement(_status_msg->local_pose.value());
          f->setInformationMatrix(_status_msg->information_matrix.value());
          // propagate estimate
          var->setEstimate(_previous_variable->estimate() * _status_msg->local_pose.value());
          _mutex.lock();
          _graph->addFactor(f);
          _mutex.unlock();
        }
        if (param_enable_closures.value()) {
          _mutex.lock();
          _variable_buffer_queue->push(var);
          _mutex.unlock();
        }

        _previous_variable = var;
        this->_need_redraw = true;
        this->draw();
        // clear and reset stuff
        _pyramid_msg.reset();
        _status_msg.reset();
        return true;
      }
    }
    return false;
  }

  void MDGraphManager::reset() {
    _pyramid_msg.reset();
    _status_msg.reset();
    _previous_variable.reset();
    _pyramid_topic = "";
    _graph->clear();
    _max_id = 0;
  }

  Matrix6f MDGraphManager::photometricClosure(Isometry3f& estimate_,
                                              MDImagePyramid* fixed_,
                                              MDImagePyramid* moving_) {
    assert((moving_ || fixed_) &&
           "MDGraphManager::photometricClosure|pyramids for photometric closures not set");

    // setting up for pairwise alignment
    param_pairwise_aligner.value()->setEstimate(estimate_);
    param_pairwise_aligner.value()->setFixed(fixed_);
    param_pairwise_aligner.value()->setMoving(moving_);
    param_pairwise_aligner.value()->compute();
    estimate_ = param_pairwise_aligner.value()->estimate();

    // scale information matrix as done in factor, front contains higher resolution dim
    const size_t& rows = fixed_->front()->rows();
    const size_t& cols = fixed_->front()->cols();
    const Matrix6f information_matrix =
      param_pairwise_aligner.value()->informationMatrix() * 1.0 / rows * cols;
    return information_matrix;
  }

  void MDGraphManager::closureCallback() {
    auto closer    = param_closer.value();
    auto validator = closer->param_loop_validator.value();
    auto detector  = closer->param_loop_det.value();
    while (!_quit_closure_thread) {
      if (_variable_buffer_queue->empty()) {
        continue;
      }

      _mutex.lock();
      MDVariableSE3Ptr v = _variable_buffer_queue->front();
      _variable_buffer_queue->pop();
      _mutex.unlock();
      _is_closure_valid = false;

      const ImageFloat& intensity = v->pyramid()->fullIntensity();
      const ImageFloat& depth     = v->pyramid()->fullDepth();

      validator->setCameraMatrix(v->pyramid()->cameraMatrix());
      validator->param_camera_type.setValue(v->pyramid()->cameraType());
      closer->setTimestamp(v->pyramid()->timestamp());
      closer->setPose(v->estimate());
      // if photometric closures are enabled store pyramid
      if (param_pairwise_aligner.value()) {
        closer->setPyramid(v->pyramid());
      }
      closer->setIntensity(intensity);
      closer->setDepth(depth);
      // try to close loop
      _is_closure_valid = closer->compute();
      if (_is_closure_valid) { // if closure is valid
        // set estimate from SVD ICP
        Isometry3f estimate = closer->estimate();
        Matrix6f infomat    = Matrix6f::Identity();

        // offset express optical sensor in the imu frame
        // if imu is present graph describes the imu motion
        const auto& off         = v->pyramid()->sensorOffset();
        Isometry3f pgo_estimate = off * estimate * off.inverse();

        // photometric alignment takes offset into consideration
        if (param_pairwise_aligner.value()) {
          _mutex.lock();
          infomat = photometricClosure(pgo_estimate, v->pyramid(), detector->match()->pyramid());
          _mutex.unlock();
        }

        // topological check for validating the closure
        const size_t indx_prev = closer->closureIndex() - 1;
        const size_t indx_next = closer->closureIndex() + 1;

        MDVariableSE3* X_prev = static_cast<MDVariableSE3*>(_graph->variable(indx_prev));
        MDVariableSE3* X_next = static_cast<MDVariableSE3*>(_graph->variable(indx_next));
        MDVariableSE3* X = static_cast<MDVariableSE3*>(_graph->variable(closer->closureIndex()));
        if (X_prev == nullptr || X_next == nullptr)
          continue;

        Isometry3f T_new_prev =
          X_prev->estimate().inverse() * X->estimate() * pgo_estimate.inverse();
        Isometry3f T_new_next =
          X_next->estimate().inverse() * X->estimate() * pgo_estimate.inverse();

        const Isometry3f T_new_prev_not = T_new_prev;
        const Isometry3f T_new_next_not = T_new_next;
        _mutex.lock();
        photometricClosure(T_new_prev, X_prev->pyramid(), v->pyramid());
        photometricClosure(T_new_next, X_next->pyramid(), v->pyramid());
        _mutex.unlock();
        const Isometry3f e1 = T_new_prev_not.inverse() * T_new_prev;
        const Isometry3f e2 = T_new_next_not.inverse() * T_new_next;

        const Eigen::AngleAxisf aa1(e1.linear());
        const Eigen::AngleAxisf aa2(e2.linear());

        std::cerr << "e PREV angle: " << aa1.angle() << " t_norm: " << e1.translation().norm()
                  << std::endl;
        std::cerr << "e NEXT angle: " << aa2.angle() << " t_norm: " << e2.translation().norm()
                  << std::endl;

        // call photometricClosure() // infomat not needed here
        // good for everything a part from os164
        // if ( fabs(aa1.angle()) > 7e-2 || fabs(aa2.angle()) > 7e-2 || e1.translation().norm() >
        // 7e-2 || e2.translation().norm() > 7e-2) { // provato on t norm 0.15, 0.10
        if (fabs(aa1.angle()) > param_angle_check.value() ||
            fabs(aa2.angle()) > param_angle_check.value() ||
            e1.translation().norm() > param_translation_check.value() ||
            e2.translation().norm() > param_translation_check.value()) {
          std::cerr << "##########################################" << std::endl;
          continue;
        }

        std::shared_ptr<SE3PosePoseGeodesicErrorFactor> f(new SE3PosePoseGeodesicErrorFactor);
        f->setVariableId(0, v->graphId());
        f->setVariableId(1, closer->closureIndex());
        // renable this for pose graph optimization
        f->setMeasurement(pgo_estimate);
        f->setInformationMatrix(infomat);
        _mutex.lock(); // concurrent writing in factor insertion
        _graph->addFactor(f);
        assert(param_solver.value() && "MDGraphManager::closureCallback|no solver set");
        param_solver.value()->setGraph(_graph);
        param_solver.value()->compute();
        _mutex.unlock();
        const auto& stats      = param_solver.value()->iterationStats();
        const auto& init_chi2  = stats.front().chi_inliers;
        const auto& final_chi2 = stats.back().chi_inliers;
        std::cerr << "pose-graph optimization - init chi2: " << init_chi2
                  << " - final chi2: " << final_chi2 << std::endl;
      }
    }
    return;
  }

  void MDGraphManager::_drawImpl(ViewerCanvasPtr gl_canvas_) const {
    // TODO: voxelize the cloud before sending, it is too big and clobbers the viewer

    if (!gl_canvas_)
      return;
    if (!_lists_created) {
      _lists_created = true;
    }

    for (auto v : _graph->variables()) {
      v.second->_drawImpl(gl_canvas_);
    }
    for (auto f : _graph->factors()) {
      f.second->_drawImpl(gl_canvas_);
    }

    if (_previous_variable) {
      const ImageFloat& intensity = _previous_variable->pyramid()->fullIntensity();
      cv::Mat cv_intensity;
      intensity.toCv(cv_intensity);
      gl_canvas_->putImage(cv_intensity);
    }
    // showing vpr frames
    if (_previous_variable && _is_closure_valid) {
      const auto loop_validator = param_closer.value()->param_loop_validator.value();
      const auto loop_det       = param_closer.value()->param_loop_det.value();
      const auto& cv_query = param_closer.value()->framesContainer()->frames().back()->cvImage();
      const auto& cv_ref   = loop_det->match()->cvImage();

      cv::Mat cv_output;
      cv::vconcat(cv_query, cv_ref, cv_output);
      cv::cvtColor(cv_output, cv_output, CV_GRAY2RGB);
      cv::Point2f shift(0, cv_query.rows);
      for (auto m : loop_validator->associations()) {
        cv::line(cv_output, m.fixed->kp.pt, m.moving->kp.pt + shift, cv::Scalar(0, 255, 0));
        cv::circle(cv_output, m.fixed->kp.pt, 2, cv::Scalar(0, 0, 255));
        cv::circle(cv_output, m.moving->kp.pt + shift, 2, cv::Scalar(255, 0, 0));
      }
      gl_canvas_->putImage(cv_output);
    }

    gl_canvas_->flush();
  }

  void MDGraphManager::publishTransform() {
    if (!platform())
      return;
    if (!_status_msg)
      return;
    const std::string& tracker_frame_id = _status_msg->frame_id.value();
    double timestamp                    = _status_msg->timestamp.value();
    auto seq                            = _status_msg->seq.value();
    TransformEventsMessagePtr transform_event(new TransformEventsMessage);
    transform_event->seq.setValue(seq);
    transform_event->timestamp.setValue(timestamp);
    transform_event->frame_id.setValue(param_map_frame_id.value());
    transform_event->topic.setValue(param_tf_dyn_topic.value());
    transform_event->events.resize(1);
    TransformEvent map_to_odom(timestamp,
                               param_map_frame_id.value(),
                               _previous_variable->estimate() *
                                 _status_msg->global_pose.value().inverse(),
                               tracker_frame_id);
    transform_event->events.setValue(0, map_to_odom);
    propagateMessage(transform_event);
  }

  using MDGraphManagerPtr = std::shared_ptr<MDGraphManager>;
} // namespace md_slam
