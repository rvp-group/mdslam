#include "pyramid_generator.h"
#include "utils.h"
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_pcl/point_cloud.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_system_utils/shell_colors.h>

#include <unistd.h>

namespace md_slam {
  using namespace std;
  using namespace srrg2_core;

  MDPyramidGenerator::MDPyramidGenerator() {
    MDNormalComputator2DCrossProduct* comp =
      dynamic_cast<MDNormalComputator2DCrossProduct*>(param_normal_computator.value().get());
    comp->param_col_gap.setValue(3);
    comp->param_row_gap.setValue(3);
    comp->param_squared_max_distance.setValue(0.3);
    param_scales.value() = vector<int>{2, 4, 8};
  }

  MDPyramidGenerator::~MDPyramidGenerator() {
  }

  void MDPyramidGenerator::setImages(const ImageUInt16& raw_depth, const BaseImage& raw_intensity) {
    assert((raw_depth.rows() == raw_intensity.rows() || raw_depth.cols() == raw_intensity.cols()) &&
           "MDPyramidGenerator::setImages | input image size mismatch");

    // bilateral filtering on depth img (only on rgbd)
    // TODO apply filtering directly on scaled image float
    // ImageUInt16 smoothed_depth(raw_depth.rows(), raw_depth.cols());
    // bilateralFiltering(smoothed_depth,
    //                    raw_depth,
    //                    param_radius_factor.value(),
    //                    _sigma_pixel,     // TODO this can be adjusted
    //                    _sigma_inv_depth, // TODO this can be adjusted
    //                    param_max_depth.value(),
    //                    _depth_scale);

    // scale fuckin depth
    _full_depth.resize(raw_depth.rows(), raw_depth.cols());
    for (size_t i = 0; i < _full_depth.size(); ++i) {
      _full_depth[i] = _depth_scale * raw_depth[i];
    }

    prepareImage(_depth,
                 _full_depth,
                 param_scales.value(param_scales.size() - 1),
                 param_row_prescaling.value(),
                 param_col_prescaling.value(),
                 true);
    allocatePyramids();

    const ImageVector3uc* rgb_image = dynamic_cast<const ImageVector3uc*>(&raw_intensity);
    if (rgb_image) {
      cv::Mat cv_rgb, cv_intensity;
      rgb_image->toCv(cv_rgb);
      cv::cvtColor(cv_rgb, cv_intensity, CV_RGB2GRAY);
      ImageUInt8 raw_intensity;
      raw_intensity.fromCv(cv_intensity);
      raw_intensity.convertTo(_full_intensity, 1. / 255);
      prepareImage(_intensity,
                   _full_intensity,
                   param_scales.value(param_scales.size() - 1),
                   param_row_prescaling.value(),
                   param_col_prescaling.value(),
                   false);
      return;
    }
    const ImageUInt8* intensity_image = dynamic_cast<const ImageUInt8*>(&raw_intensity);
    if (intensity_image) {
      intensity_image->convertTo(_full_intensity, 1. / 255);
      prepareImage(_intensity,
                   _full_intensity,
                   param_scales.value(param_scales.size() - 1),
                   param_row_prescaling.value(),
                   param_col_prescaling.value(),
                   false);
      return;
    }
    throw std::runtime_error("MDPyramidGenerator::setImages | unknown intensity");
  }

  void MDPyramidGenerator::allocatePyramids() {
    // this get called only once usually unless "static" shit change
    if (_scales_changed_flag && rows() == _normals.rows() && cols() == _normals.cols())
      return;
    _scales_changed_flag = false;

    _level_zero.resize(rows(), cols());
    _level_zero.camera_matrix = _camera_matrix_scaled;
    _level_zero.camera_type   = _camera_type;
    _level_zero.sensor_offset = _sensor_offset;
    // std::cerr << "MDPyramidGenerator::allocatePyramids:\n"
    //           << _level_zero.sensor_offset.matrix() << std::endl;
    _level_zero.min_depth             = param_min_depth.value();
    _level_zero.max_depth             = param_max_depth.value();
    _level_zero.policies[Intensity]   = (FilterPolicy) param_intensity_policy.value();
    _level_zero.thresholds[Intensity] = param_intensity_derivative_threshold.value();
    _level_zero.policies[Depth]       = (FilterPolicy) param_depth_policy.value();
    _level_zero.thresholds[Depth]     = param_depth_derivative_threshold.value();
    _level_zero.policies[Normal]      = (FilterPolicy) param_normals_policy.value();
    _level_zero.thresholds[Normal]    = param_normals_derivative_threshold.value();
    _level_zero.mask_grow_radius      = param_mask_grow_radius.value();

    switch (_camera_type) {
      case CameraType::Pinhole:
        _unprojector.reset(new PointUnprojectorPinhole());
        _unprojector->setCameraMatrix(_camera_matrix_scaled);
        break;
      case CameraType::Spherical:
        _unprojector.reset(new PointUnprojectorPolar());
        _unprojector->setCameraMatrix(_camera_matrix_scaled);
        break;
      default:
        throw std::runtime_error("MDPyramidGenerator::prepareContainers | unknown camera model");
    }
  }

  void MDPyramidGenerator::compute() {
    Chrono pyrgen_time("pyramid_generation: ", &_timings, false);
    _cloud.resize(rows(), cols());
    _cloud.fill(PointNormalIntensity3f());
    _mask = (_depth == 0);

    _pyramid_msg.reset(new MDImagePyramidMessage);
    _pyramid_msg->set(new MDImagePyramid);
    MDImagePyramid* pyr_image = _pyramid_msg->get();

    PointUnprojectorPinhole* pinhole = dynamic_cast<PointUnprojectorPinhole*>(_unprojector.get());
    PointUnprojectorPolar* polar     = dynamic_cast<PointUnprojectorPolar*>(_unprojector.get());
    assert((pinhole || polar) && "MDPyramidGenerator::compute|no unprojector set");

    int unp_mat_valid;
    if (pinhole) {
      unp_mat_valid = pinhole->computeMatrix<WithNormals>(_cloud, _depth, _intensity);
    } else {
      unp_mat_valid = polar->computeMatrix<WithNormals>(_cloud, _depth, _intensity);
    }

    assert(unp_mat_valid && "MDPyramidGenerator::compute|invalid projection");

    param_normal_computator.value()->computeNormals(_cloud);

    ImageVector3f tmp_normals(rows(), cols());
    if (param_adaptive_blur.value()) {
      // smoothing normals based on depth
      // TODO tune last values, shitty hardcoding
      const ImageUInt8 _kernels = calculateKernelImg(_depth, param_min_depth.value(), 5, 10.f);
      _cloud.copyRawFieldTo<1>(tmp_normals);
      _normals = adaptiveNormalsSmoothing(tmp_normals, _kernels);
    } else {
      // here static blurring
      _cloud.copyRawFieldTo<1>(tmp_normals);
      tmp_normals.blur(_normals, param_normals_blur_region_size.value());
    }
    // zero the normals where undefined
    _cloud.copyRawFieldFrom<1>(_normals);
    _cloud.normalize<1>();

    // expressing the lidar cloud in the imu frame
    _cloud.transformInPlace<TRANSFORM_CLASS::Isometry>(_sensor_offset);

    size_t num_levels = param_scales.size();
    _level_zero.fromCloud(_cloud);
    pyr_image->resize(num_levels);
    pyr_image->_relative_scales.resize(num_levels);
    // full resolution needed for closures
    pyr_image->setFullIntensity(_full_intensity);
    pyr_image->setFullDepth(_full_depth);
    pyr_image->setCameraMatrix(_camera_matrix_original);
    pyr_image->setCameraType(_camera_type);
    pyr_image->setSensorOffset(_sensor_offset);
    for (size_t level_num = 0; level_num < num_levels; ++level_num) {
      const float& scale = param_scales.value(level_num);
      pyr_image->at(level_num).reset(new MDPyramidLevel);
      MDPyramidLevel& pyr = *pyr_image->at(level_num);
      _level_zero.scaleTo(pyr, scale);
      int relative_scale = 1;
      if (level_num > 0) {
        relative_scale = param_scales.value(level_num) / param_scales.value(0);
        if (param_scales.value(level_num) % param_scales.value(0)) {
          cerr << "il male sia con te" << endl;
        }
      }
      pyr_image->_relative_scales[level_num] = relative_scale;
    }
  }

  bool MDPyramidGenerator::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {
    MessagePackPtr pack = std::dynamic_pointer_cast<MessagePack>(msg_);
    if (!pack) {
      std::cerr << "MDPyramidGenerator::putMessage | no pack received" << std::endl;
      return false;
    }
    ImageMessagePtr depth;
    ImageMessagePtr intensity;
    CameraInfoMessagePtr camera_info;
    for (auto& m : pack->messages) {
      if (m->topic.value() == param_depth_topic.value()) {
        depth = std::dynamic_pointer_cast<ImageMessage>(m);
        assert(depth && "MDPyramidGenerator::putMessage|depth, invalid type");
      }

      if (m->topic.value() == param_intensity_topic.value()) {
        intensity = std::dynamic_pointer_cast<ImageMessage>(m);
        assert(intensity && "MDPyramidGenerator::putMessage|intensity, invalid type");
      }

      if (m->topic.value() == param_camera_info_topic.value()) {
        camera_info = std::dynamic_pointer_cast<CameraInfoMessage>(m);
        assert(camera_info && "MDPyramidGenerator::putMessage|camera_info, invalid type");
      }
    }

    assert(depth->image()->rows() == intensity->image()->rows() &&
           "MDPyramidGenerator::putMessage|depth and intensity rows size mismatch");
    assert(depth->image()->cols() == intensity->image()->cols() &&
           "MDPyramidGenerator::putMessage|depth and intensity cols size mismatch");

    if (platform()) {
      Isometry3f sensor_offset;
      if (!platform()->getTransform(
            sensor_offset, intensity->frame_id.value(), param_base_frame_id.value())) {
        std::cerr << "MDPyramidGenerator::putMessage|waiting for transform "
                  << param_base_frame_id.value() << " " << intensity->frame_id.value() << endl;
        return false;
      } else {
        setSensorOffset(sensor_offset);
        std::cerr << "MDPyramidGenerator::putMessage | offset received\n"
                  << sensor_offset.matrix() << std::endl;
      }
    } else {
      setSensorOffset(Isometry3f::Identity());
    }

    if (param_cam_type_override.value() != CameraType::Unknown) {
      // cerr << "Override!" << param_cam_type_override.value() << endl;
      switch (param_cam_type_override.value()) {
        case Pinhole:
          camera_info->projection_model.setValue("pinhole");
          break;
        case Spherical:
          camera_info->projection_model.setValue("spherical");
          // TODO embed this in dataset
          camera_info->camera_matrix.value()(1, 1) *= -1;
          camera_info->camera_matrix.value()(0, 0) *= -1;
          break;
        default:;
      }
    }

    CameraType camera_type;
    if (camera_info->projection_model.value() == "pinhole") {
      camera_type = Pinhole;
    } else if (camera_info->projection_model.value() == "spherical") {
      camera_type = Spherical;
    } else {
      throw std::runtime_error("MDPyramidGenerator::putMessage | unknown camera_model");
    }
    setCameraType(camera_type);
    setDepthScale(camera_info->depth_scale.value());
    if (param_depth_scale_override.value() > 0)
      setDepthScale(param_depth_scale_override.value());
    setCameraMatrix(camera_info->camera_matrix.value());

    ImageUInt16* depth_16 = dynamic_cast<ImageUInt16*>(depth->image());
    assert(depth_16 && "MDPyramidGenerator::putMessage|depth not uint16");

    setImages(*depth_16, *intensity->image());
    compute();
    _pyramid_msg->seq.setValue(_seq);
    ++_seq;
    _pyramid_msg->timestamp.setValue(depth->timestamp.value());
    _pyramid_msg->get()->_timestamp = depth->timestamp.value();
    _pyramid_msg->frame_id.setValue(param_base_frame_id.value());
    _pyramid_msg->topic.setValue(param_pyramid_output_topic.value());
    propagateMessage(_pyramid_msg);
    this->_need_redraw = true;
    ActiveDrawable::draw();
    Chrono::printReport(_timings);
    return true;
  }

  void MDPyramidGenerator::_drawImpl(ViewerCanvasPtr gl_canvas_) const {
    if (!gl_canvas_)
      return;
    if (!_pyramid_msg)
      return;
    MDImagePyramid* pyr_levels = _pyramid_msg->get();
    if (!pyr_levels->numLevels())
      return;
    ImageVector3f pyr_img;
    for (const auto& pyr : pyr_levels->_levels) {
      pyr->toTiledImage(pyr_img);
      cv::Mat shown_img;
      pyr_img.toCv(shown_img);
      gl_canvas_->putImage(shown_img);
    }
    gl_canvas_->flush();
  }

} // namespace md_slam
