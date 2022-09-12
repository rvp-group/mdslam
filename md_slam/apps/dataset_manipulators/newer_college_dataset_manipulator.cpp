#include <chrono>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

#include <srrg_converters/converter.h>
#include <srrg_data_structures/platform.h>
#include <srrg_messages/instances.h>
#include <srrg_messages_ros/instances.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

#include "lidar_config_object.h"
#include "manip_utils.h"
#include "md_slam/instances.h"
#include "md_slam/utils.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace md_slam;

const char* banner[] = {
  "computes a md slam dataset from point cloud | usage rosrun md_slam "
  "ncd_manipulator -c config_file.json -o outputbag.bag [list of ordered input rosbags]",
  0};

void generateConfig(ConfigurableManager& manager_, std::string config_filename_);

int main(int argc, char** argv) {
  srrgInit(argc, argv, "ncd_manipulator");
  messages_registerTypes();
  messages_ros_registerTypes();
  md_registerTypes();

  ParseCommandLine cmd(argv, banner);
  ArgumentString config_filename(&cmd, "c", "config", "config file to read", "");
  ArgumentString output_rosbag(&cmd, "o", "output", "name of rosbag output", "");
  ArgumentFloat depth_scale(&cmd, "d", "depth-scale", "scaling factor for depth", 1e-2);
  ArgumentString in_topic_imu(
    &cmd, "tiimu", "topic-in-imu", "name of imu topic used for reading", "/os_cloud_node/imu");
  ArgumentString out_topic_intensity(&cmd,
                                     "toi",
                                     "topic-out-intensity",
                                     "name of topic intensity image to use for plumbing",
                                     "/os/image_intensity");
  ArgumentString out_topic_depth(&cmd,
                                 "tod",
                                 "topic-out-depth",
                                 "name of topic depth image to use for plumbing",
                                 "/os/image_depth");
  ArgumentString out_topic_imu(
    &cmd, "toimu", "topic-out-imu", "name of topic imu used for plumbing", "/os/imu");
  ArgumentString out_topic_camera_info(
    &cmd,
    "tk",
    "topic-camera-info",
    "name of topic camera info used for plumbing ouster intrisincs data",
    "/os/camera_info");
  ArgumentString imu_frame_id(&cmd, "fimu", "frame-imu", "name of imu frame", "/os_imu");
  ArgumentString lidar_frame_id(&cmd, "flidar", "frame-lidar", "name of lidar frame", "/os_lidar");
  ArgumentString input_configuration_filename(
    &cmd,
    "j",
    "write-configuration",
    "if output configuration set, write configuration to process dataset",
    "lidar_configuration.json");
  cmd.parse();

  ConfigurableManager manager;

  // writes configuration on file (configuration has to be given as input to process data)
  if (input_configuration_filename.isSet()) {
    generateConfig(manager, input_configuration_filename.value());
    return 0;
  }

  if (cmd.lastParsedArgs().empty()) {
    std::cerr << std::string(environ[0]) + "|ERROR, input dataset(s) not set correctly, aborting\n";
    return -1;
  }

  if (!output_rosbag.isSet()) {
    std::cerr << std::string(environ[0]) + "|ERROR, no output bag specified, aborting\n";
    return -1;
  }

  // platform to read tfs
  PlatformPtr platform(new Platform);

  // output bag
  rosbag::Bag bag_out(output_rosbag.value(), rosbag::bagmode::Write);

  std::cerr << "opening configuration [ " << FG_YELLOW(config_filename.value()) << " ]\n";
  manager.read(config_filename.value());

  // not sync imu and ouster on actual freq
  MessageROSBagSourcePtr source        = std::make_shared<MessageROSBagSource>();
  MessageSortedSourcePtr sorted_source = std::make_shared<MessageSortedSource>();
  sorted_source->param_source.setValue(source);

  auto lidar_config_obj = manager.getByName<MDLidarConfiguration>("lidar_conf");
  if (!lidar_config_obj) {
    std::cerr << std::string(environ[0]) + "|ERROR, wrong input config" << std::endl;
  }

  // printing stuff
  std::cerr << "=======================================\n";
  std::cerr << "plumbing following topics\n";
  std::cerr << "imu topic [ " << FG_YELLOW(out_topic_imu.value()) << " ]\n";
  std::cerr << "ouster camera info topic [ " << FG_YELLOW(out_topic_camera_info.value()) << " ]\n";
  std::cerr << "lidar imu offsets [ " << FG_YELLOW("/tf_static") << " ]\n";
  std::cerr << "intensity image topic [ " << FG_YELLOW(out_topic_intensity.value()) << " ]\n";
  std::cerr << "depth image topic [ " << FG_YELLOW(out_topic_depth.value())
            << " ] with depth scale [ " << FG_YELLOW(depth_scale.value()) << " ]\n";

  std::cerr << std::endl << std::endl;

  // init some stuff
  bool is_tf_set     = false;
  uint64_t seq       = 0;
  const int img_rows = lidar_config_obj->param_image_rows.value();
  const int img_cols = lidar_config_obj->param_image_cols.value();
  float hres         = 0.f;
  float vres         = 0.f;

  // better to calculate dynamically lidar intrisics
  bool calculate_dynamically_horizontal_fov = false;
  if (lidar_config_obj->param_hfov_max.value() == 0.f) {
    calculate_dynamically_horizontal_fov = true;
  } else {
    hres =
      deg2Rad(lidar_config_obj->param_hfov_max.value() - lidar_config_obj->param_hfov_min.value()) /
      img_cols;
  }

  // processing all datasets
  for (const std::string& bag_name : cmd.lastParsedArgs()) {
    std::cerr << "processing dataset [ " << FG_GREEN(bag_name) << " ]\n";
    std::cerr << std::endl;
    source->open(bag_name);

    BaseSensorMessagePtr msg = nullptr;
    while ((msg = source->getMessage())) {
      std::cerr << "message " << msg->topic.value() << " $" << msg->seq.value() << " -- ts [ "
                << std::setprecision(12) << msg->timestamp.value() << " ]\n";
      const auto msg_rostime = ros::Time(msg->timestamp.value());

      // casting messages
      PointCloud2MessagePtr point_cloud_msg =
        extractMessage<PointCloud2Message>(msg, lidar_config_obj->param_point_cloud_topic.value());
      PointIntensity3fVectorCloud raw_cloud;

      IMUMessagePtr imu_msg_ptr = extractMessage<IMUMessage>(msg, in_topic_imu.value());

      if (imu_msg_ptr) {
        sensor_msgs::ImuPtr imu_msg = Converter::convert(imu_msg_ptr);
        imu_msg->header.seq         = seq++;
        bag_out.write(out_topic_imu.value(), ros::Time(msg_rostime), imu_msg);
        continue;
      } else if (point_cloud_msg) {
        point_cloud_msg->getPointCloud(raw_cloud);
        if (raw_cloud.empty())
          std::cerr << std::string(environ[0]) +
                         "|ERROR, empty cloud obtained from message, exit program!"
                    << std::endl;
      } else if (!is_tf_set) {
        // try first with tfs, usually comes at the beginning
        TransformEventsMessagePtr msg_ptr =
          extractMessage<TransformEventsMessage>(msg, msg->topic.value());
        if (msg_ptr) {
          if (platform) {
            if (platform->add(msg)) {
              Isometry3f sensor_offset;
              if (!platform->getTransform(
                    sensor_offset, imu_frame_id.value(), lidar_frame_id.value())) {
                continue;
              }
              const tf::tfMessage tfmsg = tfFromEigMatrix(
                seq, sensor_offset, msg_rostime, imu_frame_id.value(), lidar_frame_id.value());
              bag_out.write("/tf_static", ros::Time(msg_rostime), tfmsg);
              is_tf_set = true;
            }
          }
        } else {
          continue;
        }
      } else {
        std::cerr << "not identified topic [ " << FG_RED(msg->topic.value()) << " ]\n";
        continue;
      }

      // here we go for point cloud projection

      lidar_config_obj->calculateVerticalFOV(raw_cloud);
      vres = (lidar_config_obj->vFOVmax() - lidar_config_obj->vFOVmin()) / img_rows;
      if (calculate_dynamically_horizontal_fov) {
        lidar_config_obj->calculateHorizontalFOV(raw_cloud);
        hres = (lidar_config_obj->hFOVmax() - lidar_config_obj->hFOVmin()) / img_cols;
      }

      // create images here from projections
      Matrix3f K;
      K << -1.f / hres, 0.f, img_cols / 2.f, 0.f, -1.f / vres, img_rows / 2.f, 0.f, 0.f, 1.f;

      cv::Mat cv_depth, cv_intensity;
      computeLidarImages(cv_intensity,
                         cv_depth,
                         raw_cloud,
                         K,
                         img_rows,
                         img_cols,
                         lidar_config_obj->param_min_depth.value(),
                         lidar_config_obj->param_max_depth.value(),
                         lidar_config_obj->param_max_intensity.value());

      // write img depth
      sensor_msgs::Image depth_msg;
      depth_msg.header.stamp    = msg_rostime;
      depth_msg.header.seq      = seq++;
      depth_msg.header.frame_id = lidar_frame_id.value();
      depth_msg.height          = img_rows;
      depth_msg.width           = img_cols;
      depth_msg.encoding        = "16UC1";
      depth_msg.is_bigendian    = false;
      depth_msg.step            = img_cols * 2;
      depth_msg.data.resize(sizeof(uint16_t) * img_rows * img_cols);
      uint16_t* dest = (uint16_t*) &(depth_msg.data[0]);
      for (int r = 0; r < img_rows; ++r) {
        for (int c = 0; c < img_cols; ++c, ++dest) {
          *dest = cv_depth.at<float>(r, c) * 1.f / depth_scale.value();
        }
      }
      bag_out.write(out_topic_depth.value(), ros::Time(msg_rostime), depth_msg);
      // write img intensity
      cv_bridge::CvImage intensity_bridge;
      intensity_bridge.image           = cv_intensity;
      intensity_bridge.header.frame_id = lidar_frame_id.value();
      intensity_bridge.encoding        = sensor_msgs::image_encodings::MONO8;
      intensity_bridge.header.stamp    = msg_rostime;
      intensity_bridge.header.seq      = seq++;
      bag_out.write(
        out_topic_intensity.value(), ros::Time(msg_rostime), intensity_bridge.toImageMsg());

      // writing one camera info
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.seq      = seq++;
      camera_info.header.frame_id = lidar_frame_id.value();
      camera_info.header.stamp    = msg_rostime;
      camera_info.height          = img_rows;
      camera_info.width           = img_cols;
      camera_info.K               = {
        K(0, 0), K(0, 1), K(0, 2), K(1, 0), K(1, 1), K(1, 2), K(2, 0), K(2, 1), K(2, 2)};
      bag_out.write(out_topic_camera_info.value(), ros::Time(msg_rostime), camera_info);

      // viz
      cv::imshow("depth", cv_depth / lidar_config_obj->param_max_depth.value());
      cv::imshow("intensity", cv_intensity);

      cv::waitKey(1);
    }
  }

  std::cerr << "dataset written successfully [ " << FG_GREEN(output_rosbag.value()) << " ]\n";
  bag_out.close();
}

void generateConfig(ConfigurableManager& manager_, std::string config_filename_) {
  auto lidar_config_obj = manager_.create<MDLidarConfiguration>("lidar_conf");
  std::cerr
    << "writing template configuration to use this executable [ " << FG_YELLOW(config_filename_)
    << " ] \n ... modify this configuration and pass this with -c flag as input to the program!"
    << std::endl;
  manager_.write(config_filename_);
  std::cerr << "done!\n";
}
