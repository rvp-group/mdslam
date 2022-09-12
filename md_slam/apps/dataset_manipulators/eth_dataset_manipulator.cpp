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
#include <srrg_converters/translator_utils.h>
#include <srrg_messages_ros/instances.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

#include "manip_utils.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace md_slam;

const char* banner[] = {
  "computes an md slam dataset from eth rgbd dataset format (old freiburg rgdb format) ",
  0};

struct AssociationRecord {
  double rgb_timestamp;
  std::string rgb_file;
  double depth_timestamp;
  std::string depth_file;
};

struct ImuMeasurement {
  double time;
  Vector3f accelerometer;
  Vector3f gyroscope; // omega
};

void readImuMeasurements(std::vector<ImuMeasurement>& imu_meas_, const std::string& filename_) {
  std::string line;
  std::ifstream fi(filename_.c_str());
  getline(fi, line, '\n'); // ignore the first line
  double time = 0., acc_x = 0., acc_y = 0., acc_z = 0., gyro_x = 0., gyro_y = 0., gyro_z = 0.;
  while (!fi.eof()) {
    getline(fi, line, '\n');
    sscanf(line.c_str(),
           "%lf %lf %lf %lf %lf %lf %lf",
           &time,
           &gyro_x,
           &gyro_y,
           &gyro_z,
           &acc_x,
           &acc_y,
           &acc_z);

    ImuMeasurement z_imu;
    z_imu.time          = time;
    z_imu.accelerometer = Vector3f(acc_x, acc_y, acc_z);
    z_imu.gyroscope     = Vector3f(gyro_x, gyro_y, gyro_z);
    imu_meas_.emplace_back(z_imu);
  }
  std::cerr << "read number of imu measurements [ " << imu_meas_.size() << " ]" << std::endl;
}

void readAssociation(std::vector<AssociationRecord>& target_, const std::string& filename_) {
  std::ifstream fi(filename_.c_str());
  if (!fi.good())
    return;
  while (fi) {
    AssociationRecord record;
    fi >> record.rgb_timestamp >> record.rgb_file >> record.depth_timestamp >> record.depth_file;
    target_.emplace_back(record);
  }
  std::cerr << "read number of items [ " << target_.size() << " ]" << std::endl;
}

bool readCameraMatrix(Matrix3f& camera_matrix_, const std::string& filename_) {
  camera_matrix_.setIdentity();
  std::ifstream is(filename_.c_str());
  if (!is.good())
    return false;
  is >> camera_matrix_(0, 0) >> camera_matrix_(1, 1) >> camera_matrix_(0, 2) >>
    camera_matrix_(1, 2);
  std::cerr << "loaded camera matrix\n" << camera_matrix_ << std::endl;
  return true;
}

int main(int argc, char** argv) {
  srrgInit(argc, argv, "eth_manipulator");
  messages_registerTypes();
  messages_ros_registerTypes();

  ParseCommandLine cmd(argv, banner);
  ArgumentString arg_input_filename(
    &cmd,
    "i",
    "input",
    "path of input association file, usually called associated.txt ",
    "associated.txt");
  ArgumentString arg_camera_matrix_file(
    &cmd,
    "k",
    "camera-matrix",
    "camera matrix file, must be single raw txt file of the format <fx fy cx cy>",
    "calibration.txt");
  ArgumentString arg_output_rosbag(&cmd, "o", "output", "name of rosbag output", "");
  ArgumentString arg_imu_filename(
    &cmd, "m", "imu", "path of imu measurements, format <timestamp ax ay az gx gy gz>", "");

  // tf stuff
  ArgumentString arg_imu_frame_id(&cmd, "fimu", "frame-imu", "name of imu frame", "/rgbd_imu");
  ArgumentString arg_cam_frame_id(&cmd, "fcam", "frame-cam", "name of cam frame", "/rgbd_cam");

  // output topics
  ArgumentString arg_out_topic_imu(
    &cmd, "toimu", "topic-out-imu", "name of topic imu used for plumbing", "/rgbd/imu");
  ArgumentString arg_out_topic_rgb(&cmd,
                                   "toi",
                                   "topic-out-intensity",
                                   "name of topic intensity image to use for plumbing",
                                   "/rgbd/image_rgb");
  ArgumentString arg_out_topic_depth(&cmd,
                                     "tod",
                                     "topic-out-depth",
                                     "name of topic depth image to use for plumbing",
                                     "/rgbd/image_depth");

  ArgumentString arg_out_topic_camera_info(
    &cmd,
    "tk",
    "topic-camera-info",
    "name of topic camera info used for plumbing rgbd intrisincs data",
    "/rgbd/camera_info");

  cmd.parse();

  if (!arg_input_filename.isSet()) {
    std::cerr << std::string(environ[0]) + "|ERROR, no input association file\n";
    return -1;
  }

  if (!arg_output_rosbag.isSet()) {
    std::cerr << std::string(environ[0]) + "|ERROR, no output bag specified\n";
    return -1;
  }

  // read camera matrix
  Matrix3f camera_matrix;
  if (!readCameraMatrix(camera_matrix, arg_camera_matrix_file.value())) {
    std::cerr << std::string(environ[0]) + "|ERROR, no camera matrix file specified\n" << std::endl;
    return -1;
  }

  // read rgb and depth images
  std::vector<AssociationRecord> records;
  readAssociation(records, arg_input_filename.value());

  if (records.empty()) {
    std::cerr << std::string(environ[0]) + "|ERROR, no valid input specified\n" << std::endl;
    return -1;
  }

  // read imu
  std::vector<ImuMeasurement> imu_measurements;
  if (arg_imu_filename.isSet())
    readImuMeasurements(imu_measurements, arg_imu_filename.value());

  if (imu_measurements.empty()) {
    std::cerr << FG_YELLOW("WARNING, no valid imu input specified\n") << std::endl;
  }

  // start writing output bag
  uint64_t seq = 0;
  rosbag::Bag bag_out(arg_output_rosbag.value(), rosbag::bagmode::Write);

  size_t j = 0;
  for (size_t i = 1; i < records.size(); ++i) {
    const auto record              = records.at(i);
    const double& prev_record_time = records.at(i - 1).rgb_timestamp;
    const double& curr_record_time = record.rgb_timestamp;
    // if imu is not present this is never done
    while (j < imu_measurements.size() && imu_measurements.at(j).time <= curr_record_time) {
      if (imu_measurements.at(j).time >= prev_record_time) {
        // process imu
        const auto& imu_meas = imu_measurements.at(j);
        std::cerr << "imu message "
                  << " $" << seq << " -- ts [ " << std::setprecision(12) << imu_meas.time << " ]\n";
        sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
        imu_msg->header.seq      = seq++;
        imu_msg->header.stamp    = ros::Time(imu_meas.time);
        imu_msg->header.frame_id = "imu";
        EIGEN_TO_ROS_VEC3(imu_msg->linear_acceleration, imu_meas.accelerometer);
        EIGEN_TO_ROS_VEC3(imu_msg->angular_velocity, imu_meas.gyroscope);
        bag_out.write(arg_out_topic_imu.value(), ros::Time(imu_meas.time), imu_msg);
      }
      j++;
    }

    // if one of the two is none than skip, this is risky
    if (record.rgb_file.length() == 0 || record.depth_file.length() == 0) {
      std::cerr << FG_YELLOW("skipping img msgs") << std::endl;
      continue;
    }

    // process imayges
    const auto msg_rostime = ros::Time(curr_record_time);

    // load and write img rgb
    std::cerr << "rgb message "
              << " $" << seq << " -- ts [ " << std::setprecision(12) << curr_record_time << " ]\n";

    ImageVector3uc rgb;
    cv::Mat cv_rgb;
    if (!loadImage(rgb, record.rgb_file)) {
      std::cerr << "failure in loading rgb [ " << record.rgb_file << " ]" << std::endl;
      return -1;
    }
    rgb.toCv(cv_rgb);

    cv_bridge::CvImage rgb_bridge;
    rgb_bridge.header.stamp    = msg_rostime;
    rgb_bridge.header.seq      = seq++;
    rgb_bridge.header.frame_id = arg_cam_frame_id.value();
    rgb_bridge.image           = cv_rgb;
    rgb_bridge.encoding        = "8UC3";
    bag_out.write(arg_out_topic_rgb.value(), ros::Time(msg_rostime), rgb_bridge.toImageMsg());

    // load and write img depth
    std::cerr << "depth message "
              << " $" << seq << " -- ts [ " << std::setprecision(12) << curr_record_time << " ]\n";

    ImageUInt16 depth_img;
    if (!loadImage(depth_img, record.depth_file)) {
      std::cerr << "failure in loading depth [ " << record.depth_file << " ]" << std::endl;
      std::cerr << "EXIT" << std::endl;
      exit(EXIT_FAILURE);
    }

    // write img depth
    sensor_msgs::Image depth_msg;
    depth_msg.header.stamp    = msg_rostime;
    depth_msg.header.seq      = seq++;
    depth_msg.header.frame_id = arg_cam_frame_id.value();
    depth_msg.height          = depth_img.rows();
    depth_msg.width           = depth_img.cols();
    depth_msg.encoding        = "16UC1";
    depth_msg.is_bigendian    = false;
    depth_msg.step            = depth_img.cols() * 2;
    std::cerr << "contains an imu message!" << std::endl;

    depth_msg.data.resize(sizeof(uint16_t) * depth_img.rows() * depth_img.cols());
    uint16_t* dest = (uint16_t*) &(depth_msg.data[0]);
    for (size_t r = 0; r < depth_img.rows(); ++r) {
      for (size_t c = 0; c < depth_img.cols(); ++c, ++dest) {
        *dest = depth_img(r, c);
      }
    }
    bag_out.write(arg_out_topic_depth.value(), msg_rostime, depth_msg);

    std::cerr << "camera matrix message "
              << " $" << seq << " -- ts [ " << std::setprecision(12) << curr_record_time << " ]\n";
    // writing one camera info
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.stamp    = msg_rostime;
    camera_info.header.frame_id = arg_cam_frame_id.value();
    camera_info.header.seq      = seq++;
    camera_info.height          = depth_img.rows();
    camera_info.width           = depth_img.cols();
    const auto& K               = camera_matrix;
    camera_info.K               = {
      K(0, 0), K(0, 1), K(0, 2), K(1, 0), K(1, 1), K(1, 2), K(2, 0), K(2, 1), K(2, 2)};
    bag_out.write(arg_out_topic_camera_info.value(), ros::Time(msg_rostime), camera_info);

    // diplay images for debugging purposes
    cv::imshow("rgb", cv_rgb);
    cv::Mat cv_depth;
    depth_img.toCv(cv_depth);
    cv::imshow("depth", cv_depth);
    cv::waitKey(1);
  }

  std::cerr << "dataset written successfully [ " << FG_GREEN(arg_output_rosbag.value()) << " ]\n";
  bag_out.close();
}
