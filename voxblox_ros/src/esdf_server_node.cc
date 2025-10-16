#include "voxblox_ros/esdf_server.h"

#include <gflags/gflags.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  // ros::init(argc, argv, "voxblox");
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  auto nh = rclcpp::Node::make_shared("voxblox");

  voxblox::EsdfServer node(nh.get());

  rclcpp::spin(nh);
  return 0;
}
