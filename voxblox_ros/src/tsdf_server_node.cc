#include <rclcpp/rclcpp.hpp>
#include "voxblox_ros/tsdf_server.h"

#include <gflags/gflags.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  rclcpp::Node::SharedPtr node_ptr = rclcpp::Node::make_shared("voxblox_node");
  voxblox::TsdfServer node(node_ptr.get());

  rclcpp::spin(node_ptr);
  return 0;
}
