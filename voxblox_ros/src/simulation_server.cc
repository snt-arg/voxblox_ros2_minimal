#include "voxblox_ros/simulation_server.h"

#include <rclcpp/rclcpp.hpp>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/esdf_occ_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/simulation/simulation_world.h>
#include <voxblox/utils/evaluation_utils.h>

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "voxblox_ros/conversions.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {

void SimulationServer::getServerConfigFromRosParam(rclcpp::Node* node_ptr) {
  // Settings for simulation.
  // nh_private_.param("tsdf_voxel_size", voxel_size_, voxel_size_);
  // nh_private_.param("tsdf_voxels_per_side", voxels_per_side_,
  // voxels_per_side_); nh_private.param("incremental", incremental_,
  // incremental_); nh_private.param("generate_mesh", generate_mesh_,
  // generate_mesh_);

  // nh_private.param("visualize", visualize_, visualize_);
  // nh_private.param("visualization_slice_level", visualization_slice_level_,
  //                  visualization_slice_level_);

  // nh_private.param("generate_occupancy", generate_occupancy_,
  //                  generate_occupancy_);
  // nh_private.param("add_robot_pose", add_robot_pose_, add_robot_pose_);
  // nh_private.param("truncation_distance", truncation_distance_,
  //                  truncation_distance_);

  // nh_private.param("depth_camera_resolution_u", depth_camera_resolution_[0],
  //                  depth_camera_resolution_[0]);
  // nh_private.param("depth_camera_resolution_v", depth_camera_resolution_[1],
  //                  depth_camera_resolution_[1]);

  // nh_private.param("fov_h_rad", fov_h_rad_, fov_h_rad_);

  // nh_private.param("max_dist", max_dist_, max_dist_);
  // nh_private.param("min_dist", min_dist_, min_dist_);

  // nh_private.param("num_viewpoints", num_viewpoints_, num_viewpoints_);
  node_ptr->declare_parameter("tsdf_voxel_size", voxel_size_);
  node_ptr->declare_parameter("tsdf_voxels_per_side", voxels_per_side_);
  node_ptr->declare_parameter("incremental", incremental_);
  node_ptr->declare_parameter("generate_mesh", generate_mesh_);
  node_ptr->declare_parameter("visualize", visualize_);
  node_ptr->declare_parameter("visualization_slice_level",
                              visualization_slice_level_);
  node_ptr->declare_parameter("generate_occupancy", generate_occupancy_);
  node_ptr->declare_parameter("add_robot_pose", add_robot_pose_);
  node_ptr->declare_parameter("truncation_distance", truncation_distance_);
  node_ptr->declare_parameter("depth_camera_resolution_u",
                              depth_camera_resolution_[0]);
  node_ptr->declare_parameter("depth_camera_resolution_v",
                              depth_camera_resolution_[1]);
  node_ptr->declare_parameter("fov_h_rad", fov_h_rad_);
  node_ptr->declare_parameter("max_dist", max_dist_);
  node_ptr->declare_parameter("min_dist", min_dist_);
  node_ptr->declare_parameter("num_viewpoints", num_viewpoints_);

  node_ptr->get_parameter("tsdf_voxel_size", voxel_size_);
  node_ptr->get_parameter("tsdf_voxels_per_side", voxels_per_side_);
  node_ptr->get_parameter("incremental", incremental_);
  node_ptr->get_parameter("generate_mesh", generate_mesh_);
  node_ptr->get_parameter("visualize", visualize_);
  node_ptr->get_parameter("visualization_slice_level",
                          visualization_slice_level_);
  node_ptr->get_parameter("generate_occupancy", generate_occupancy_);
  node_ptr->get_parameter("add_robot_pose", add_robot_pose_);
  node_ptr->get_parameter("truncation_distance", truncation_distance_);
  node_ptr->get_parameter("depth_camera_resolution_u",
                          depth_camera_resolution_[0]);
  node_ptr->get_parameter("depth_camera_resolution_v",
                          depth_camera_resolution_[1]);
  node_ptr->get_parameter("fov_h_rad", fov_h_rad_);
  node_ptr->get_parameter("max_dist", max_dist_);
  node_ptr->get_parameter("min_dist", min_dist_);
  node_ptr->get_parameter("num_viewpoints", num_viewpoints_);

  // NOTE(mfehr): needed because ros params does not support size_t.
  int max_attempts_to_generate_viewpoint =
      static_cast<int>(max_attempts_to_generate_viewpoint_);
  // nh_private.param("max_attempts_to_generate_viewpoint",
  //                  max_attempts_to_generate_viewpoint,
  //                  max_attempts_to_generate_viewpoint);
  node_ptr->declare_parameter("max_attempts_to_generate_viewpoint",
                              max_attempts_to_generate_viewpoint);
  node_ptr->get_parameter("max_attempts_to_generate_viewpoint",
                          max_attempts_to_generate_viewpoint);
  CHECK_GT(max_attempts_to_generate_viewpoint, 0);
  max_attempts_to_generate_viewpoint_ =
      static_cast<size_t>(max_attempts_to_generate_viewpoint);

  // nh_private.param("world_frame", world_frame_, world_frame_);
  node_ptr->declare_parameter("world_frame", world_frame_);
  node_ptr->get_parameter("world_frame", world_frame_);
}

SimulationServer::SimulationServer(
    rclcpp::Node* node_ptr, const EsdfMap::Config& esdf_config,
    const EsdfIntegrator::Config& esdf_integrator_config,
    const TsdfMap::Config& tsdf_config,
    const TsdfIntegratorBase::Config& tsdf_integrator_config)
    : node_ptr_(node_ptr),
      voxel_size_(tsdf_config.tsdf_voxel_size),
      voxels_per_side_(tsdf_config.tsdf_voxels_per_side),
      world_frame_("world"),
      generate_occupancy_(false),
      visualize_(true),
      visualization_slice_level_(2.0),
      generate_mesh_(true),
      incremental_(true),
      add_robot_pose_(false),
      truncation_distance_(tsdf_integrator_config.default_truncation_distance),
      esdf_max_distance_(esdf_integrator_config.max_distance_m),
      max_attempts_to_generate_viewpoint_(50u),
      depth_camera_resolution_(Eigen::Vector2i(320, 240)),
      fov_h_rad_(1.5708),
      max_dist_(10.0),
      min_dist_(0.5),
      num_viewpoints_(50),
      world_(new SimulationWorld()) {
  CHECK_EQ(voxel_size_, tsdf_config.tsdf_voxel_size);
  CHECK_EQ(voxel_size_, esdf_config.esdf_voxel_size);
  CHECK_EQ(static_cast<size_t>(voxels_per_side_),
           tsdf_config.tsdf_voxels_per_side);
  CHECK_EQ(static_cast<size_t>(voxels_per_side_),
           esdf_config.esdf_voxels_per_side);

  getServerConfigFromRosParam(node_ptr);

  tsdf_gt_.reset(new Layer<TsdfVoxel>(voxel_size_, voxels_per_side_));
  esdf_gt_.reset(new Layer<EsdfVoxel>(voxel_size_, voxels_per_side_));

  tsdf_test_.reset(new Layer<TsdfVoxel>(voxel_size_, voxels_per_side_));
  esdf_test_.reset(new Layer<EsdfVoxel>(voxel_size_, voxels_per_side_));

  tsdf_integrator_.reset(
      new MergedTsdfIntegrator(tsdf_integrator_config, tsdf_test_.get()));

  EsdfIntegrator::Config esdf_integrator_config_copy = esdf_integrator_config;
  esdf_integrator_config_copy.clear_sphere_radius = min_dist_;

  esdf_integrator_.reset(new EsdfIntegrator(
      esdf_integrator_config_copy, tsdf_test_.get(), esdf_test_.get()));

  if (generate_occupancy_) {
    occ_test_.reset(new Layer<OccupancyVoxel>(voxel_size_, voxels_per_side_));

    OccupancyIntegrator::Config occ_integrator_config;
    occ_integrator_config.max_ray_length_m =
        tsdf_integrator_config.max_ray_length_m;
    occ_integrator_.reset(
        new OccupancyIntegrator(occ_integrator_config, occ_test_.get()));

    EsdfOccIntegrator::Config esdf_occ_config;
    esdf_occ_config.max_distance_m = esdf_max_distance_;
    esdf_occ_config.default_distance_m = esdf_max_distance_;
    esdf_occ_integrator_.reset(new EsdfOccIntegrator(
        esdf_occ_config, occ_test_.get(), esdf_test_.get()));
  }

  // ROS stuff.
  // GT
  // tsdf_gt_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
  //     "tsdf_gt", 1, true);

  // esdf_gt_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
  //     "esdf_gt", 1, true);
  // tsdf_gt_mesh_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
  //     "tsdf_gt_mesh", 1, true);

  // // Test
  // tsdf_test_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
  //     "tsdf_test", 1, true);
  // esdf_test_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
  //     "esdf_test", 1, true);
  // tsdf_test_mesh_pub_ =
  // nh_private_.advertise<visualization_msgs::MarkerArray>(
  //     "tsdf_test_mesh", 1, true);

  /* view_ptcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB>
     >( "view_ptcloud_pub", 1, true); */
  tsdf_gt_pub_ =
      node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("tsdf_gt", 1);
  esdf_gt_pub_ =
      node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("esdf_gt", 1);
  tsdf_gt_mesh_pub_ =
      node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "tsdf_gt_mesh", 1);
  tsdf_test_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "tsdf_test", 1);
  esdf_test_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "esdf_test", 1);
  tsdf_test_mesh_pub_ =
      node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "tsdf_test_mesh", 1);
  view_ptcloud_pub_ =
      node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "view_ptcloud_pub", 1);

  // Set random seed to a fixed value.
  srand(0);
}

SimulationServer::SimulationServer(rclcpp::Node* node_ptr)
    : SimulationServer(node_ptr, getEsdfMapConfigFromRosParam(node_ptr),
                       getEsdfIntegratorConfigFromRosParam(node_ptr),
                       getTsdfMapConfigFromRosParam(node_ptr),
                       getTsdfIntegratorConfigFromRosParam(node_ptr)) {}

bool SimulationServer::generatePlausibleViewpoint(FloatingPoint min_distance,
                                                  Point* ray_origin,
                                                  Point* ray_direction) const {
  CHECK_NOTNULL(world_);
  // Generate a viewpoint at least min_distance from any objects (if you want
  // just outside an object, just call this with min_distance = 0).

  FloatingPoint max_distance = min_distance * 2.0;

  // Figure out the dimensions of the space.
  Point space_min = world_->getMinBound() / 2.0;
  Point space_size = world_->getMaxBound() - space_min / 2.0;

  Point position = Point::Zero();
  bool success = false;
  // Generate a position, and check if it's ok.
  for (size_t i = 0; i < max_attempts_to_generate_viewpoint_; ++i) {
    position.setRandom();
    // Make this span the whole space.
    position =
        space_size.cwiseProduct(position + Point::Ones()) / 2.0 + space_min;

    if (world_->getDistanceToPoint(position, max_distance) >= min_distance) {
      success = true;
      break;
    }
  }
  if (!success) {
    return false;
  }

  // Now that we have a position, generate the ray direction.
  *ray_origin = position;
  ray_direction->setRandom();
  ray_direction->normalize();
  return true;
}

void SimulationServer::generateSDF() {
  Pointcloud ptcloud;
  Colors colors;

  Point view_origin(0.0, 0.0, 2.0);
  Point view_direction(0.0, 1.0, 0.0);
  view_direction.normalize();

  pcl::PointCloud<pcl::PointXYZRGB> ptcloud_pcl;

  for (int i = 0; i < num_viewpoints_; ++i) {
    if (!generatePlausibleViewpoint(min_dist_, &view_origin, &view_direction)) {
      // ROS_WARN(
      //     "Could not generate enough viewpoints. Generated: %d, Needed: %d",
      //     i, num_viewpoints_);
      RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Could not generate enough viewpoints. Generated: %d, Needed: "
          "%d",
          i, num_viewpoints_);
      break;
    }

    ptcloud.clear();
    colors.clear();

    CHECK_NOTNULL(world_);
    world_->getPointcloudFromViewpoint(view_origin, view_direction,
                                       depth_camera_resolution_, fov_h_rad_,
                                       max_dist_, &ptcloud, &colors);

    // Get T_G_C from ray origin and ray direction.
    Transformation T_G_C(view_origin,
                         Eigen::Quaternion<FloatingPoint>::FromTwoVectors(
                             Point(0.0, 0.0, 1.0), view_direction));

    // Transform back into camera frame.
    Pointcloud ptcloud_C;
    transformPointcloud(T_G_C.inverse(), ptcloud, &ptcloud_C);

    // Put into the real map.
    tsdf_integrator_->integratePointCloud(T_G_C, ptcloud_C, colors);

    if (generate_occupancy_) {
      occ_integrator_->integratePointCloud(T_G_C, ptcloud_C);
    }

    if (add_robot_pose_) {
      esdf_integrator_->addNewRobotPosition(view_origin);
    }

    const bool clear_updated_flag = true;
    if (incremental_) {
      esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
    }

    // Convert to a XYZRGB pointcloud.
    if (visualize_) {
      ptcloud_pcl.header.frame_id = world_frame_;
      pcl::PointXYZRGB point;
      point.x = view_origin.x();
      point.y = view_origin.y();
      point.z = view_origin.z();

      ptcloud_pcl.push_back(point);

      sensor_msgs::msg::PointCloud2 ptcloud_msg;
      pcl::toROSMsg(ptcloud_pcl, ptcloud_msg);
      view_ptcloud_pub_->publish(ptcloud_msg);
      // ros::spinOnce();
    }
  }

  // Generate ESDF in batch.
  if (!incremental_) {
    if (generate_occupancy_) {
      esdf_occ_integrator_->updateFromOccLayerBatch();
    }

    esdf_integrator_->updateFromTsdfLayerBatch();

    // Other batch options for reference:
    // esdf_integrator_->updateFromTsdfLayerBatchFullEuclidean();
    // esdf_integrator_->updateFromTsdfLayerBatchOccupancy();
  }
}

void SimulationServer::evaluate() {
  // First evaluate the TSDF vs ground truth...
  // Use only observed points.

  const double tsdf_rmse = utils::evaluateLayersRmse(*tsdf_gt_, *tsdf_test_);
  const double esdf_rmse = utils::evaluateLayersRmse(*esdf_gt_, *esdf_test_);

  // ROS_INFO_STREAM("TSDF RMSE: " << tsdf_rmse << " ESDF RMSE: " << esdf_rmse);

  // ROS_INFO_STREAM("Mesh Timings: " << std::endl << timing::Timing::Print());
  RCLCPP_INFO_STREAM(node_ptr_->get_logger(),
                     "TSDF RMSE: " << tsdf_rmse << " ESDF RMSE: " << esdf_rmse);
  RCLCPP_INFO_STREAM(node_ptr_->get_logger(),
                     "Mesh Timings: " << std::endl
                                      << timing::Timing::Print());
}

void SimulationServer::visualize() {
  if (!visualize_) {
    return;
  }

  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  pointcloud.header.frame_id = world_frame_;
  createDistancePointcloudFromTsdfLayerSlice(
      *tsdf_gt_, 2, visualization_slice_level_, &pointcloud);
  // createDistancePointcloudFromTsdfLayer(*tsdf_gt_, &pointcloud);

  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  tsdf_gt_pub_->publish(pointcloud_msg);

  pointcloud.clear();
  createDistancePointcloudFromEsdfLayerSlice(
      *esdf_gt_, 2, visualization_slice_level_, &pointcloud);
  // createDistancePointcloudFromEsdfLayer(*esdf_gt_, &pointcloud);
  sensor_msgs::msg::PointCloud2 pointcloud_msg_esdf;
  pcl::toROSMsg(pointcloud, pointcloud_msg_esdf);
  esdf_gt_pub_->publish(pointcloud_msg_esdf);

  pointcloud.clear();
  createDistancePointcloudFromTsdfLayerSlice(
      *tsdf_test_, 2, visualization_slice_level_, &pointcloud);

  sensor_msgs::msg::PointCloud2 pointcloud_msg_tsdf;
  pcl::toROSMsg(pointcloud, pointcloud_msg_tsdf);
  tsdf_test_pub_->publish(pointcloud_msg_tsdf);
  // createDistancePointcloudFromTsdfLayer(*tsdf_test_, &pointcloud);
  // tsdf_test_pub_.publish(pointcloud);

  pointcloud.clear();
  createDistancePointcloudFromEsdfLayerSlice(
      *esdf_test_, 2, visualization_slice_level_, &pointcloud);
  // createDistancePointcloudFromEsdfLayer(*esdf_test_, &pointcloud);
  // esdf_test_pub_.publish(pointcloud);
  sensor_msgs::msg::PointCloud2 pointcloud_msg_esdf_test;
  pcl::toROSMsg(pointcloud, pointcloud_msg_esdf_test);
  esdf_test_pub_->publish(pointcloud_msg_esdf_test);

  if (generate_mesh_) {
    // Generate TSDF GT mesh.
    MeshIntegratorConfig mesh_config;
    MeshLayer::Ptr mesh(new MeshLayer(tsdf_gt_->block_size()));
    MeshIntegrator<TsdfVoxel> mesh_integrator(mesh_config, tsdf_gt_.get(),
                                              mesh.get());

    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.resize(1);
    ColorMode color_mode = ColorMode::kNormals;
    fillMarkerWithMesh(mesh, color_mode, &marker_array.markers[0]);
    marker_array.markers[0].header.frame_id = world_frame_;
    tsdf_gt_mesh_pub_->publish(marker_array);

    // Also generate test mesh
    MeshLayer::Ptr mesh_test(new MeshLayer(tsdf_test_->block_size()));
    MeshIntegrator<TsdfVoxel> mesh_integrator_test(
        mesh_config, tsdf_test_.get(), mesh_test.get());
    mesh_integrator_test.generateMesh(only_mesh_updated_blocks,
                                      clear_updated_flag);
    marker_array.markers.clear();
    marker_array.markers.resize(1);
    fillMarkerWithMesh(mesh_test, color_mode, &marker_array.markers[0]);
    marker_array.markers[0].header.frame_id = world_frame_;
    tsdf_test_mesh_pub_->publish(marker_array);
  }
  // ros::spinOnce();
}

void SimulationServer::run() {
  prepareWorld();
  generateSDF();
  evaluate();
  visualize();
}

}  // namespace voxblox
