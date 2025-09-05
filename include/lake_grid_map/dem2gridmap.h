#pragma once
// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cstdio>
#include <grid_map_msgs/msg/grid_map.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "lake_grid_map/DEM.h"

#include <string>


/*!
 * Loads an image and saves it as layer 'elevation' of a grid map.
 * The grid map is published and can be viewed in Rviz.
 */
class Dem2Gridmap : public rclcpp::Node
{
public:
  Dem2Gridmap();
  virtual ~Dem2Gridmap();

  private:

  // Reads and verifies the ROS parameters.
  bool readParameters();
  
  // Load the dem img as a DEM, and stores it in the elevation_map of the grid_map
  void loadDemInGridmap();
  void addColorLayer();
  void setLocalToGridmapTransform();
  void keepAlive();
  void publishGridmap();
  
  
  // State Machine to initialize the grid_map and set static transforms
  void initCallback();
  
  // Initialize static transform_
  // From the transformation from utm to a local reference frame (parent_transform_)
  // And from the bbox coordinates (map origin) 
  // Computes the transformation from this local reference frame to the map frame
  void initializeStaticTransform();

  // Compute the Z offset from the current robot pose to the current corresponding Z in the map
  // Correct static_transform_ with this offset
  void correctZfromT(const geometry_msgs::msg::TransformStamped& t);


  // Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;

  // Grid map data.
  grid_map::GridMap map_;

  // Resolution of the grid map.
  double resolution_;

  // Metrics bbox to store mins and maxs
  DEM::BBox bbox_;

  // DEM to load and store data
  DEM dem_;
  
  // Constant offset deduced in DEM values
  double initial_offset_z_;

  // Frame id of the grid map (i.e. utm_local_symphonie)
  std::string map_frame_id_;
  std::string map_center_frame_id_;
  std::string map_center_corrected_frame_id_;
  
  // Frame id of the parent. (a local reference_frame, i.e. utm_local_gte)
  std::string parent_frame_id_;
 
  // Path to DEM image to load 
  std::string img_filename_;
  std::string color_img_filename_;

  // Flags for the initialization
  bool init_complete_;
  bool got_parent_transform_;
  bool wait_for_odom_;
  bool do_color_;
  
  // ROS timers and TF buffers, listeners, broadcasters
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::TimerBase::SharedPtr init_timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_{nullptr};
 
  // Stores the transformation from utm to a local reference frame 
  // (ie. from tum to utm_local_gte or utm_local_lake)
  // if wait_for_odom is true:
  // This local reference frame is expected to be the localization frame of the robot
  geometry_msgs::msg::Transform parent_transform_;

  // Stores the transformation from the local reference frame to the map frame
  // (ie from utm_local_gte to utm_local_symphonie)
  geometry_msgs::msg::TransformStamped static_transform_;
  
  //Then, grid_map frame is centered on the image, x up, y left
  //To account for that, we rotate from 90 deg counter clockwise
  //And add an offset to the center
  //We publish it as a child else its untractable 
  geometry_msgs::msg::TransformStamped local_to_gridmap_transform_;
};
