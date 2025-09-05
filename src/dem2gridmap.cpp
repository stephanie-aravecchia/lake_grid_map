#include "lake_grid_map/dem2gridmap.h"

/*!
 * ROS Wrapper on DEM
 * DEM class: loads an image representation of DEM, with associated metadata, and stores as elevation matrix
 * This wrapper convert it in a grid_map msgs, stores it in an elevation layer, and publishes it
 * This wrapper also correct the Z offset from the robot localization to the DEM frame
*/
Dem2Gridmap::Dem2Gridmap()
: Node("dem_to_gridmap"),
  map_(grid_map::GridMap({"elevation"})),
  init_complete_(false),
  got_parent_transform_(false),
  wait_for_odom_(true),
  do_color_(false)
{
  // Read the ROS parameters, loads the DEM and stores it into the grid_map map_
  readParameters();
  map_.setBasicLayers({"elevation"});
  loadDemInGridmap();
  setLocalToGridmapTransform(); 
  // Create the publisher with Keep Last and History Depth = 1
  // Reliable
  // Durabilty: transient local (similar to latching topics)
  rclcpp::QoS qos = rclcpp::QoS(1);
  qos.reliable();
  qos.transient_local();
  grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", qos);
 
  // Initialize tf broadcasters, listeners and buffers
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create a timer calling initCallback: the state machine that set the grid_map and publishes it
  init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Dem2Gridmap::initCallback, this));
 
  //Is this needed?
  timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&Dem2Gridmap::keepAlive, this));
}

Dem2Gridmap::~Dem2Gridmap() {}

void Dem2Gridmap::keepAlive() {return;}

// Reads and verifies the ROS parameters.
bool Dem2Gridmap::readParameters() {

  this->declare_parameter("frame_id", std::string("utm_local"));
  this->declare_parameter("parent_frame_id", std::string("utm_local_gte"));
  this->declare_parameter("dem_file", std::string(""));
  this->declare_parameter("color_img_file", std::string(""));
  this->declare_parameter("rate", rclcpp::ParameterValue(1.0));
  this->declare_parameter("resolution", rclcpp::ParameterValue(0.0));
  this->declare_parameter("xmin", rclcpp::ParameterValue(0.0));
  this->declare_parameter("ymin", rclcpp::ParameterValue(0.0));
  this->declare_parameter("zmin", rclcpp::ParameterValue(0.0));
  this->declare_parameter("xmax", rclcpp::ParameterValue(0.0));
  this->declare_parameter("ymax", rclcpp::ParameterValue(0.0));
  this->declare_parameter("zmax", rclcpp::ParameterValue(0.0));
  this->declare_parameter("wait_for_odom", rclcpp::ParameterValue(true));

  map_frame_id_   = this->get_parameter("frame_id").as_string();
  parent_frame_id_ = this->get_parameter("parent_frame_id").as_string();
  img_filename_  = this->get_parameter("dem_file").as_string();
  color_img_filename_  = this->get_parameter("color_img_file").as_string();
  resolution_   = this->get_parameter("resolution").as_double();

  bbox_.xmin = this->get_parameter("xmin").as_double();
  bbox_.ymin = this->get_parameter("ymin").as_double();
  bbox_.zmin = this->get_parameter("zmin").as_double();
  bbox_.xmax = this->get_parameter("xmax").as_double();
  bbox_.ymax = this->get_parameter("ymax").as_double();
  bbox_.zmax = this->get_parameter("zmax").as_double();

  wait_for_odom_  = this->get_parameter("wait_for_odom").as_bool();
  
  if (color_img_filename_ != "") {
    do_color_ = true;
  }
  
  return true;
}

// Load the dem img as a DEM, and stores it in the elevation_map of the grid_map
void Dem2Gridmap::loadDemInGridmap() { 
  RCLCPP_INFO(
      this->get_logger(),
      "Loading map: %s", img_filename_.c_str());

  dem_ = DEM(img_filename_, bbox_, resolution_);

  //Initialize the map
  grid_map::GridMapCvConverter::initializeFromImage(dem_.data(), resolution_, map_, grid_map::Position::Zero());
  RCLCPP_INFO(
    this->get_logger(),
    "Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
    map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
  
  // Store the DEM data into the elevation layer
  cv::Mat tmp = dem_.data();
  grid_map::GridMapCvConverter::addLayerFromImage<double, 1>(tmp, std::string("elevation"), map_);

  // As the origin of the frame in grid_map is the center,
  // We make it to 0
  double centerval = map_.atPosition("elevation", grid_map::Position(0, 0));
  initial_offset_z_ = centerval;
  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    map_.at(
      "elevation",
      *it) -=initial_offset_z_;
  }
  //grid_map::GridMapCvConverter::addColorLayerFromImage(*msg, "color", map_);
    RCLCPP_INFO(
      this->get_logger(),
      "Layer Elevation added.");
  if (do_color_) {
    addColorLayer();
  }
}

// Load the color_image and add it to the grid_map as a new layer
void Dem2Gridmap::addColorLayer() { 
  RCLCPP_INFO(
      this->get_logger(),
      "Loading color img: %s", color_img_filename_.c_str());

  // Load color image and convert to BGR
  cv::Mat tmp = cv::imread(color_img_filename_.c_str(), cv::IMREAD_COLOR);
  assert(!tmp.empty());
  cv::Mat imgBGR = tmp.clone();
  //cv::cvtColor(tmp, imgBGR, cv::COLOR_RGB2BGR);
  cv::cvtColor(tmp, imgBGR, cv::COLOR_BGR2RGB);

  //grid_map::GridMapCvConverter::addLayerFromImage<uint8_t, 3>(tmp, std::string("color"), map_);
  //That should expect BGR, and that's what opencv should load.. what am i missing?
  grid_map::GridMapCvConverter::addColorLayerFromImage<uint8_t, 3>(tmp, std::string("color"), map_);

  //grid_map::GridMapCvConverter::addColorLayerFromImage(*msg, "color", map_);
    RCLCPP_INFO(
      this->get_logger(),
      "Layer color added");
  }
void Dem2Gridmap::publishGridmap(){
  if (init_complete_){
    //First broadcast transfom to map frame
    local_to_gridmap_transform_.header.stamp = this->get_clock()->now();
    static_tf_broadcaster_->sendTransform(local_to_gridmap_transform_);
    
    // Publish as grid map.
    auto message = grid_map::GridMapRosConverter::toMessage(map_);
    message->header.stamp = this->get_clock()->now(); 
    message->header.frame_id = map_frame_id_+"_gridmap"; 
    grid_map_publisher_->publish(std::move(message));
      RCLCPP_INFO(
        this->get_logger(),
        "Grid Map published!");
      //Debug Z:
    double centerval = map_.atPosition("elevation", grid_map::Position(0, 0));
      RCLCPP_INFO(
        this->get_logger(),
        "Center value in grid_map: %f", centerval);
    }
    
  }
// State Machine to initialize the grid_map and set static transforms
void Dem2Gridmap::initCallback() {
  if (!init_complete_)
  {
    // STEP 1
    // Wait for the parent_transfom
    // typically the transform from utm to some local coordinate frame close to the experimental field
    // parent_frame_id_ is that local coordinate frame
    if (!got_parent_transform_)
    {
      //If we can't transform, return
      RCLCPP_INFO(this->get_logger(), " ROS time now: %f", this->get_clock()->now().seconds());
      std::string errStr;
      tf_buffer_->canTransform("utm", parent_frame_id_.c_str(), tf2::TimePointZero,
                                    tf2::durationFromSec(1), &errStr);
      if (!tf_buffer_->canTransform("utm", parent_frame_id_.c_str(), tf2::TimePointZero,
                                    tf2::durationFromSec(1), &errStr))
      {
        RCLCPP_ERROR(this->get_logger(), "Cannot transform target: %s", errStr.c_str());
        return;
      }
      // If we can transform
      // Store this initial transform from utm to local coordinate frame into parent_transform_
      // Initialize static_transform with this initial parent_transform
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_buffer_->lookupTransform(
        "utm", parent_frame_id_,
        tf2::TimePointZero);
        parent_transform_ = t.transform;
        initializeStaticTransform();
        RCLCPP_INFO(this->get_logger(), "Initial static transform set.");
        got_parent_transform_ = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          "utm", parent_frame_id_.c_str(), ex.what());
        return;
      }
    }
    else
    {
      // STEP 2
      // Now that the static transform is set (STEP 1),
      // We wait for the current robot pose 
      //    to correct the z offset from the robot localization frame to the map frame
      RCLCPP_INFO(this->get_logger(), "wait for odom is %d ", wait_for_odom_);
      if (wait_for_odom_) {
        // If we can't transform, return
        std::string errStr;
        if (!tf_buffer_->canTransform(map_frame_id_, "base_footprint", tf2::TimePointZero,
                                      tf2::durationFromSec(1), &errStr))
        {
          RCLCPP_ERROR(this->get_logger(), "Cannot transform target: %s", errStr.c_str());
          return;
        }
        // Now that we can, we have the robot pose
        // Correct the z offset from the current robot pose
        // Publish the grid_map, and we're done
        geometry_msgs::msg::TransformStamped t;
        try {
          t = tf_buffer_->lookupTransform(
          map_frame_id_, "base_link",
          tf2::TimePointZero);
          correctZfromT(t);
          RCLCPP_INFO(this->get_logger(), "Final static transform set.");
          init_complete_ = true;
          publishGridmap();
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            map_frame_id_.c_str(), "base_link", ex.what());
          return;
        }
        // If we don't need this correction
        // Publish the grid_map, and we're done
      } else {
          RCLCPP_INFO(this->get_logger(), "will publish the gridmap");
          init_complete_ = true;
          publishGridmap();
      }
    }

  }
}
// Initialize static transform_
// From the transformation from utm to a local reference frame (parent_transform_)
// And from the bbox coordinates (map origin) 
// Computes the transformation from this local reference frame to the map frame
// header.frame_id is parent_frame_id_:  the local reference frame (i.e. utm_local_gte or utm_local_lake)
// child_frame_id is the map frame, from the bbox (i.e. utm_local_lake)
void Dem2Gridmap::initializeStaticTransform() {
      // Transform utm to frame_id is given by bbox
      double orig_x = bbox_.xmin;
      double orig_y = bbox_.ymin;

      // We want to publish it with parent == parent_frame_id (utm_local_gte)
      // parent transform is utm to parent_frame_id
      // z is ignored on this step
      double parent_x = parent_transform_.translation.x;
      double parent_y = parent_transform_.translation.y;

      static_transform_.header.frame_id = parent_frame_id_;
      static_transform_.child_frame_id = map_frame_id_;
      static_transform_.transform.translation.x = orig_x - parent_x;
      static_transform_.transform.translation.y = orig_y - parent_y;
      static_transform_.transform.translation.z = 0;
      
      //For UTM transforms, only translations apply
      static_transform_.transform.rotation.x = 0.; 
      static_transform_.transform.rotation.y = 0.;
      static_transform_.transform.rotation.z = 0.;
      static_transform_.transform.rotation.w = 1.;

      //And broadcast it 
      static_transform_.header.stamp = this->get_clock()->now();
      static_tf_broadcaster_->sendTransform(static_transform_);

}

// Compute the Z offset from the current robot pose to the current corresponding Z in the map
// Correct local_to_gridmap_static_transform_ with this offset
// For completeness, we discard here any translation on z from base_link to base_footprint.
// Improvement left TODO if needed someday
void Dem2Gridmap::correctZfromT(const geometry_msgs::msg::TransformStamped& t){
        // In the map_frame, before correction, the position of the robot is:
        double odom_current_x = t.transform.translation.x;
        double odom_current_y = t.transform.translation.y;
        double odom_current_z = t.transform.translation.z;
        
        // First we convert the current position in the map (in the gridmap frame, following their convention)
        double size_x = map_.getLength().x();
        double size_y = map_.getLength().y();
        double px = (odom_current_y-size_x/2.); 
        double py = -(odom_current_x-size_y/2); 
        double map_target_z_at_odom_position = map_.atPosition("elevation", grid_map::Position(px, py));
        
        // We compute the correction to apply, and apply it to the transform of the gridmap frame
        double offset_from_robot = odom_current_z - map_target_z_at_odom_position;
        local_to_gridmap_transform_.transform.translation.z += offset_from_robot;

}
// Local Transform from top left corner in UTM to the gridmap center as requested in doc
// Origin of frame is center, x up, y left_
void Dem2Gridmap::setLocalToGridmapTransform() {
      double size_x = map_.getLength().x();
      double size_y = map_.getLength().y();
      // Correct also the Z offset
      local_to_gridmap_transform_.header.frame_id = map_frame_id_;
      local_to_gridmap_transform_.child_frame_id = map_frame_id_+"_gridmap";
      //Translation to the center
      local_to_gridmap_transform_.transform.translation.x = size_y/2.;
      local_to_gridmap_transform_.transform.translation.y = size_x/2.;
      local_to_gridmap_transform_.transform.translation.z = 0;
      //Rotation 90 degres clockwise
      tf2::Quaternion q;
      q.setRPY(0,0,M_PI_2);
      local_to_gridmap_transform_.transform.rotation = tf2::toMsg(q); 

}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  std::cout<<"Dem2Gridmap launching" << std::endl;
  rclcpp::spin(std::make_shared<Dem2Gridmap>());
  rclcpp::shutdown();
  return 0;
}
