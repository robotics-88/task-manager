/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/task_manager.h"
#include "bag_recorder/Rosbag.h"

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>  
#include <float.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>

#include <pcl_ros/transforms.h>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 1.0; // Goal reached if within a meter
}

namespace task_manager
{
TaskManager::TaskManager(ros::NodeHandle& node)
    : private_nh_("~")
    , nh_(node)
    , simulate_(false)
    , offline_(false)
    , save_pcd_(false)
    , save_pcd_frame_("utm")
    , utm_tf_init_(false)
    , hello_decco_manager_(node)
    , enable_autonomy_(false)
    , enable_exploration_(false)
    , ardupilot_(true)
    , use_failsafes_(false)
    , target_altitude_(2.0)
    , min_altitude_(2.0)
    , max_altitude_(10.0)
    , max_dist_to_polygon_(300.0)
    , map_tf_init_(false)
    , tf_listener_(tf_buffer_)
    , home_utm_zone_(-1)
    , mavros_map_frame_("map")
    , slam_map_frame_("slam_map")
    , slam_pose_topic_("decco/pose")
    , last_ui_heartbeat_stamp_(ros::Time::now())
    , ui_hb_threshold_(5.0)
    , do_record_(true)
    , bag_active_(false)
    , record_config_name_("r88_default")
    , drone_state_manager_(node)
    , cmd_history_("")
    , explore_action_client_("explore", true)
    , health_check_s_(5.0)
    , path_planner_topic_("/kd_pointcloud_accumulated")
    , costmap_topic_("/costmap_node/costmap/costmap")
    , lidar_topic_("/cloud_registered")
    , thermal_topic_("/thermal_cam/image_rect_color")
    , attollo_topic_("/mapir_rgn/image_rect")
    , mapir_topic_("/mapir_rgn/image_rect_color")
    , mapir_rgb_topic_("/mapir_rgn/image_rect_color")
    , rosbag_topic_("/record/heartbeat")
    , did_takeoff_(false)
    , is_armed_(false)
    , explicit_global_params_(false)
    , estimated_drone_speed_(2.0)
    , battery_failsafe_safety_factor_(2.0)
    , do_slam_(false)
{
    private_nh_.param<bool>("enable_autonomy", enable_autonomy_, enable_autonomy_);
    private_nh_.param<bool>("enable_exploration", enable_exploration_, enable_exploration_);
    private_nh_.param<bool>("ardupilot", ardupilot_, ardupilot_);
    private_nh_.param<bool>("use_failsafes", use_failsafes_, use_failsafes_);
    private_nh_.param<float>("default_altitude_m", target_altitude_, target_altitude_);
    private_nh_.param<float>("min_altitude", min_altitude_, min_altitude_);
    private_nh_.param<float>("max_altitude", max_altitude_, max_altitude_);
    private_nh_.param<double>("max_dist_to_polygon", max_dist_to_polygon_, max_dist_to_polygon_);

    std::string goal_topic = "/mavros/setpoint_position/local";
    private_nh_.param<std::string>("goal_topic", goal_topic, goal_topic);
    private_nh_.param<bool>("do_slam", do_slam_, do_slam_);
    private_nh_.param<bool>("do_record", do_record_, do_record_);
    private_nh_.param<std::string>("mavros_map_frame", mavros_map_frame_, mavros_map_frame_);
    private_nh_.param<std::string>("base_frame", mavros_base_frame_, mavros_base_frame_);
    private_nh_.param<std::string>("slam_map_frame", slam_map_frame_, slam_map_frame_);
    private_nh_.param<std::string>("path_planner_topic", path_planner_topic_, path_planner_topic_);
    private_nh_.param<std::string>("slam_pose_topic", slam_pose_topic_, slam_pose_topic_);
    private_nh_.param<std::string>("costmap_topic", costmap_topic_, costmap_topic_);
    private_nh_.param<std::string>("lidar_topic", lidar_topic_, lidar_topic_);
    private_nh_.param<std::string>("attollo_topic", attollo_topic_, attollo_topic_);
    private_nh_.param<std::string>("mapir_topic", mapir_topic_, mapir_topic_);
    private_nh_.param<std::string>("mapir_rgb_topic", mapir_rgb_topic_, mapir_rgb_topic_);
    private_nh_.param<std::string>("thermal_topic", thermal_topic_, thermal_topic_);
    private_nh_.param<std::string>("rosbag_topic", rosbag_topic_, rosbag_topic_);
    private_nh_.param<bool>("offline", offline_, offline_);
    private_nh_.param<bool>("simulate", simulate_, simulate_);
    private_nh_.param<bool>("save_pcd", save_pcd_, save_pcd_);
    private_nh_.param<std::string>("save_pcd_frame", save_pcd_frame_, save_pcd_frame_);
    private_nh_.param<std::string>("data_directory", burn_dir_prefix_, burn_dir_prefix_);
    private_nh_.param<bool>("explicit_global", explicit_global_params_, explicit_global_params_);
    private_nh_.param<double>("estimated_drone_speed", estimated_drone_speed_, estimated_drone_speed_);
    estimated_drone_speed_ = estimated_drone_speed_ < 1 ? 1.0 : estimated_drone_speed_; // This protects against a later potential div by 0
    private_nh_.param<double>("battery_failsafe_safety_factor", battery_failsafe_safety_factor_, battery_failsafe_safety_factor_);
    private_nh_.param<bool>("do_slam", do_slam_, do_slam_);
    private_nh_.param<bool>("do_mapir", do_mapir_, do_mapir_);
    private_nh_.param<bool>("do_mapir_rgb", do_mapir_rgb_, do_mapir_rgb_);
    private_nh_.param<bool>("do_attollo", do_attollo_, do_attollo_);
    private_nh_.param<bool>("do_thermal_cam", do_thermal_, do_thermal_);
    int lidar_type;
    private_nh_.param<int>("lidar_type", lidar_type, lidar_type);

    hello_decco_manager_.setFrames(mavros_map_frame_, slam_map_frame_);

    // SLAM pose sub
    decco_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(slam_pose_topic_, 10, &TaskManager::deccoPoseCallback, this);

    // TODO eventually remove this, just a patch for TX data where separate bags for MAVROS/Attollo
    if (!explicit_global_params_) {
        map_tf_timer_ = nh_.createTimer(ros::Duration(1.0), &TaskManager::mapTfTimerCallback, this);
    }
    else {
        map_tf_timer_ = nh_.createTimer(ros::Duration(1.0), &TaskManager::mapTfTimerCallbackNoGlobal, this);
        global_pose_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10);
    }

    mode_monitor_timer_ = private_nh_.createTimer(ros::Duration(1.0),
                               [this](const ros::TimerEvent&) { modeMonitor(); });

    // Health pubs/subs
    health_pub_ = nh_.advertise<std_msgs::String>("/mapversation/health_report", 10);
    if (do_slam_) {
        path_planner_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(path_planner_topic_, 10, &TaskManager::pathPlannerCallback, this);
        // Pointcloud republisher only if SLAM running
        pointcloud_repub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered_map", 10);
        registered_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 10, &TaskManager::registeredPclCallback, this);
    }
    costmap_sub_ = nh_.subscribe<map_msgs::OccupancyGridUpdate>(costmap_topic_, 10, &TaskManager::costmapCallback, this);
    if (lidar_type == 2) {
        lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(lidar_topic_, 10, &TaskManager::pointcloudCallback, this);
    }
    else if (lidar_type == 4) {
        lidar_sub_ = nh_.subscribe<livox_ros_driver::CustomMsg>(lidar_topic_, 10, &TaskManager::livoxCallback, this);
    }
    if (do_mapir_) {
        mapir_sub_ = nh_.subscribe<sensor_msgs::Image>(mapir_topic_, 10, &TaskManager::mapirCallback, this);
    }
    else if (do_mapir_rgb_) {
        mapir_sub_ = nh_.subscribe<sensor_msgs::Image>(mapir_rgb_topic_, 10, &TaskManager::mapirCallback, this);
    }
    if (do_attollo_) {
        attollo_sub_ = nh_.subscribe<sensor_msgs::Image>(attollo_topic_, 10, &TaskManager::attolloCallback, this);
    }
    if (do_thermal_) {
        thermal_sub_ = nh_.subscribe<sensor_msgs::Image>(thermal_topic_, 10, &TaskManager::thermalCallback, this);
    }
    rosbag_sub_ = nh_.subscribe<std_msgs::String>(rosbag_topic_, 10, &TaskManager::rosbagCallback, this);

    // Geo/map state services
    geopoint_service_ = nh_.advertiseService("/slam2geo", &TaskManager::convert2Geo, this);

    // MAVROS
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    local_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    vision_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    // Heartbeat timer
    int heartbeat_hz = 1;
    heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0 / heartbeat_hz), &TaskManager::heartbeatTimerCallback, this);

    // Recording
    start_record_pub_ = nh_.advertise<bag_recorder::Rosbag>("/record/start", 5);
    stop_record_pub_ = nh_.advertise<std_msgs::String>("/record/stop", 5);
    if (offline_) {
        map_yaw_sub_ = nh_.subscribe<std_msgs::Float64>("map_yaw", 10, &TaskManager::mapYawCallback, this);
        if (save_pcd_) {
            pcl_save_ .reset(new pcl::PointCloud<pcl::PointXYZI>());
        }
    }
    else {
        map_yaw_pub_ = nh_.advertise<std_msgs::Float64>("map_yaw", 5, true);
    }

    // Task status pub
    task_pub_ = nh_.advertise<messages_88::TaskStatus>("task_status", 10);
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(goal_topic, 10, &TaskManager::goalCallback, this);
    task_msg_.enable_autonomy = false;
    task_msg_.enable_exploration = false;

    // Mapversation subscriber
    mapver_sub_ = nh_.subscribe<std_msgs::String>("/mapversation/to_decco", 10, &TaskManager::packageFromMapversation, this);

    // Initialize home pos struct
    home_pos_.header.frame_id = mavros_map_frame_;
    home_pos_.pose.position.x = 0;
    home_pos_.pose.position.y = 0;
    home_pos_.pose.position.z = target_altitude_;

    initDroneStateManager();
}

TaskManager::~TaskManager(){
    if (offline_ && save_pcd_) {
        if (pcl_save_->size() > 0) {
            std::string file_name = save_pcd_frame_ + ".pcd";
            std::string all_points_dir(ros::package::getPath("task_manager_88") + "/PCD/");
            if (!boost::filesystem::exists(all_points_dir)) {
                boost::filesystem::create_directory(all_points_dir);
            }
            all_points_dir += file_name;
            pcl::PCDWriter pcd_writer;
            std::cout << "current scan saved to /PCD/" << file_name<<std::endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_save_);
        }
        else {
            std::cout << "No pointclouds to save" << std::endl;
        }
    }
}

void TaskManager::packageFromMapversation(const std_msgs::String::ConstPtr &msg) {
    json mapver_json = json::parse(msg->data);
    std::string topic = mapver_json["topic"];
    json gossip_json = mapver_json["gossip"];
    if (topic == "burn_unit_send") {
        makeBurnUnitJson(gossip_json);
    }
    else if (topic == "target_polygon") {
        json burn_unit = hello_decco_manager_.polygonToBurnUnit(gossip_json);
        makeBurnUnitJson(burn_unit);
    }
    else if (topic == "target_setpoint") {
        setpointResponse(gossip_json);
    }
    else if (topic == "emergency") {
        std::string severity = gossip_json["severity"];
        emergencyResponse(severity);
    }
}

void TaskManager::mapTfTimerCallback(const ros::TimerEvent&) {

    // Get drone heading
    if (!offline_) {
        double yaw = 0;
        if (!drone_state_manager_.getMapYaw(yaw)) {
            ROS_WARN_THROTTLE(10, "Waiting for heading from autopilot...");
            return;
        }

        ROS_INFO("Initial heading: %f", yaw);

        // Convert yaw from NED to ENU
        yaw = -yaw + 90.0;
        // Convert to radians
        map_yaw_ = yaw * M_PI / 180.0;
        std_msgs::Float64 yaw_msg;
        yaw_msg.data = map_yaw_;
        map_yaw_pub_.publish(yaw_msg);
    }
    else {
        ROS_WARN("Waiting for heading from rosbag...");
        ros::topic::waitForMessage<std_msgs::Float64>("map_yaw", nh_);
    }


    // Get roll, pitch for map stabilization
    sensor_msgs::Imu mavros_init_imu;
    ros::Rate r(0.1);
    while (!drone_state_manager_.getImu(mavros_init_imu)) {
        r.sleep();
        std::cout << "waiting for IMU initial" << std::endl;
    }
    tf2::Quaternion quatmav(mavros_init_imu.orientation.x, mavros_init_imu.orientation.y, mavros_init_imu.orientation.z, mavros_init_imu.orientation.w);
    tf2::Matrix3x3 m(quatmav);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;

    // Fill in data
    map_to_slam_tf_.header.frame_id = mavros_map_frame_;
    map_to_slam_tf_.header.stamp = ros::Time::now();
    map_to_slam_tf_.child_frame_id = slam_map_frame_;
    map_to_slam_tf_.transform.translation.x = 0.0;
    map_to_slam_tf_.transform.translation.y = 0.0;
    map_to_slam_tf_.transform.translation.z = 0.0;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw); // TODO, get rid of map_yaw record topic? It's the same as IMU yaw which is also recorded.
    map_yaw_ = yaw;
    quat_tf.normalize();

    geometry_msgs::Quaternion quat;
    tf2::convert(quat_tf, quat);
    map_to_slam_tf_.transform.rotation = quat;

    // Save inverse
    tf2::Transform map2, slam2;
    tf2::convert(map_to_slam_tf_.transform, map2);
    slam2 = map2.inverse();
    geometry_msgs::Transform slam2_tf;
    tf2::convert(slam2, slam2_tf);
    slam_to_map_tf_.header.frame_id = slam_map_frame_;
    slam_to_map_tf_.header.stamp = map_to_slam_tf_.header.stamp;
    slam_to_map_tf_.child_frame_id = mavros_map_frame_;
    slam_to_map_tf_.transform = slam2_tf;

    // Send transform and stop timer
    static_tf_broadcaster_.sendTransform(map_to_slam_tf_);

    map_tf_timer_.stop();
    map_tf_init_ = true;

    ROS_INFO("waiting for global...");
    drone_state_manager_.waitForGlobal();
    while (home_utm_zone_ < 0) {
        home_utm_zone_ = drone_state_manager_.getUTMZone();
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }
    ROS_INFO("Got global, UTM zone: %d. LL : (%f, %f)", home_utm_zone_, drone_state_manager_.getCurrentGlobalPosition().latitude, drone_state_manager_.getCurrentGlobalPosition().longitude);
    double utm_x, utm_y;
    drone_state_manager_.initUTM(utm_x, utm_y);
    hello_decco_manager_.setUtmOffsets(utm_x, utm_y);
    ROS_INFO("UTM offsets: (%f, %f)", utm_x, utm_y);
    std::cout << "utm x: " << utm_x << "\nutm y: " << utm_y << "\nmap yaw: " << map_yaw_ << std::endl;
    utm2map_tf_.header.frame_id = "utm";
    utm2map_tf_.header.stamp = ros::Time::now();
    utm2map_tf_.child_frame_id = mavros_map_frame_;
    utm2map_tf_.transform.translation.x = utm_x;
    utm2map_tf_.transform.translation.y = utm_y;
    utm2map_tf_.transform.translation.z = 0;
    utm2map_tf_.transform.rotation.x = 0;
    utm2map_tf_.transform.rotation.y = 0;
    utm2map_tf_.transform.rotation.z = 0;
    utm2map_tf_.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(utm2map_tf_);
    utm_tf_init_ = true;

    current_status_ = CurrentStatus::INITIALIZED;
    health_pub_timer_ = private_nh_.createTimer(health_check_s_,
                               [this](const ros::TimerEvent&) { publishHealth(); });
}

void TaskManager::mapTfTimerCallbackNoGlobal(const ros::TimerEvent&) {

    private_nh_.param<double>("map_yaw", map_yaw_, map_yaw_);
    double utm_x, utm_y;
    private_nh_.param<double>("utm_y", utm_y, utm_y);
    private_nh_.param<double>("utm_x", utm_x, utm_x);
    private_nh_.param<int>("utm_zone", home_utm_zone_, home_utm_zone_);

    // Fill in data
    map_to_slam_tf_.header.frame_id = mavros_map_frame_;
    map_to_slam_tf_.header.stamp = ros::Time::now();
    map_to_slam_tf_.child_frame_id = slam_map_frame_;
    map_to_slam_tf_.transform.translation.x = 0.0;
    map_to_slam_tf_.transform.translation.y = 0.0;
    map_to_slam_tf_.transform.translation.z = 0.0;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, map_yaw_);

    geometry_msgs::Quaternion quat;
    tf2::convert(quat_tf, quat);
    map_to_slam_tf_.transform.rotation = quat;

    // Send transform and stop timer
    static_tf_broadcaster_.sendTransform(map_to_slam_tf_);
    map_tf_timer_.stop();
    map_tf_init_ = true;

    hello_decco_manager_.setUtmOffsets(utm_x, utm_y);
    ROS_INFO("UTM offsets: (%f, %f)", utm_x, utm_y);
    std::cout << "utm x: " << utm_x << "\nutm y: " << utm_y << "\nmap yaw: " << map_yaw_ << std::endl;
    utm2map_tf_.header.frame_id = "utm";
    utm2map_tf_.header.stamp = ros::Time::now();
    utm2map_tf_.child_frame_id = mavros_map_frame_;
    utm2map_tf_.transform.translation.x = utm_x;
    utm2map_tf_.transform.translation.y = utm_y;
    utm2map_tf_.transform.translation.z = 0;
    utm2map_tf_.transform.rotation.x = 0;
    utm2map_tf_.transform.rotation.y = 0;
    utm2map_tf_.transform.rotation.z = 0;
    utm2map_tf_.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(utm2map_tf_);

    double lat, lon;
    hello_decco_manager_.utmToLL(utm_x, utm_y, home_utm_zone_, lat, lon);
    sensor_msgs::NavSatFix nav_msg;
    nav_msg.header.frame_id = "base_link";
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.latitude = lat;
    nav_msg.longitude = lon;
    global_pose_pub_.publish(nav_msg);

    current_status_ = CurrentStatus::INITIALIZED;
    health_pub_timer_ = private_nh_.createTimer(health_check_s_,
                               [this](const ros::TimerEvent&) { publishHealth(); });
}

void TaskManager::failsafe() {
    cmd_history_.append("Failsafe init. Failsafes active?" + std::to_string(use_failsafes_) + "\n");
    ROS_WARN("Failsafe init. Failsafes active? %d", use_failsafes_);
    if (use_failsafes_) {
        drone_state_manager_.setMode(land_mode_);
        stop();
    }
}

void TaskManager::deccoPoseCallback(const geometry_msgs::PoseStampedConstPtr &slam_pose) {

    // Transform decco pose (in slam_map frame) and publish it in mavros_map frame as /mavros/vision_pose/pose
    if (!map_tf_init_) {
        return;
    }

    geometry_msgs::TransformStamped tf;
    try {
        tf = tf_buffer_.lookupTransform(mavros_map_frame_, slam_map_frame_, ros::Time(0));
    } catch (tf2::TransformException & ex) {
        ROS_WARN_THROTTLE(10, "Cannot publish vision pose as map<>slam_map tf not yet available");
        return;
    }

    // Apply the transform to the drone pose
    geometry_msgs::PoseStamped msg_body_pose;
    geometry_msgs::PoseStamped slam = *slam_pose;

    tf2::doTransform(slam, msg_body_pose, tf);
    msg_body_pose.header.frame_id = mavros_map_frame_;
    msg_body_pose.header.stamp = slam_pose->header.stamp;

    vision_pose_publisher_.publish(msg_body_pose);

    last_slam_pos_stamp_ = slam_pose->header.stamp;

    // Map tf for offline
    if (explicit_global_params_) {
        geometry_msgs::PointStamped point_in, point_out;
        point_in.header = slam_pose->header;
        point_in.point.x = 0;
        point_in.point.y = 0;
        point_in.point.z = 0;
        tf2::doTransform(point_in, point_out, utm2map_tf_);
        double lat, lon;
        hello_decco_manager_.utmToLL(point_out.point.x, point_out.point.y, home_utm_zone_, lat, lon);
        sensor_msgs::NavSatFix nav_msg;
        nav_msg.header.frame_id = mavros_base_frame_;
        nav_msg.header.stamp = slam_pose->header.stamp;
        nav_msg.latitude = lat;
        nav_msg.longitude = lon;
        global_pose_pub_.publish(nav_msg);

    }
}

void TaskManager::emergencyResponse(const std::string severity) {
    ROS_WARN("Emergency response initiated, level %s.", severity.c_str());
    // Immediately set to hover
    drone_state_manager_.setMode(loiter_mode_);
    cmd_history_.append("Emergency init with severity " + severity + "\n");

    // Then respond based on severity 
    if (severity == "PAUSE") {
        // NOTICE = PAUSE
        // TODO, tell exploration to stop searching frontiers. For now, will keep blacklisting them, but the drone is in loiter mode. Currently no way to pick back up and set to guided mode (here or in HD)
    }
    else if (severity == "LAND") {
        // EMERGENCY = LAND IMMEDIATELY
        pauseOperations();
        drone_state_manager_.setMode(land_mode_);
    }
    else if (severity == "RTL") {
        // CRITICAL = RTL
        pauseOperations();
        drone_state_manager_.setMode(rtl_mode_);
    }
}

bool TaskManager::convert2Geo(messages_88::Geopoint::Request& req, messages_88::Geopoint::Response& resp) {
    // Sanity check UTM
    if (home_utm_zone_ != drone_state_manager_.getUTMZone()) {
        ROS_WARN("UTM zones crossed. Home UTM: %d. Now UTM: %d", home_utm_zone_, drone_state_manager_.getUTMZone());
        return false;
        // TODO decide what to do about it
    }
    geometry_msgs::PointStamped in, out;
    in.header.frame_id = slam_map_frame_;
    in.header.stamp = ros::Time(0);
    in.point.x = req.slam_position.x;
    in.point.y = req.slam_position.y;
    in.point.z = req.slam_position.z;
    hello_decco_manager_.mapToGeopoint(in, out, -map_yaw_);
    resp.utm_position.x = out.point.x;
    resp.utm_position.y = out.point.y;
    return true;
}

void TaskManager::heartbeatTimerCallback(const ros::TimerEvent&) {
    sensor_msgs::NavSatFix hb = drone_state_manager_.getCurrentGlobalPosition();
    geometry_msgs::PoseStamped local = drone_state_manager_.getCurrentLocalPosition();
    geometry_msgs::Quaternion quat_flu = local.pose.orientation;
    double yaw = drone_state_manager_.getCompass();
    json j = {
        {"latitude", hb.latitude},
        {"longitude", hb.longitude},
        {"altitude", hb.altitude},
        {"heading", yaw},
        {"header", {
            {"frame_id", hb.header.frame_id},
            {"stamp", hb.header.stamp.toSec()}
        }}
    };
    hello_decco_manager_.packageToMapversation("decco_heartbeat", j);

    ros::Time now_time = ros::Time::now();
    float interval = (now_time - last_ui_heartbeat_stamp_).toSec();
    // TODO needs also check if drone state is in air/action
    if (interval > ui_hb_threshold_) {
        messages_88::Emergency emergency;
        // TODO fill in pause msg
        // emergency_client_.call(emergency);
    }
}

void TaskManager::uiHeartbeatCallback(const json &msg) {
    last_ui_heartbeat_stamp_ = ros::Time::now();
}

bool TaskManager::pauseOperations() {
    // TODO figure out how to pause and restart
    actionlib::SimpleClientGoalState goal_state = explore_action_client_.getState();
    if (goal_state == actionlib::SimpleClientGoalState::ACTIVE) {
        explore_action_client_.cancelAllGoals();
    }
    return true;
}

void TaskManager::initDroneStateManager() {
    // Drone state manager setup
    drone_state_manager_.setAutonomyEnabled(enable_autonomy_);
    task_msg_.enable_autonomy = enable_autonomy_;
    task_msg_.enable_exploration = enable_exploration_;

    if (ardupilot_) {
        land_mode_ = "LAND";
        loiter_mode_ = "LOITER";
        rtl_mode_ = "RTL";
        guided_mode_ = "GUIDED";
    }
    else {
        land_mode_ = "AUTO.LAND";
        loiter_mode_ = "AUTO.LOITER";
        rtl_mode_ = "AUTO.RTL";
        guided_mode_ = "OFFBOARD";
        drone_state_manager_.setMode("POSCTL");
    }
}

bool TaskManager::getReadyForAction() {
    cmd_history_.append("Get ready command received.\n ");
    getDroneReady();
    return true;
}

void TaskManager::getReadyForExplore() {
    cmd_history_.append("Get ready to explore command received.\n ");
    bool needs_transit = false;
    geometry_msgs::PoseStamped target_position;
    if (!isInside(current_polygon_, drone_state_manager_.getCurrentLocalPosition().pose.position)) {
        cmd_history_.append("Transit to explore required.\n ");
        needs_transit = true;
        // Find nearest point on the polygon
        // Check polygon area and distance
        double min_dist;
        if (!polygonDistanceOk(min_dist, target_position, current_polygon_)) {
            ROS_WARN("Polygon rejected, exceeds maximum starting distance threshold.");
            return;
        }
        else {
            ROS_DEBUG("Polygon received, distance ok.");
        }
        ROS_INFO("target position x,y was %f, %f", target_position.pose.position.x, target_position.pose.position.y);
        target_position.pose.position.z = target_altitude_;
    }

    if (task_msg_.enable_autonomy) {
        getDroneReady();
    }

    boost::uuids::random_generator generator;
    boost::uuids::uuid u = generator();
    messages_88::ExploreGoal explore_goal;
    if (task_msg_.enable_autonomy && needs_transit) {
        padNavTarget(target_position);
        messages_88::NavToPointGoal nav_goal;
        nav_goal.point = target_position.pose.position;
        nav_goal.uuid.data = boost::uuids::to_string(u);
        explore_goal.uuid.data = boost::uuids::to_string(u);
        startNav2PointTask(nav_goal);
    }
    else {
        explore_goal.uuid.data = "NO_TRANSIT";
    }
    explore_goal.polygon = current_polygon_;
    explore_goal.altitude = target_altitude_;
    explore_goal.min_altitude = min_altitude_;
    explore_goal.max_altitude = max_altitude_;
    if (task_msg_.enable_exploration) {
        explore_action_client_.waitForServer();
        sendExploreTask(explore_goal);
        cmd_history_.append("Sent explore goal.\n");
        hello_decco_manager_.updateBurnUnit(current_index_, "ACTIVE");
    }
    return;
}

void TaskManager::startNav2PointTask(messages_88::NavToPointGoal &nav_goal) {
    messages_88::NavToPointGoal goal = nav_goal;
    current_status_ = CurrentStatus::NAVIGATING;
    geometry_msgs::Point point = goal.point;
    current_target_ = point;
    geometry_msgs::PoseStamped target;
    target.header.frame_id = slam_map_frame_;
    target.header.stamp = ros::Time::now();
    target.pose.position = point;
    local_pos_pub_.publish(target);
    while (!(isInside(current_polygon_, drone_state_manager_.getCurrentLocalPosition().pose.position) || current_status_ == CurrentStatus::WAITING_TO_EXPLORE)) {
        ros::Duration(1.0).sleep();    
    }
}

void TaskManager::sendExploreTask(messages_88::ExploreGoal &goal) {
    cmd_history_.append("Sending explore goal.\n");
    current_explore_goal_ = goal;
    // TODO inspect uuid/waiting logic now that drone state mgr lives here instead of monarch
    std::string uuid = current_explore_goal_.uuid.data; // If no transit, uuid=NO_TRANSIT, otherwise should match transit uuid

    if (uuid == "NO_TRANSIT" || current_status_ == CurrentStatus::WAITING_TO_EXPLORE) {
        startExploreTask();
    }
    else {
        status_timer_ = private_nh_.createTimer(ros::Duration(1.0),
                               [this](const ros::TimerEvent&) { readyToExplore(); });
    }

}

void TaskManager::readyToExplore() {
    if (current_status_ == CurrentStatus::WAITING_TO_EXPLORE) {
        status_timer_.stop();
        startExploreTask();
    }
}

void TaskManager::startExploreTask() {
    // Start with a rotation command in case no frontiers immediately processed, will be overridden with first exploration goal
    geometry_msgs::Twist vel;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = M_PI_2; // PI/2 rad/s
    local_vel_pub_.publish(vel);

    current_status_ = CurrentStatus::EXPLORING;
    explore_action_client_.sendGoal(current_explore_goal_);
}

void TaskManager::stop() {
    stopBag();
    pauseOperations();
}

void TaskManager::modeMonitor() {
    std::string mode = drone_state_manager_.getFlightMode();
    bool armed = drone_state_manager_.getIsArmed();
    bool in_air = drone_state_manager_.getIsInAir();
    if (!is_armed_ && armed) {
        cmd_history_.append("Manually set armed state to true. \n ");
        is_armed_ = true;
    }
    if (is_armed_ && !bag_active_) {
        cmd_history_.append("Checking start bag record due to arming detected. Armed: " + std::to_string(armed) + ", flight mode: " + mode + "\n");
        // Handle recording during manual take off
        startBag();
    }
    if (is_armed_ && !armed) {
        cmd_history_.append("Disarm detected. \n ");
        // Handle save bag during disarm (manual or auton)
        stop();
        is_armed_ = false; // Reset so can restart if another arming
    }

    // Update home position
    home_pos_.header.stamp = ros::Time::now();

    // This is the primary state machine
    switch (current_status_) {
        case CurrentStatus::EXPLORING:
        {
            // Check if we can make it home on current battery levels
            if (!isBatteryOk()) {
                std::string action_string = "Battery level low, returning home";
                ROS_WARN("%s", action_string.c_str());
                cmd_history_.append(action_string);
                pauseOperations();
                doRtl88();
            }
            
            // Check action client status to see if complete
            actionlib::SimpleClientGoalState goal_state = explore_action_client_.getState();
            bool is_aborted = goal_state == actionlib::SimpleClientGoalState::ABORTED;
            bool is_lost = goal_state == actionlib::SimpleClientGoalState::LOST;
            bool is_completed = goal_state == actionlib::SimpleClientGoalState::SUCCEEDED;
            if (is_aborted || is_lost || is_completed) {
                ROS_INFO("explore action client state: %s", explore_action_client_.getState().getText().c_str());
                std::string action_string = "Exploration complete, action client status: " + explore_action_client_.getState().getText() + ", sending SLAM origin as position target. \n";
                cmd_history_.append(action_string);
                doRtl88();
                hello_decco_manager_.updateBurnUnit(current_index_, "COMPLETED");
            }
        }
        break;

        case CurrentStatus::RTL_88:
        {
            if (drone_state_manager_.getCurrentLocalPosition().pose.position == home_pos_.pose.position) {
                ROS_INFO("RTL_88 completed, landing");
                drone_state_manager_.setMode(land_mode_);
                current_status_ = CurrentStatus::LANDING;
            }
        }
        break;
    }


    task_msg_.header.stamp = ros::Time::now();
    task_msg_.cmd_history.data = cmd_history_.c_str();
    task_msg_.current_status.data = getStatusString();
    task_pub_.publish(task_msg_);
    json task_json = makeTaskJson();
    hello_decco_manager_.packageToMapversation("task_status", task_json);
    // std_msgs::String task_json_msg;
    // task_json_msg.data = task_json.dump();
    // task_json_pub_.publish(task_json_msg);
}

void TaskManager::doRtl88() {
    ROS_INFO("Doing RTL 88");
    local_pos_pub_.publish(home_pos_);
    current_status_ = CurrentStatus::RTL_88;
}

bool TaskManager::isBatteryOk() {
    geometry_msgs::Point location = drone_state_manager_.getCurrentLocalPosition().pose.position;
    double distance = sqrt(location.x * location.x + location.y * location.y);
    double time_to_home = distance / estimated_drone_speed_;
    double flight_time_remaining = drone_state_manager_.getFlightTimeRemaining();

    return (flight_time_remaining > battery_failsafe_safety_factor_ * time_to_home);
}

void TaskManager::startBag() {
    if (bag_active_) {
        return;
    }
    cmd_history_.append("Bag starting, prefix " + burn_dir_prefix_ + " .\n ");
    bag_recorder::Rosbag start_bag_msg;
    start_bag_msg.data_dir = burn_dir_prefix_;
    start_bag_msg.bag_name = "decco";
    start_bag_msg.config = record_config_name_;
    start_bag_msg.header.stamp = ros::Time::now();
    start_record_pub_.publish(start_bag_msg);
    bag_active_ = true;
}

void TaskManager::stopBag() {
    if (!bag_active_) {
        return;
    }
    cmd_history_.append("Stopping bag.\n");
    std_msgs::String stop_msg;
    stop_msg.data = record_config_name_;
    stop_record_pub_.publish(stop_msg);
    bag_active_ = false;
}

void TaskManager::getDroneReady() {
    ROS_INFO("getting ready for action");
    if (do_record_) {
        startBag();
    }
    if (!drone_state_manager_.readyForAction()) {
        did_takeoff_ = drone_state_manager_.getReadyForAction();
        cmd_history_.append("Drone ready for action result: " + std::to_string(did_takeoff_) + "\n");
    }
}

bool TaskManager::isInside(const geometry_msgs::Polygon& polygon, const geometry_msgs::Point& point)
{
  // Determine if the given point is inside the polygon using the number of crossings method
  // https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
  int n = polygon.points.size();
  int cross = 0;
  // Loop from i = [0 ... n - 1] and j = [n - 1, 0 ... n - 2]
  // Ensures first point connects to last point
  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    // Check if the line to x,y crosses this edge
    if ( ((polygon.points[i].y > point.y) != (polygon.points[j].y > point.y))
           && (point.x < (polygon.points[j].x - polygon.points[i].x) * (point.y - polygon.points[i].y) /
            (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x) )
    {
      cross++;
    }
  }
  // Return true if the number of crossings is odd
  return cross % 2 > 0;
}

bool TaskManager::polygonDistanceOk(double &min_dist, geometry_msgs::PoseStamped &target, geometry_msgs::Polygon &map_region) {
    // Medium check, computes distance to nearest point on 2 most likely polygon edges
    // Polygon is already in map coordinates, i.e., expressed in meters from UAS home
    min_dist = DBL_MAX;
    int closest_point_ind = 0;
    for (int ii=0; ii < map_region.points.size(); ii++) {
        double d = std::pow(map_region.points.at(ii).x, 2) + std::pow(map_region.points.at(ii).y, 2);
        if (d < min_dist) {
            min_dist = d;
            closest_point_ind = ii;
        }
    }
    // Find line intersection with each segment connecting to closest point
    geometry_msgs::Point32 point1, point2, closest_point = map_region.points.at(closest_point_ind);
    int ind1, ind2;
    if (closest_point_ind == 0) {
        ind1 = map_region.points.size() - 1;
    }
    else {
        ind1 = closest_point_ind - 1;
    }
    if (closest_point_ind == map_region.points.size() - 1) {
        ind2 = 0;
    }
    else {
        ind2 = closest_point_ind + 1;
    }
    point1 = map_region.points.at(ind1);
    point2 = map_region.points.at(ind2);

    // Compute intersection
    bool intersection1 = false, intersection2 = false;
    geometry_msgs::Point my_position = drone_state_manager_.getCurrentLocalPosition().pose.position;
    // Compute first edge
    double dx1 = closest_point.x - point1.x;
    double dy1 = closest_point.y - point1.y;
    double m1 = dy1 / dx1;
    // TODO add check for either = 0
    double b1 = closest_point.y - m1 * closest_point.x;
    double mstar1 = -1 / m1;
    double bstar1 = my_position.y + mstar1 * my_position.x;
    double xstar1 = (mstar1 * my_position.x + bstar1 - b1) / m1;
    double ystar1 = mstar1 * xstar1 + bstar1;
    double dist1 = DBL_MAX;
    if (xstar1 > std::min(closest_point.x, point1.x) && xstar1 < std::max(closest_point.x, point1.x) && ystar1 > std::min(closest_point.y, point1.y) && ystar1 < std::max(closest_point.y, point1.y)) {
        // Point is inside the line segment
        intersection1 = true;
        dist1 = std::sqrt(std::pow(my_position.x - xstar1, 2) + std::pow(my_position.y - ystar1, 2));
    }
    // Compute second edge
    double dx2 = closest_point.x - point2.x;
    double dy2 = closest_point.y - point2.y;
    double m2 = dy2 / dx2;
    // TODO add check for either = 0
    double b2 = closest_point.y - m2 * closest_point.x;
    double mstar2 = -1 / m2;
    double bstar2 = my_position.y + mstar2 * my_position.x;
    double xstar2 = (mstar2 * my_position.x + bstar2 - b2) / m2;
    double ystar2 = mstar2 * xstar2 + bstar2;
    double dist2 = DBL_MAX;
    if (xstar2 > std::min(closest_point.x, point2.x) && xstar2 < std::max(closest_point.x, point2.x) && ystar2 > std::min(closest_point.y, point2.y) && ystar2 < std::max(closest_point.y, point2.y)) {
        // Point is inside the line segment
        intersection2 = true;
        dist2 = std::sqrt(std::pow(my_position.x - xstar2, 2) + std::pow(my_position.y - ystar2, 2));
    }
    geometry_msgs::Point target_position;
    if (intersection1 && intersection2) {
        if (dist1 < dist2) {
            min_dist = dist1;
            target_position.x = xstar1;
            target_position.y = ystar1;
        }
        else {
            min_dist = dist2;
            target_position.x = xstar2;
            target_position.y = ystar2;
        }
    }
    else if (intersection1) {
        min_dist = dist1;
        target_position.x = xstar1;
        target_position.y = ystar1;
    }
    else if (intersection2) {
        min_dist = dist2;
        target_position.x = xstar2;
        target_position.y = ystar2;
    }
    else {
        target_position.x = closest_point.x;
        target_position.y = closest_point.y;
    }
    target.pose.position = target_position;

    if (min_dist > std::pow(max_dist_to_polygon_, 2)) {
        ROS_WARN("Max dist exceeded (%f m), will not execute flight.", std::sqrt(min_dist));
        return false;
    }
    return true;
}

void TaskManager::padNavTarget(geometry_msgs::PoseStamped &target) {
    // Add 2m to ensure fully inside polygon, otherwise exploration won't start
    float padding = 2.0;
    geometry_msgs::Point my_position = drone_state_manager_.getCurrentLocalPosition().pose.position;
    double dif_x = target.pose.position.x - my_position.x;
    double dif_y = target.pose.position.y - my_position.y;
    double norm = sqrt(std::pow(dif_x, 2) + std::pow(dif_y, 2));
    double normed_dif_x = dif_x / norm;
    double normed_dif_y = dif_y / norm;
    target.pose.position.x += normed_dif_x * padding;
    target.pose.position.y += normed_dif_y * padding;

}

std::string TaskManager::getStatusString() {
    switch (current_status_) {
        case CurrentStatus::ON_START:           return "ON_START";
        case CurrentStatus::EXPLORING:          return "EXPLORING";
        case CurrentStatus::WAITING_TO_EXPLORE: return "WAITING_TO_EXPLORE";
        case CurrentStatus::INITIALIZED:        return "INITIALIZED";
        case CurrentStatus::NAVIGATING:         return "NAVIGATING";
        case CurrentStatus::RTL_88:             return "RTL_88";
        case CurrentStatus::TAKING_OFF:         return "TAKING_OFF";
        case CurrentStatus::LANDING:            return "LANDING";
        default:                                return "unknown";
    }
}

void TaskManager::publishHealth() {
    // TODO fill in json string and publish
    ros::Duration half_dur = ros::Duration(0.5 * health_check_s_.toSec() );
    bool explore_healthy = explore_action_client_.waitForServer(half_dur);
    auto jsonObjects = json::object();
    ros::Time t = ros::Time::now();
    json header = {
        {"frame_id", "decco"},
        {"stamp", t.toSec()},
    };
    jsonObjects["header"] = header;

    auto healthObjects = json::array();
    // 1) Path planner. TODO, replace with ROS node binding
    bool path_healthy = (t - last_path_planner_stamp_ < health_check_s_);
    if (!path_healthy && do_slam_) {
        cmd_history_.append("Failsafe triggered by path unhealthy. \n");
        failsafe();
    }
    json j = {
        {"name", "lidar"},
        {"label", "LiDAR"},
        {"isHealthy", (t - last_lidar_stamp_ < health_check_s_)}
    };
    if (do_slam_) {
        // SLAM position
        bool slam_healthy = (t - last_slam_pos_stamp_ < health_check_s_);
        if (!slam_healthy) {
            cmd_history_.append("Failsafe triggered by SLAM unhealthy. \n");
            failsafe();
        }
        j = {
            {"name", "slamPosition"},
            {"label", "SLAM position"},
            {"isHealthy", slam_healthy}
        };
        healthObjects.push_back(j);
        j = {
            {"name", "pathPlanner"},
            {"label", "Path planner"},
            {"isHealthy", path_healthy}
        };
        healthObjects.push_back(j);
        // costmap
        j = {
            {"name", "costmap"},
            {"label", "Costmap"},
            {"isHealthy", (t - last_costmap_stamp_ < health_check_s_)}
        };
        healthObjects.push_back(j);
        // 6) Explore
        j = {
            {"name", "explore"},
            {"label", "Explore Service"},
            {"isHealthy", explore_healthy}
        };
        healthObjects.push_back(j);
    }
    // MAPIR
    if (do_mapir_ || do_mapir_rgb_) {
        j = {
            {"name", "mapir"},
            {"label", "MAPIR Camera"},
            {"isHealthy", (t - last_mapir_stamp_ < health_check_s_)}
        };
        healthObjects.push_back(j);
    }
    // Attollo
    if (do_attollo_) {
        j = {
            {"name", "attollo"},
            {"label", "Attollo Camera"},
            {"isHealthy", (t - last_attollo_stamp_ < health_check_s_)}
        };
        healthObjects.push_back(j);
    }
    // Thermal
    if (do_thermal_) {
        j = {
            {"name", "thermal"},
            {"label", "Thermal Camera"},
            {"isHealthy", (t - last_thermal_stamp_ < health_check_s_)}
        };
        healthObjects.push_back(j);
    }
    // ROS bag
    if (is_armed_ && !simulate_) {
        j = {
            {"name", "rosbag"},
            {"label", "ROS bag"},
            {"isHealthy", (t - last_rosbag_stamp_ < health_check_s_)}
        };
        healthObjects.push_back(j);
    }
    jsonObjects["healthIndicators"] = healthObjects;
    hello_decco_manager_.packageToMapversation("health_report", jsonObjects);
}

void TaskManager::registeredPclCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (!map_tf_init_) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr reg_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg (*msg, *reg_cloud);

    pcl_ros::transformPointCloud(*reg_cloud, *map_cloud, map_to_slam_tf_.transform);

    sensor_msgs::PointCloud2 map_cloud_ros;
    pcl::toROSMsg(*map_cloud, map_cloud_ros);
    map_cloud_ros.header.frame_id = mavros_map_frame_;
    pointcloud_repub_.publish(map_cloud_ros);

    if (utm_tf_init_ && offline_ & save_pcd_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = map_cloud;
        if (save_pcd_frame_ == "utm") {
            pcl::PointCloud<pcl::PointXYZI>::Ptr utm_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl_ros::transformPointCloud(*map_cloud, *utm_cloud, utm2map_tf_.transform);
            cloud = utm_cloud;
        }
        *pcl_save_ += *cloud;
    }
}

void TaskManager::pathPlannerCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    last_path_planner_stamp_ = msg->header.stamp;
}

void TaskManager::costmapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr &msg) {
    last_costmap_stamp_ = msg->header.stamp;
}

void TaskManager::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    last_lidar_stamp_ =  msg->header.stamp;
}

void TaskManager::livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    last_lidar_stamp_ =  msg->header.stamp;
}

void TaskManager::mapirCallback(const sensor_msgs::ImageConstPtr &msg) {
    last_mapir_stamp_ = msg->header.stamp;
}

void TaskManager::attolloCallback(const sensor_msgs::ImageConstPtr &msg) {
    last_attollo_stamp_ = msg->header.stamp;
}

void TaskManager::thermalCallback(const sensor_msgs::ImageConstPtr &msg) {
    last_thermal_stamp_ = msg->header.stamp;
}

void TaskManager::rosbagCallback(const std_msgs::StringConstPtr &msg) {
    last_rosbag_stamp_ = ros::Time::now();
}

void TaskManager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    goal_ = *msg;
}

void TaskManager::makeBurnUnitJson(json burn_unit) {
    if (!enable_exploration_) {
        ROS_WARN("Exploration disabled, burn unit ignored.");
        return;
    }
    if (!map_tf_init_) {
        ROS_WARN("Not ready for flight, try again after initialized.");
        return;
    }
    std::string name = burn_unit["name"];
    burn_dir_prefix_ = burn_dir_prefix_ + name + "/";
    hello_decco_manager_.makeBurnUnitJson(burn_unit, home_utm_zone_);
    current_index_ = hello_decco_manager_.initBurnUnit(current_polygon_);
    if (current_index_ < 0) {
        ROS_WARN("No burn polygon was found, all are already complete.");
        std::string burn_status_string = "No burn units subpolygons were incomplete, not exploring. \n";
        cmd_history_.append(burn_status_string);
    }
    else {
        getReadyForExplore();
    }
}

void TaskManager::setpointResponse(json &json_msg) {
    // ATM, this response is purely a testing function. 
    startBag();
}

void TaskManager::mapYawCallback(const std_msgs::Float64::ConstPtr &msg) {
    map_yaw_ = msg->data;
    ROS_INFO("got map yaw: %f", map_yaw_);
}

json TaskManager::makeTaskJson() {
    json j;
    j["flightMode"] = drone_state_manager_.getFlightMode();
    double xval = goal_.pose.position.x;
    double yval = goal_.pose.position.y;
    json goalArray;
    goalArray.push_back(xval);
    goalArray.push_back(yval);
    j["goal"] = goalArray;
    j["taskStatus"] = getStatusString();
    j["minAltitude"] = min_altitude_;
    j["maxAltitude"] = max_altitude_;
    j["targetAltitude"] = target_altitude_;
    j["flightMinLeft"] = (int)(drone_state_manager_.getFlightTimeRemaining() / 60.f);
    return j;
}

}