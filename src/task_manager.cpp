/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/task_manager.h"

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <float.h>

#include <sensor_msgs/msg/image.hpp>
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "task_manager/decco_utilities.h"
#include "task_manager/LawnmowerPattern.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace task_manager
{
TaskManager::TaskManager(std::shared_ptr<flight_controller_interface::FlightControllerInterface> fci)
    : Node("task_manager")
    , current_task_(Task::INITIALIZING)
    , flight_controller_interface_(fci)
    , task_manager_loop_duration_(1.0)
    , simulate_(false)
    , offline_(false)
    , save_pcd_(false)
    , utm_tf_init_(false)
    , enable_autonomy_(false)
    , use_failsafes_(false)
    , target_altitude_(3.0)
    , target_agl_(3.0)
    , min_altitude_(2.0)
    , min_agl_(2.0)
    , max_altitude_(10.0)
    , max_agl_(10.0)
    , altitude_offset_(0.0)
    , home_elevation_(0.0)
    , max_dist_to_polygon_(300.0)
    , flightleg_area_acres_(3.0)
    , map_tf_init_(false)
    , home_utm_zone_(-1)
    , mavros_map_frame_("map")
    , slam_map_frame_("slam_map")
    , slam_pose_topic_("decco/pose")
    , ui_hb_threshold_(5.0)
    , do_record_(true)
    , bag_active_(false)
    , record_config_file_("")
    , burn_unit_name_("")
    , cmd_history_("")
    , lawnmower_started_(false)
    , health_check_pub_duration_(rclcpp::Duration(5.0, 0))
    , path_planner_topic_("/kd_pointcloud_accumulated")
    , costmap_topic_("/costmap_node/costmap/costmap_updates")
    , lidar_topic_("/cloud_registered")
    , thermal_topic_("/thermal_cam/image_rect_color")
    , attollo_topic_("/mapir_rgn/image_rect")
    , mapir_topic_("/mapir_rgn/image_rect_color")
    , mapir_rgb_topic_("/mapir_rgn/image_rect_color")
    , rosbag_topic_("/record/heartbeat")
    , explore_action_result_(rclcpp_action::ResultCode::UNKNOWN)
    , is_armed_(false)
    , in_autonomous_flight_(false)
    , explicit_global_params_(false)
    , init_remote_id_message_sent_(false)
    , takeoff_attempts_(0)
    , estimated_drone_speed_(2.0)
    , battery_failsafe_safety_factor_(2.0)
    , do_slam_(false)
    , lidar_pitch_(0.0)
    , lidar_x_(0.0)
    , lidar_z_(0.0)
    , needs_takeoff_(false)
    , last_rid_updated_timestamp_(0)
    , operator_id_("")
    , hd_drone_id_(0)
    , start_time_(0)
    , last_lidar_stamp_(0, 0, RCL_ROS_TIME)
    , last_slam_pos_stamp_(0, 0, RCL_ROS_TIME)
    , last_path_planner_stamp_(0, 0, RCL_ROS_TIME)
    , last_costmap_stamp_(0, 0, RCL_ROS_TIME)
    , last_mapir_stamp_(0, 0, RCL_ROS_TIME)
    , last_thermal_stamp_(0, 0, RCL_ROS_TIME)
    , last_attollo_stamp_(0, 0, RCL_ROS_TIME)
    , last_rosbag_stamp_(0, 0, RCL_ROS_TIME)
    , last_health_pub_stamp_(0, 0, RCL_ROS_TIME)
    , last_preflight_check_log_stamp_(0, 0, RCL_ROS_TIME)
    , last_ui_heartbeat_stamp_(0, 0, RCL_ROS_TIME)
    , lidar_timeout_(rclcpp::Duration::from_seconds(0.5))
    , slam_timeout_(rclcpp::Duration::from_seconds(0.5))
    , path_timeout_(rclcpp::Duration::from_seconds(0.5))
    , costmap_timeout_(rclcpp::Duration::from_seconds(3.0))
    , explore_timeout_(1s)
    , mapir_timeout_(rclcpp::Duration::from_seconds(1.0))
    , attollo_timeout_(rclcpp::Duration::from_seconds(1.0))
    , thermal_timeout_(rclcpp::Duration::from_seconds(1.0))
    , rosbag_timeout_(rclcpp::Duration::from_seconds(1.0))
{
}

void TaskManager::initialize() {
    RCLCPP_INFO(this->get_logger(), "TM entered init");

    this->declare_parameter("enable_autonomy", enable_autonomy_);
    this->declare_parameter("use_failsafes", use_failsafes_);
    this->declare_parameter("default_alt", target_altitude_);
    this->declare_parameter("min_alt", min_altitude_);
    this->declare_parameter("max_alt", max_altitude_);
    this->declare_parameter("max_dist_to_polygon", max_dist_to_polygon_);
    std::string goal_topic = "/mavros/setpoint_position/local";
    this->declare_parameter("goal_topic", goal_topic);
    this->declare_parameter("do_slam", do_slam_);
    this->declare_parameter("do_record", do_record_);
    this->declare_parameter("mavros_map_frame", mavros_map_frame_);
    this->declare_parameter("base_frame", mavros_base_frame_);
    this->declare_parameter("slam_map_frame", slam_map_frame_);
    this->declare_parameter("path_planner_topic", path_planner_topic_);
    this->declare_parameter("slam_pose_topic", slam_pose_topic_);
    this->declare_parameter("costmap_topic", costmap_topic_);
    this->declare_parameter("lidar_topic", lidar_topic_);
    this->declare_parameter("attollo_topic", attollo_topic_);
    this->declare_parameter("mapir_topic", mapir_topic_);
    this->declare_parameter("mapir_rgb_topic", mapir_rgb_topic_);
    this->declare_parameter("thermal_topic", thermal_topic_);
    this->declare_parameter("rosbag_topic", rosbag_topic_);
    this->declare_parameter("offline", offline_);
    this->declare_parameter("save_pcd", save_pcd_);
    this->declare_parameter("simulate", simulate_);
    this->declare_parameter("data_directory", data_directory_);
    this->declare_parameter("record_config_file", record_config_file_);
    this->declare_parameter("explicit_global", explicit_global_params_);
    this->declare_parameter("estimated_drone_speed", estimated_drone_speed_);
    estimated_drone_speed_ = estimated_drone_speed_ < 1 ? 1.0 : estimated_drone_speed_; // this protects against a later potential div by 0
    this->declare_parameter("battery_failsafe_safety_factor", battery_failsafe_safety_factor_);
    this->declare_parameter("do_mapir", do_mapir_);
    this->declare_parameter("do_mapir_rgb", do_mapir_rgb_);
    this->declare_parameter("do_attollo", do_attollo_);
    this->declare_parameter("do_thermal_cam", do_thermal_);
    int lidar_type;
    this->declare_parameter("lidar_type", lidar_type);
    this->declare_parameter("lidar_pitch", lidar_pitch_);
    this->declare_parameter("lidar_x", lidar_x_);
    this->declare_parameter("lidar_z", lidar_z_);
    this->declare_parameter("flightleg_area_acres", flightleg_area_acres_);

    // Now get parameters
    this->get_parameter("enable_autonomy", enable_autonomy_);
    this->get_parameter("use_failsafes", use_failsafes_);
    this->get_parameter("default_alt", target_altitude_);
    this->get_parameter("min_alt", min_altitude_);
    this->get_parameter("max_alt", max_altitude_);
    this->get_parameter("max_dist_to_polygon", max_dist_to_polygon_);
    this->get_parameter("goal_topic", goal_topic);
    this->get_parameter("do_slam", do_slam_);
    this->get_parameter("do_record", do_record_);
    this->get_parameter("mavros_map_frame", mavros_map_frame_);
    this->get_parameter("base_frame", mavros_base_frame_);
    this->get_parameter("slam_map_frame", slam_map_frame_);
    this->get_parameter("path_planner_topic", path_planner_topic_);
    this->get_parameter("slam_pose_topic", slam_pose_topic_);
    this->get_parameter("costmap_topic", costmap_topic_);
    this->get_parameter("lidar_topic", lidar_topic_);
    this->get_parameter("attollo_topic", attollo_topic_);
    this->get_parameter("mapir_topic", mapir_topic_);
    this->get_parameter("mapir_rgb_topic", mapir_rgb_topic_);
    this->get_parameter("thermal_topic", thermal_topic_);
    this->get_parameter("rosbag_topic", rosbag_topic_);
    this->get_parameter("offline", offline_);
    this->get_parameter("save_pcd", save_pcd_);
    this->get_parameter("simulate", simulate_);
    this->get_parameter("data_directory", data_directory_);
    this->get_parameter("record_config_file", record_config_file_);
    this->get_parameter("explicit_global", explicit_global_params_);
    this->get_parameter("estimated_drone_speed", estimated_drone_speed_);
    estimated_drone_speed_ = estimated_drone_speed_ < 1 ? 1.0 : estimated_drone_speed_; // this protects against a later potential div by 0
    this->get_parameter("battery_failsafe_safety_factor", battery_failsafe_safety_factor_);
    this->get_parameter("do_mapir", do_mapir_);
    this->get_parameter("do_mapir_rgb", do_mapir_rgb_);
    this->get_parameter("do_attollo", do_attollo_);
    this->get_parameter("do_thermal_cam", do_thermal_);
    this->get_parameter("lidar_type", lidar_type);
    this->get_parameter("lidar_pitch", lidar_pitch_);
    this->get_parameter("lidar_x", lidar_x_);
    this->get_parameter("lidar_z", lidar_z_);
    this->get_parameter("flightleg_area_acres", flightleg_area_acres_);

    // Init agl altitude params
    max_agl_ = max_altitude_;
    min_agl_ = min_altitude_;
    target_agl_ = target_altitude_;
    
    // SLAM pose sub
    slam_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(slam_pose_topic_, 10, std::bind(&TaskManager::slamPoseCallback, this, _1));

    // Health pubs/subs
    health_pub_ = this->create_publisher<std_msgs::msg::String>("/mapversation/health_report", 10);
    if (do_slam_) {
        path_planner_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(path_planner_topic_, 10, std::bind(&TaskManager::pathPlannerCallback, this, _1));
        // Pointcloud republisher only if SLAM running
        pointcloud_repub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_map", 10);

        // TODO change this back to cloud_registered
        registered_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered_map", 10, std::bind(&TaskManager::registeredPclCallback, this, _1));
    }
    costmap_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(costmap_topic_, 10, std::bind(&TaskManager::costmapCallback, this, _1));
    if (lidar_type == 2) {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic_, 10, std::bind(&TaskManager::pointcloudCallback, this, _1));
    }
    else if (lidar_type == 4) {
        livox_lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar_topic_, 10, std::bind(&TaskManager::livoxCallback, this, _1));
    }
    if (do_mapir_) {
        mapir_sub_ = this->create_subscription<sensor_msgs::msg::Image>(mapir_topic_, 10, std::bind(&TaskManager::mapirCallback, this, _1));
    }
    else if (do_mapir_rgb_) {
        mapir_sub_ = this->create_subscription<sensor_msgs::msg::Image>(mapir_rgb_topic_, 10, std::bind(&TaskManager::mapirCallback, this, _1));
    }
    if (do_attollo_) {
        attollo_sub_ = this->create_subscription<sensor_msgs::msg::Image>(attollo_topic_, 10, std::bind(&TaskManager::attolloCallback, this, _1));
    }
    if (do_thermal_) {
        thermal_sub_ = this->create_subscription<sensor_msgs::msg::Image>(thermal_topic_, 10, std::bind(&TaskManager::thermalCallback, this, _1));
    }

    rosbag_sub_ = this->create_subscription<std_msgs::msg::String>(rosbag_topic_, 10, std::bind(&TaskManager::rosbagCallback, this, _1));

    // Explore action setup
    explore_action_client_ = rclcpp_action::create_client<Explore>(this, "explore");
    auto send_explore_goal_options = rclcpp_action::Client<Explore>::SendGoalOptions();
    send_explore_goal_options.goal_response_callback =
      std::bind(&TaskManager::explore_goal_response_callback, this, std::placeholders::_1);
    send_explore_goal_options.feedback_callback =
      std::bind(&TaskManager::explore_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_explore_goal_options.result_callback =
      std::bind(&TaskManager::explore_result_callback, this, std::placeholders::_1);

    // Geo/map state services
    geopoint_service_ = this->create_service<messages_88::srv::Geopoint>("slam2geo", std::bind(&TaskManager::convert2Geo, this, _1, std::placeholders::_2, std::placeholders::_3));
    elevation_map_service_ = this->create_service<messages_88::srv::GetMapData>("get_map_data", std::bind(&TaskManager::getMapData, this, _1, std::placeholders::_2, std::placeholders::_3));

    mavros_geofence_client_ = this->create_client<mavros_msgs::srv::WaypointPush>("/mavros/geofence/push");

    // If running slam, pass goals through path planner, which receives goal_topic and plans a path. 
    // Otherwise, send direct setpoint to mavros. 
    if (do_slam_)
        position_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
    else
        position_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", rclcpp::SensorDataQoS());

    local_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    vision_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lawnmower", 10);

    // Remote ID
    odid_basic_id_pub_ = this->create_publisher<mavros_msgs::msg::BasicID>("/mavros/open_drone_id/basic_id", 10);
    odid_operator_id_pub_ = this->create_publisher<mavros_msgs::msg::OperatorID>("/mavros/open_drone_id/operator_id", 10);
    odid_self_id_pub_ = this->create_publisher<mavros_msgs::msg::SelfID>("/mavros/open_drone_id/self_id", 10);
    odid_system_pub_ = this->create_publisher<mavros_msgs::msg::System>("/mavros/open_drone_id/system", 10);
    odid_system_update_pub_ = this->create_publisher<mavros_msgs::msg::SystemUpdate>("/mavros/open_drone_id/system_update", 10);

    // Recording
    if (offline_) {
        map_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>("map_yaw", 10, std::bind(&TaskManager::mapYawCallback, this, _1));
        if (save_pcd_) {
            pcl_save_ .reset(new pcl::PointCloud<pcl::PointXYZI>());
        }
    }
    else {
        map_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("map_yaw", 5);
    }

    // Task status pub
    task_pub_ = this->create_publisher<messages_88::msg::TaskStatus>("task_status", 10);
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(goal_topic, 10, std::bind(&TaskManager::goalCallback, this, _1));
    task_msg_.enable_autonomy = enable_autonomy_;
    task_msg_.enable_exploration = true;

    // tymbal pub/subs
    tymbal_hd_pub_ = this->create_publisher<std_msgs::msg::String>("/tymbal/to_hello_decco", 10);
    tymbal_sub_ = this->create_subscription<std_msgs::msg::String>("/tymbal/to_decco", 10, std::bind(&TaskManager::packageFromTymbal, this, _1));
    // Maybe the above topic should be called "from_hello_decco", feels a little more readable
    tymbal_puddle_pub_ = this->create_publisher<std_msgs::msg::String>("/tymbal/to_puddle", 10);
    map_region_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/map_region", 10);


    // TF
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    flight_controller_interface_->setAutonomyEnabled(enable_autonomy_);

    // Start timers
    health_check_timer_ = this->create_wall_timer(100ms, std::bind(&TaskManager::checkHealth, this));
    task_manager_timer_ = this->create_wall_timer(std::chrono::duration<float>(task_manager_loop_duration_), std::bind(&TaskManager::runTaskManager, this));
    heartbeat_timer_ = this->create_wall_timer(1s, std::bind(&TaskManager::heartbeatTimerCallback, this));
    odid_timer_ = this->create_wall_timer(1s, std::bind(&TaskManager::odidTimerCallback, this));

    // Initialize hello decco manager
    hello_decco_manager_ = std::make_shared<hello_decco_manager::HelloDeccoManager>(flightleg_area_acres_, mavros_map_frame_);

}

TaskManager::~TaskManager() {

    if (offline_ && save_pcd_) {
        if (pcl_save_->size() > 0) {
            std::string file_name = "utm.pcd";
            std::string task_manager_dir;
            std::string package_name = "task_manager";
            try {
                task_manager_dir = ament_index_cpp::get_package_share_directory("task_manager");
                RCLCPP_INFO(this->get_logger(), "Package '%s' found at: %s", package_name.c_str(), task_manager_dir.c_str());
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Could not find package '%s': %s", package_name.c_str(), e.what());
            }
            std::string pcd_save_dir = task_manager_dir + "/PCD/";

            if (!boost::filesystem::exists(pcd_save_dir)) {
                boost::filesystem::create_directory(pcd_save_dir);
            }
            pcd_save_dir += file_name;
            pcl::PCDWriter pcd_writer;
            RCLCPP_INFO(this->get_logger(), "current scan saved to /PCD/%s", file_name.c_str());
            pcd_writer.writeBinary(pcd_save_dir, *pcl_save_);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "No pointclouds to save");
        }
    }
}

void TaskManager::runTaskManager() {

    // Check arm status and make sure bag recording is happening properly.
    checkArmStatus();

    checkFailsafes();

    switch (current_task_) {
        case Task::INITIALIZING: {
            if (initialized()) {
                logEvent(EventType::STATE_MACHINE, Severity::LOW, "Drone initialized");
                if (!offline_) {
                    flight_controller_interface_->setMode(flight_controller_interface_->guided_mode_);

                    if (flight_controller_interface_->getDroneReadyToArm()){
                        logEvent(EventType::STATE_MACHINE, Severity::LOW, "Preflight checks passed, ready to arm");
                        updateCurrentTask(Task::READY);
                    }
                    else {
                        logEvent(EventType::STATE_MACHINE, Severity::LOW, "Waiting for preflight checks to pass");
                        updateCurrentTask(Task::PREFLIGHT_CHECK);
                    }
                }
                else {
                    updateCurrentTask(Task::READY);
                }
            }
            explore_action_result_ = rclcpp_action::ResultCode::UNKNOWN;
            break;
        }
        case Task::PREFLIGHT_CHECK: {
            // TODO sim should pass arming checks too, they don't for some weird reasons.
            // Like loop rate 222 (which can be fixed by putting ClockSpeed: 0.8 in settings.json)
            // However, other things still fail. Eventually we should fix this and set ARMING_CHECK to 1 in ardupilot
            if (flight_controller_interface_->getDroneReadyToArm() || simulate_) {
                logEvent(EventType::STATE_MACHINE, Severity::LOW, "Preflight checks passed, ready to arm");
                updateCurrentTask(Task::READY);
            } 
            else if (this->get_clock()->now() - last_preflight_check_log_stamp_ > rclcpp::Duration::from_seconds(10.0)) {
                logEvent(EventType::FLIGHT_CONTROL, Severity::MEDIUM,
                         "Preflight check failed due to " + flight_controller_interface_->getPreflightCheckReasons());
                last_preflight_check_log_stamp_ = this->get_clock()->now();
            }
            break;
        }
        case Task::READY: {
            // this flag gets triggered when received a burn unit
            if (needs_takeoff_) 
            {
                if (takeoff_attempts_ > 5) {
                    logEvent(EventType::STATE_MACHINE, Severity::MEDIUM, "Takeoff failed after 5 attempts");
                    needs_takeoff_ = false;
                    takeoff_attempts_ = 0;
                }
                else {
                    if (!enable_autonomy_) {
                        logEvent(EventType::STATE_MACHINE, Severity::MEDIUM, "Not taking off, autonomy not enabled");
                        needs_takeoff_ = false;
                    }
                    else {
                        logEvent(EventType::STATE_MACHINE, Severity::LOW, "Starting takeoff");
                        startTakeoff();
                    }
                    
                }
            }
            else if (flight_controller_interface_->getIsArmed()) {
                logEvent(EventType::STATE_MACHINE, Severity::LOW, "Drone armed manually");
                updateCurrentTask(Task::MANUAL_FLIGHT);
            }
            break;
        }
        case Task::MANUAL_FLIGHT: {
            if (!hello_decco_manager_->getElevationInit()) {
                hello_decco_manager_->getHomeElevation(home_elevation_);
                RCLCPP_INFO(this->get_logger(),"got home elevation in manual mode : %f", home_elevation_);
            }
            if (!flight_controller_interface_->getIsArmed()) {
                logEvent(EventType::STATE_MACHINE, Severity::LOW, "Drone disarmed manually");
                updateCurrentTask(Task::COMPLETE);
            }
            break;
        }
        case Task::PAUSE: {
            // Handle resuming exploration when that is available.
            break;
        }
        case Task::TAKING_OFF: {
            // Once we reach takeoff altitude, transition to next flight state
            if (flight_controller_interface_->getAltitudeAGL() > (target_altitude_ - 1)) {
                // If not in polygon, start navigation task
                if (!decco_utilities::isInside(map_polygon_, flight_controller_interface_->getCurrentLocalPosition().pose.position)) {
                    logEvent(EventType::STATE_MACHINE, Severity::LOW, "Transiting to designated survey unit");
                    startTransit();
                }
                else {
                    logEvent(EventType::STATE_MACHINE, Severity::LOW, "Starting exploration");
                    startExploration();
                }
            }
            break;
        }
        case Task::IN_TRANSIT: {
            if (decco_utilities::isInside(map_polygon_, flight_controller_interface_->getCurrentLocalPosition().pose.position)) {
                logEvent(EventType::STATE_MACHINE, Severity::LOW, "Starting exploration");
                startExploration();
            }
            break;
        }
        case Task::EXPLORING: {   
            // Check action client status to see if complete
            if (explore_action_result_ == rclcpp_action::ResultCode::ABORTED || 
                explore_action_result_ == rclcpp_action::ResultCode::CANCELED ||
                explore_action_result_ == rclcpp_action::ResultCode::SUCCEEDED) {

                logEvent(EventType::STATE_MACHINE, Severity::LOW, "Initiating RTL_88 due to exploration action stopped. "); // TODO add result parsing to string
                auto fs_msg = hello_decco_manager_->updateFlightStatus("COMPLETED", this->get_clock()->now());
                tymbal_hd_pub_->publish(fs_msg);
                startRtl88();
            }

            break;
        }
        case Task::LAWNMOWER: {
            if (lawnmower_started_ && lawnmower_points_.empty()) {
                logEvent(EventType::STATE_MACHINE, Severity::LOW, "Lawnmower complete, doing RTL_88");
                startRtl88();
            }
            else {
                if (lawnmower_started_ && !lawnmowerGoalComplete()) {
                    break;
                }
                else {
                    getLawnmowerGoal();
                    position_setpoint_pub_->publish(goal_);
                    lawnmower_started_ = true;
                }
            }
            break;
        }
        case Task::RTL_88: {
            bool at_home_position = decco_utilities::isInAcceptanceRadius(flight_controller_interface_->getCurrentLocalPosition().pose.position,
                                                                          home_pos_.pose.position,
                                                                          1.0);
            if (at_home_position) {
                logEvent(EventType::STATE_MACHINE, Severity::LOW, "RTL_88 completed, landing");
                startLanding();
            }
            break;
        }
        case Task::LANDING:
        case Task::FAILSAFE_LANDING: {
            if (!flight_controller_interface_->getIsArmed()) {
                logEvent(EventType::STATE_MACHINE, Severity::LOW, "Flight complete");
                updateCurrentTask(Task::COMPLETE);
            }
            break;
        }
        case Task::COMPLETE: {
            in_autonomous_flight_ = false;
            // Return to preflight check as we have already initialized. 
            updateCurrentTask(Task::PREFLIGHT_CHECK);
            break;
        }
        default: {
            break;
        }
    }

    task_msg_.header.stamp = this->get_clock()->now();
    task_msg_.cmd_history.data = cmd_history_.c_str();
    task_msg_.current_status.data = getTaskString(current_task_);
    task_pub_->publish(task_msg_);
    json task_json = makeTaskJson();
    auto ts_msg = hello_decco_manager_->packageToTymbalHD("task_status", task_json, this->get_clock()->now());
    tymbal_hd_pub_->publish(ts_msg);

    json path_json;
    path_json["timestamp"] = static_cast<float>(flight_controller_interface_->getCurrentGlobalPosition().header.stamp.sec
                                + static_cast<float>(flight_controller_interface_->getCurrentGlobalPosition().header.stamp.nanosec) / 1e9);
    path_json["latitude"] = flight_controller_interface_->getCurrentGlobalPosition().latitude;
    path_json["longitude"] = flight_controller_interface_->getCurrentGlobalPosition().longitude;
    // hello_decco_manager_->packageToTymbalPuddle("/flight", path_json);
}

void TaskManager::startTakeoff() {

    // Set home position
    home_pos_.header.stamp = this->get_clock()->now();
    home_pos_.header.frame_id = mavros_map_frame_;
    home_pos_.pose.position.x = flight_controller_interface_->getCurrentLocalPosition().pose.position.x;
    home_pos_.pose.position.y = flight_controller_interface_->getCurrentLocalPosition().pose.position.y;
    home_pos_.pose.position.z = target_altitude_;

    // this is a redundant check but probably good to keep
    if (!task_msg_.enable_autonomy) {
        logEvent(EventType::STATE_MACHINE, Severity::MEDIUM, "Not taking off, autonomy disabled");
        return;
    }

    if (flight_controller_interface_->takeOff()) {
        if (do_record_) {
            startBag();
        }
        in_autonomous_flight_ = true;
        updateCurrentTask(Task::TAKING_OFF);
        needs_takeoff_ = false;
        takeoff_attempts_ = 0; 
    } 
    else {
        logEvent(EventType::FLIGHT_CONTROL, Severity::MEDIUM, "Takeoff request failed. Retrying.");
        takeoff_attempts_++;
    }

}

void TaskManager::startTransit() {    
    padNavTarget(initial_transit_point_);

    initial_transit_point_.header.frame_id = mavros_map_frame_;
    initial_transit_point_.header.stamp = this->get_clock()->now();
    position_setpoint_pub_->publish(initial_transit_point_);

    updateCurrentTask(Task::IN_TRANSIT);
}

void TaskManager::startExploration() {
    if (survey_type_ == SurveyType::SUPER) {
        updateCurrentTask(Task::LAWNMOWER);
        return;
    }

    current_explore_goal_.polygon = map_polygon_;
    current_explore_goal_.altitude = target_altitude_;
    current_explore_goal_.min_altitude = min_altitude_;
    current_explore_goal_.max_altitude = max_altitude_;

    explore_action_client_->wait_for_action_server();

    // Start with a rotation command in case no frontiers immediately processed, will be overridden with first exploration goal
    geometry_msgs::msg::Twist vel;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = M_PI_2; // PI/2 rad/s
    local_vel_pub_->publish(vel);

    if (!explore_action_client_->wait_for_action_server()) {
        RCLCPP_WARN(this->get_logger(), "Explore action client not available after waiting");
    }
    explore_action_client_->async_send_goal(current_explore_goal_);

    logEvent(EventType::STATE_MACHINE, Severity::LOW, "Sending explore goal");
    auto fs_msg = hello_decco_manager_->updateFlightStatus("ACTIVE", this->get_clock()->now());
    tymbal_hd_pub_->publish(fs_msg);

    updateCurrentTask(Task::EXPLORING);
}

void TaskManager::startRtl88() {
    pauseOperations();

    if (flight_controller_interface_->getFlightMode() != flight_controller_interface_->guided_mode_) {
        flight_controller_interface_->setMode(flight_controller_interface_->guided_mode_);
    }

    position_setpoint_pub_->publish(home_pos_);
    updateCurrentTask(Task::RTL_88);
}

void TaskManager::startLanding() {
    flight_controller_interface_->setMode(flight_controller_interface_->land_mode_);
    updateCurrentTask(Task::LANDING);
}

void TaskManager::startFailsafeLanding() {
    pauseOperations();
    flight_controller_interface_->setMode(flight_controller_interface_->land_mode_);
    updateCurrentTask(Task::FAILSAFE_LANDING);
}

void TaskManager::startPause() {
    flight_controller_interface_->setMode(flight_controller_interface_->brake_mode_);
    updateCurrentTask(Task::PAUSE);
}

void TaskManager::updateCurrentTask(Task task) {
    std::string task_str = getTaskString(task);
    logEvent(EventType::TASK_STATUS, Severity::LOW, "Current task updated to " + task_str);
    current_task_ = task;
}

TaskManager::Task TaskManager::getCurrentTask() {
    return current_task_;
}

void TaskManager::checkHealth() {
    rclcpp::Time now = this->get_clock()->now();

    health_checks_.battery_ok = isBatteryOk();
    health_checks_.lidar_ok = now - last_lidar_stamp_ < lidar_timeout_;
    health_checks_.slam_ok = now - last_slam_pos_stamp_ < slam_timeout_;
    health_checks_.path_ok = now - last_path_planner_stamp_ < path_timeout_;
    health_checks_.costmap_ok = now - last_costmap_stamp_ < costmap_timeout_;
    health_checks_.explore_ok = explore_action_client_.get()->action_server_is_ready();
    health_checks_.mapir_ok = now - last_mapir_stamp_ < mapir_timeout_;
    health_checks_.attollo_ok = now - last_attollo_stamp_ < attollo_timeout_;
    health_checks_.thermal_ok = now - last_thermal_stamp_ < thermal_timeout_;
    health_checks_.rosbag_ok = now - last_rosbag_stamp_ < rosbag_timeout_;

    if (now - last_health_pub_stamp_ > health_check_pub_duration_) {
        publishHealth();
        last_health_pub_stamp_ = now;
    }
}

void TaskManager::checkFailsafes() {

    if (!in_autonomous_flight_)
        return;

    // Check for manual takeover
    std::string mode = flight_controller_interface_->getFlightMode();
    if (mode == "STABILIZE" || 
        mode == "ALT_HOLD" ||
        mode == "POSHOLD") {
        logEvent(EventType::FLIGHT_CONTROL, Severity::MEDIUM, "Manual takeover initiated");
        updateCurrentTask(Task::MANUAL_FLIGHT);
        in_autonomous_flight_ = false;
        pauseOperations();
        return;
    }

    // Check for failsafe landing conditions
    std::string failsafe_reason = "";
    bool need_failsafe_landing = false;
    if (!health_checks_.slam_ok && do_slam_) {
        failsafe_reason += "SLAM unhealthy, ";
        need_failsafe_landing = true;
    }
    if (!health_checks_.path_ok && do_slam_) {
        failsafe_reason += "Path unhealthy, ";
        need_failsafe_landing = true;
    }
    if (need_failsafe_landing) {
        if (current_task_ != Task::FAILSAFE_LANDING) {
            if (use_failsafes_) {
                logEvent(EventType::FAILSAFE, Severity::HIGH, "Failsafe landing initiated due to " + failsafe_reason);
                startFailsafeLanding();
            }
            else {
                if (current_task_ != Task::PAUSE) {
                    logEvent(EventType::FAILSAFE, Severity::MEDIUM, "Failsafe landing requested for "
                                                                    + failsafe_reason + " but failsafes not active");
                }
            }
        }
        // Return here because this is the most extreme failsafe and we don't need to check others
        return;
    }

    // Check for RTL 88 conditions
    // For now, only battery low will trigger this, but using this structure for future additions
    std::string rtl_88_reason = "";
    bool need_rtl_88 = false;
    if (!health_checks_.battery_ok) {
        rtl_88_reason += "battery level low, ";
        need_rtl_88 = true;
    } 
    if (need_rtl_88 &&
        current_task_ != Task::RTL_88 &&
        current_task_ != Task::LANDING &&
        current_task_ != Task::COMPLETE) {

        logEvent(EventType::STATE_MACHINE, Severity::MEDIUM, "Starting RTL 88 due to " + rtl_88_reason);
        startRtl88();
    }
}

bool TaskManager::initialized() {

    // Wait for FCI to finish initializing
    if (!flight_controller_interface_->getDroneInitalized())
        return false;

    // Get drone heading
    if (!offline_) {
        double yaw = 0;
        if (!flight_controller_interface_->getMapYaw(yaw)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Waiting for heading from autopilot...");
            return false;
        }

        logEvent(EventType::INFO, Severity::LOW, "Initial heading: " + std::to_string(yaw));

        // Convert yaw from NED to ENU
        yaw = -yaw + 90.0;
        // Convert to radians
        map_yaw_ = yaw * M_PI / 180.0;
        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = map_yaw_;
        map_yaw_pub_->publish(yaw_msg);
    }

    // Get roll, pitch for map stabilization
    geometry_msgs::msg::Quaternion init_orientation;
    logEvent(EventType::INFO, Severity::LOW, "Initializing IMU");
    if (!flight_controller_interface_->getAveragedOrientation(init_orientation)) {
        return false;
    }
    tf2::Quaternion quatmav(init_orientation.x, init_orientation.y, init_orientation.z, init_orientation.w);
    tf2::Matrix3x3 m(quatmav);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw, %f", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
    map_yaw_ = yaw;

    // If using tilted lidar, add the lidar pitch to the map to slam tf, since
    // the lidar is used as the basis for the slam map frame
    pitch += -lidar_pitch_;


    double offset_x = -lidar_x_;
    double offset_y = 0.0;
    double offset_z = -lidar_z_;

    // Fill in data
    map_to_slam_tf_.header.frame_id = mavros_map_frame_;
    map_to_slam_tf_.header.stamp = this->get_clock()->now();
    map_to_slam_tf_.child_frame_id = slam_map_frame_;
    
    // Rotate the lidar offsets in base_link frame to map_frame
    map_to_slam_tf_.transform.translation.x = offset_x * cos(-map_yaw_) + offset_y * sin(-map_yaw_); 
    map_to_slam_tf_.transform.translation.y = -offset_x * sin(-map_yaw_) + offset_y * cos(-map_yaw_); 
    map_to_slam_tf_.transform.translation.z = offset_z;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, map_yaw_); // TODO, get rid of map_yaw_? It's the same as IMU yaw. Also confirm IMU is ok and not being affected by param setup on launch
    quat_tf.normalize();

    geometry_msgs::msg::Quaternion quat;
    tf2::convert(quat_tf, quat);
    map_to_slam_tf_.transform.rotation = quat;

    // Save inverse
    tf2::Transform map2, slam2;
    tf2::convert(map_to_slam_tf_.transform, map2);
    slam2 = map2.inverse();
    geometry_msgs::msg::Transform slam2_tf;
    tf2::convert(slam2, slam2_tf);
    slam_to_map_tf_.header.frame_id = slam_map_frame_;
    slam_to_map_tf_.header.stamp = map_to_slam_tf_.header.stamp;
    slam_to_map_tf_.child_frame_id = mavros_map_frame_;
    slam_to_map_tf_.transform = slam2_tf;

    // Send transform and stop timer
    tf_static_broadcaster_->sendTransform(map_to_slam_tf_);

    map_tf_timer_.reset();
    map_tf_init_ = true;

    logEvent(EventType::INFO, Severity::LOW, "Waiting for global position");
    home_utm_zone_ = flight_controller_interface_->getUTMZone();
    if (home_utm_zone_ < 0) {
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Got global, UTM zone: %d. LL : (%f, %f)", home_utm_zone_, flight_controller_interface_->getCurrentGlobalPosition().latitude, flight_controller_interface_->getCurrentGlobalPosition().longitude);
    double utm_x, utm_y;
    flight_controller_interface_->initUTM(utm_x, utm_y);
    hello_decco_manager_->setUtm(utm_x, utm_y, home_utm_zone_);
    RCLCPP_INFO(this->get_logger(), "UTM offsets: (%f, %f)", utm_x, utm_y);
    RCLCPP_INFO(this->get_logger(), "Map yaw: %f", map_yaw_ * 180 / M_PI);
    utm2map_tf_.header.frame_id = "utm";
    utm2map_tf_.header.stamp = this->get_clock()->now();
    utm2map_tf_.child_frame_id = mavros_map_frame_;
    utm2map_tf_.transform.translation.x = utm_x;
    utm2map_tf_.transform.translation.y = utm_y;
    utm2map_tf_.transform.translation.z = 0;
    utm2map_tf_.transform.rotation.x = 0;
    utm2map_tf_.transform.rotation.y = 0;
    utm2map_tf_.transform.rotation.z = 0;
    utm2map_tf_.transform.rotation.w = 1;
    tf_static_broadcaster_->sendTransform(utm2map_tf_);
    utm_tf_init_ = true;

    logEvent(EventType::INFO, Severity::LOW, "Got global position, UTM zone, and map yaw");

    return true;
}

void TaskManager::map2UtmPoint(geometry_msgs::msg::PointStamped &in, geometry_msgs::msg::PointStamped &out) {
    in.header.frame_id = slam_map_frame_;
    in.header.stamp = this->get_clock()->now();
    tf_buffer_->transform(in, out, "utm");
}

bool TaskManager::convert2Geo(const std::shared_ptr<rmw_request_id_t>/*request_header*/,
                                const std::shared_ptr<messages_88::srv::Geopoint::Request> req,
                                const std::shared_ptr<messages_88::srv::Geopoint::Response> resp) {
    // Sanity check UTM
    if (home_utm_zone_ != flight_controller_interface_->getUTMZone()) {
        logEvent(EventType::INFO, Severity::LOW, "UTM zones crossed. Home UTM: " + std::to_string(home_utm_zone_) + 
                                                 "Now UTM: " + std::to_string(flight_controller_interface_->getUTMZone()));
        return false;
        // TODO decide what to do about it
    }
    geometry_msgs::msg::PointStamped in, out;
    in.point.x = req->slam_position.x;
    in.point.y = req->slam_position.y;
    in.point.z = req->slam_position.z;
    map2UtmPoint(in, out);
    resp->utm_position.x = out.point.x;
    resp->utm_position.y = out.point.y;
    double lat, lon;
    decco_utilities::utmToLL(out.point.x, out.point.y, home_utm_zone_, lat, lon);
    resp->latitude = lat;
    resp->longitude = lon;
    return true;
}

bool TaskManager::getMapData(const std::shared_ptr<rmw_request_id_t>/*request_header*/,
                             const std::shared_ptr<messages_88::srv::GetMapData::Request> req,
                             const std::shared_ptr<messages_88::srv::GetMapData::Response> resp){
    geometry_msgs::msg::Point map_point = req->map_position;
    int width = req->width;
    int height = req->height;
    geometry_msgs::msg::PointStamped in, out;
    in.point = map_point;
    map2UtmPoint(in, out);
    RCLCPP_INFO(this->get_logger(), "at intput (%f, %f) output utm was (%f, %f)", in.point.x, in.point.y, out.point.x, out.point.y);
    sensor_msgs::msg::Image chunk;
    double ret_altitude;
    bool worked = hello_decco_manager_->getElevationValue(out.point.x, out.point.y, ret_altitude);
    resp->success = worked;
    if (!worked) {
        logEvent(EventType::INFO, Severity::HIGH, "Failed to get elevation data! Cannot proceed in terrain.");
        return false;
    }
    resp->tif_mat = chunk;
    resp->ret_altitude = ret_altitude;

    RCLCPP_INFO(this->get_logger(), "Altitude at utm: %fm", ret_altitude);

    if (req->adjust_params) {
        altitude_offset_ = ret_altitude - home_elevation_;
        resp->home_offset = altitude_offset_;
        target_altitude_ = target_agl_  + altitude_offset_;
        resp->target_altitude = target_altitude_;
        RCLCPP_INFO(this->get_logger(),"setting new alt params with home elev %f, alt offset %f, and target alt: %f.", home_elevation_, altitude_offset_, target_altitude_);
        setAltitudeParams(max_agl_ + altitude_offset_, min_agl_ + altitude_offset_, target_altitude_);
    }
}

void TaskManager::heartbeatTimerCallback() {
    sensor_msgs::msg::NavSatFix hb = flight_controller_interface_->getCurrentGlobalPosition();
    geometry_msgs::msg::PoseStamped local = flight_controller_interface_->getCurrentLocalPosition();
    double altitudeAgl = flight_controller_interface_->getAltitudeAGL();
    geometry_msgs::msg::Quaternion quat_flu = local.pose.orientation;
    double yaw = flight_controller_interface_->getCompass();
    json j = {
        {"latitude", hb.latitude},
        {"longitude", hb.longitude},
        {"altitude", (int)altitudeAgl},
        {"heading", yaw},
        {"header", {
            {"frame_id", hb.header.frame_id},
            {"stamp", static_cast<float>(hb.header.stamp.sec) + static_cast<float>(hb.header.stamp.nanosec) / 1e9}
        }}
    };
    if (in_autonomous_flight_) {
        j["deccoId"] = hd_drone_id_;
        j["startTime"] = start_time_;
    }
    auto hb_msg = hello_decco_manager_->packageToTymbalHD("decco_heartbeat", j, this->get_clock()->now());
    tymbal_hd_pub_->publish(hb_msg);

    // rclcpp::Time now_time = this->get_clock()->now();
    // float interval = (now_time - last_ui_heartbeat_stamp_).seconds();
    // TODO needs also check if drone state is in air/action
    // if (interval > ui_hb_threshold_) {
    //     messages_88::srv::Emergency emergency;
    //     // TODO fill in pause msg
    //     // emergency_client_.call(emergency);
    // }
}

// Publish the ODID messages that require updates
void TaskManager::odidTimerCallback() {

    // Self ID
    mavros_msgs::msg::SelfID self_id;
    self_id.header.stamp = this->get_clock()->now();
    if (current_task_ == Task::FAILSAFE_LANDING) {
        self_id.description_type = mavros_msgs::msg::SelfID::MAV_ODID_DESC_TYPE_EMERGENCY;
        self_id.description = "FAILSAFE. CAUTION";
    }
    else {
        self_id.description_type = mavros_msgs::msg::SelfID::MAV_ODID_DESC_TYPE_TEXT;
        self_id.description = getTaskString(current_task_);
    }
    odid_self_id_pub_->publish(self_id);
}

void TaskManager::uiHeartbeatCallback(const json &msg) {
    last_ui_heartbeat_stamp_ = this->get_clock()->now();
}

bool TaskManager::pauseOperations() {

    // TODO figure out how to pause and restart
    if (explore_action_goal_handle_) {
        auto status = explore_action_goal_handle_->get_status();
        if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
            explore_action_client_->async_cancel_all_goals();
        }
    }

    return true;
}

void TaskManager::checkArmStatus() {
    std::string mode = flight_controller_interface_->getFlightMode();
    bool armed = flight_controller_interface_->getIsArmed();
    if (!is_armed_ && armed) {
        logEvent(EventType::INFO, Severity::LOW, "Manually set armed state to true");
        is_armed_ = true;
    }
    if (is_armed_ && !bag_active_ && do_record_) {
        logEvent(EventType::INFO, Severity::LOW, "Armed but bag not active"); 
        startBag();
    }
    if (is_armed_ && !armed) {
        logEvent(EventType::INFO, Severity::MEDIUM, "Disarm detected");
        stopBag();
        pauseOperations();
        is_armed_ = false; // Reset so can restart if another arming
    }
}

bool TaskManager::isBatteryOk() {
    geometry_msgs::msg::Point location = flight_controller_interface_->getCurrentLocalPosition().pose.position;
    double distance = sqrt(location.x * location.x + location.y * location.y);
    double time_to_home = distance / estimated_drone_speed_;
    double flight_time_remaining = flight_controller_interface_->getFlightTimeRemaining();

    return (flight_time_remaining > battery_failsafe_safety_factor_ * time_to_home);
}

void TaskManager::startBag() {
    if (bag_active_) {
        return;
    }
    logEvent(EventType::INFO, Severity::LOW, "Bag starting, dir: " + data_directory_);

    std::shared_ptr<rclcpp::Node> bag_record_node = rclcpp::Node::make_shared("bag_record_client");
    auto bag_recorder_client = bag_record_node->create_client<bag_recorder_2::srv::Record>("/bag_recorder/record");
    auto req = std::make_shared<bag_recorder_2::srv::Record::Request>();

    if (!bag_recorder_client->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Bag recorder not available. Not recording bag.");
        return;
    }

    req->start = true;
    req->config_file = record_config_file_;
    req->data_directory = data_directory_ + burn_unit_name_;

    auto result = bag_recorder_client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(bag_record_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (!result.get()->success) {
            RCLCPP_WARN(this->get_logger(), "Failed to start bag");
        }
        else {
            bag_active_ = true;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service /bag_recorder/record");
    }
}

void TaskManager::stopBag() {
    if (!bag_active_) {
        return;
    }
    std::shared_ptr<rclcpp::Node> bag_record_node = rclcpp::Node::make_shared("bag_record_client");
    auto bag_recorder_client = bag_record_node->create_client<bag_recorder_2::srv::Record>("/bag_recorder/record");
    auto req = std::make_shared<bag_recorder_2::srv::Record::Request>();

    if (!bag_recorder_client->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Bag recorder client not available. Not stop bag.");
        return;
    }

    req->start = false;

    auto result = bag_recorder_client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(bag_record_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (!result.get()->success) {
            RCLCPP_WARN(this->get_logger(), "Failed to stop bag");
        }
        else {
            bag_active_ = false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service /bag_recorder/record");
    }
}

bool TaskManager::polygonDistanceOk(geometry_msgs::msg::PoseStamped &target, geometry_msgs::msg::Polygon &map_region) {

    if (decco_utilities::isInside(map_polygon_, flight_controller_interface_->getCurrentLocalPosition().pose.position))
        return true;

    // Medium check, computes distance to nearest point on 2 most likely polygon edges
    // Polygon is already in map coordinates, i.e., expressed in meters from UAS home
    double min_dist = DBL_MAX;
    unsigned closest_point_ind = 0;
    for (unsigned ii=0; ii < map_region.points.size(); ii++) {
        double d = std::pow(map_region.points.at(ii).x, 2) + std::pow(map_region.points.at(ii).y, 2);
        if (d < min_dist) {
            min_dist = d;
            closest_point_ind = ii;
        }
    }
    // Find line intersection with each segment connecting to closest point
    geometry_msgs::msg::Point32 point1, point2, closest_point = map_region.points.at(closest_point_ind);
    unsigned ind1, ind2;
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

    // Compute intersections

    geometry_msgs::msg::Point drone_location_64 = flight_controller_interface_->getCurrentLocalPosition().pose.position;
    geometry_msgs::msg::Point32 drone_location;
    drone_location.x = drone_location_64.x;
    drone_location.y = drone_location_64.y;
    
    geometry_msgs::msg::Point32 intersection_point_1;
    bool intersection1 = decco_utilities::intersectsOrthogonal(closest_point, point1, drone_location, intersection_point_1);
    double dist1 = decco_utilities::distance_xy(drone_location, intersection_point_1);

    geometry_msgs::msg::Point32 intersection_point_2;
    bool intersection2 = decco_utilities::intersectsOrthogonal(closest_point, point2, drone_location, intersection_point_2);
    double dist2 = decco_utilities::distance_xy(drone_location, intersection_point_2);

    geometry_msgs::msg::Point target_position;
    if (intersection1 && intersection2) {

        if (dist1 < dist2) {
            min_dist = dist1;
            target_position.x = intersection_point_1.x;
            target_position.y = intersection_point_1.y;
        }
        else {
            min_dist = dist2;
            target_position.x = intersection_point_2.x;
            target_position.y = intersection_point_2.y;
        }
    }
    else if (intersection1) {
        min_dist = dist1;
        target_position.x = intersection_point_1.x;
        target_position.y = intersection_point_1.y;
    }
    else if (intersection2) {
        min_dist = dist2;
        target_position.x = intersection_point_2.x;
        target_position.y = intersection_point_2.y;
    }
    else {
        target_position.x = closest_point.x;
        target_position.y = closest_point.y;
    }
    target.pose.position = target_position;
    target.pose.position.z = target_altitude_;

    if (min_dist > std::pow(max_dist_to_polygon_, 2)) {
        logEvent(EventType::STATE_MACHINE, Severity::MEDIUM, 
                    "Max dist to polygon exceeded, will not execute flight. Max distance is: " + 
                    std::to_string(max_dist_to_polygon_) + "m, Closest point: " +
                    std::to_string(std::sqrt(min_dist)) + "m");
        return false;
    }
    return true;
}

void TaskManager::padNavTarget(geometry_msgs::msg::PoseStamped &target) {
    // Add 2m to ensure fully inside polygon, otherwise exploration won't start
    float padding = 2.0;
    geometry_msgs::msg::Point my_position = flight_controller_interface_->getCurrentLocalPosition().pose.position;
    double dif_x = target.pose.position.x - my_position.x;
    double dif_y = target.pose.position.y - my_position.y;
    double norm = sqrt(std::pow(dif_x, 2) + std::pow(dif_y, 2));
    double normed_dif_x = dif_x / norm;
    double normed_dif_y = dif_y / norm;
    target.pose.position.x += normed_dif_x * padding;
    target.pose.position.y += normed_dif_y * padding;
}

std::string TaskManager::getTaskString(Task task) {
    switch (task) {
        case Task::INITIALIZING:           return "INITIALIZING";
        case Task::PREFLIGHT_CHECK:        return "PREFLIGHT_CHECK";
        case Task::READY:                  return "READY";
        case Task::MANUAL_FLIGHT:          return "MANUAL_FLIGHT";
        case Task::PAUSE:                  return "PAUSE";
        case Task::EXPLORING:              return "EXPLORING";
        case Task::LAWNMOWER:              return "LAWNMOWER";
        case Task::IN_TRANSIT:             return "IN_TRANSIT";
        case Task::RTL_88:                 return "RTL_88";
        case Task::TAKING_OFF:             return "TAKING_OFF";
        case Task::LANDING:                return "LANDING";
        case Task::FAILSAFE_LANDING:       return "FAILSAFE_LANDING";
        case Task::COMPLETE:               return "COMPLETE";
        default:                           return "unknown";
    }
}

std::string TaskManager::getEventTypeString(EventType type) {
    switch (type) {
        case EventType::TASK_STATUS:    return "TASK_STATUS";
        case EventType::STATE_MACHINE:  return "STATE_MACHINE";
        case EventType::FLIGHT_CONTROL: return "FLIGHT_CONTROL";
        case EventType::FAILSAFE:       return "FAILSAFE";
        case EventType::INFO:           return "INFO";
        default:                        return "unknown";
    }
}

std::string TaskManager::getSeverityString(Severity sev) {
    switch (sev) {
        case Severity::LOW:      return "LOW";
        case Severity::MEDIUM:   return "MEDIUM";
        case Severity::HIGH:     return "HIGH";
        default:                 return "unknown";
    }
}

void TaskManager::slamPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr slam_pose) {

    slam_pose_ = *slam_pose;

    last_slam_pos_stamp_ = this->get_clock()->now();

    // Transform decco pose (in slam_map frame) and publish it in mavros_map frame as /mavros/vision_pose/pose
    if (!map_tf_init_) {
        return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(mavros_map_frame_, slam_map_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Cannot publish vision pose as map<>slam_map tf not yet available");
        return;
    }

    // Apply the transform to the drone pose
    geometry_msgs::msg::PoseStamped msg_body_pose;
    geometry_msgs::msg::PoseStamped slam = *slam_pose;

    tf2::doTransform(slam, msg_body_pose, tf);
    msg_body_pose.header.frame_id = mavros_map_frame_;
    msg_body_pose.header.stamp = slam_pose->header.stamp;

    vision_pose_publisher_->publish(msg_body_pose);

    // Map tf for offline
    if (explicit_global_params_) {
        geometry_msgs::msg::PointStamped point_in, point_out;
        point_in.header = slam_pose->header;
        point_in.point.x = 0;
        point_in.point.y = 0;
        point_in.point.z = 0;
        tf2::doTransform(point_in, point_out, utm2map_tf_);
        double lat, lon;
        decco_utilities::utmToLL(point_out.point.x, point_out.point.y, home_utm_zone_, lat, lon);
        sensor_msgs::msg::NavSatFix nav_msg;
        nav_msg.header.frame_id = mavros_base_frame_;
        nav_msg.header.stamp = slam_pose->header.stamp;
        nav_msg.latitude = lat;
        nav_msg.longitude = lon;
        global_pose_pub_->publish(nav_msg);

    }
}

void TaskManager::registeredPclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!map_tf_init_) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr reg_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg (*msg, *reg_cloud);

    pcl_ros::transformPointCloud(*reg_cloud, *map_cloud, map_to_slam_tf_);

    sensor_msgs::msg::PointCloud2 map_cloud_ros;
    pcl::toROSMsg(*map_cloud, map_cloud_ros);
    map_cloud_ros.header.frame_id = mavros_map_frame_;
    pointcloud_repub_->publish(map_cloud_ros);

    if (utm_tf_init_ && offline_ & save_pcd_) {
        RCLCPP_INFO(this->get_logger(), "Adding pcl to pcl_save");

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = map_cloud;
        // if (save_pcd_frame_ == "utm") {
            pcl::PointCloud<pcl::PointXYZI>::Ptr utm_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl_ros::transformPointCloud(*map_cloud, *utm_cloud, utm2map_tf_);
            cloud = utm_cloud;
        // }
        *pcl_save_ += *cloud;
    }
}

void TaskManager::pathPlannerCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_path_planner_stamp_ = this->get_clock()->now();
}

void TaskManager::costmapCallback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
    last_costmap_stamp_ = this->get_clock()->now();
}

void TaskManager::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_lidar_stamp_ =  this->get_clock()->now();
}

void TaskManager::livoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    last_lidar_stamp_ = this->get_clock()->now();
}

void TaskManager::mapirCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_mapir_stamp_ = this->get_clock()->now();
}

void TaskManager::attolloCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_attollo_stamp_ = this->get_clock()->now();
}

void TaskManager::thermalCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_thermal_stamp_ = this->get_clock()->now();
}

void TaskManager::rosbagCallback(const std_msgs::msg::String::SharedPtr msg) {
    last_rosbag_stamp_ = this->get_clock()->now();
}

void TaskManager::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_ = *msg;
}

void TaskManager::mapYawCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    map_yaw_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Got map yaw: %f", map_yaw_ * 180 / M_PI);
}

void TaskManager::explore_goal_response_callback(const ExploreGoalHandle::SharedPtr & goal_handle) {
    if (!goal_handle) {
      logEvent(INFO, MEDIUM, "Explore goal rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      explore_action_goal_handle_ = goal_handle;
    }
}

void TaskManager::explore_feedback_callback(ExploreGoalHandle::SharedPtr, const std::shared_ptr<const Explore::Feedback> feedback) {
    // TODO?
}

void TaskManager::explore_result_callback(const ExploreGoalHandle::WrappedResult & result) {
    explore_action_result_ = (rclcpp_action::ResultCode) result.code;
    switch (explore_action_result_) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        logEvent(INFO, LOW, "Goal succeeded");
        return;
      case rclcpp_action::ResultCode::ABORTED:
        logEvent(INFO, MEDIUM, "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        logEvent(INFO, MEDIUM, "Goal was canceled");
        return;
      default:
        logEvent(INFO, MEDIUM, "Unknown goal result code");
        return;
    }
}

void TaskManager::packageFromTymbal(const std_msgs::msg::String::SharedPtr msg) {
    json mapver_json = json::parse(msg->data);
    if (!mapver_json.contains("topic")) {
        RCLCPP_ERROR(this->get_logger(), "Received tymbal message without topic, ignoring.");
        logEvent(TASK_STATUS, HIGH, "Bad message sent to drone, ignoring.");
        return;
    }
    std::string topic = static_cast<std::string>(mapver_json["topic"]);
    json gossip_json = mapver_json["gossip"];
    if (topic == "flight_send") {
        acceptFlight(gossip_json);
    }
    else if (topic == "target_setpoint") {
        setpointResponse(gossip_json);
    }
    else if (topic == "emergency") {
        std::string severity = static_cast<std::string>(gossip_json["severity"]);
        emergencyResponse(severity);
    }
    else if (topic == "altitudes") {
        altitudesResponse(gossip_json);
    }
    else if (topic == "heartbeat") {
        remoteIDResponse(gossip_json);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Got a tymbal topic I don't understand.");
    }
}

void TaskManager::acceptFlight(json flight) {

    if (!map_tf_init_) {
        logEvent(EventType::STATE_MACHINE, Severity::MEDIUM, "Not ready for flight, try again after initialized");
        return;
    }
    burn_unit_name_ = static_cast<std::string>(flight["burnUnitName"]);
    hd_drone_id_ = flight["droneId"];
    start_time_ = decco_utilities::rosTimeToMilliseconds(this->get_clock()->now());

    hello_decco_manager_->setDroneLocationLocal(slam_pose_);
    geometry_msgs::msg::Polygon polygon;

    // Process flight via HDM
    rclcpp::Time timestamp = this->get_clock()->now();
    auto receipt = hello_decco_manager_->flightReceipt(flight, timestamp);
    tymbal_hd_pub_->publish(receipt);

    auto burn_unit_rcv = hello_decco_manager_->acceptFlight(flight, polygon, home_elevation_, timestamp);
    tymbal_hd_pub_->publish(burn_unit_rcv);

    RCLCPP_INFO(this->get_logger(), "Got home elevation : %f", home_elevation_);

    // Now publish polygon visualization
    auto vis = hello_decco_manager_->visualizePolygon(timestamp);
    map_region_pub_->publish(vis);

    auto req = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    if (!hello_decco_manager_->polygonToGeofence(polygon, req)) {
        auto result = mavros_geofence_client_->async_send_request(req);
        // TODO see if there is a clean way to handle result - I think I had issues with this last time I tried
    }
    else {
        logEvent(EventType::STATE_MACHINE, Severity::MEDIUM, "Geofence invalid, not setting geofence");
    }
    
    map_polygon_ = hello_decco_manager_->getMapPolygon();
    if (!polygonDistanceOk(initial_transit_point_, map_polygon_)) {
        logEvent(EventType::STATE_MACHINE, Severity::MEDIUM, "Polygon rejected, exceeds maximum starting distance threshold");
        return;
    }

    std::string type = static_cast<std::string>(flight["type"]);
    if (type == "PERI") {
        getLawnmowerPattern(map_polygon_, lawnmower_points_);
        visualizeLawnmower();
        survey_type_ = SurveyType::SUPER;
        RCLCPP_INFO(this->get_logger(),"lawnmower starting. ");
    }
    else {
        survey_type_ = SurveyType::SUB;
    }

    needs_takeoff_ = true;

}

// TODO where should this live?
void TaskManager::getLawnmowerPattern(const geometry_msgs::msg::Polygon &polygon, std::vector<geometry_msgs::msg::PoseStamped> &lawnmower_points) {
    std::vector<lawnmower::Point> polygon_points;
    for (int ii = 0; ii < polygon.points.size(); ii++) {
        lawnmower::Point pt;
        pt.x = polygon.points.at(ii).x;
        pt.y = polygon.points.at(ii).y;
        polygon_points.push_back(pt);
    }
    std::vector<lawnmower::Point> points = lawnmower::generateLawnmowerPattern(polygon_points, 10);
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = mavros_map_frame_;
    pose_stamped.header.stamp = this->get_clock()->now();

    // TODO outer loop for testing, remove
    for (int j = 0; j < 10; j++) {
        for (int ii = 0; ii < points.size(); ii++) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = points.at(ii).x;
            pose.position.y = points.at(ii).y;
            pose.position.z = target_altitude_;
            pose_stamped.pose = pose;
            lawnmower_points.push_back(pose_stamped);
        }
    }
    
}

// TODO remove this once confirmed
void TaskManager::visualizeLawnmower()
{
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 0.5;

  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = mavros_map_frame_;
  m.header.stamp = this->get_clock()->now();
  m.ns = "lawnmower";
  m.color = red;
  // lives forever
  m.lifetime = rclcpp::Duration(0.0, 0.0);
  m.frame_locked = true;

  m.action = visualization_msgs::msg::Marker::ADD;
  int id = 0;
  for (auto& frontier : lawnmower_points_) {
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = id;
    m.pose.position = frontier.pose.position;
    m.pose.orientation.w = 1.0;
    double scale = 1.5;
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = red;
    markers.push_back(m);
    ++id;
  }

  marker_pub_->publish(markers_msg);
}

void TaskManager::getLawnmowerGoal() {
    goal_ = lawnmower_points_.at(0);

    // Determine heading
    geometry_msgs::msg::PoseStamped current_pos = flight_controller_interface_->getCurrentLocalPosition();
    double x_diff = goal_.pose.position.x - current_pos.pose.position.x;
    double y_diff = goal_.pose.position.y - current_pos.pose.position.y;
    double yaw_target = atan2(y_diff, x_diff);
    tf2::Quaternion setpoint_q;
    setpoint_q.setRPY(0.0, 0.0, yaw_target);
    tf2::convert(setpoint_q, goal_.pose.orientation);
    
    lawnmower_points_.erase(lawnmower_points_.begin());
}

bool TaskManager::lawnmowerGoalComplete() {
    geometry_msgs::msg::PoseStamped current_pos = flight_controller_interface_->getCurrentLocalPosition();
    double dist_sq = std::pow(current_pos.pose.position.x - goal_.pose.position.x, 2) + std::pow(current_pos.pose.position.y - goal_.pose.position.y, 2);
    double min_dist = std::pow(3, 2); // within 2m
    return dist_sq < min_dist;
}

void TaskManager::setpointResponse(json &json_msg) {
    // ATM, this response is purely a testing function. 
    startBag();
}

void TaskManager::emergencyResponse(const std::string severity) {
    logEvent(EventType::FAILSAFE, Severity::HIGH, severity + " initiated due to pilot request");

    // Then respond based on severity 
    if (severity == "PAUSE") {
        // NOTICE = PAUSE
        // TODO, tell exploration to stop searching frontiers. For now, will keep blacklisting them, but the drone is in PAUSE mode. Currently no way to pick back up and set to guided mode (here or in HD)
        // Immediately set to hover
        startPause();
    }
    else if (severity == "LAND") {
        // EMERGENCY = LAND IMMEDIATELY
        startFailsafeLanding();
    }
    else if (severity == "RTL") {
        // CRITICAL = RTL
        startRtl88();
    }
}

void TaskManager::altitudesResponse(json &json_msg) {

    if (!json_msg["max_altitude"].is_number() || 
        !json_msg["min_altitude"].is_number() || 
        !json_msg["default_altitude"].is_number()) {
        RCLCPP_WARN(this->get_logger(), "Altitude message from mapversation contains invalid data");
        return;
    }

    max_agl_ = json_msg["max_altitude"];
    min_agl_ = json_msg["min_altitude"];
    target_agl_= json_msg["default_altitude"];

    // Set altitude params in all nodes that use them
    setAltitudeParams(max_agl_ + altitude_offset_, min_agl_ + altitude_offset_, target_agl_ + altitude_offset_);

    // TODO verify but I think can remove this. only sets values for this node anyways.
    this->set_parameter(rclcpp::Parameter("max_alt", max_agl_ + altitude_offset_));
    this->set_parameter(rclcpp::Parameter("min_alt", min_agl_ + altitude_offset_));
    this->set_parameter(rclcpp::Parameter("default_alt", target_agl_ + altitude_offset_));
}

void TaskManager::setAltitudeParams(const double max, const double min, const double target) {
    // Set altitude params in all nodes that use them
    auto parameter = rcl_interfaces::msg::Parameter();
    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

    auto client = this->create_client<rcl_interfaces::srv::SetParametersAtomically>("universal_altitude_params"); // E.g.: serviceName = "/turtlesim/set_parameters_atomically"

    parameter.name = "max_alt";  // E.g.: parameter_name = "background_b"
    parameter.value.type = 3;          //  bool = 1,    int = 2,        float = 3,     string = 4
    parameter.value.double_value = max; // .bool_value, .integer_value, .double_value, .string_value
    request->parameters.push_back(parameter);

    parameter.name = "min_alt";  // E.g.: parameter_name = "background_b"
    parameter.value.type = 3;          //  bool = 1,    int = 2,        float = 3,     string = 4
    parameter.value.double_value = min; // .bool_value, .integer_value, .double_value, .string_value
    request->parameters.push_back(parameter);

    parameter.name = "default_alt";  // E.g.: parameter_name = "background_b"
    parameter.value.type = 3;          //  bool = 1,    int = 2,        float = 3,     string = 4
    parameter.value.double_value = target; // .bool_value, .integer_value, .double_value, .string_value
    request->parameters.push_back(parameter);

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "universal altitude param service not available, waiting again..."); 
    }
    auto result = client->async_send_request(request);
}

void TaskManager::remoteIDResponse(json &json) {

    // Unpack JSON here for convenience
    int uas_id_str = json["uas_id"].is_number_integer() ? (int)json["uas_id"] : 0;
    int operator_id_str = json["operator_id"].is_number_integer() ? (int)json["operator_id"] : 0;
    float operator_latitude = json["operator_latitude"].is_number_float() ? (float)json["operator_latitude"] : 0.f;
    float operator_longitude = json["operator_longitude"].is_number_float() ? (float)json["operator_longitude"] : 0.f;
    float operator_altitude_geo = json["operator_altitude_geo"].is_number_float() ? (float)json["operator_altitude_geo"] : 0.f;
    int timestamp = json["timestamp"].is_number_integer() ? (int)json["timestamp"] : 0; 
    
    if (!init_remote_id_message_sent_) {
        // Basic ID
        mavros_msgs::msg::BasicID basic_id;
        basic_id.header.stamp = this->get_clock()->now();
        basic_id.id_type = mavros_msgs::msg::BasicID::MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID;
        basic_id.ua_type = mavros_msgs::msg::BasicID::MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR;
        basic_id.uas_id = std::to_string(uas_id_str);
        odid_basic_id_pub_->publish(basic_id);

        // Operator ID
        mavros_msgs::msg::OperatorID operator_id;
        operator_id.header.stamp = this->get_clock()->now();
        operator_id.operator_id_type = mavros_msgs::msg::OperatorID::MAV_ODID_OPERATOR_ID_TYPE_CAA;
        operator_id.operator_id = std::to_string(operator_id_str);
        odid_operator_id_pub_->publish(operator_id);
        operator_id_ = std::to_string(operator_id_str);

        // System
        // this should probably just be published at startup, and System Update published here
        mavros_msgs::msg::System system;
        system.header.stamp = this->get_clock()->now();
        system.operator_location_type = mavros_msgs::msg::System::MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF; // TODO dynamic operator location
        system.classification_type = mavros_msgs::msg::System::MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED;
        system.operator_latitude = operator_latitude * 1E7;
        system.operator_longitude = operator_longitude * 1E7;
        system.operator_altitude_geo = operator_altitude_geo;
        system.timestamp = timestamp;
        odid_system_pub_->publish(system);

        init_remote_id_message_sent_ = true;
    }

    // SystemUpdate
    mavros_msgs::msg::SystemUpdate system_update;
    system_update.header.stamp = this->get_clock()->now();
    system_update.operator_latitude = operator_latitude * 1E7;
    system_update.operator_longitude = operator_longitude * 1E7;
    system_update.operator_altitude_geo = operator_altitude_geo;
    if (timestamp > last_rid_updated_timestamp_) {
        system_update.timestamp = timestamp;
        last_rid_updated_timestamp_ = timestamp;
    }
    else {
        system_update.timestamp = this->get_clock()->now().seconds();
    }
    odid_system_update_pub_->publish(system_update);

}

void TaskManager::publishHealth() {

    auto jsonObjects = json::object();
    json header = {
        {"frame_id", "decco"},
        {"stamp", this->get_clock()->now().seconds()},
    };
    jsonObjects["header"] = header;

    auto healthObjects = json::array();

    json j = {
        {"name", "battery"},
        {"label", "Battery"},
        {"isHealthy", health_checks_.battery_ok}
    };
    healthObjects.push_back(j);
    j = {
        {"name", "lidar"},
        {"label", "LiDAR"},
        {"isHealthy", health_checks_.lidar_ok}
    };
    healthObjects.push_back(j);
    if (do_slam_) {
        j = {
            {"name", "slamPosition"},
            {"label", "SLAM position"},
            {"isHealthy", health_checks_.slam_ok}
        };
        healthObjects.push_back(j);
        j = {
            {"name", "pathPlanner"},
            {"label", "Path planner"},
            {"isHealthy", health_checks_.path_ok}
        };
        healthObjects.push_back(j);
        // costmap
        j = {
            {"name", "costmap"},
            {"label", "Costmap"},
            {"isHealthy", health_checks_.costmap_ok}
        };
        healthObjects.push_back(j);
        // 6) Explore
        j = {
            {"name", "explore"},
            {"label", "Explore Service"},
            {"isHealthy", health_checks_.explore_ok}
        };
        healthObjects.push_back(j);
    }
    // MAPIR
    if (do_mapir_ || do_mapir_rgb_) {
        j = {
            {"name", "mapir"},
            {"label", "MAPIR Camera"},
            {"isHealthy", health_checks_.mapir_ok}
        };
        healthObjects.push_back(j);
    }
    // Attollo
    if (do_attollo_) {
        j = {
            {"name", "attollo"},
            {"label", "Attollo Camera"},
            {"isHealthy", health_checks_.attollo_ok}
        };
        healthObjects.push_back(j);
    }
    // Thermal
    if (do_thermal_) {
        j = {
            {"name", "thermal"},
            {"label", "Thermal Camera"},
            {"isHealthy", health_checks_.thermal_ok}
        };
        healthObjects.push_back(j);
    }
    // ROS bag
    if (is_armed_ && !simulate_) {
        j = {
            {"name", "rosbag"},
            {"label", "ROS bag"},
            {"isHealthy", health_checks_.rosbag_ok}
        };
        healthObjects.push_back(j);
    }

    jsonObjects["healthIndicators"] = healthObjects;
    auto hr_msg = hello_decco_manager_->packageToTymbalHD("health_report", jsonObjects, this->get_clock()->now());
    tymbal_hd_pub_->publish(hr_msg);
}

void TaskManager::logEvent(EventType type, Severity sev, std::string description) {
    switch (sev) {
        case Severity::LOW:
        {
            RCLCPP_INFO(this->get_logger(), "%s", description.c_str());
            break;
        }
        case Severity::MEDIUM:
        {
            RCLCPP_WARN(this->get_logger(), "%s", description.c_str());
            break;
        }
        case Severity::HIGH:
        {
            RCLCPP_ERROR(this->get_logger(), "%s", description.c_str());
            break;
        }
    }

    cmd_history_.append(description + "\n");

    json j;
    j["flightId"] = 1; // TODO
    j["level"] = getSeverityString(sev);
    j["droneId"] = 1;
    j["timestamp"] = decco_utilities::rosTimeToMilliseconds(this->get_clock()->now());
    j["type"] = getEventTypeString(type);
    j["description"] = description.substr(0, 256); // Limit string size to 256

    auto event_msg = hello_decco_manager_->packageToTymbalHD("event", j, this->get_clock()->now());
    tymbal_hd_pub_->publish(event_msg);

    if (in_autonomous_flight_) {
        j["deccoId"] = hd_drone_id_;
        j["startTime"] = start_time_;
        sensor_msgs::msg::NavSatFix hb = flight_controller_interface_->getCurrentGlobalPosition();
        json ll_json = {
            {"latitude", hb.latitude},
            {"longitude", hb.longitude}
        };
        j["location"] = ll_json;
        auto puddle_msg = hello_decco_manager_->packageToTymbalPuddle("/flight-event", j);
        tymbal_puddle_pub_->publish(puddle_msg);
    }
}

json TaskManager::makeTaskJson() {
    json j;
    j["flightMode"] = flight_controller_interface_->getFlightMode();
    double xval = goal_.pose.position.x;
    double yval = goal_.pose.position.y;
    json goalArray;
    goalArray.push_back(xval);
    goalArray.push_back(yval);
    j["goal"] = goalArray;
    j["taskStatus"] = getTaskString(current_task_);
    j["minAltitude"] = min_altitude_;
    j["maxAltitude"] = max_altitude_;
    j["targetAltitude"] = target_altitude_;
    j["flightMinLeft"] = (int)(flight_controller_interface_->getFlightTimeRemaining() / 60.f);
    j["operatorID"] = operator_id_;
    j["rawVoltage"] = (int)(flight_controller_interface_->getBatteryVoltage() * 100);
    j["readyToArm"] = flight_controller_interface_->getDroneReadyToArm();
    j["isArmed"] = flight_controller_interface_->getIsArmed();
    return j;
}

}