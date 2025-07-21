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

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "task_manager/LawnmowerPattern.h"
#include "task_manager/decco_utilities.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
using json = nlohmann::json;

namespace task_manager {
TaskManager::TaskManager(
    std::shared_ptr<flight_controller_interface::FlightControllerInterface> fci)
    : Node("task_manager"),
      perception_modules_loaded_(false),
      has_lidar_(false),
      has_thermal_(false),
      has_camera_(false),
      current_task_(Task::INITIALIZING),
      flight_controller_interface_(fci),
      task_manager_loop_duration_(1.0),
      simulate_(false),
      offline_(false),
      utm_tf_init_(false),
      enable_autonomy_(false),
      use_failsafes_(false),
      do_trail_(false),
      target_altitude_(3.0),
      target_agl_(3.0),
      min_altitude_(2.0),
      min_agl_(2.0),
      max_altitude_(10.0),
      max_agl_(10.0),
      altitude_offset_(0.0),
      home_elevation_(0.0),
      max_dist_to_polygon_(300.0),
      flightleg_area_acres_(3.0),
      largest_distance_(0.0),
      angle_diff_count_(0),
      map_tf_init_(false),
      home_utm_zone_(-1),
      mavros_map_frame_("map"),
      slam_map_frame_("slam_map"),
      slam_pose_topic_("decco/pose"),
      do_record_(true),
      recording_(false),
      record_config_file_(""),
      burn_unit_name_(""),
      cmd_history_(""),
      lawnmower_started_(false),
      setpoint_started_(false),
      health_check_pub_duration_(rclcpp::Duration(5.0, 0)),
      path_planner_topic_("/path_planner/heartbeat"),
      lidar_topic_("/cloud_registered"),
      rosbag_topic_("/record/heartbeat"),
      initialized_(false),
      is_armed_(false),
      in_autonomous_flight_(false),
      has_setpoint_(false),
      perception_path_(""),
      explicit_global_params_(false),
      init_remote_id_message_sent_(false),
      takeoff_attempts_(0),
      estimated_drone_speed_(2.0),
      battery_failsafe_safety_factor_(2.0),
      do_slam_(false),
      lidar_pitch_(0.0),
      lidar_x_(0.0),
      lidar_z_(0.0),
      needs_takeoff_(false),
      last_rid_updated_timestamp_(0),
      operator_id_(""),
      hd_drone_id_(0),
      start_time_(0),
      last_path_planner_stamp_(0, 0, RCL_ROS_TIME),
      last_rosbag_stamp_(0, 0, RCL_ROS_TIME),
      last_health_pub_stamp_(0, 0, RCL_ROS_TIME),
      last_preflight_check_log_stamp_(0, 0, RCL_ROS_TIME),
      last_ui_heartbeat_stamp_(0, 0, RCL_ROS_TIME),
      lidar_timeout_(rclcpp::Duration::from_seconds(0.5)),
      vision_pose_timeout_(rclcpp::Duration::from_seconds(0.5)),
      path_timeout_(rclcpp::Duration::from_seconds(3.0)),
      rosbag_timeout_(rclcpp::Duration::from_seconds(1.0)),
      has_mission_(false),
      num_cameras_(0) {

    RCLCPP_INFO(this->get_logger(), "TM entered init");

    this->declare_parameter("enable_autonomy", enable_autonomy_);
    this->declare_parameter("use_failsafes", use_failsafes_);
    this->declare_parameter("default_alt", target_altitude_);
    this->declare_parameter("min_alt", min_altitude_);
    this->declare_parameter("max_alt", max_altitude_);
    this->declare_parameter("max_dist_to_polygon", max_dist_to_polygon_);
    this->declare_parameter("do_trail", do_trail_);

    std::string goal_topic = "/mavros/setpoint_position/local";
    this->declare_parameter("goal_topic", goal_topic);
    this->declare_parameter("do_slam", do_slam_);
    this->declare_parameter("do_record", do_record_);
    this->declare_parameter("mavros_map_frame", mavros_map_frame_);
    this->declare_parameter("base_frame", mavros_base_frame_);
    this->declare_parameter("slam_map_frame", slam_map_frame_);
    this->declare_parameter("path_planner_topic", path_planner_topic_);
    this->declare_parameter("slam_pose_topic", slam_pose_topic_);
    this->declare_parameter("lidar_topic", lidar_topic_);
    this->declare_parameter("rosbag_topic", rosbag_topic_);
    this->declare_parameter("offline", offline_);
    this->declare_parameter("simulate", simulate_);
    this->declare_parameter("data_directory", data_directory_);
    this->declare_parameter("record_config_file", record_config_file_);
    this->declare_parameter("explicit_global", explicit_global_params_);
    this->declare_parameter("estimated_drone_speed", estimated_drone_speed_);
    this->declare_parameter("battery_failsafe_safety_factor", battery_failsafe_safety_factor_);
    this->declare_parameter("lidar_pitch", lidar_pitch_);
    this->declare_parameter("lidar_x", lidar_x_);
    this->declare_parameter("lidar_z", lidar_z_);
    this->declare_parameter("flightleg_area_acres", flightleg_area_acres_);
    this->declare_parameter("perception_file", perception_path_);
    this->declare_parameter("num_cameras", num_cameras_);
    this->declare_parameter("has_lidar", has_lidar_);
    this->declare_parameter("has_thermal", has_thermal_);
    this->declare_parameter("has_camera", has_camera_);

    // Now get parameters
    this->get_parameter("enable_autonomy", enable_autonomy_);
    this->get_parameter("use_failsafes", use_failsafes_);
    this->get_parameter("default_alt", target_altitude_);
    this->get_parameter("min_alt", min_altitude_);
    this->get_parameter("max_alt", max_altitude_);
    this->get_parameter("max_dist_to_polygon", max_dist_to_polygon_);
    this->get_parameter("do_trail", do_trail_);
    this->get_parameter("goal_topic", goal_topic);
    this->get_parameter("do_slam", do_slam_);
    this->get_parameter("do_record", do_record_);
    this->get_parameter("mavros_map_frame", mavros_map_frame_);
    this->get_parameter("base_frame", mavros_base_frame_);
    this->get_parameter("slam_map_frame", slam_map_frame_);
    this->get_parameter("path_planner_topic", path_planner_topic_);
    this->get_parameter("slam_pose_topic", slam_pose_topic_);
    this->get_parameter("lidar_topic", lidar_topic_);
    this->get_parameter("rosbag_topic", rosbag_topic_);
    this->get_parameter("offline", offline_);
    this->get_parameter("simulate", simulate_);
    this->get_parameter("data_directory", data_directory_);
    this->get_parameter("record_config_file", record_config_file_);
    this->get_parameter("explicit_global", explicit_global_params_);
    this->get_parameter("estimated_drone_speed", estimated_drone_speed_);
    estimated_drone_speed_ =
        estimated_drone_speed_ < 1
            ? 1.0
            : estimated_drone_speed_; // this protects against a later potential div by 0
    this->get_parameter("battery_failsafe_safety_factor", battery_failsafe_safety_factor_);
    this->get_parameter("lidar_pitch", lidar_pitch_);
    this->get_parameter("lidar_x", lidar_x_);
    this->get_parameter("lidar_z", lidar_z_);
    this->get_parameter("flightleg_area_acres", flightleg_area_acres_);
    this->get_parameter("perception_file", perception_path_);
    this->get_parameter("num_cameras", num_cameras_);
    this->get_parameter("has_lidar", has_lidar_);
    this->get_parameter("has_thermal", has_thermal_);
    this->get_parameter("has_camera", has_camera_);

    // Init agl altitude params
    max_agl_ = max_altitude_;
    min_agl_ = min_altitude_;
    target_agl_ = target_altitude_;

    // Clicked point sub
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&TaskManager::clickedPointCallback, this, _1));

    // SLAM pose sub
    slam_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        slam_pose_topic_, 10, std::bind(&TaskManager::slamPoseCallback, this, _1));

    // Health pubs/subs
    health_pub_ = this->create_publisher<std_msgs::msg::String>("/mapversation/health_report", 10);
    rclcpp::QoS hb_qos(10);
    hb_qos.liveliness();
    if (do_slam_) {
        path_planner_sub_ = this->create_subscription<std_msgs::msg::Header>(
            path_planner_topic_, hb_qos, std::bind(&TaskManager::pathPlannerCallback, this, _1));
        // Pointcloud republisher only if SLAM running
        pointcloud_repub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_map", 10);
        registered_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10, std::bind(&TaskManager::registeredPclCallback, this, _1));
    }

    rosbag_sub_ = this->create_subscription<std_msgs::msg::String>(
        rosbag_topic_, 10, std::bind(&TaskManager::rosbagCallback, this, _1));

    // Geo/map state services
    geopoint_service_ = this->create_service<messages_88::srv::Geopoint>(
        "slam2geo", std::bind(&TaskManager::convert2Geo, this, _1, std::placeholders::_2,
                              std::placeholders::_3));
    elevation_map_service_ = this->create_service<messages_88::srv::GetMapData>(
        "get_map_data", std::bind(&TaskManager::getMapData, this, _1, std::placeholders::_2,
                                  std::placeholders::_3));

    mavros_geofence_client_ =
        this->create_client<mavros_msgs::srv::WaypointPush>("/mavros/geofence/push");

    goal_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

    local_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lawnmower", 10);

    // Remote ID
    odid_basic_id_pub_ =
        this->create_publisher<mavros_msgs::msg::OpenDroneIDBasicID>("/mavros/open_drone_id/basic_id", 10);
    odid_operator_id_pub_ = this->create_publisher<mavros_msgs::msg::OpenDroneIDOperatorID>(
        "/mavros/open_drone_id/operator_id", 10);
    odid_self_id_pub_ =
        this->create_publisher<mavros_msgs::msg::OpenDroneIDSelfID>("/mavros/open_drone_id/self_id", 10);
    odid_system_pub_ =
        this->create_publisher<mavros_msgs::msg::OpenDroneIDSystem>("/mavros/open_drone_id/system", 10);
    odid_system_update_pub_ = this->create_publisher<mavros_msgs::msg::OpenDroneIDSystemUpdate>(
        "/mavros/open_drone_id/system_update", 10);

    // Task status pub
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10, std::bind(&TaskManager::goalCallback, this, _1));
    task_msg_.enable_autonomy = enable_autonomy_;

    map_region_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/map_region", 10);

    // TF
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    flight_controller_interface_->setAutonomyEnabled(enable_autonomy_);

    // Start timers
    health_check_timer_ =
        this->create_wall_timer(100ms, std::bind(&TaskManager::checkHealth, this));
    task_manager_timer_ =
        this->create_wall_timer(std::chrono::duration<float>(task_manager_loop_duration_),
                                std::bind(&TaskManager::runTaskManager, this));
    odid_timer_ = this->create_wall_timer(1s, std::bind(&TaskManager::odidTimerCallback, this));

    // To be tested later
    // utm_tf_update_timer_ = this->create_wall_timer(1s, std::bind(&TaskManager::updateUTMTF,
    // this));

    // Initialize hello decco manager
    elevation_manager_ = std::make_shared<elevation_manager::ElevationManager>(
        flightleg_area_acres_, mavros_map_frame_);

    // Tif pubs for visualization
    tif_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/tif_grid", 10);
    tif_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/tif_pcl", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                        .transient_local());

    trigger_recording_pub_ =
        this->create_publisher<std_msgs::msg::String>("/trigger_recording", 10);
    
    // REST pub/sub
    rest_capabilities_pub_ =
        this->create_publisher<std_msgs::msg::String>("capabilities",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    rest_status_pub_ =
        this->create_publisher<std_msgs::msg::String>("rest_status", 10);

    rest_log_pub_ =
        this->create_publisher<std_msgs::msg::String>("rest_log", 10);

    rest_mission_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/frontend/run_mission", 10, std::bind(&TaskManager::missionCallback, this, _1));

    rest_toggle_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/frontend/toggle_module", 10, std::bind(&TaskManager::toggleCallback, this, _1));

    rest_emergency_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/frontend/emergency", 10, std::bind(&TaskManager::emergencyCallback, this, _1));

    rest_remote_id_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/frontend/remote_id", 10, std::bind(&TaskManager::remoteIDResponse, this, _1));

    path_manager_cancel_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/path_manager/cancel", 10);
}

TaskManager::~TaskManager() {}

void TaskManager::runTaskManager() {

    // Check arm status and make sure bag recording is happening properly.
    checkArmStatus();
    checkFailsafes();
    if (!initialized_) {
        loadPerceptionRegistry();
        checkMissions();
    }
    updateStatus();

    switch (current_task_) {
    case Task::INITIALIZING: {
        if (!initialized_) {
            initialize();
        } else {
            RCLCPP_INFO(this->get_logger(), "Drone initialized");
            if (!offline_) {
                flight_controller_interface_->setMode(flight_controller_interface_->guided_mode_);

                if (flight_controller_interface_->getDroneReadyToArm()) {
                    RCLCPP_INFO(this->get_logger(), "Preflight checks passed, ready to arm");
                    updateCurrentTask(Task::READY);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Waiting for preflight checks to pass");
                    updateCurrentTask(Task::PREFLIGHT_CHECK);
                }
            } else {
                updateCurrentTask(Task::READY);
            }
        }
        break;
    }
    case Task::PREFLIGHT_CHECK: {
        // TODO sim should pass arming checks too, they don't for some weird reasons.
        // Like loop rate 222 (which can be fixed by putting ClockSpeed: 0.8 in settings.json)
        // However, other things still fail. Eventually we should fix this and set ARMING_CHECK to 1
        // in ardupilot
        if (flight_controller_interface_->getDroneReadyToArm() || simulate_) {
            RCLCPP_INFO(this->get_logger(), "Preflight checks passed, ready to arm");
            updateCurrentTask(Task::READY);
        } else if (this->get_clock()->now() - last_preflight_check_log_stamp_ >
                   rclcpp::Duration::from_seconds(10.0)) {
            RCLCPP_INFO(this->get_logger(), "Preflight check failed due to %s",
                        flight_controller_interface_->getPreflightCheckReasons().c_str());
            last_preflight_check_log_stamp_ = this->get_clock()->now();
        }
        break;
    }
    case Task::READY: {
        // Return to preflight check state if not armed and drone isn't ready to arm
        if (!flight_controller_interface_->getIsArmed() &&
            !flight_controller_interface_->getDroneReadyToArm() && !simulate_) {
            updateCurrentTask(Task::PREFLIGHT_CHECK);
            break;
        }
        // this flag gets triggered when received a burn unit or setpoint
        if (needs_takeoff_) {
            if (takeoff_attempts_ > 5) {
                RCLCPP_WARN(this->get_logger(), "Takeoff failed after 5 attempts");
                needs_takeoff_ = false;
                takeoff_attempts_ = 0;
            } else {
                if (!enable_autonomy_) {
                    RCLCPP_WARN(this->get_logger(), "Not taking off, autonomy not enabled");
                    needs_takeoff_ = false;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Starting takeoff");
                    startTakeoff();
                }
            }
        }
        break;
    }
    case Task::MANUAL_FLIGHT: {
        // Allow for moving to setpoint from manual mode
        if (has_mission_) {
            startMission();
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
            if (has_mission_) {
                startMission();
            }
        }
        break;
    }
    case Task::MISSION: {
        // Handle mission tasks
        if (current_mission_.type == MissionType::LAWNMOWER) {
            // TODO generate lawnmower pattern from polygon
            if (lawnmower_started_ && lawnmower_points_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Lawnmower complete, doing RTL_88");
                startRtl88();
            } else {
                if (lawnmower_started_ && !lawnmowerGoalComplete()) {
                    break;
                } else {
                    bool got_goal = getLawnmowerGoal();
                    if (got_goal) {
                        goal_pos_pub_->publish(goal_);
                        lawnmower_started_ = true;
                    }
                }
            }
        } else if (current_mission_.type == MissionType::TRAIL_FOLLOW) {
            // TODO
        } else if (current_mission_.type == MissionType::SETPOINT) {
            if (!setpoint_started_) {
                goal_.pose.position = current_mission_.setpoint;
                goal_pos_pub_->publish(goal_);
                setpoint_started_ = true;
            } else {
                bool reached = decco_utilities::isInAcceptanceRadius(
                    flight_controller_interface_->getCurrentLocalPosition().pose.position,
                    goal_.pose.position, 3.0);
                if (reached) {
                    has_setpoint_ = false;
                    setpoint_started_ = false;
                    updateCurrentTask(Task::READY);
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No mission set, returning to READY state");
            updateCurrentTask(Task::READY);
        }
        break;
    }
    case Task::RTL_88: {
        bool at_home_position = decco_utilities::isInAcceptanceRadius(
            flight_controller_interface_->getCurrentLocalPosition().pose.position,
            home_pos_.pose.position, 1.0);
        if (at_home_position) {
            RCLCPP_INFO(this->get_logger(), "RTL_88 completed, landing");
            startLanding();
        }
        break;
    }
    case Task::LANDING:
    case Task::FAILSAFE_LANDING: {

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
}

void TaskManager::startTakeoff() {

    // Set home position
    home_pos_.header.stamp = this->get_clock()->now();
    home_pos_.header.frame_id = mavros_map_frame_;
    home_pos_.pose.position.x =
        flight_controller_interface_->getCurrentLocalPosition().pose.position.x;
    home_pos_.pose.position.y =
        flight_controller_interface_->getCurrentLocalPosition().pose.position.y;
    home_pos_.pose.position.z = target_altitude_;

    // this is a redundant check but probably good to keep
    if (!task_msg_.enable_autonomy) {
        RCLCPP_WARN(this->get_logger(), "Not taking off, autonomy disabled");
        return;
    }

    if (!offline_ && do_record_) {
        if (!recording_) {
            startRecording();
            rclcpp::sleep_for(
                1s); // Wait a second after before arming to get some extra data before takeoff
        } else
            RCLCPP_INFO(this->get_logger(),
                        "Recording flag already true, not starting new recording");
    }

    if (flight_controller_interface_->takeOff(target_altitude_)) {
        in_autonomous_flight_ = true;
        updateCurrentTask(Task::TAKING_OFF);
        needs_takeoff_ = false;
        takeoff_attempts_ = 0;
        is_armed_ = true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Takeoff request failed. Retrying.");
        takeoff_attempts_++;
    }
}

void TaskManager::startRtl88() {
    pauseOperations();

    if (flight_controller_interface_->getFlightMode() !=
        flight_controller_interface_->guided_mode_) {
        flight_controller_interface_->setMode(flight_controller_interface_->guided_mode_);
    }

    goal_pos_pub_->publish(home_pos_);
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
    RCLCPP_INFO(this->get_logger(), "Current task updated to %s", task_str.c_str());
    current_task_ = task;
}

TaskManager::Task TaskManager::getCurrentTask() {
    return current_task_;
}

void TaskManager::checkMissions()
  {
    if (!perception_modules_loaded_) {
        RCLCPP_WARN(this->get_logger(), "Perception modules not loaded yet, cannot check missions");
        return;
    }
    json capabilities_json;
    capabilities_json["missions"] = json::array();

    // Build a copy of perception_status_ into a simple json object at the end
    //    (for publishing)
    json perception_module_states = json::object();
    for (const auto &module : perception_modules_) {
        perception_module_states[module.second.module_name] = {
            {"active", module.second.is_active},
            {"togglable", module.second.togglable}
        };
    }

    // Hardware status
    std::map<std::string, bool> hardware_status = {
      {"lidar", has_lidar_},
      {"thermal", has_thermal_},
      {"camera", has_camera_}
    };


    // Now iterate over each mission JSON file in the “missions/” folder
    std::string pkg_path = ament_index_cpp::get_package_share_directory("task_manager");
    std::string missions_dir = pkg_path + "/missions";

    for (const auto &entry : std::filesystem::directory_iterator(missions_dir)) {
      if (entry.path().extension() != ".json") {
        continue;
      }
      std::ifstream file(entry.path());
      if (!file.is_open()) {
        continue;
      }

      json mission_json;
      try {
        file >> mission_json;
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to parse mission file: %s", entry.path().c_str());
        continue;
      }

      // 4a) Extract mission name
      std::string name = mission_json["mission"]["name"].get<std::string>();

      if (mission_json["requirements"]["slam"].get<bool>() && !do_slam_) {
        RCLCPP_WARN(this->get_logger(), "Skipping mission %s, requires SLAM mode", name.c_str());
        continue; // Skip missions that require SLAM if not in SLAM mode
      }

      // 4b) Gather required/optional perception modules from mission definition
      std::set<std::string> required_perception;
      std::set<std::string> optional_perception;
      
      std::string geometry_type = "";
      if (mission_json["requirements"].contains("input_geometry")) {
        geometry_type = mission_json["requirements"]["input_geometry"].get<std::string>();
      }

      if (mission_json["requirements"].contains("perception")
          && mission_json["requirements"]["perception"].contains("required"))
      {
        for (const auto &mod : mission_json["requirements"]["perception"]["required"]) {
          required_perception.insert(mod.get<std::string>());
        }
      }
      if (mission_json["requirements"].contains("perception")
          && mission_json["requirements"]["perception"].contains("optional"))
      {
        for (const auto &mod : mission_json["requirements"]["perception"]["optional"]) {
          optional_perception.insert(mod.get<std::string>());
        }
      }

    // 4c) Determine hardware needed by “required” perception
    std::set<std::string> all_required_hardware;
    for (const auto &mod : required_perception) {
        auto it = perception_modules_.find(mod);
        if (it != perception_modules_.end()) {
            for (const auto &hw : it->second.hardware) {
                all_required_hardware.insert(hw);
            }
        }
    }

    // 4d) If an optional module is actually “on”, it also brings its hardware
    for (const auto &mod : optional_perception) {
        auto it = perception_modules_.find(mod);
        if (it != perception_modules_.end() && it->second.is_active) {
            for (const auto &hw : it->second.hardware) {
                all_required_hardware.insert(hw);
            }
        }
    }

      // 4e) TODO this is meaningless rn, change it to checking topics
      bool hardware_ok = true;
      for (const auto &hw : all_required_hardware) {
        if (!hardware_status.count(hw) || !hardware_status[hw]) {
          hardware_ok = false;
          break;
        }
      }
      if (!hardware_ok) {
        // skip this mission entirely if required hardware is missing
        continue;
      }

      // 4f) Check perception “on/off” constraints
      bool mission_available = true;
      std::vector<std::string> unmet;

      // Any required perception that is not “on” makes mission unavailable
      for (const auto &mod : required_perception) {
        auto it = perception_modules_.find(mod);
        if (it == perception_modules_.end() || !it->second.is_active) {
            mission_available = false;
            unmet.push_back(mod);
        }
      }
      // Any optional perception that is off just goes into “requires_activation”
      for (const auto &mod : optional_perception) {
        auto it = perception_modules_.find(mod);
        if (it == perception_modules_.end() || !it->second.is_active) {
            unmet.push_back(mod);
        }
      }

      // 4g) Build JSON entry for this mission
      json mission_entry;
      mission_entry["name"] = name;
      mission_entry["available"] = mission_available;
      mission_entry["geometry_type"] = geometry_type;
      if (!unmet.empty()) {
        mission_entry["requires_activation"] = unmet;
      }

      capabilities_json["missions"].push_back(mission_entry);
    }

    // 5) Finally, embed the perception module states and publish
    capabilities_json["perception_modules"] = perception_module_states;

    auto msg = std_msgs::msg::String();
    msg.data = capabilities_json.dump();
    rest_capabilities_pub_->publish(msg);
}

void TaskManager::loadPerceptionRegistry() {
    if (perception_modules_loaded_) {
        return;
    }
    std::ifstream f(perception_path_);
    std::cout << "Loading perception registry from: " << perception_path_ << std::endl;
    if (!f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open perception registry file: %s",
                     perception_path_.c_str());
        return;
    }
    json perception_registry = json::parse(f);
    std::cout << "Perception registry loaded successfully: " << perception_registry.dump(2) << std::endl;

    // For each module in perception_registry, look up its node name and hardware requirements
    for (auto& [module_name, info] : perception_registry.items()) {
        PerceptionModule module;
        module.module_name = module_name;
        module.node_name = info["node"].get<std::string>();

        bool skip_module = false;
        for (const auto& hw : info["hardware_required"]) {
            std::string hw_str = hw.get<std::string>();
            std::cout << "Hardware required for module " << module_name << ": " << hw_str << std::endl;

            if (hw_str == "lidar" && has_lidar_) {
                module.hardware.push_back(hw_str);
            } else if (hw_str == "thermal" && has_thermal_) {
                module.hardware.push_back(hw_str);
            } else if (hw_str == "camera" && has_camera_) {
                module.hardware.push_back(hw_str);
            } else {
                RCLCPP_WARN(this->get_logger(), "Perception module %s requires unsupported hardware: %s",
                            module_name.c_str(), hw_str.c_str());
                skip_module = true;
            }
        }
        if (skip_module) {
            RCLCPP_WARN(this->get_logger(), "Skipping perception module %s due to unsupported hardware",
                        module_name.c_str());
            continue;
        }

        if (do_slam_ && (module.module_name == "obstacle_avoidance")) {
            module.togglable = false;
            module.is_active = true; // These modules are always active in SLAM mode and cannot be turned off
        }
        else {
            module.togglable = true;
            module.is_active = false;
        }
        perception_modules_[module_name] = module;

        // Setup parameter monitoring for this module
        std::string param_name = "/task_manager/" + module.node_name + "/set_node_active";
        this->declare_parameter(param_name, module.is_active);
        RCLCPP_INFO(this->get_logger(), "Declared parameter %s", param_name.c_str());
    }
    perception_modules_loaded_ = true;
}

void TaskManager::updateStatus() {
    json status_json;
    // Update task message
    std::string task_string = getTaskString(current_task_);
    status_json["status"] = task_string;

    status_json["distance"] = flight_controller_interface_->getHomeDistance();

    status_json["speed"] = flight_controller_interface_->getGroundSpeed();

    status_json["num_cameras"] = num_cameras_;

    // Publish status
    std_msgs::msg::String status_msg;
    status_msg.data = status_json.dump();
    rest_status_pub_->publish(status_msg);
}

void TaskManager::publishLog(LogLevel level, const std::string &message) {
    json log_json;
    std::string level_string;
    switch (level) {
        case LogLevel::NORMAL: level_string = "NORMAL"; break;
        case LogLevel::INFO: level_string = "INFO"; break;
        case LogLevel::WARN: level_string = "WARN"; break;
        case LogLevel::ERROR: level_string = "ERROR"; break;
    }
    log_json["level"] = level_string;
    log_json["message"] = "DRONE::" + message;
    std_msgs::msg::String log_msg;
    log_msg.data = log_json.dump();
    rest_log_pub_->publish(log_msg);
}

void TaskManager::checkHealth() {
    rclcpp::Time now = this->get_clock()->now();

    health_checks_.battery_ok = isBatteryOk();
    health_checks_.slam_ok =
        now - flight_controller_interface_->getLastVisionPosePubStamp() < vision_pose_timeout_;
    health_checks_.path_ok = now - last_path_planner_stamp_ < path_timeout_;
    health_checks_.rosbag_ok = now - last_rosbag_stamp_ < rosbag_timeout_;

    // What to do with this?
}

void TaskManager::checkFailsafes() {

    if (!in_autonomous_flight_)
        return;

    // Check for manual takeover
    std::string mode = flight_controller_interface_->getFlightMode();
    if (mode == "STABILIZE" || mode == "ALT_HOLD" || mode == "POSHOLD") {
        RCLCPP_WARN(this->get_logger(), "Manual takeover initiated");
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
                RCLCPP_ERROR(this->get_logger(), "Failsafe landing initiated due to %s",
                             failsafe_reason.c_str());
                startFailsafeLanding();
            } else {
                if (current_task_ != Task::PAUSE) {
                    RCLCPP_INFO(this->get_logger(),
                                "Failsafe landing requested for %s but failsafes not active",
                                failsafe_reason.c_str());
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
    if (need_rtl_88 && current_task_ != Task::RTL_88 && current_task_ != Task::LANDING &&
        current_task_ != Task::COMPLETE) {

        RCLCPP_INFO(this->get_logger(), "Starting RTL 88 due to %s", rtl_88_reason.c_str());
        startRtl88();
    }
}

void TaskManager::initialize() {

    // Wait for FCI to finish initializing
    if (!flight_controller_interface_->getDroneInitalized())
        return;

    // Get roll, pitch for map stabilization
    RCLCPP_INFO(this->get_logger(), "Getting initial IMU");
    geometry_msgs::msg::Quaternion init_orientation =
        flight_controller_interface_->getInitOrientation();

    // Check for valid init orientation
    if (init_orientation.x == 0 && init_orientation.y == 0 && init_orientation.z == 0 &&
        init_orientation.w == 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get initial orientation");
        return;
    }

    tf2::Quaternion quatmav(init_orientation.x, init_orientation.y, init_orientation.z,
                            init_orientation.w);
    tf2::Matrix3x3 m(quatmav);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "Initial drone RPY (ENU): [%f, %f, %f]", roll * 180 / M_PI,
                pitch * 180 / M_PI, yaw * 180 / M_PI);
    double heading = 90.0 - yaw * 180 / M_PI;
    if (heading < 0) {
        heading += 360;
    }
    RCLCPP_INFO(this->get_logger(), "Initial drone heading: %f", heading);

    // If using tilted lidar, add the lidar pitch to the map to slam tf, since
    // the lidar is used as the basis for the slam map frame
    // Offsets are lidar IMU offsets relative to base_link, since lidar IMU is basis for slam_map
    pitch += lidar_pitch_;

    double offset_x = lidar_x_;
    double offset_y = 0.0;
    double offset_z = lidar_z_;

    // Fill in data
    map_to_slam_tf_.header.frame_id = mavros_map_frame_;
    map_to_slam_tf_.header.stamp = this->get_clock()->now();
    map_to_slam_tf_.child_frame_id = slam_map_frame_;

    // Rotate the lidar offsets in base_link frame to map_frame
    map_to_slam_tf_.transform.translation.x = offset_x * cos(-yaw) + offset_y * sin(-yaw);
    map_to_slam_tf_.transform.translation.y = -offset_x * sin(-yaw) + offset_y * cos(-yaw);
    map_to_slam_tf_.transform.translation.z = offset_z;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    quat_tf.normalize();

    geometry_msgs::msg::Quaternion quat;
    tf2::convert(quat_tf, quat);
    map_to_slam_tf_.transform.rotation = quat;

    // Send transform and stop timer
    tf_static_broadcaster_->sendTransform(map_to_slam_tf_);

    map_tf_init_ = true;
    flight_controller_interface_->setMapSlamTf(map_to_slam_tf_);

    RCLCPP_INFO(this->get_logger(), "Waiting for global position");
    home_utm_zone_ = flight_controller_interface_->getUTMZone();
    if (home_utm_zone_ < 0) {
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Got global, UTM zone: %d. LL : (%f, %f)", home_utm_zone_,
                flight_controller_interface_->getCurrentGlobalPosition().latitude,
                flight_controller_interface_->getCurrentGlobalPosition().longitude);
    flight_controller_interface_->initUTM(home_utm_x_, home_utm_y_);
    elevation_manager_->setUtm(home_utm_x_, home_utm_y_, home_utm_zone_);
    RCLCPP_INFO(this->get_logger(), "UTM offsets: (%f, %f)", home_utm_x_, home_utm_y_);
    utm2map_tf_.header.frame_id = "utm";
    utm2map_tf_.header.stamp = this->get_clock()->now();
    utm2map_tf_.child_frame_id = mavros_map_frame_;
    utm2map_tf_.transform.translation.x = home_utm_x_;
    utm2map_tf_.transform.translation.y = home_utm_y_;
    utm2map_tf_.transform.translation.z = 0;
    utm2map_tf_.transform.rotation.x = 0;
    utm2map_tf_.transform.rotation.y = 0;
    utm2map_tf_.transform.rotation.z = 0;
    utm2map_tf_.transform.rotation.w = 1;
    tf_static_broadcaster_->sendTransform(utm2map_tf_);
    utm_tf_init_ = true;

    initialized_ = true;

    if (offline_) {
        if (elevation_manager_->getHomeElevation(home_elevation_)) {
            RCLCPP_INFO(this->get_logger(), "Got home elevation : %f", home_elevation_);
            publishTif();
        } else {
            RCLCPP_WARN(this->get_logger(), "No elevation, can only perform manual flight.");
        }
    }

    return;
}

void TaskManager::map2UtmPoint(geometry_msgs::msg::PointStamped &in,
                               geometry_msgs::msg::PointStamped &out) {
    in.header.frame_id = mavros_map_frame_;
    in.header.stamp = this->get_clock()->now();
    tf_buffer_->transform(in, out, "utm");
}

bool TaskManager::convert2Geo(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                              const std::shared_ptr<messages_88::srv::Geopoint::Request> req,
                              const std::shared_ptr<messages_88::srv::Geopoint::Response> resp) {
    // Sanity check UTM
    if (home_utm_zone_ != flight_controller_interface_->getUTMZone()) {
        RCLCPP_INFO(this->get_logger(), "UTM zones crossed. Home UTM: %s, Now UTM:, %s",
                    std::to_string(home_utm_zone_).c_str(),
                    std::to_string(flight_controller_interface_->getUTMZone()).c_str());
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
    resp->home_altitude = home_elevation_;
    return true;
}

void TaskManager::updateUTMTF() {
    if (!utm_tf_init_) {
        return;
    }

    auto ll = flight_controller_interface_->getCurrentGlobalPosition();

    double px, py;
    decco_utilities::llToUtm(ll.latitude, ll.longitude, home_utm_zone_, px, py);

    double dx = px - home_utm_x_;
    double dy = py - home_utm_y_;

    // Only correct map<>utm if we are far enough away from the origin, due to limited gps accuracy
    double distance = sqrt(dx * dx + dy * dy);
    if (distance < 10.0)
        return;

    double utm_angle = atan2(dy, dx);

    auto pos = flight_controller_interface_->getCurrentLocalPosition();
    double map_angle = atan2(pos.pose.position.y, pos.pose.position.x);

    double angle_diff = map_angle - utm_angle;

    // Add to average angle diff
    avg_angle_diff_ = (avg_angle_diff_ * angle_diff_count_ + angle_diff) / (angle_diff_count_ + 1);
    angle_diff_count_++;

    // For testing when needed
    RCLCPP_INFO(this->get_logger(), "UTM: [%f, %f]", dx, dy);
    RCLCPP_INFO(this->get_logger(), "Map: [%f, %f]", pos.pose.position.x, pos.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Map angle: %f, UTM angle: %f", map_angle * 180.0 / M_PI,
                utm_angle * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "Angle diff: %f", angle_diff * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "Avg angle diff: %f", avg_angle_diff_ * 180.0 / M_PI);

    // Convert to quaternion and send TF
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, avg_angle_diff_);
    quat_tf.normalize();

    geometry_msgs::msg::Quaternion quat;
    tf2::convert(quat_tf, quat);

    utm2map_tf_.transform.rotation = quat;

    utm2map_tf_.header.stamp = this->get_clock()->now();
    tf_static_broadcaster_->sendTransform(utm2map_tf_);
}

bool TaskManager::getElevationAtPoint(geometry_msgs::msg::PointStamped &point, double &elevation) {
    geometry_msgs::msg::PointStamped point_utm;
    map2UtmPoint(point, point_utm);
    bool worked =
        elevation_manager_->getElevationValue(point_utm.point.x, point_utm.point.y, elevation);
    if (!worked) {
        RCLCPP_INFO(this->get_logger(), "Failed to get elevation at map point [%f, %f]",
                    point.point.x, point.point.y);
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Altitude at [%f, %f]: %fm", point.point.x, point.point.y,
                elevation);

    return true;
}

bool TaskManager::getMapData(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                             const std::shared_ptr<messages_88::srv::GetMapData::Request> req,
                             const std::shared_ptr<messages_88::srv::GetMapData::Response> resp) {
    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped.point = req->map_position;
    double ret_altitude;
    if (!getElevationAtPoint(point_stamped, ret_altitude)) {
        RCLCPP_WARN(this->get_logger(),
                    "Elevation at requested point not found, not adjusting for terrain");
        resp->success = false;
        return false;
    }
    sensor_msgs::msg::Image chunk;
    resp->tif_mat = chunk;
    resp->ret_altitude = ret_altitude - home_elevation_;

    if (req->adjust_params) {
        // Get alt at start
        geometry_msgs::msg::PoseStamped local =
            flight_controller_interface_->getCurrentLocalPosition();
        geometry_msgs::msg::PointStamped in, out;
        in.point.x = local.pose.position.x;
        in.point.y = local.pose.position.y;
        map2UtmPoint(in, out);
        double my_altitude;
        if (!elevation_manager_->getElevationValue(out.point.x, out.point.y, my_altitude)) {
            RCLCPP_WARN(this->get_logger(),
                        "Elevation at current position not found, not adjusting for terrain");
            resp->success = false;
            return false;
        }
        double my_offset = my_altitude - home_elevation_;
        // Update internal
        altitude_offset_ = ret_altitude - home_elevation_;
        target_altitude_ = target_agl_ + altitude_offset_;
        min_altitude_ = std::min(min_agl_ + altitude_offset_, min_agl_ + my_offset);
        max_altitude_ = std::max(max_agl_ + altitude_offset_, max_agl_ + my_offset);
        // Send service
        resp->home_offset = altitude_offset_;
        resp->target_altitude = target_altitude_;
        resp->min_altitude = min_altitude_;
        resp->max_altitude = max_altitude_;

        this->set_parameter(rclcpp::Parameter("max_alt", max_altitude_));
        this->set_parameter(rclcpp::Parameter("min_alt", min_altitude_));
        this->set_parameter(rclcpp::Parameter("default_alt", target_altitude_));
    }

    resp->success = true;

    return true;
}

void TaskManager::publishTif() {
    auto tif_grid = elevation_manager_->getTifGrid();
    auto cloud = elevation_manager_->getTifCloud();
    tif_grid_pub_->publish(tif_grid);
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = tif_grid.header.frame_id;
    cloud_msg.header.stamp = tif_grid.header.stamp;
    tif_pcl_pub_->publish(cloud_msg);
}

void TaskManager::remoteIDResponse(std_msgs::msg::String::SharedPtr msg) {
    json json = json::parse(msg->data);

    // Unpack JSON here for convenience
    int uas_id_str = json["uas_id"].is_number_integer() ? (int)json["uas_id"] : 0;
    int operator_id_str = json["operator_id"].is_number_integer() ? (int)json["operator_id"] : 0;
    float operator_latitude =
        json["operator_latitude"].is_number_float() ? (float)json["operator_latitude"] : 0.f;
    float operator_longitude =
        json["operator_longitude"].is_number_float() ? (float)json["operator_longitude"] : 0.f;
    float operator_altitude_geo = json["operator_altitude_geo"].is_number_float()
                                      ? (float)json["operator_altitude_geo"]
                                      : 0.f;
    int timestamp = json["timestamp"].is_number_integer() ? (int)json["timestamp"] : 0;

    if (!init_remote_id_message_sent_) {
        // Basic ID
        mavros_msgs::msg::OpenDroneIDBasicID basic_id;
        basic_id.header.stamp = this->get_clock()->now();
        basic_id.id_type = mavros_msgs::msg::OpenDroneIDBasicID::ID_TYPE_CAA_REGISTRATION_ID;
        basic_id.ua_type = mavros_msgs::msg::OpenDroneIDBasicID::UA_TYPE_HELICOPTER_OR_MULTIROTOR;
        basic_id.uas_id = std::to_string(uas_id_str);
        odid_basic_id_pub_->publish(basic_id);

        // Operator ID
        mavros_msgs::msg::OpenDroneIDOperatorID operator_id;
        operator_id.header.stamp = this->get_clock()->now();
        operator_id.operator_id_type = mavros_msgs::msg::OpenDroneIDOperatorID::ID_TYPE_CAA;
        operator_id.operator_id = std::to_string(operator_id_str);
        odid_operator_id_pub_->publish(operator_id);
        operator_id_ = std::to_string(operator_id_str);

        // System
        // this should probably just be published at startup, and System Update published here
        mavros_msgs::msg::OpenDroneIDSystem system;
        system.header.stamp = this->get_clock()->now();
        system.operator_location_type =
            mavros_msgs::msg::OpenDroneIDSystem::LOCATION_TYPE_TAKEOFF; // TODO dynamic operator location
        system.classification_type =
            mavros_msgs::msg::OpenDroneIDSystem::CLASSIFICATION_TYPE_UNDECLARED;
        system.operator_latitude = operator_latitude * 1E7;
        system.operator_longitude = operator_longitude * 1E7;
        system.operator_altitude_geo = operator_altitude_geo;
        odid_system_pub_->publish(system);

        init_remote_id_message_sent_ = true;
    }

    // SystemUpdate
    mavros_msgs::msg::OpenDroneIDSystemUpdate system_update;
    system_update.header.stamp = this->get_clock()->now();
    system_update.operator_latitude = operator_latitude * 1E7;
    system_update.operator_longitude = operator_longitude * 1E7;
    system_update.operator_altitude_geo = operator_altitude_geo;
    if (timestamp > last_rid_updated_timestamp_) {
        last_rid_updated_timestamp_ = timestamp;
    }
    odid_system_update_pub_->publish(system_update);
}

// Publish the ODID messages that require updates
void TaskManager::odidTimerCallback() {

    // Self ID
    mavros_msgs::msg::OpenDroneIDSelfID self_id;
    self_id.header.stamp = this->get_clock()->now();
    self_id.description_type = mavros_msgs::msg::OpenDroneIDSelfID::DESC_TYPE_TEXT;
    self_id.description = "FLIGHT";
    odid_self_id_pub_->publish(self_id);
}

bool TaskManager::pauseOperations() {
    if (current_task_ == Task::PAUSE) {
        RCLCPP_WARN(this->get_logger(), "Already in PAUSE task, not pausing again");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Pausing operations");
    path_manager_cancel_pub_->publish(home_pos_);
    if (current_task_ != Task::MANUAL_FLIGHT) {
        startPause();
    }
    else {
        updateCurrentTask(Task::PAUSE);
    }

    // TODO decide how to save state for restart
    return true;
}

void TaskManager::checkArmStatus() {
    bool armed = flight_controller_interface_->getIsArmed();
    if (!is_armed_ && armed) {
        RCLCPP_INFO(this->get_logger(), "Drone armed manually");
        is_armed_ = true;

        // Only get arm status in offline mode, rest is not needed
        if (offline_)
            return;

        updateCurrentTask(Task::MANUAL_FLIGHT);
        if (elevation_manager_->getHomeElevation(home_elevation_)) {
            RCLCPP_INFO(this->get_logger(), "Got home elevation : %f", home_elevation_);
            publishTif();
        } else {
            RCLCPP_WARN(this->get_logger(), "No elevation, can only perform manual flight.");
        }

        if (do_record_) {
            if (!recording_)
                startRecording();
            else
                RCLCPP_INFO(this->get_logger(),
                            "Recording flag already true, not starting new recording");
        }
    }
    if (is_armed_ && !armed) {
        RCLCPP_INFO(this->get_logger(), "Disarm detected");
        updateCurrentTask(Task::COMPLETE);
        if (recording_) {
            rclcpp::sleep_for(2s); // Wait a bit to make sure we have all messages in bag through
                                   // full end of flight
            stopRecording();
        }
        is_armed_ = false; // Reset so can restart if another arming
    }
}

bool TaskManager::isBatteryOk() {
    geometry_msgs::msg::Point location =
        flight_controller_interface_->getCurrentLocalPosition().pose.position;
    double distance = sqrt(location.x * location.x + location.y * location.y);
    double time_to_home = distance / estimated_drone_speed_;
    double flight_time_remaining = flight_controller_interface_->getFlightTimeRemaining();

    return (flight_time_remaining > battery_failsafe_safety_factor_ * time_to_home);
}

void TaskManager::startRecording() {

    std::string start_time = decco_utilities::get_time_str();
    std::string flight_directory = data_directory_ + burn_unit_name_ + "/flight_" + start_time;
    if (!boost::filesystem::exists(flight_directory)) {
        boost::filesystem::create_directories(flight_directory);
    }
    RCLCPP_INFO(this->get_logger(), "Recording starting, dir: %s", flight_directory.c_str());

    // Handle bag
    std::shared_ptr<rclcpp::Node> bag_record_node = rclcpp::Node::make_shared("bag_record_client");
    auto bag_recorder_client =
        bag_record_node->create_client<bag_recorder_2::srv::Record>("/bag_recorder/record");

    if (!bag_recorder_client->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Bag recorder not available. Not recording bag.");
    } else {
        auto req = std::make_shared<bag_recorder_2::srv::Record::Request>();
        req->start = true;
        req->config_file = record_config_file_;
        req->data_directory = flight_directory;

        auto result = bag_recorder_client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(bag_record_node, result, 1s) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if (!result.get()->success) {
                RCLCPP_WARN(this->get_logger(), "Failed to start bag");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service /bag_recorder/record");
        }
    }

    auto msg = std_msgs::msg::String();
    msg.data = flight_directory;
    trigger_recording_pub_->publish(msg);

    recording_ = true;
}

void TaskManager::stopRecording() {

    auto msg = std_msgs::msg::String();
    msg.data = "";
    trigger_recording_pub_->publish(msg);

    // Deal with rosbag
    std::shared_ptr<rclcpp::Node> bag_record_node = rclcpp::Node::make_shared("bag_record_client");
    auto bag_recorder_client =
        bag_record_node->create_client<bag_recorder_2::srv::Record>("/bag_recorder/record");

    if (!bag_recorder_client->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Bag recorder client not available. Not stopping bag.");
    } else {
        auto req = std::make_shared<bag_recorder_2::srv::Record::Request>();
        req->start = false;

        auto result = bag_recorder_client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(bag_record_node, result, 1s) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if (!result.get()->success) {
                RCLCPP_WARN(this->get_logger(), "Failed to stop bag");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service /bag_recorder/record");
        }
    }

    recording_ = false;
}

std::string TaskManager::getTaskString(Task task) {
    switch (task) {
    case Task::INITIALIZING:
        return "INITIALIZING";
    case Task::PREFLIGHT_CHECK:
        return "PREFLIGHT_CHECK";
    case Task::READY:
        return "READY";
    case Task::MANUAL_FLIGHT:
        return "MANUAL_FLIGHT";
    case Task::PAUSE:
        return "PAUSE";
    case Task::MISSION:
        return "MISSION";
    case Task::RTL_88:
        return "RTL_88";
    case Task::TAKING_OFF:
        return "TAKING_OFF";
    case Task::LANDING:
        return "LANDING";
    case Task::FAILSAFE_LANDING:
        return "FAILSAFE_LANDING";
    case Task::COMPLETE:
        return "COMPLETE";
    default:
        return "unknown";
    }
}

TaskManager::MissionType TaskManager::getMissionType(std::string mission_type) {
    if (mission_type == "TRAIL_FOLLOW") {
        return MissionType::TRAIL_FOLLOW;
    } else if (mission_type == "LAWNMOWER") {
        return MissionType::LAWNMOWER;
    } else if (mission_type == "SETPOINT") {
        return MissionType::SETPOINT;
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown mission type: %s", mission_type.c_str());
        return MissionType::NONE;
    }
}

void TaskManager::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    double elevation;
    getElevationAtPoint(*msg, elevation);
}

void TaskManager::slamPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr slam_pose) {

    slam_pose_ = *slam_pose;

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
    pcl::fromROSMsg(*msg, *reg_cloud);

    pcl_ros::transformPointCloud(*reg_cloud, *map_cloud, map_to_slam_tf_);

    sensor_msgs::msg::PointCloud2 map_cloud_ros;
    pcl::toROSMsg(*map_cloud, map_cloud_ros);
    map_cloud_ros.header.frame_id = mavros_map_frame_;
    pointcloud_repub_->publish(map_cloud_ros);
}

void TaskManager::pathPlannerCallback(const std_msgs::msg::Header::SharedPtr msg) {
    last_path_planner_stamp_ = this->get_clock()->now();
}

void TaskManager::rosbagCallback(const std_msgs::msg::String::SharedPtr msg) {
    last_rosbag_stamp_ = this->get_clock()->now();
}

void TaskManager::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_ = *msg;
}

void TaskManager::missionCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received mission: %s", msg->data.c_str());
    publishLog(LogLevel::INFO, "Received mission: " + json::parse(msg->data).dump(2));
    if (current_task_ == Task::MANUAL_FLIGHT || current_task_ == Task::MISSION ){
        json mission_json = json::parse(msg->data);
        RCLCPP_INFO(this->get_logger(), "Accepting new mission");
        publishLog(LogLevel::INFO, "Accepting new mission: " + msg->data);
        parseMission(mission_json);
        startMission();
    }
    else if (current_task_ == Task::READY) {
        json mission_json = json::parse(msg->data);
        RCLCPP_INFO(this->get_logger(), "Preparing for mission");
        parseMission(mission_json);
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Cannot switch to mission from current task: %s",
                    getTaskString(current_task_).c_str());
        publishLog(LogLevel::WARN, "Cannot switch to mission from current task: " +
                   getTaskString(current_task_));
        return;
    }
}

void TaskManager::toggleCallback(const std_msgs::msg::String::SharedPtr msg) {
    json toggle_json = json::parse(msg->data);
    if (!toggle_json.contains("module_name") || !toggle_json.contains("active")) {
        RCLCPP_ERROR(this->get_logger(), "Invalid toggle message format received from frontend");
        return;
    }

    std::string module_name = toggle_json["module_name"];
    if (perception_modules_.find(module_name) == perception_modules_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Unknown perception module: %s. Not toggling.", module_name.c_str());
        return;
    }
    bool active = toggle_json["active"];

    RCLCPP_INFO(this->get_logger(), "Toggling local param for module '%s' %s",
                module_name.c_str(), active ? "ON" : "OFF");

    std::string param_name = "/task_manager/" + perception_modules_[module_name].node_name + "/set_node_active";

    try {
        rclcpp::Parameter param(param_name, active);
        this->set_parameter(param);
        perception_modules_[module_name].is_active = active;
        checkMissions();
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "Parameter '%s' not declared. Make sure it exists.", param_name.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set param: %s", e.what());
    }
}

void TaskManager::emergencyCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received emergency message: %s", msg->data.c_str());
    publishLog(LogLevel::ERROR, "Received emergency message: " + msg->data);
    switch (msg->data[0])  // Assuming first character indicates the emergency type
    {
    case 'e':  // Emergency stop
        RCLCPP_ERROR(this->get_logger(), "Emergency stop requested");
        pauseOperations();
        startFailsafeLanding();
        break;
    case 'p':  // Emergency pause
        RCLCPP_INFO(this->get_logger(), "Emergency pause requested");
        if (current_task_ != Task::PAUSE) {
            pauseOperations();
        } else {
            RCLCPP_WARN(this->get_logger(), "Already in PAUSE task, ignoring request");
        }
        break;
    case 'r':  // Emergency rtl
        RCLCPP_INFO(this->get_logger(), "Emergency RTL requested");
        if (current_task_ != Task::RTL_88 && current_task_ != Task::LANDING &&
            current_task_ != Task::COMPLETE) {
            startRtl88();
        } else {
            RCLCPP_WARN(this->get_logger(), "Already in RTL task, ignoring request");
        }
        break;
    
    default:
        break;
    }
}

bool TaskManager::parseMission(json mission_json) {
    if (!mission_json.contains("type")) {
        RCLCPP_ERROR(this->get_logger(), "Mission JSON missing 'mission.type'");
        return false;
    }

    std::string type = mission_json["type"];
    std::string file_name = type;
    std::transform(file_name.begin(), file_name.end(), file_name.begin(), ::tolower);

    // Load full mission definition from disk
    std::string pkg_path = ament_index_cpp::get_package_share_directory("task_manager");
    std::string mission_path = pkg_path + "/missions/" + file_name + ".json";

    std::ifstream file(mission_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open mission file: %s", mission_path.c_str());
        return false;
    }

    json mission_def;
    try {
        file >> mission_def;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse mission file: %s", e.what());
        return false;
    }

    std::string geometry_type;
    if (mission_def.contains("requirements") && mission_def["requirements"].contains("input_geometry")) {
        geometry_type = mission_def["requirements"]["input_geometry"].get<std::string>();
    }

    Mission mission;
    mission.completed = false;

    // Geometry validation
    if (geometry_type == "polygon") {
        if (!mission_json.contains("polygon") || !mission_json["polygon"].is_array()) {
            RCLCPP_ERROR(this->get_logger(), "Mission '%s' requires a 'geometry' array", type.c_str());
            return false;
        }

        for (const auto &pt : mission_json["polygon"]) {
            if (!pt.contains("lat") || !pt.contains("lon")) {
                RCLCPP_ERROR(this->get_logger(), "Each polygon point must have 'lat' and 'lon'");
                return false;
            }
            geometry_msgs::msg::Point32 p;
            double x,y;
            elevation_manager_->llToMap(pt["lat"], pt["lon"], x, y);
            p.x = x;
            p.y = y;
            p.z = target_altitude_;  // default altitude
            mission.polygon.points.push_back(p);
        }

    } else if (geometry_type == "point") {
        if (!mission_json.contains("setpoint")) {
            RCLCPP_ERROR(this->get_logger(), "Mission '%s' requires a 'setpoint'", type.c_str());
            return false;
        }
        const auto& sp = mission_json["setpoint"];
        if (!sp.contains("lat") || !sp.contains("lon")) {
            RCLCPP_ERROR(this->get_logger(), "Setpoint must include 'lat' and 'lon'");
            return false;
        }
        double x,y;
        elevation_manager_->llToMap(sp["lat"], sp["lon"], x, y);
        mission.setpoint.x = x;
        mission.setpoint.y = y;
        mission.setpoint.z = target_altitude_;  // default altitude
    } else if (!geometry_type.empty()) {
        RCLCPP_WARN(this->get_logger(), "Unhandled geometry type: '%s'", geometry_type.c_str());
        return false;
    }

    if (mission_json.contains("dem")) {
        std::string dem_file = mission_json["dem"];
        std::string dem_path = std::string(std::getenv("HOME")) + "/r88_public/dems/" + dem_file;
        if (!std::filesystem::exists(dem_path)) {
            RCLCPP_ERROR(this->get_logger(), "Could not find DEM file: %s", dem_path.c_str());
            return false;
        }
        bool elev = elevation_manager_->elevationInitializer(dem_path);
        if (!elev) {
            RCLCPP_ERROR(this->get_logger(), "Found but could not load DEM file: %s", dem_path.c_str());
        }
    }
    else {
        RCLCPP_WARN(this->get_logger(), "No DEM file in json, proceeding without elevation correction: %s", mission_json.dump(2).c_str());
    }

    // Accept mission
    mission.type = getMissionType(type);
    current_mission_ = mission;
    has_mission_ = true;
    if (!is_armed_) {
        needs_takeoff_ = true;
    }
    return true;
}

void TaskManager::startMission() {
    switch (current_mission_.type) {
        case MissionType::TRAIL_FOLLOW:
            updateCurrentTask(Task::MISSION);
            current_mission_.type = MissionType::TRAIL_FOLLOW;
            break;
        case MissionType::LAWNMOWER:
            updateCurrentTask(Task::MISSION);
            current_mission_.type = MissionType::LAWNMOWER;
            break;
        case MissionType::SETPOINT:
            updateCurrentTask(Task::MISSION);
            current_mission_.type = MissionType::SETPOINT;
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown mission type in startMission");
    }
}

// TODO where should this live?
void TaskManager::startTrailFollowing(bool start) {
    // Start trail goal enabled in pcl analysis
    auto parameter = rcl_interfaces::msg::Parameter();
    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

    auto client = this->create_client<rcl_interfaces::srv::SetParametersAtomically>(
        "trail_enabled_service"); // E.g.: serviceName = "/turtlesim/set_parameters_atomically"

    parameter.name = "trails_enabled";  // E.g.: parameter_name = "background_b"
    parameter.value.type = 1;           //  bool = 1,    int = 2,        float = 3,     string = 4
    parameter.value.bool_value = start; // .bool_value, .integer_value, .double_value, .string_value
    request->parameters.push_back(parameter);

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "universal altitude param service not available, waiting again...");
    }
    auto result = client->async_send_request(request);
}

bool TaskManager::getLawnmowerPattern(
    const geometry_msgs::msg::Polygon &polygon,
    std::vector<geometry_msgs::msg::PoseStamped> &lawnmower_points) {
    std::vector<lawnmower::Point> polygon_points;
    for (int ii = 0; ii < polygon.points.size(); ii++) {
        lawnmower::Point pt;
        pt.x = polygon.points.at(ii).x;
        pt.y = polygon.points.at(ii).y;
        polygon_points.push_back(pt);
    }
    std::vector<lawnmower::Point> points = lawnmower::generateLawnmowerPattern(polygon_points, 10);
    if (points.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate lawnmower pattern");
        return false;
    }
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = mavros_map_frame_;
    pose_stamped.header.stamp = this->get_clock()->now();

    for (int ii = 0; ii < points.size(); ii++) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = points.at(ii).x;
        pose.position.y = points.at(ii).y;
        pose.position.z = target_altitude_;
        pose_stamped.pose = pose;
        lawnmower_points_.push_back(pose_stamped);
    }
    return true;
}

// TODO remove this once confirmed
void TaskManager::visualizeLawnmower() {
    std_msgs::msg::ColorRGBA red;
    red.r = 1.0;
    red.g = 0;
    red.b = 0;
    red.a = 0.5;

    visualization_msgs::msg::MarkerArray markers_msg;
    std::vector<visualization_msgs::msg::Marker> &markers = markers_msg.markers;
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
    for (auto &frontier : lawnmower_points_) {
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

bool TaskManager::getLawnmowerGoal() {
    if (lawnmower_points_.empty()) {
        bool success = getLawnmowerPattern(current_mission_.polygon, lawnmower_points_);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get lawnmower pattern");
            return false;
        }
    }
    goal_ = lawnmower_points_.at(0);

    // Determine heading
    geometry_msgs::msg::PoseStamped current_pos =
        flight_controller_interface_->getCurrentLocalPosition();
    double x_diff = goal_.pose.position.x - current_pos.pose.position.x;
    double y_diff = goal_.pose.position.y - current_pos.pose.position.y;
    double yaw_target = atan2(y_diff, x_diff);
    tf2::Quaternion setpoint_q;
    setpoint_q.setRPY(0.0, 0.0, yaw_target);
    tf2::convert(setpoint_q, goal_.pose.orientation);

    lawnmower_points_.erase(lawnmower_points_.begin());
    return true;
}

bool TaskManager::lawnmowerGoalComplete() {
    geometry_msgs::msg::PoseStamped current_pos =
        flight_controller_interface_->getCurrentLocalPosition();
    double dist_sq = std::pow(current_pos.pose.position.x - goal_.pose.position.x, 2) +
                     std::pow(current_pos.pose.position.y - goal_.pose.position.y, 2);
    double min_dist = std::pow(3, 2); // within 2m
    return dist_sq < min_dist;
}

} // namespace task_manager