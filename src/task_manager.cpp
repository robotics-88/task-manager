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

#include <mavros_msgs/BasicID.h>
#include <mavros_msgs/OperatorID.h>
#include <mavros_msgs/SelfID.h>
#include <mavros_msgs/System.h>
#include <mavros_msgs/SystemUpdate.h>

#include <pcl_ros/point_cloud.h>
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
    , current_task_(CurrentTask::INITIALIZING)
    , task_manager_loop_duration_(1.0)
    , simulate_(false)
    , offline_(false)
    , hello_decco_manager_(node)
    , enable_autonomy_(false)
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
    , health_check_pub_duration_(5.0)
    , path_planner_topic_("/kd_pointcloud_accumulated")
    , costmap_topic_("/costmap_node/costmap/costmap_updates")
    , lidar_topic_("/cloud_registered")
    , thermal_topic_("/thermal_cam/image_rect_color")
    , attollo_topic_("/mapir_rgn/image_rect")
    , mapir_topic_("/mapir_rgn/image_rect_color")
    , mapir_rgb_topic_("/mapir_rgn/image_rect_color")
    , rosbag_topic_("/record/heartbeat")
    , is_armed_(false)
    , in_autonomous_flight_(false)
    , explicit_global_params_(false)
    , init_remote_id_message_sent_(false)
    , takeoff_attempts_(0)
    , estimated_drone_speed_(2.0)
    , battery_failsafe_safety_factor_(2.0)
    , do_slam_(false)
    , needs_takeoff_(false)
    , last_rid_updated_timestamp_(0)
    , operator_id_("")
    , lidar_timeout_(0.5)
    , slam_timeout_(0.5)
    , path_timeout_(0.5)
    , costmap_timeout_(3.0)
    , explore_timeout_(1.0)
    , mapir_timeout_(1.0)
    , attollo_timeout_(1.0)
    , thermal_timeout_(1.0)
    , rosbag_timeout_(1.0)
{
    private_nh_.param<bool>("enable_autonomy", enable_autonomy_, enable_autonomy_);
    private_nh_.param<bool>("ardupilot", ardupilot_, ardupilot_);
    private_nh_.param<bool>("use_failsafes", use_failsafes_, use_failsafes_);
    private_nh_.param<float>("default_alt", target_altitude_, target_altitude_);
    private_nh_.param<float>("min_alt", min_altitude_, min_altitude_);
    private_nh_.param<float>("max_alt", max_altitude_, max_altitude_);
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

    // SLAM pose sub
    decco_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(slam_pose_topic_, 10, &TaskManager::deccoPoseCallback, this);

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

    // Remote ID
    odid_basic_id_pub_ = nh_.advertise<mavros_msgs::BasicID>("/mavros/open_drone_id/basic_id", 10);
    odid_operator_id_pub_ = nh_.advertise<mavros_msgs::OperatorID>("/mavros/open_drone_id/operator_id", 10);
    odid_self_id_pub_ = nh_.advertise<mavros_msgs::SelfID>("/mavros/open_drone_id/self_id", 10);
    odid_system_pub_ = nh_.advertise<mavros_msgs::System>("/mavros/open_drone_id/system", 10);
    odid_system_update_pub_ = nh_.advertise<mavros_msgs::SystemUpdate>("/mavros/open_drone_id/system_update", 10);
    odid_timer_ = nh_.createTimer(ros::Duration(1.0), &TaskManager::odidTimerCallback, this);

    // Heartbeat timer
    int heartbeat_hz = 1;
    heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0 / heartbeat_hz), &TaskManager::heartbeatTimerCallback, this);

    // Recording
    start_record_pub_ = nh_.advertise<bag_recorder::Rosbag>("/record/start", 5);
    stop_record_pub_ = nh_.advertise<std_msgs::String>("/record/stop", 5);
    if (offline_) {
        map_yaw_sub_ = nh_.subscribe<std_msgs::Float64>("map_yaw", 10, &TaskManager::mapYawCallback, this);
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

    health_check_timer_ = private_nh_.createTimer(ros::Duration(0.1),
                               [this](const ros::TimerEvent&) { checkHealth(); });

    task_manager_timer_ = private_nh_.createTimer(task_manager_loop_duration_,
                               [this](const ros::TimerEvent&) { runTaskManager(); });

    health_pub_timer_ = private_nh_.createTimer(health_check_pub_duration_,
                               [this](const ros::TimerEvent&) { publishHealth(); });
}

TaskManager::~TaskManager(){}

void TaskManager::runTaskManager() {

    // Check arm status and make sure bag recording is happening properly.
    checkArmStatus();

    checkFailsafes();

    // Update home position timestamp
    home_pos_.header.stamp = ros::Time::now();

    switch (current_task_) {
        case CurrentTask::INITIALIZING: {
            drone_state_manager_.setMode(guided_mode_);
            if (getMapTf()) {
                if (drone_state_manager_.getDroneReadyToArm())
                    current_task_ = CurrentTask::READY;
                else
                    current_task_ = CurrentTask::PREFLIGHT_CHECK_FAIL;
            }
            break;
        }
        case CurrentTask::PREFLIGHT_CHECK_FAIL: {
            // TODO sim should pass arming checks too, they don't for some weird reasons.
            // Like loop rate 222 (which can be fixed by putting ClockSpeed: 0.8 in settings.json)
            // However, other things still fail. Eventually we should fix this and set ARMING_CHECK to 1 in ardupilot
            if (drone_state_manager_.getDroneReadyToArm() || simulate_)
                current_task_ = CurrentTask::READY;
            break;
        }
        case CurrentTask::READY: {
            // This flag gets triggered when received a burn unit
            if (needs_takeoff_) 
            {
                if (takeoff_attempts_ > 5) {
                    ROS_WARN("Takeoff failed after 5 attempts");
                    needs_takeoff_ = false;
                    takeoff_attempts_ = 0;
                }
                else {
                    startTakeoff();
                }
            }
            else if (drone_state_manager_.getIsArmed()) {
                current_task_ = CurrentTask::MANUAL_FLIGHT;
            }
            break;
        }
        case CurrentTask::MANUAL_FLIGHT: {
            if (!drone_state_manager_.getIsArmed()) {
                current_task_ = CurrentTask::COMPLETE;
            }
            break;
        }
        case CurrentTask::LOITER: {
            if (drone_state_manager_.getFlightMode() != "LOITER") {
                current_task_ = CurrentTask::MANUAL_FLIGHT;
            }
            break;
        }
        case CurrentTask::TAKING_OFF: {
            // Once we reach takeoff altitude, transition to next flight state
            if (drone_state_manager_.getAltitudeAGL() > (target_altitude_ - 1)) {
                // If not in polygon, start navigation task
                if (!isInside(current_polygon_, drone_state_manager_.getCurrentLocalPosition().pose.position)) {
                    startTransit();
                }
                else {
                    startExploration();
                }
            }
            break;
        }
        case CurrentTask::IN_TRANSIT: {
            if (isInside(current_polygon_, drone_state_manager_.getCurrentLocalPosition().pose.position)) {
                startExploration();
            }
            break;
        }
        case CurrentTask::EXPLORING: {            
            // Check action client status to see if complete
            actionlib::SimpleClientGoalState goal_state = explore_action_client_.getState();
            if (goal_state == actionlib::SimpleClientGoalState::ABORTED || 
                goal_state == actionlib::SimpleClientGoalState::LOST ||
                goal_state == actionlib::SimpleClientGoalState::SUCCEEDED
                ) {

                startRtl88("exploration " + explore_action_client_.getState().getText());
                hello_decco_manager_.updateBurnUnit(current_index_, "COMPLETED");
            }

            break;
        }
        case CurrentTask::RTL_88: {
            if (drone_state_manager_.getCurrentLocalPosition().pose.position == home_pos_.pose.position) {
                ROS_INFO("RTL_88 completed, landing");
                drone_state_manager_.setMode(land_mode_);
                current_task_ = CurrentTask::LANDING;
            }
            break;
        }
        case CurrentTask::LANDING:
        case CurrentTask::FAILSAFE_LANDING: {
            if (!drone_state_manager_.getIsArmed()) {
                current_task_ = CurrentTask::COMPLETE;
            }
            break;
        }
        case CurrentTask::COMPLETE: {
            in_autonomous_flight_ = false;
            ROS_INFO("Flight complete, reinitializing");
            ros::Duration(1.0).sleep(); // Quick sleep to let drone settle down before reinitializing
            current_task_ = CurrentTask::INITIALIZING;
            break;
        }
        default: {
            break;
        }
    }

    task_msg_.header.stamp = ros::Time::now();
    task_msg_.cmd_history.data = cmd_history_.c_str();
    task_msg_.current_status.data = getStatusString();
    task_pub_.publish(task_msg_);
    json task_json = makeTaskJson();
    hello_decco_manager_.packageToMapversation("task_status", task_json);
}

void TaskManager::startTakeoff() {

    // This is a redundant check but probably good to keep
    if (!task_msg_.enable_autonomy) {
        ROS_WARN("Not taking off, autonomy disabled");
        return;
    }

    if (drone_state_manager_.takeOff()) {
        if (do_record_) {
            startBag();
        }
        in_autonomous_flight_ = true;
        current_task_ = CurrentTask::TAKING_OFF;
        needs_takeoff_ = false;
        takeoff_attempts_ = 0; 
    } 
    else {
        ROS_INFO("Takeoff request failed. Retrying.");
        takeoff_attempts_++;
    }

}

void TaskManager::startTransit() {    
    padNavTarget(initial_transit_point_);

    current_target_ = initial_transit_point_.pose.position;
    initial_transit_point_.header.frame_id = slam_map_frame_;
    initial_transit_point_.header.stamp = ros::Time::now();
    local_pos_pub_.publish(initial_transit_point_);

    current_task_ = CurrentTask::IN_TRANSIT;
}

void TaskManager::startExploration() {

    current_explore_goal_.polygon = current_polygon_;
    current_explore_goal_.altitude = target_altitude_;
    current_explore_goal_.min_altitude = min_altitude_;
    current_explore_goal_.max_altitude = max_altitude_;

    explore_action_client_.waitForServer();
    cmd_history_.append("Sending explore goal.\n");

    // Start with a rotation command in case no frontiers immediately processed, will be overridden with first exploration goal
    geometry_msgs::Twist vel;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = M_PI_2; // PI/2 rad/s
    local_vel_pub_.publish(vel);

    explore_action_client_.sendGoal(current_explore_goal_);

    cmd_history_.append("Sent explore goal.\n");
    hello_decco_manager_.updateBurnUnit(current_index_, "ACTIVE");

    current_task_ = CurrentTask::EXPLORING;
}

void TaskManager::startRtl88(std::string reason) {
    std::string str = "Initializing RTL 88 due to " + reason;
    ROS_INFO("%s", str.c_str());
    cmd_history_.append(str + "\n");
    pauseOperations();
    local_pos_pub_.publish(home_pos_);
    current_task_ = CurrentTask::RTL_88;
}

void TaskManager::startFailsafeLanding(std::string reason) {
    std::string str = "Initializing failsafe landing due to " + reason;
    ROS_INFO("%s", str.c_str());
    cmd_history_.append(str + "\n");
    pauseOperations();
    drone_state_manager_.setMode(land_mode_);
    current_task_ = CurrentTask::FAILSAFE_LANDING;
    
}

void TaskManager::startLoiter(std::string reason) {
    std::string str = "Initializing loiter due to " + reason;
    ROS_INFO("%s", str.c_str());
    cmd_history_.append(str + "\n");
    drone_state_manager_.setMode(loiter_mode_);
    current_task_ = CurrentTask::LOITER;
}

void TaskManager::checkHealth() {
    ros::Time now = ros::Time::now();

    health_checks_.battery_ok = isBatteryOk();
    health_checks_.lidar_ok = now - last_lidar_stamp_ < lidar_timeout_;
    health_checks_.slam_ok = now - last_slam_pos_stamp_ < slam_timeout_;
    health_checks_.path_ok = now - last_path_planner_stamp_ < path_timeout_;
    health_checks_.costmap_ok = now - last_costmap_stamp_ < costmap_timeout_;
    health_checks_.explore_ok = explore_action_client_.waitForServer(explore_timeout_);
    health_checks_.mapir_ok = now - last_mapir_stamp_ < mapir_timeout_;
    health_checks_.attollo_ok = now - last_attollo_stamp_ < attollo_timeout_;
    health_checks_.thermal_ok = now - last_thermal_stamp_ < thermal_timeout_;
    health_checks_.rosbag_ok = now - last_rosbag_stamp_ < rosbag_timeout_;
}

void TaskManager::checkFailsafes() {

    if (in_autonomous_flight_) {

        // Check for manual takeover
        if (drone_state_manager_.getFlightMode() != drone_state_manager_.getLastSetFlightMode()) {
            ROS_WARN("WARNING: Manual takeover initiated");
            current_task_ = CurrentTask::MANUAL_FLIGHT;
            in_autonomous_flight_ = false;
            pauseOperations();
            return;
        }

        // Check for failsafe landing conditions
        std::string failsafe_reason = "";
        bool need_failsafe_landing = false;
        if (!health_checks_.slam_ok) {
            failsafe_reason += "SLAM unhealthy, ";
            need_failsafe_landing = true;
        }
        if (!health_checks_.path_ok) {
            failsafe_reason += "Path unhealthy, ";
            need_failsafe_landing = true;
        }
        if (need_failsafe_landing) {
            if (current_task_ != CurrentTask::FAILSAFE_LANDING) {
                if (use_failsafes_) {
                    startFailsafeLanding(failsafe_reason);
                }
                else {
                    if (current_task_ != CurrentTask::LOITER) {
                        startLoiter("failsafe landing requested but failsafes not active");
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
            current_task_ != CurrentTask::RTL_88 &&
            current_task_ != CurrentTask::LANDING &&
            current_task_ != CurrentTask::COMPLETE) {

            startRtl88(rtl_88_reason);
        }
    }
}

bool TaskManager::getMapTf() {

    // Get drone heading
    if (!offline_) {
        double yaw = 0;
        if (!drone_state_manager_.getMapYaw(yaw)) {
            ROS_WARN_THROTTLE(10, "Waiting for heading from autopilot...");
            return false;
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
    ros::Rate r(0.5);
    ROS_INFO("Initializing IMU");
    while (!drone_state_manager_.initializeImu(mavros_init_imu)) {
        r.sleep();
    }
    tf2::Quaternion quatmav(mavros_init_imu.orientation.x, mavros_init_imu.orientation.y, mavros_init_imu.orientation.z, mavros_init_imu.orientation.w);
    tf2::Matrix3x3 m(quatmav);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Roll: %f, Pitch: %f, Yaw, %f", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);

    // Fill in data
    map_to_slam_tf_.header.frame_id = mavros_map_frame_;
    map_to_slam_tf_.header.stamp = ros::Time::now();
    map_to_slam_tf_.child_frame_id = slam_map_frame_;
    map_to_slam_tf_.transform.translation.x = 0.0;
    map_to_slam_tf_.transform.translation.y = 0.0;
    map_to_slam_tf_.transform.translation.z = 0.0;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, map_yaw_); // TODO, get rid of map_yaw_? It's the same as IMU yaw. Also confirm IMU is ok and not being affected by param setup on launch
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

    ROS_INFO("Waiting for global...");
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
    ROS_INFO("Map yaw: %f", map_yaw_ * 180 / M_PI);
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

    return true;
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
    tf_buffer_.transform(in, out, "utm");
    resp.utm_position.x = out.point.x;
    resp.utm_position.y = out.point.y;
    return true;
}

void TaskManager::heartbeatTimerCallback(const ros::TimerEvent&) {
    sensor_msgs::NavSatFix hb = drone_state_manager_.getCurrentGlobalPosition();
    geometry_msgs::PoseStamped local = drone_state_manager_.getCurrentLocalPosition();
    double altitudeAgl = drone_state_manager_.getAltitudeAGL();
    geometry_msgs::Quaternion quat_flu = local.pose.orientation;
    double yaw = drone_state_manager_.getCompass();
    json j = {
        {"latitude", hb.latitude},
        {"longitude", hb.longitude},
        {"altitude", (int)altitudeAgl},
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

// Publish the ODID messages that require updates
void TaskManager::odidTimerCallback(const ros::TimerEvent&) {

    // Self ID
    mavros_msgs::SelfID self_id;
    self_id.header.stamp = ros::Time::now();
    if (current_task_ == CurrentTask::FAILSAFE_LANDING) {
        self_id.description_type = mavros_msgs::SelfID::MAV_ODID_DESC_TYPE_EMERGENCY;
        self_id.description = "FAILSAFE. CAUTION";
    }
    else {
        self_id.description_type = mavros_msgs::SelfID::MAV_ODID_DESC_TYPE_TEXT;
        self_id.description = getStatusString();
    }
    odid_self_id_pub_.publish(self_id);
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
    task_msg_.enable_exploration = true; // TODO just remove this

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

void TaskManager::checkArmStatus() {
    std::string mode = drone_state_manager_.getFlightMode();
    bool armed = drone_state_manager_.getIsArmed();
    if (!is_armed_ && armed) {
        cmd_history_.append("Manually set armed state to true. \n");
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
        stopBag();
        pauseOperations();
        is_armed_ = false; // Reset so can restart if another arming
    }
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

bool TaskManager::polygonDistanceOk(geometry_msgs::PoseStamped &target, geometry_msgs::Polygon &map_region) {

    if (isInside(current_polygon_, drone_state_manager_.getCurrentLocalPosition().pose.position))
        return true;

    // Medium check, computes distance to nearest point on 2 most likely polygon edges
    // Polygon is already in map coordinates, i.e., expressed in meters from UAS home
    double min_dist = DBL_MAX;
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
    switch (current_task_) {
        case CurrentTask::INITIALIZING:           return "INITIALIZING";
        case CurrentTask::PREFLIGHT_CHECK_FAIL:   return "PREFLIGHT_CHECK_FAIL";
        case CurrentTask::READY:                  return "READY";
        case CurrentTask::MANUAL_FLIGHT:          return "MANUAL_FLIGHT";
        case CurrentTask::LOITER:                 return "LOITER";
        case CurrentTask::EXPLORING:              return "EXPLORING";
        case CurrentTask::IN_TRANSIT:             return "IN_TRANSIT";
        case CurrentTask::RTL_88:                 return "RTL_88";
        case CurrentTask::TAKING_OFF:             return "TAKING_OFF";
        case CurrentTask::LANDING:                return "LANDING";
        case CurrentTask::FAILSAFE_LANDING:       return "FAILSAFE_LANDING";
        case CurrentTask::COMPLETE:               return "COMPLETE";
        default:                                  return "unknown";
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

void TaskManager::registeredPclCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (!map_tf_init_) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr reg_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg (*msg, *reg_cloud);

    pcl_ros::transformPointCloud(*reg_cloud, *map_cloud, map_to_slam_tf_.transform);

    sensor_msgs::PointCloud2 map_cloud_ros;
    pcl::toROSMsg(*map_cloud, map_cloud_ros);
    map_cloud_ros.header.frame_id = mavros_map_frame_;
    pointcloud_repub_.publish(map_cloud_ros);
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

void TaskManager::mapYawCallback(const std_msgs::Float64::ConstPtr &msg) {
    map_yaw_ = msg->data;
    ROS_INFO("Got map yaw: %f", map_yaw_ * 180 / M_PI);
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
    else if (topic == "altitudes") {
        altitudesResponse(gossip_json);
    }
    else if (topic == "heartbeat") {
        remoteIDResponse(gossip_json);
    }
}

void TaskManager::makeBurnUnitJson(json burn_unit) {

    if (!map_tf_init_) {
        ROS_WARN("Not ready for flight, try again after initialized.");
        return;
    }
    std::string name = burn_unit["name"];
    burn_dir_prefix_ = burn_dir_prefix_ + name + "/";
    hello_decco_manager_.makeBurnUnitJson(burn_unit, home_utm_zone_);
    current_index_ = hello_decco_manager_.initBurnUnit(current_polygon_);
    if (current_index_ < 0) {
        std::string str = "No burn polygon was found, all are already complete, not exploring.";
        ROS_WARN("%s", str.c_str());
        cmd_history_.append(str + "\n");
        return;
    }
    
    if (!polygonDistanceOk(initial_transit_point_, current_polygon_)) {
        std::string str = "Polygon rejected, exceeds maximum starting distance threshold.";
        ROS_WARN("%s", str.c_str());
        cmd_history_.append(str + "\n");
        return;
    }

    needs_takeoff_ = true;
}

void TaskManager::setpointResponse(json &json_msg) {
    // ATM, this response is purely a testing function. 
    startBag();
}

void TaskManager::emergencyResponse(const std::string severity) {
    std::string str = "Emergency response initiated with level " + severity;
    ROS_WARN("%s", str.c_str());
    cmd_history_.append(str + "\n");

    // Then respond based on severity 
    if (severity == "PAUSE") {
        // NOTICE = PAUSE
        // TODO, tell exploration to stop searching frontiers. For now, will keep blacklisting them, but the drone is in loiter mode. Currently no way to pick back up and set to guided mode (here or in HD)
        // Immediately set to hover
        startLoiter("pilot request");
    }
    else if (severity == "LAND") {
        // EMERGENCY = LAND IMMEDIATELY
        startFailsafeLanding("pilot request");
    }
    else if (severity == "RTL") {
        // CRITICAL = RTL
        startRtl88("pilot request");
    }
}

void TaskManager::altitudesResponse(json &json_msg) {

    if (!json_msg["max_altitude"].is_number() || 
        !json_msg["min_altitude"].is_number() || 
        !json_msg["default_altitude"].is_number()) {
        ROS_WARN("Altitude message from mapversation contains invalid data");
        return;
    }

    float max_altitude = json_msg["max_altitude"];
    float min_altitude = json_msg["min_altitude"];
    float default_altitude = json_msg["default_altitude"];

    // Set altitude params in all nodes that use them
    ros::param::set("/task_manager/max_alt", max_altitude);
    ros::param::set("/task_manager/min_alt", min_altitude);
    ros::param::set("/task_manager/default_alt", default_altitude);

    // It's simpler just to set the task manager altitude data here instead of re-fetching the param dynamically
    max_altitude_ = max_altitude;
    min_altitude_ = min_altitude;
    target_altitude_ = default_altitude;

    ros::param::set("/path_planning_node/search/max_alt", max_altitude);
    ros::param::set("/path_planning_node/search/min_alt", min_altitude);
}

void TaskManager::remoteIDResponse(json &json) {

    // Unpack JSON here for convenience
    std::string uas_id_str = json["uas_id"].is_null() ? "" : json["uas_id"];
    std::string operator_id_str = json["operator_id"].is_null() ? "" : json["operator_id"];
    float operator_latitude = json["operator_latitude"].is_number_float() ? (float)json["operator_latitude"] : 0.f;
    float operator_longitude = json["operator_longitude"].is_number_float() ? (float)json["operator_longitude"] : 0.f;
    float operator_altitude_geo = json["operator_altitude_geo"].is_number_float() ? (float)json["operator_altitude_geo"] : 0.f;
    int timestamp = json["timestamp"].is_number_integer() ? (int)json["timestamp"] : 0; 
    
    if (!init_remote_id_message_sent_) {
        // Basic ID
        mavros_msgs::BasicID basic_id;
        basic_id.header.stamp = ros::Time::now();
        basic_id.id_type = mavros_msgs::BasicID::MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID;
        basic_id.ua_type = mavros_msgs::BasicID::MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR;
        basic_id.uas_id = uas_id_str;
        odid_basic_id_pub_.publish(basic_id);

        // Operator ID
        mavros_msgs::OperatorID operator_id;
        operator_id.header.stamp = ros::Time::now();
        operator_id.operator_id_type = mavros_msgs::OperatorID::MAV_ODID_OPERATOR_ID_TYPE_CAA;
        operator_id.operator_id = operator_id_str;
        odid_operator_id_pub_.publish(operator_id);
        operator_id_ = operator_id_str;

        // System
        // This should probably just be published at startup, and System Update published here
        mavros_msgs::System system;
        system.header.stamp = ros::Time::now();
        system.operator_location_type = mavros_msgs::System::MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF; // TODO dynamic operator location
        system.classification_type = mavros_msgs::System::MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED;
        system.operator_latitude = operator_latitude * 1E7;
        system.operator_longitude = operator_longitude * 1E7;
        system.operator_altitude_geo = operator_altitude_geo;
        system.timestamp = timestamp;
        odid_system_pub_.publish(system);

        init_remote_id_message_sent_ = true;
    }

    // SystemUpdate
    mavros_msgs::SystemUpdate system_update;
    system_update.header.stamp = ros::Time::now();
    system_update.operator_latitude = operator_latitude * 1E7;
    system_update.operator_longitude = operator_longitude * 1E7;
    system_update.operator_altitude_geo = operator_altitude_geo;
    if (timestamp > last_rid_updated_timestamp_) {
        system_update.timestamp = timestamp;
        last_rid_updated_timestamp_ = timestamp;
    }
    else {
        system_update.timestamp = ros::Time::now().toSec();
    }
    odid_system_update_pub_.publish(system_update);

}

void TaskManager::publishHealth() {

    auto jsonObjects = json::object();
    json header = {
        {"frame_id", "decco"},
        {"stamp", ros::Time::now().toSec()},
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
    hello_decco_manager_.packageToMapversation("health_report", jsonObjects);
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
    j["operatorID"] = operator_id_;
    j["rawVoltage"] = (int)(drone_state_manager_.getBatteryVoltage() * 100);
    j["readyToArm"] = drone_state_manager_.getDroneReadyToArm();
    j["isArmed"] = drone_state_manager_.getIsArmed();
    return j;
}

}