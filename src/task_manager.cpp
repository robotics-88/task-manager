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
    , offline_(false)
    , hello_decco_manager_(node)
    , enable_autonomy_(false)
    , enable_exploration_(false)
    , ardupilot_(true)
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
    , mapir_topic_("/mapir_rgn/image_rect")
    , rosbag_topic_("/record/heartbeat")
    , did_save_(false)
    , did_takeoff_(false)
    , is_armed_(false)
    , explicit_global_params_(false)
    , do_slam_(false)
{
    private_nh_.param<bool>("enable_autonomy", enable_autonomy_, enable_autonomy_);
    private_nh_.param<bool>("enable_exploration", enable_exploration_, enable_exploration_);
    private_nh_.param<bool>("ardupilot", ardupilot_, ardupilot_);
    private_nh_.param<float>("default_altitude_m", target_altitude_, target_altitude_);
    private_nh_.param<float>("min_altitude", min_altitude_, min_altitude_);
    private_nh_.param<float>("max_altitude", max_altitude_, max_altitude_);
    private_nh_.param<double>("max_dist_to_polygon", max_dist_to_polygon_, max_dist_to_polygon_);

    std::string goal_topic = "/mavros/setpoint_position/local";
    private_nh_.param<std::string>("goal_topic", goal_topic, goal_topic);
    private_nh_.param<bool>("do_record", do_record_, do_record_);
    private_nh_.param<std::string>("mavros_map_frame", mavros_map_frame_, mavros_map_frame_);
    private_nh_.param<std::string>("slam_map_frame", slam_map_frame_, slam_map_frame_);
    private_nh_.param<std::string>("path_planner_topic", path_planner_topic_, path_planner_topic_);
    private_nh_.param<std::string>("slam_pose_topic", slam_pose_topic_, slam_pose_topic_);
    private_nh_.param<std::string>("costmap_topic", costmap_topic_, costmap_topic_);
    private_nh_.param<std::string>("lidar_topic", lidar_topic_, lidar_topic_);
    private_nh_.param<std::string>("mapir_topic", mapir_topic_, mapir_topic_);
    private_nh_.param<std::string>("rosbag_topic", rosbag_topic_, rosbag_topic_);
    private_nh_.param<bool>("offline", offline_, offline_);
    private_nh_.param<std::string>("data_directory", burn_dir_prefix_, burn_dir_prefix_);
    private_nh_.param<bool>("explicit_global", explicit_global_params_, explicit_global_params_);
    private_nh_.param<bool>("do_slam", do_slam_, do_slam_);

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
    path_planner_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(path_planner_topic_, 10, &TaskManager::pathPlannerCallback, this);
    costmap_sub_ = nh_.subscribe<map_msgs::OccupancyGridUpdate>(costmap_topic_, 10, &TaskManager::costmapCallback, this);
    lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(lidar_topic_, 10, &TaskManager::lidarCallback, this);
    mapir_sub_ = nh_.subscribe<sensor_msgs::Image>(mapir_topic_, 10, &TaskManager::mapirCallback, this);
    attollo_sub_ = nh_.subscribe<sensor_msgs::Image>("/attollo_cam/image_rect", 10, &TaskManager::attolloCallback, this);
    rosbag_sub_ = nh_.subscribe<std_msgs::String>(rosbag_topic_, 10, &TaskManager::rosbagCallback, this);

    // Geo/map state services
    geopoint_service_ = nh_.advertiseService("/slam2geo", &TaskManager::convert2Geo, this);

    // MAVROS
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    local_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    vision_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    // Heartbeat handling
    heartbeat_pub_ = nh_.advertise<std_msgs::String>("/mapversation/monarch_heartbeat", 10);
    int heartbeat_hz = 1;
    heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0 / heartbeat_hz), &TaskManager::heartbeatTimerCallback, this);
    ui_heartbeat_subscriber_ = nh_.subscribe<std_msgs::String>("/mapversation/ui_heartbeat", 10, &TaskManager::uiHeartbeatCallback, this);

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
    task_json_pub_ = nh_.advertise<std_msgs::String>("/mapversation/task_status", 10);
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(goal_topic, 10, &TaskManager::goalCallback, this);
    task_msg_.enable_autonomy = false;
    task_msg_.enable_exploration = false;

    // Burn unit subscribers
    burn_unit_sub_ = nh_.subscribe<std_msgs::String>("/mapversation/burn_unit_send", 10, &TaskManager::makeBurnUnitJson, this);
    target_polygon_subscriber_ = nh_.subscribe<geometry_msgs::Polygon>("/mapversation/target_polygon", 10, &TaskManager::targetPolygonCallback, this);
    target_setpoint_subscriber_ = nh_.subscribe<sensor_msgs::NavSatFix>("/mapversation/target_setpoint", 10, &TaskManager::targetSetpointCallback, this);
    emergency_subscriber_ = nh_.subscribe<mavros_msgs::StatusText>("/mapversation/emergency", 10, &TaskManager::emergencyResponse, this);

    // Logs created in offline mode
    vegetation_save_client_ = private_nh_.serviceClient<messages_88::Save>("/vegetation/save");
    tree_save_client_ = private_nh_.serviceClient<messages_88::Save>("/species_mapper/save");
    std::string data_folder = ros::package::getPath("task_manager_88") + "/logs/";
    std::string time_local = boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time());
    time_local.replace(time_local.find(" "), 1, "_");
    log_dir_ = data_folder + time_local + "/";
    if ((do_record_ || offline_) && !boost::filesystem::exists(log_dir_)) {
        ROS_INFO("Folder did not exist, creating directory: %s", log_dir_.c_str());
        boost::filesystem::create_directories(log_dir_);
    }

    initDroneStateManager();
}

TaskManager::~TaskManager(){}

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

    current_status_ = CurrentStatus::INITIALIZED;
    health_pub_timer_ = private_nh_.createTimer(health_check_s_,
                               [this](const ros::TimerEvent&) { publishHealth(); });
}

void TaskManager::failsafe() {
    cmd_history_.append("Failsafe init. \n");
    drone_state_manager_.setMode(land_mode_);
    stop();
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
        nav_msg.header.frame_id = "base_link";
        nav_msg.header.stamp = slam_pose->header.stamp;
        nav_msg.latitude = lat;
        nav_msg.longitude = lon;
        global_pose_pub_.publish(nav_msg);

    }
}

void TaskManager::emergencyResponse(const mavros_msgs::StatusText::ConstPtr &msg) {
    ROS_WARN("Emergency response initiated, level %d.", msg->severity);
    // Immediately set to hover
    drone_state_manager_.setMode(loiter_mode_);
    cmd_history_.append("Emergency init with severity " + std::to_string(msg->severity) + "\n");

    // Then respond based on severity 
    // (message definitions at http://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/StatusText.html)
    int level = msg->severity;
    if (level == 5) {
        // NOTICE = PAUSE
        // TODO, tell exploration to stop searching frontiers. For now, will keep blacklisting them, but the drone is in loiter mode. Currently no way to pick back up and set to guided mode (here or in HD)
    }
    else if (level == 0) {
        // EMERGENCY = LAND IMMEDIATELY
        pauseOperations();
        drone_state_manager_.setMode(land_mode_);
    }
    else if (level == 2) {
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
    geometry_msgs::PoseStamped local = drone_state_manager_.getCurrentSlamPosition();
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
    std::string s = j.dump();
    std_msgs::String hb_string;
    hb_string.data = s;
    heartbeat_pub_.publish(hb_string);

    ros::Time now_time = ros::Time::now();
    float interval = (now_time - last_ui_heartbeat_stamp_).toSec();
    // TODO needs also check if drone state is in air/action
    if (interval > ui_hb_threshold_) {
        messages_88::Emergency emergency;
        // TODO fill in pause msg
        // emergency_client_.call(emergency);
    }
}

void TaskManager::uiHeartbeatCallback(const std_msgs::String::ConstPtr &msg) {
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
    if (!isInside(current_polygon_, drone_state_manager_.getCurrentSlamPosition().pose.position)) {
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
    while (!(isInside(current_polygon_, drone_state_manager_.getCurrentSlamPosition().pose.position) || current_status_ == CurrentStatus::WAITING_TO_EXPLORE)) {
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
    did_save_ = false;
}

void TaskManager::stop() {
    stopBag();
    if (!did_save_) {
        messages_88::Save save_msg;
        save_msg.request.directory.data = log_dir_;
        vegetation_save_client_.call(save_msg);
        tree_save_client_.call(save_msg);
        did_save_ = true;
    }
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
        did_save_ = false;
    }
    if (is_armed_ && !armed) {
        cmd_history_.append("Disarm detected. \n ");
        // Handle save bag during disarm (manual or auton)
        stop();
        is_armed_ = false; // Reset so can restart if another arming
    }
    geometry_msgs::PoseStamped home_pos;
    home_pos.header.frame_id = slam_map_frame_;
    home_pos.header.stamp = ros::Time::now();
    home_pos.pose.position.x = 0;
    home_pos.pose.position.y = 0;
    home_pos.pose.position.z = current_explore_goal_.altitude;
    if (current_status_ == CurrentStatus::EXPLORING) {
        // Check action client status to see if complete
        actionlib::SimpleClientGoalState goal_state = explore_action_client_.getState();
        bool is_aborted = goal_state == actionlib::SimpleClientGoalState::ABORTED;
        bool is_lost = goal_state == actionlib::SimpleClientGoalState::LOST;
        bool is_completed = goal_state == actionlib::SimpleClientGoalState::SUCCEEDED;
        if (is_aborted || is_lost || is_completed) {
            ROS_INFO("explore action client state: %s", explore_action_client_.getState().getText().c_str());
            std::string action_string = "Exploration complete, action client status: " + explore_action_client_.getState().getText() + ", sending SLAM origin as position target. \n";
            cmd_history_.append(action_string);
            local_pos_pub_.publish(home_pos);
            current_status_ = CurrentStatus::RTL_88;
            hello_decco_manager_.updateBurnUnit(current_index_, "COMPLETED");
        }
    }
    if (current_status_ == CurrentStatus::RTL_88) { 
        if (drone_state_manager_.getCurrentSlamPosition().pose.position == home_pos.pose.position) {
            drone_state_manager_.setMode(land_mode_);
            current_status_ = CurrentStatus::LANDING;
        }
    }
    task_msg_.header.stamp = ros::Time::now();
    task_msg_.cmd_history.data = cmd_history_.c_str();
    task_msg_.current_status.data = getStatusString();
    task_pub_.publish(task_msg_);
    json task_json = makeTaskJson();
    std_msgs::String task_json_msg;
    task_json_msg.data = task_json.dump();
    task_json_pub_.publish(task_json_msg);
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
    geometry_msgs::Point my_position = drone_state_manager_.getCurrentSlamPosition().pose.position;
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
    geometry_msgs::Point my_position = drone_state_manager_.getCurrentSlamPosition().pose.position;
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
    // 1) Path planner
    bool path_healthy = (t - last_path_planner_stamp_ < health_check_s_);
    if (!path_healthy && do_slam_) {
        cmd_history_.append("Failsafe triggered by path unhealthy. \n");
        failsafe();
    }
    json j = {
        {"name", "pathPlanner"},
        {"label", "Path planner"},
        {"isHealthy", path_healthy}
    };
    healthObjects.push_back(j);
    // 2) SLAM position
    bool slam_healthy = (t - last_slam_pos_stamp_ < health_check_s_);
    if (!slam_healthy && do_slam_) {
        cmd_history_.append("Failsafe triggered by SLAM unhealthy. \n");
        failsafe();
    }
    j = {
        {"name", "slamPosition"},
        {"label", "SLAM position"},
        {"isHealthy", slam_healthy}
    };
    healthObjects.push_back(j);
    // 3) costmap
    j = {
        {"name", "costmap"},
        {"label", "Costmap"},
        {"isHealthy", (t - last_costmap_stamp_ < health_check_s_)}
    };
    healthObjects.push_back(j);
    // 4) LiDAR
    j = {
        {"name", "lidar"},
        {"label", "LiDAR"},
        {"isHealthy", (t - last_lidar_stamp_ < health_check_s_)}
    };
    healthObjects.push_back(j);
    // 5) MAPIR
    j = {
        {"name", "mapir"},
        {"label", "MAPIR Camera"},
        {"isHealthy", (t - last_mapir_stamp_ < health_check_s_)}
    };
    healthObjects.push_back(j);
    // 5) Attollo
    j = {
        {"name", "attollo"},
        {"label", "Attollo Camera"},
        {"isHealthy", (t - last_attollo_stamp_ < health_check_s_)}
    };
    healthObjects.push_back(j);
    // 6) Explore
    j = {
        {"name", "explore"},
        {"label", "Explore Service"},
        {"isHealthy", explore_healthy}
    };
    healthObjects.push_back(j);
    // 7) ROS bag
    j = {
        {"name", "rosbag"},
        {"label", "ROS bag"},
        {"isHealthy", (t - last_rosbag_stamp_ < health_check_s_)}
    };
    healthObjects.push_back(j);
    jsonObjects["healthIndicators"] = healthObjects;

    std::string s = jsonObjects.dump();
    std_msgs::String health_string;
    health_string.data = s;
    health_pub_.publish(health_string);
}

void TaskManager::pathPlannerCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    last_path_planner_stamp_ = msg->header.stamp;
}

void TaskManager::costmapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr &msg) {
    last_costmap_stamp_ = msg->header.stamp;
}

void TaskManager::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // TODO do we want to change this to point to raw pointcloud? If registered, not independent from SLAM position, but raw requires handling multiple data types.
    last_lidar_stamp_ =  msg->header.stamp;
}

void TaskManager::mapirCallback(const sensor_msgs::ImageConstPtr &msg) {
    last_mapir_stamp_ = msg->header.stamp;
}

void TaskManager::attolloCallback(const sensor_msgs::ImageConstPtr &msg) {
    last_attollo_stamp_ = msg->header.stamp;
}

void TaskManager::rosbagCallback(const std_msgs::StringConstPtr &msg) {
    last_rosbag_stamp_ = ros::Time::now();
}

void TaskManager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    goal_ = *msg;
}

void TaskManager::makeBurnUnitJson(const std_msgs::String::ConstPtr &msg) {
    if (!enable_exploration_) {
        ROS_WARN("Exploration disabled, burn unit ignored.");
        return;
    }
    if (!map_tf_init_) {
        ROS_WARN("Not ready for flight, try again after initialized.");
        return;
    }
    json burn_unit = json::parse(msg->data);
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

// Below are purely test methods, to eventually be deprecated in favor of burn units
void TaskManager::targetPolygonCallback(const geometry_msgs::Polygon::ConstPtr &msg) {
    json burn_unit = hello_decco_manager_.polygonToBurnUnit(*msg);
    makeBurnUnitJson(burn_unit);
}

void TaskManager::targetSetpointCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    // ATM, this response is purely a testing function. All the drone should do is take off and hover.
    ROS_INFO("setpoint received, ll: %f, %f", msg->latitude, msg->longitude);
    startBag();
    // TODO: check dist (need mavros sub position) below threshold, check mavros status, takeoff etc if needed, go to point
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
    return j;
}

}