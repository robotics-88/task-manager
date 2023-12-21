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

#include <task_manager/json.hpp>
using json = nlohmann::json;

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
    , map_tf_init_(false)
    , tf_listener_(tf_buffer_)
    , mavros_map_frame_("map")
    , slam_map_frame_("slam_map")
    , slam_pose_topic_("decco/pose")
    , do_record_(true)
    , bag_active_(false)
    , record_config_name_("r88_default")
    , drone_state_manager_(node)
    , max_dist_to_polygon_(300.0)
    , cmd_history_("")
    , explore_action_client_("explore", true)
    , health_check_s_(5.0)
    , costmap_topic_("/costmap_node/costmap/costmap")
    , lidar_topic_("/livox/lidar")
    , mapir_topic_("/mapir_rgn/image_rect")
    , rosbag_topic_("/record/heartbeat")
    , did_save_(false)
    , did_takeoff_(false)
{
    std::string goal_topic = "/mavros/setpoint_position/local";
    private_nh_.param<std::string>("goal_topic", goal_topic, goal_topic);
    private_nh_.param<bool>("do_record", do_record_, do_record_);
    private_nh_.param<std::string>("mavros_map_frame", mavros_map_frame_, mavros_map_frame_);
    private_nh_.param<std::string>("slam_map_frame", slam_map_frame_, slam_map_frame_);
    private_nh_.param<std::string>("slam_pose_topic", slam_pose_topic_, slam_pose_topic_);
    private_nh_.param<std::string>("costmap_topic", costmap_topic_, costmap_topic_);
    private_nh_.param<std::string>("lidar_topic", lidar_topic_, lidar_topic_);
    private_nh_.param<std::string>("mapir_topic", mapir_topic_, mapir_topic_);
    private_nh_.param<std::string>("rosbag_topic", rosbag_topic_, rosbag_topic_);

    // Subscribe to MAVROS and SLAM pose topics to 
    slam_pose_sub_.subscribe(nh_, slam_pose_topic_, 10);
    mavros_pose_sub_.subscribe(nh_, "/mavros/local_position/pose", 10);
    sync_.reset(new Sync(MySyncPolicy(10), mavros_pose_sub_, slam_pose_sub_));
    sync_->registerCallback(boost::bind(&TaskManager::syncedPoseCallback, this, _1, _2));

    mode_monitor_timer_ = private_nh_.createTimer(ros::Duration(1.0),
                               [this](const ros::TimerEvent&) { modeMonitor(); });

    // Health pubs/subs
    health_pub_ = nh_.advertise<std_msgs::String>("/mapversation/health_report", 10);
    costmap_sub_ = nh_.subscribe<map_msgs::OccupancyGridUpdate>(costmap_topic_, 10, &TaskManager::costmapCallback, this);
    lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(lidar_topic_, 10, &TaskManager::lidarCallback, this);
    mapir_sub_ = nh_.subscribe<sensor_msgs::Image>(mapir_topic_, 10, &TaskManager::mapirCallback, this);
    rosbag_sub_ = nh_.subscribe<std_msgs::String>(rosbag_topic_, 10, &TaskManager::rosbagCallback, this);
    health_pub_timer_ = private_nh_.createTimer(health_check_s_,
                               [this](const ros::TimerEvent&) { publishHealth(); });

    // Drone state services
    drone_state_service_ = nh_.advertiseService("/init_drone_state", &TaskManager::initDroneStateManager, this);
    drone_ready_service_ = nh_.advertiseService("/prep_drone_action", &TaskManager::getReadyForAction, this);
    drone_explore_service_ = nh_.advertiseService("/prep_drone_explore", &TaskManager::getReadyForExplore, this);
    drone_position_service_ = nh_.advertiseService("/get_drone_position", &TaskManager::getDronePosition, this);
    emergency_service_ = nh_.advertiseService("/emergency_response", &TaskManager::emergencyResponse, this);

    // MAVROS
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    local_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    // Recording
    start_record_pub_ = nh_.advertise<bag_recorder::Rosbag>("/record/start", 5);
    stop_record_pub_ = nh_.advertise<std_msgs::String>("/record/stop", 5);

    task_pub_ = nh_.advertise<messages_88::TaskStatus>("task_status", 10);
    task_msg_.enable_autonomy = false;
    task_msg_.enable_exploration = false;

    vegetation_save_client_ = private_nh_.serviceClient<messages_88::Save>("/vegetation/save");
    tree_save_client_ = private_nh_.serviceClient<messages_88::Save>("/species_mapper/save");
    std::string data_folder = ros::package::getPath("task_manager_88") + "/logs/";
    std::string time_local = boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time());
    time_local.replace(time_local.find(" "), 1, "_");
    directory_ = data_folder + time_local + "/";
    if (do_record_ && !boost::filesystem::exists(directory_)) {
        ROS_INFO("Folder did not exist, creating directory: %s", directory_.c_str());
        boost::filesystem::create_directories(directory_);
    }
}

TaskManager::~TaskManager(){}

void TaskManager::syncedPoseCallback(const geometry_msgs::PoseStampedConstPtr &mavros_pose, const geometry_msgs::PoseStampedConstPtr &slam_pose) {
    last_mavros_pos_stamp_ = mavros_pose->header.stamp;
    last_slam_pos_stamp_ = slam_pose->header.stamp;
    if (current_status_ == CurrentStatus::NAVIGATING) {
        if (mavros_pose->pose.position == current_target_ || isInside(current_explore_goal_.polygon, mavros_pose->pose.position)) {
            // Within a meter of target or inside polygon
            current_status_ = CurrentStatus::WAITING_TO_EXPLORE;
        }
    }
    // Attempt at dynamic tf: FROM mavros TO slam
    // Compute the rotation and translation between the mavros and slam base_link estimates
    // double xdif, ydif, zdif;
    // xdif = mavros_pose->pose.position.x - slam_pose->pose.position.x;
    // ydif = mavros_pose->pose.position.y - slam_pose->pose.position.y;
    // zdif = mavros_pose->pose.position.z - slam_pose->pose.position.z;
    // tf2::Quaternion mavros_quat, slam_quat_inv, slam_quat, map_quat;
    // tf2::convert(mavros_pose->pose.orientation, mavros_quat);
    // tf2::convert(slam_pose->pose.orientation, slam_quat);
    // slam_quat_inv = slam_quat;
    // slam_quat_inv.setW(-1 * slam_quat.getW());
    // map_quat = mavros_quat * slam_quat_inv;
    // map_quat.normalize();
    // geometry_msgs::Quaternion tf_quat;
    // tf2::convert(map_quat, tf_quat);

    // geometry_msgs::TransformStamped map_to_slam_tf;
    // map_to_slam_tf.header.frame_id = mavros_map_frame_;
    // map_to_slam_tf.header.stamp = slam_pose->header.stamp;
    // map_to_slam_tf.child_frame_id = slam_map_frame_;
    // map_to_slam_tf.transform.translation.x = xdif;
    // map_to_slam_tf.transform.translation.y = ydif;
    // map_to_slam_tf.transform.translation.z = zdif;
    // map_to_slam_tf.transform.rotation = tf_quat;
    // tf_broadcaster_.sendTransform(map_to_slam_tf);

    // Below assumes static tf
    if (map_tf_init_) {
        return;
    }
    geometry_msgs::TransformStamped map_to_slam_tf;
    map_to_slam_tf.header.frame_id = mavros_map_frame_;
    map_to_slam_tf.header.stamp = ros::Time::now();
    map_to_slam_tf.child_frame_id = slam_map_frame_;

    map_to_slam_tf.transform.translation.x = mavros_pose->pose.position.x;
    map_to_slam_tf.transform.translation.y = mavros_pose->pose.position.y;
    map_to_slam_tf.transform.translation.z = mavros_pose->pose.position.z;

    geometry_msgs::Quaternion quat = mavros_pose->pose.orientation;
    map_to_slam_tf.transform.rotation = quat;
    static_tf_broadcaster_.sendTransform(map_to_slam_tf);

    map_tf_init_ = true;
}

bool TaskManager::emergencyResponse(messages_88::Emergency::Request& req, messages_88::Emergency::Response& resp) {
    ROS_WARN("Emergency response initiated, level %d.", req.status.severity);
    // Immediately set to hover
    drone_state_manager_.setMode(loiter_mode_);
    cmd_history_.append("Emergency init with severity " + std::to_string(req.status.severity) + "\n");

    // Then respond based on severity 
    // (message definitions at http://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/StatusText.html)
    int level = req.status.severity;
    if (level == 5) {
        // NOTICE = PAUSE
        pauseOperations();
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
    return true;
}

bool TaskManager::pauseOperations() {
    actionlib::SimpleClientGoalState goal_state = explore_action_client_.getState();
    if (goal_state == actionlib::SimpleClientGoalState::ACTIVE) {
        explore_action_client_.cancelAllGoals();
    }
    if (explore_action_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
        return false;
    }
    return true;
}

bool TaskManager::initDroneStateManager(messages_88::InitDroneState::Request& req, messages_88::InitDroneState::Response& resp) {
    // Drone state manager setup
    drone_state_manager_.setAutonomyEnabled(req.enable_autonomy);
    task_msg_.enable_autonomy = req.enable_autonomy;
    task_msg_.enable_exploration = req.enable_exploration;

    drone_state_manager_.setSafetyArea();

    if (req.ardupilot) {
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
    return true;
}

bool TaskManager::getReadyForAction(messages_88::PrepareDrone::Request& req, messages_88::PrepareDrone::Response& resp) {
    cmd_history_.append("Get ready command received.\n ");
    getDroneReady();
    return true;
}

bool TaskManager::getReadyForExplore(messages_88::PrepareExplore::Request& req, messages_88::PrepareExplore::Response& resp) {
    cmd_history_.append("Get ready to explore command received.\n ");
    bool needs_transit = false;
    geometry_msgs::PoseStamped target_position;
    current_polygon_ = req.polygon;
    if (!isInside(current_polygon_, drone_state_manager_.getCurrentLocalPosition())) {
        cmd_history_.append("Transit to explore required.\n ");
        needs_transit = true;
        // Find nearest point on the polygon
        // Check polygon area and distance
        double min_dist;
        if (!polygonDistanceOk(min_dist, target_position, req.polygon)) {
            ROS_WARN("Polygon rejected, exceeds maximum starting distance threshold.");
            resp.success = false;
            return false;
        }
        else {
            ROS_DEBUG("Polygon received, distance ok.");
        }
        ROS_INFO("target position x,y was %f, %f", target_position.pose.position.x, target_position.pose.position.y);
        target_position.pose.position.z = req.altitude;
    }

    if (task_msg_.enable_autonomy) {
        getDroneReady();
    }

    boost::uuids::random_generator generator;
    boost::uuids::uuid u = generator();
    messages_88::ExploreGoal explore_goal;
    if (task_msg_.enable_autonomy && needs_transit) {
        messages_88::NavToPointGoal nav_goal;
        nav_goal.point = target_position.pose.position;
        nav_goal.uuid.data = boost::uuids::to_string(u);
        explore_goal.uuid.data = boost::uuids::to_string(u);
        startNav2PointTask(nav_goal);
    }
    else {
        explore_goal.uuid.data = "NO_TRANSIT";
    }
    explore_goal.polygon = req.polygon;
    explore_goal.altitude = req.altitude;
    explore_goal.min_altitude = req.min_altitude;
    explore_goal.max_altitude = req.max_altitude;
    if (task_msg_.enable_exploration) {
        explore_action_client_.waitForServer();
        sendExploreTask(explore_goal);
        cmd_history_.append("Sent explore goal.\n");
        resp.success = true;
    }
    return true;
}

bool TaskManager::getDronePosition(messages_88::GetPosition::Request& req, messages_88::GetPosition::Response& resp) {
    resp.global = drone_state_manager_.getCurrentGlobalPosition();
    resp.local = drone_state_manager_.getCurrentLocalPosition();
    return true;
}

void TaskManager::startNav2PointTask(messages_88::NavToPointGoal &nav_goal) {
    messages_88::NavToPointGoal goal = nav_goal;
    current_status_ = CurrentStatus::NAVIGATING;
    geometry_msgs::Point point = goal.point;
    current_target_ = point;
    geometry_msgs::PoseStamped target;
    target.pose.position = point;
    local_pos_pub_.publish(target);
    while (!(isInside(current_polygon_, drone_state_manager_.getCurrentLocalPosition()) || current_status_ == CurrentStatus::WAITING_TO_EXPLORE)) {
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
        save_msg.request.directory.data = directory_;
        vegetation_save_client_.call(save_msg);
        tree_save_client_.call(save_msg);
        did_save_ = true;
    }

    // TODO stop any active goals
}

void TaskManager::modeMonitor() {
    std::string mode = drone_state_manager_.getFlightMode();
    bool armed = drone_state_manager_.getIsArmed();
    bool in_air = drone_state_manager_.getIsInAir();
    if (!did_takeoff_ && in_air) {
        // Should have been set to true during takeoff, but just in case
        cmd_history_.append("Manually set takeoff to true. \n ");
        did_takeoff_ = true;
    }
    if (armed && !bag_active_) {
        cmd_history_.append("Checking start bag record due to arming detected. In air: " + std::to_string(in_air) + ", flight mode: " + mode + "\n");
        // Handle recording during manual take off
        startBag();
        did_save_ = false;
    }
    if (did_takeoff_ && !drone_state_manager_.getIsArmed()) {
        cmd_history_.append("Disarm detected. \n ");
        // Handle save bag during land (manual or auton)
        stop();
        did_takeoff_ = false; // Reset so can restart if another takeoff
    }
    task_msg_.header.stamp = ros::Time::now();
    task_msg_.cmd_history.data = cmd_history_.c_str();
    task_msg_.current_status.data = getStatusString();
    task_pub_.publish(task_msg_);
}

void TaskManager::startBag() {
    if (bag_active_) {
        return;
    }
    cmd_history_.append("Bag starting.\n ");
    bag_recorder::Rosbag start_bag_msg;
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
    geometry_msgs::Point my_position = drone_state_manager_.getCurrentLocalPosition();
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

std::string TaskManager::getStatusString() {
    switch (current_status_) {
        case 0:
            return "ON_START";
        case 1:
            return "EXPLORING";
        case 2:
            return "WAITING_TO_EXPLORE";
        case 3:
            return "HOVERING";
        case 4:
            return "NAVIGATING";
        case 5:
            return "TAKING_OFF";
        case 6:
            return "LANDING";
        default:
            return "unknown";
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
    // 1) MAVROS position
    json j = {
        {"name", "mavrosPosition"},
        {"label", "MAVROS position"},
        {"isHealthy", (t - last_mavros_pos_stamp_ < health_check_s_)}
    };
    healthObjects.push_back(j);
    // 2) SLAM position
    j = {
        {"name", "slamPosition"},
        {"label", "SLAM position"},
        {"isHealthy", (t - last_slam_pos_stamp_ < health_check_s_)}
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
    // 6) Explore
    j = {
        {"name", "explore"},
        {"label", "Explore Service"},
        {"isHealthy", explore_healthy}
    };
    healthObjects.push_back(j);
    jsonObjects["healthIndicators"] = healthObjects;

    std::string s = jsonObjects.dump();
    std_msgs::String health_string;
    health_string.data = s;
    health_pub_.publish(health_string);
}

void TaskManager::costmapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr &msg) {
    last_costmap_stamp_ = msg->header.stamp;
}

void TaskManager::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    last_lidar_stamp_ =  msg->header.stamp;
}

void TaskManager::mapirCallback(const sensor_msgs::ImageConstPtr &msg) {
    last_mapir_stamp_ = msg->header.stamp;
}

void TaskManager::rosbagCallback(const std_msgs::StringConstPtr &msg) {
    last_rosbag_stamp_ = ros::Time::now();
}

}