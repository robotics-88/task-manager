/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/task_manager.h"
#include "bag_recorder/Rosbag.h"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>  
#include <float.h>
#include <geometry_msgs/Twist.h>

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
    , do_record_(true)
    , bag_active_(false)
    , record_config_name_("r88_default")
    , drone_state_manager_(node)
    , max_dist_to_polygon_(300.0)
    , nav2point_action_server_(private_nh_, "nav2point", false)
    , explore_action_server_(private_nh_, "explore", false)
    , explore_action_client_("explore", true)
{
    std::string goal_topic = "/mavros/setpoint_position/local";
    private_nh_.param<std::string>("goal_topic", goal_topic, goal_topic);
    private_nh_.param<bool>("do_record", do_record_, do_record_);

    mode_monitor_timer_ = private_nh_.createTimer(ros::Duration(1.0),
                               [this](const ros::TimerEvent&) { modeMonitor(); });

    // Drone state services
    drone_state_service_ = nh_.advertiseService("/init_drone_state", &TaskManager::initDroneStateManager, this);
    drone_ready_service_ = nh_.advertiseService("/prep_drone_action", &TaskManager::getReadyForAction, this);
    drone_explore_service_ = nh_.advertiseService("/prep_drone_explore", &TaskManager::getReadyForExplore, this);
    drone_position_service_ = nh_.advertiseService("/get_drone_position", &TaskManager::getDronePosition, this);
    emergency_service_ = nh_.advertiseService("/emergency_response", &TaskManager::emergencyResponse, this);

    // Create action server (called by UI/Monarch)
    nav2point_action_server_.registerGoalCallback(boost::bind(&TaskManager::startNav2PointTask, this));
    nav2point_action_server_.registerPreemptCallback(boost::bind(&TaskManager::stop, this));
    nav2point_action_server_.start();

    // Create action server (called by UI/Monarch)
    explore_action_server_.registerGoalCallback(boost::bind(&TaskManager::receivedExploreTask, this));
    explore_action_server_.registerPreemptCallback(boost::bind(&TaskManager::stop, this));
    explore_action_server_.start();

    // MAVROS
    mavros_local_pos_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &TaskManager::localPositionCallback, this);
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    local_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    // Recording
    start_record_pub_ = nh_.advertise<bag_recorder::Rosbag>("/record/start", 5);
    stop_record_pub_ = nh_.advertise<std_msgs::String>("/record/stop", 5);
}

TaskManager::~TaskManager(){}

void TaskManager::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (current_status_ == CurrentStatus::NAVIGATING) {
        if (msg->pose.position == current_target_) {
            // Within a meter of target (TODO, add or already inside polygon: || isInside(current_explore_goal_.polygon, msg->pose.position)
            current_status_ = CurrentStatus::WAITING_TO_EXPLORE;
        }
    }
}

bool TaskManager::emergencyResponse(messages_88::Emergency::Request& req, messages_88::Emergency::Response& resp) {
    ROS_WARN("Emergency response initiated, level %d.", req.status.severity);
    // Immediately set to hover
    drone_state_manager_.setMode(loiter_mode_);

    // Then respond based on severity 
    // (message definitions at http://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/StatusText.html)
    int level = req.status.severity;
    if (level == 5) {
        // NOTICE = PAUSE
        drone_state_manager_.pauseOperations();
    }
    else if (level == 0) {
        // EMERGENCY = LAND IMMEDIATELY
        drone_state_manager_.pauseOperations();
        drone_state_manager_.setMode(land_mode_);
    }
    else if (level == 2) {
        // CRITICAL = RTL
        drone_state_manager_.pauseOperations();
        drone_state_manager_.setMode(rtl_mode_);
    }
    return true;
}

bool TaskManager::initDroneStateManager(messages_88::InitDroneState::Request& req, messages_88::InitDroneState::Response& resp) {
    // Drone state manager setup
    drone_state_manager_.setAutonomyEnabled(req.enable_autonomy);

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

bool TaskManager::getReadyForAction(messages_88::PrepareDroneAction::Request& req, messages_88::PrepareDroneAction::Response& resp) {
    getDroneReady();
    return true;
}

bool TaskManager::getReadyForExplore(messages_88::PrepareExplore::Request& req, messages_88::PrepareExplore::Response& resp) {

    bool needs_transit = false;
    geometry_msgs::PoseStamped target_position;
    if (!isInside(req.polygon, drone_state_manager_.getCurrentLocalPosition())) {
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

    getDroneReady();

    drone_state_manager_.actionClientsAvailable();
    boost::uuids::random_generator generator;
    boost::uuids::uuid u = generator();
    messages_88::ExploreGoal explore_goal;
    if (needs_transit) {
        messages_88::NavToPointGoal nav_goal;
        nav_goal.point = target_position.pose.position;
        nav_goal.uuid.data = boost::uuids::to_string(u);
        explore_goal.uuid.data = boost::uuids::to_string(u);
        drone_state_manager_.setTransitGoal(nav_goal);
    }
    else {
        explore_goal.uuid.data = "NO_TRANSIT";
    }
    explore_goal.polygon = req.polygon;
    explore_goal.altitude = req.altitude;
    explore_goal.min_altitude = req.min_altitude;
    explore_goal.max_altitude = req.max_altitude;
    drone_state_manager_.setExploreGoal(explore_goal);
    resp.success = true;
    return true;
}

bool TaskManager::getDronePosition(messages_88::GetPosition::Request& req, messages_88::GetPosition::Response& resp) {
    resp.global = drone_state_manager_.getCurrentGlobalPosition();
    resp.local = drone_state_manager_.getCurrentLocalPosition();
    return true;
}

void TaskManager::startNav2PointTask() {
    messages_88::NavToPointGoalConstPtr goal = nav2point_action_server_.acceptNewGoal();
    current_status_ = CurrentStatus::NAVIGATING;
    geometry_msgs::Point point = goal->point;
    current_target_ = point;
    geometry_msgs::PoseStamped target;
    target.pose.position = point;
    local_pos_pub_.publish(target);
}

void TaskManager::receivedExploreTask() {
    messages_88::ExploreGoalConstPtr goal = explore_action_server_.acceptNewGoal();
    current_explore_goal_ = *goal;
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
    // TODO stop any active goals
}

void TaskManager::modeMonitor() {
    std::string mode = drone_state_manager_.getFlightMode();
    bool in_air = drone_state_manager_.getIsInAir();
    if (in_air && !bag_active_ && mode != land_mode_) {
        // Handle recording during manual take off
        startBag();
    }
    if (bag_active_ && mode == land_mode_) {
        // Handle save bag during manual land
        stopBag();
    }
}

void TaskManager::startBag() {
    if (bag_active_) {
        return;
    }
    bag_recorder::Rosbag start_bag_msg;
    start_bag_msg.bag_name = "decco";
    start_bag_msg.config = record_config_name_;
    start_bag_msg.header.stamp = ros::Time::now();
    start_record_pub_.publish(start_bag_msg);
    bag_active_ = true;
}

void TaskManager::stopBag() {
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
        drone_state_manager_.getReadyForAction();
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

}