/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/drone_state_manager.h"
#include "messages_88/ExploreAction.h"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <GeographicLib/PolygonArea.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>  
#include <float.h>

namespace drone_state_manager
{
DroneStateManager::DroneStateManager(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
  , autonomy_active_(false)
  , enable_autonomy_(false)
  , enable_exploration_(false)
  , target_altitude_(2.0)
  , min_altitude_(2.0)
  , max_altitude_(10.0)
  , max_distance_(2.0)
  , ardupilot_(true)
  , explore_action_client_("/task_manager/explore", true)
  , navigate_action_client_("/task_manager/nav2point", true)
  , mavros_global_pos_topic_("/mavros/global_position/global")
  , mavros_state_topic_("/mavros/state")
  , mavros_alt_topic_("/mavros/global_position/rel_alt")
  , arming_topic_("/mavros/cmd/arming")
  , set_mode_topic_("/mavros/set_mode")
  , takeoff_topic_("/mavros/cmd/takeoff")
  , altitude_set_(false)
  , connected_(false)
  , armed_(false)
  , in_air_(false)
  , in_guided_mode_(false)
  , service_wait_duration_(2.0)
{
    // Set params from launch file 
    private_nh_.param<float>("default_altitude_m", target_altitude_, target_altitude_);
    private_nh_.param<float>("max_altitude", max_altitude_, max_altitude_);
    private_nh_.param<float>("max_distance", max_distance_, max_distance_);
    private_nh_.param<std::string>("mavros_global_pos_topic", mavros_global_pos_topic_, mavros_global_pos_topic_);
    private_nh_.param<std::string>("mavros_state_topic", mavros_state_topic_, mavros_state_topic_);
    private_nh_.param<std::string>("mavros_arming_topic", arming_topic_, arming_topic_);
    private_nh_.param<std::string>("mavros_set_mode_topic", set_mode_topic_, set_mode_topic_);
    private_nh_.param<std::string>("mavros_takeoff_topic", takeoff_topic_, takeoff_topic_);
    private_nh_.param<std::string>("mavros_alt_topic", mavros_alt_topic_, mavros_alt_topic_);
    private_nh_.param<bool>("ardupilot", ardupilot_, ardupilot_);

    safety_area_viz_ = nh_.advertise<geometry_msgs::PolygonStamped>("safety_box", 10);

    // Set subscribers for Mavros
    mavros_global_pos_subscriber_ = nh_.subscribe<sensor_msgs::NavSatFix>(mavros_global_pos_topic_, 10, &DroneStateManager::globalPositionCallback, this);
    mavros_local_pos_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &DroneStateManager::localPositionCallback, this);
    mavros_state_subscriber_ = nh_.subscribe<mavros_msgs::State>(mavros_state_topic_, 10, &DroneStateManager::statusCallback, this);
    mavros_alt_subscriber_ = nh_.subscribe<std_msgs::Float64>(mavros_alt_topic_, 10, &DroneStateManager::altitudeCallback, this);

    setSafetyArea();
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

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
        setMode("POSCTL");
    }
}

DroneStateManager::~DroneStateManager() {
    arming_client_.shutdown();
    set_mode_client_.shutdown();
    takeoff_client_.shutdown();
}

void DroneStateManager::setAutonomyEnabled(bool enabled) {
    enable_autonomy_ = enabled;
    if (enable_autonomy_) {
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_topic_);
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_topic_);
        takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>(takeoff_topic_);
    }
    else {
        arming_client_.shutdown();
        set_mode_client_.shutdown();
        takeoff_client_.shutdown();
    }
}

void DroneStateManager::setExplorationEnabled(bool enabled) {
    enable_exploration_ = enabled;
}

geometry_msgs::Point DroneStateManager::getCurrentLocalPosition() {
    return current_pose_.pose.position;
}

sensor_msgs::NavSatFix DroneStateManager::getCurrentGlobalPosition() {
    return current_ll_;
}

double DroneStateManager::getAltitudeAGL() {
    if (altitude_set_) {
        return current_altitude_;
    }
    else {
        return -1;
    }
}

std::string DroneStateManager::getFlightMode() {
    return current_mode_;
}

bool DroneStateManager::getIsInAir() {
    return in_air_;
}

bool DroneStateManager::getAutonomyActive() {
    return autonomy_active_;
}

void DroneStateManager::globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_ll_ = *msg;
}

void DroneStateManager::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose_ = *msg;
}

void DroneStateManager::statusCallback(const mavros_msgs::State::ConstPtr & msg) {
    // TODO add system status with enum matching MAV_STATE defined by Mavlink
    connected_ = msg->connected;
    armed_ = msg->armed;
    current_mode_ = msg->mode;
    in_guided_mode_ = current_mode_ == guided_mode_;
}

void DroneStateManager::altitudeCallback(const std_msgs::Float64::ConstPtr & msg) {
    in_air_ = msg->data > 1.0 && armed_;
    current_altitude_ = msg->data;
    if (!altitude_set_) {
        altitude_set_ = true;
    }
}

bool DroneStateManager::setGuided() {
    if (in_guided_mode_) {
        return true;
    }
    if (!enable_autonomy_) {
        ROS_WARN("Autonomy disabled in setGuided.");
        return false;
    }
    if (!ardupilot_) {
        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20.0);
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;

        // PX4 does not accept a switch to offboard mode unless points are already streaming
        for(int i = 100; ros::ok() && i > 0; --i){
            local_pos_pub_.publish(pose);
            rate.sleep();
        }
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = guided_mode_;

    set_mode_client_.waitForExistence(service_wait_duration_);
    if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
        autonomy_active_ = true;
        return true;
    }
    else {
        ROS_WARN("Guided mode failed.");
        autonomy_active_ = false;
        return false;
    }

}

bool DroneStateManager::setMode(std::string mode) {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = mode;
    set_mode_client_.waitForExistence(service_wait_duration_);
    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("Mode enabled: %s", mode.c_str());
        return true;
    }
    else {
        ROS_WARN("Mode failed: %s", mode.c_str());
        return false;
    }
}

bool DroneStateManager::arm() {
    if (armed_) {
        return true;
    }
    if (!enable_autonomy_) {
        ROS_WARN("Autonomy disabled in arming.");
        return false;
    }
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arming_client_.waitForExistence(service_wait_duration_);
    if( arming_client_.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
        return true;
    }
    else {
        ROS_INFO("Arming failed");
        return false;
    }
}

bool DroneStateManager::takeOff() {
    if (in_air_) {
        ROS_WARN("Takeoff command received while in air.");
        return true;
    }
    if (!enable_autonomy_) {
        ROS_WARN("Autonomy disabled in takeoff.");
        return false;
    }
    if (!in_guided_mode_) {
        setGuided();
    }
    if (!armed_) {
        arm();
    }

    mavros_msgs::CommandTOL takeoff_request;
    takeoff_request.request.altitude = target_altitude_;
    takeoff_client_.waitForExistence(service_wait_duration_);
    int attempts = 0;
    while (!in_air_ && !takeoff_request.response.success && attempts < 10)
    {
        ros::Duration(.1).sleep();
        takeoff_client_.call(takeoff_request);
        ros::spinOnce();
        attempts++;
    }
    if (!(in_air_ || takeoff_request.response.success)) {
        return false;
    }
    return true;
}

bool DroneStateManager::pauseOperations() {
    actionlib::SimpleClientGoalState goal_state = explore_action_client_.getState();
    if (goal_state == actionlib::SimpleClientGoalState::ACTIVE) {
        explore_action_client_.cancelAllGoals();
    }
    if (explore_action_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
        return false;
    }
    return true;
}

bool DroneStateManager::setTransitGoal(messages_88::NavToPointGoal nav_goal) {
    // TODO add valid checking
    navigate_action_client_.sendGoal(nav_goal);
    return true;
}

bool DroneStateManager::setExploreGoal(messages_88::ExploreGoal explore_goal) {
    // TODO add valid checking
    explore_action_client_.sendGoal(explore_goal);
    return true;
}

bool DroneStateManager::actionClientsAvailable() {
    explore_action_client_.waitForServer(ros::Duration(10.0));
    navigate_action_client_.waitForServer(ros::Duration(10.0));
    if (explore_action_client_.isServerConnected() && navigate_action_client_.isServerConnected()) {
        return true;
    }
    return false;
}

bool DroneStateManager::readyForAction() {
    return (connected_ && armed_ && in_air_ && in_guided_mode_);
}

bool DroneStateManager::getReadyForAction() {
    int attempts = 0;
    while (!altitude_set_ && attempts < 20) {
        ros::spinOnce();
        attempts++;
    }
    if (in_air_) {
        return true;
    }
    if (!enable_autonomy_) {
        ROS_WARN("Autonomy disabled in getReady.");
        return false;;
    }
    // TODO: setSafetyArea();
    bool guided = false, armed = false, takeoff = false;
    if (!in_guided_mode_) {
        ROS_INFO("setting guided mode to %s", guided_mode_.c_str());
        guided = setGuided();
        ros::spinOnce();
    }
    if (!armed_) {
        ROS_INFO("arming");
        armed = arm();
        ros::spinOnce();
    }
    if (!in_air_) {
        ros::spinOnce();
        ros::Duration(5).sleep();
        takeoff = takeOff();
        ros::spinOnce();
        ros::Duration(10).sleep();
    }
    return guided && armed && (in_air_ || takeoff);
}

bool DroneStateManager::setSafetyArea() {
    // TODO set MAVROS bounding polygon based on exploration region
}

}