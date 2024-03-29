/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/drone_state_manager.h"
#include "messages_88/ExploreAction.h"
#include "messages_88/Battery.h"

#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/WaypointClear.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>  
#include <float.h>
namespace drone_state_manager
{
DroneStateManager::DroneStateManager(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
  , offline_(false)
  , simulate_(false)
  , do_slam_(false)
  , autonomy_active_(false)
  , enable_autonomy_(false)
  , enable_exploration_(false)
  , target_altitude_(2.0)
  , min_altitude_(2.0)
  , max_altitude_(10.0)
  , max_distance_(2.0)
  , ardupilot_(true)
  , mavros_global_pos_topic_("/mavros/global_position/global")
  , mavros_state_topic_("/mavros/state")
  , mavros_alt_topic_("/mavros/global_position/rel_alt")
  , arming_topic_("/mavros/cmd/arming")
  , set_mode_topic_("/mavros/set_mode")
  , takeoff_topic_("/mavros/cmd/takeoff")
  , compass_count_(0)
  , altitude_set_(false)
  , connected_(false)
  , armed_(false)
  , in_air_(false)
  , in_guided_mode_(false)
  , service_wait_duration_(2.0)
  , detected_utm_zone_(-1)
  , utm_set_(false)
  , battery_size_(5.2)
  , estimated_current_(20.0)
  , msg_rate_timer_dt_(5.0)
  , imu_rate_(120.0)
  , local_pos_rate_(60.0)
  , battery_rate_(10.0)
  , all_stream_rate_(5.0)
{
    // Set params from launch file 
    private_nh_.param<float>("default_altitude_m", target_altitude_, target_altitude_);
    private_nh_.param<float>("max_altitude", max_altitude_, max_altitude_);
    private_nh_.param<float>("max_distance", max_distance_, max_distance_);
    private_nh_.param<float>("battery_size", battery_size_, battery_size_);
    private_nh_.param<float>("estimated_current", estimated_current_, estimated_current_);
    private_nh_.param<std::string>("mavros_global_pos_topic", mavros_global_pos_topic_, mavros_global_pos_topic_);
    private_nh_.param<std::string>("mavros_state_topic", mavros_state_topic_, mavros_state_topic_);
    private_nh_.param<std::string>("mavros_arming_topic", arming_topic_, arming_topic_);
    private_nh_.param<std::string>("mavros_set_mode_topic", set_mode_topic_, set_mode_topic_);
    private_nh_.param<std::string>("mavros_takeoff_topic", takeoff_topic_, takeoff_topic_);
    private_nh_.param<std::string>("mavros_alt_topic", mavros_alt_topic_, mavros_alt_topic_);
    private_nh_.param<bool>("ardupilot", ardupilot_, ardupilot_);
    private_nh_.param<float>("imu_rate", imu_rate_, imu_rate_);
    private_nh_.param<float>("local_pos_rate", local_pos_rate_, local_pos_rate_);
    private_nh_.param<float>("all_stream_rate", all_stream_rate_, all_stream_rate_);
    private_nh_.param<bool>("offline", offline_, offline_);

    // Change arducopter param map if using slam pos src
    private_nh_.param<bool>("do_slam", do_slam_, do_slam_);
    if (do_slam_) {
        param_map_[ "EK3_SRC1_POSXY" ] = 6;
        param_map_[ "EK3_SRC1_VELXY" ] = 6;
        param_map_[ "EK3_SRC1_POSZ" ] = 6;
        param_map_[ "EK3_SRC1_VELZ" ] = 6;
        param_map_[ "EK3_SRC1_YAW" ] = 6;
        param_map_[ "VISO_TYPE" ] = 1;
    }
    
    // Add a stream rate modifier in simulation b/c arducopter loop rate is slow
    private_nh_.param<bool>("simulate", simulate_, simulate_);
    if (simulate_)
        stream_rate_modifier_ = 300.f / 222.f;
    else
        stream_rate_modifier_ = 1.f;

    safety_area_viz_ = nh_.advertise<geometry_msgs::PolygonStamped>("safety_box", 10);

    // Set subscribers for Mavros
    mavros_global_pos_subscriber_ = nh_.subscribe<sensor_msgs::NavSatFix>(mavros_global_pos_topic_, 10, &DroneStateManager::globalPositionCallback, this);
    mavros_local_pos_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &DroneStateManager::localPositionCallback, this);
    mavros_state_subscriber_ = nh_.subscribe<mavros_msgs::State>(mavros_state_topic_, 10, &DroneStateManager::statusCallback, this);
    mavros_alt_subscriber_ = nh_.subscribe<std_msgs::Float64>(mavros_alt_topic_, 10, &DroneStateManager::altitudeCallback, this);
    mavros_imu_subscriber_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &DroneStateManager::imuCallback, this);
    mavros_compass_subscriber_ = nh_.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, &DroneStateManager::compassCallback, this);
    mavros_battery_subscriber_ = nh_.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 10, &DroneStateManager::batteryCallback, this);

    // Slam pose subscriber
    slam_pose_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>("/decco/pose", 10, &DroneStateManager::slamPoseCallback, this);

    battery_pub_ = nh_.advertise<messages_88::Battery>("/decco/battery", 10);

    if (!offline_) {
        // Run initial mavlink stream request, just so we can get drone data immediately
        requestMavlinkStreams();

        attempts_ = 0;
        drone_init_timer_ = private_nh_.createTimer(ros::Duration(1.0), &DroneStateManager::initializeDrone, this);
        msg_rate_timer_ = private_nh_.createTimer(ros::Duration(msg_rate_timer_dt_), &DroneStateManager::checkMsgRates, this);
    }

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

    // fill recent current vector with starting estimated current from param
    for (int i = 0; i < 10; i++) {
        recent_currents_.push_back(estimated_current_);
    }
}

DroneStateManager::~DroneStateManager() {
    arming_client_.shutdown();
    set_mode_client_.shutdown();
    takeoff_client_.shutdown();
}

void DroneStateManager::initializeDrone(const ros::TimerEvent &event) {

    // Try setting a dummy param to check if param fetch has completed
    if (!param_fetch_complete_) {

        // Wait an extra long on 
        if (attempts_ == 0) {

            // Param fetch takes about 50 seconds in sim, 15 seconds on drone
            int approx_time_to_fetch = simulate_ ? 50 : 15;
            for (int i = 0; i < approx_time_to_fetch; i++) {
                ROS_INFO_THROTTLE(5, "Drone state manager waiting for param fetch to complete");
                ros::Duration(1.0).sleep();
            }
        }

        // Run the action
        auto param_set_client = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
        mavros_msgs::ParamSet param_set_srv;
        param_set_srv.request.param_id = "ACRO_RP_RATE"; // Use this as we don't care about acro mode stuff.
        param_set_srv.request.value.real = 360.0;
        param_set_client.call(param_set_srv);

        // Check success
        if (!param_set_srv.response.success) {
            if (attempts_ == 10) {
                ROS_ERROR("Setting parameter failed after 10 attempts");
                drone_init_timer_.stop();
            }
            ROS_WARN_THROTTLE(5, "Param set failed (param fetch may still be occuring)");

            attempts_++;
            return;
        }
        else {
            param_fetch_complete_ = true;
            attempts_ = 0;
            ROS_INFO("Param fetch complete");
            ROS_INFO("Drone state manager waiting for message rate checker to run");
            requestMavlinkStreams();
        }
    }

    if (!stream_rates_ok_) {
        
        // Don't continue if message rate checker hasn't run enough times
        if (check_msg_rates_counter_ < 2) {
            return;
        }

        // Check success
        if (!battery_rate_ok_ || !imu_rate_ok_) {
            if (attempts_ == 5) {
                ROS_ERROR("MAVlink stream rates failed after 5 attempts");
                drone_init_timer_.stop();
            }

            requestMavlinkStreams();
            attempts_++;
            return;
        }
        else {
            stream_rates_ok_ = true;
            attempts_ = 0;
            ROS_INFO("Mavlink streaming rates OK");
            ROS_INFO("Clearing Geofence");
        }
    }

    // Clear previous geofence
    if (!geofence_clear_ok_) {

        // Run the action
        auto geofence_clear_client = nh_.serviceClient<mavros_msgs::WaypointClear>("/mavros/geofence/clear");
        geofence_clear_client.waitForExistence();
        mavros_msgs::WaypointClear waypoint_clear_srv;
        geofence_clear_client.call(waypoint_clear_srv);

        // Check success
        if (!waypoint_clear_srv.response.success) {
            if (attempts_ == 3) {
                ROS_ERROR("Geofence clear failed after 3 attempts");
                drone_init_timer_.stop();
            }
            ROS_WARN("Geofence clear failed, trying again in 1s");
            attempts_++;
            return;
        }
        else {
            geofence_clear_ok_ = true;
            attempts_ = 0;
            ROS_INFO("Clearing mission");
        }
    }

    // Clear any existing mission (we don't use missions, this is just for safety)
    if (!mission_clear_ok_) {

        // Run the action
        auto mission_clear_client = nh_.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
        mission_clear_client.waitForExistence();
        mavros_msgs::WaypointClear waypoint_clear_srv;
        mission_clear_client.call(waypoint_clear_srv);

        // Check success
        if (!waypoint_clear_srv.response.success) {
            if (attempts_ == 3) {
                ROS_ERROR("Mission clear failed after 3 attempts");
                drone_init_timer_.stop();
            }
            ROS_WARN("Mission clear failed, trying again in 1s");
            attempts_++;
            return;
        }
        else {
            mission_clear_ok_ = true;
            attempts_ = 0;
            ROS_INFO("Setting Arducopter heading source to Compass");
        }
    }

    // Set heading source to compass. Also acts as check on whether parameter fetch is complete
    if (!heading_src_ok_) {
            
        // Run the action
        auto param_set_client = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
        param_set_client.waitForExistence();
        mavros_msgs::ParamSet param_set_srv;
        param_set_srv.request.param_id = "EK3_SRC1_YAW";
        param_set_srv.request.value.integer = 1; // 1 = Compass, 6 = ExternalNav
        param_set_client.call(param_set_srv);

        // Check success
        if (!param_set_srv.response.success) {
            if (attempts_ == 3) {
                ROS_ERROR("EKF heading source param set failed after 3 attempts");
                drone_init_timer_.stop();
            }
            ROS_WARN("EKF heading source param set failed");
            attempts_++;
            return;
        }
        else {
            heading_src_ok_ = true;
            attempts_ = 0;
            ROS_INFO("Getting initial compass heading");
        }
    }


    // Get current compass heading, and set it as 'home' compass heading
    // I.e. the compass heading of the drone when the ROS code is started
    if (!compass_init_ok_) {

        // Wait 3 ticks after setting heading source to Compass before gathering compass
        if (compass_wait_counter_ < 3) {
            compass_wait_counter_++;
            return;
        }

        // Check success
        if (!compass_received_) {
            if (attempts_ == 3) {
                ROS_ERROR("Compass orientation not received after 3 attempts");
                drone_init_timer_.stop();
            }
            ROS_WARN("Compass orientation not yet received, trying again in 1s");
            attempts_++;
            return;
        }
        else {
            compass_init_ok_ = true;
            home_compass_hdg_ = compass_hdg_;
            attempts_ = 0;
            ROS_INFO("Initial compass heading: %f", home_compass_hdg_);
            ROS_INFO("Setting Arducopter params");
        }
    }

    // Set arducopter params
    if (!param_set_ok_) {

        // Run the action
        auto param_set_client = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
        mavros_msgs::ParamSet param_set_srv;

        std::map<std::string, int>::iterator it;

        for (it = param_map_.begin(); it != param_map_.end(); it++) {
            param_set_srv.request.param_id = it->first;
            param_set_srv.request.value.integer = it->second;
            param_set_client.call(param_set_srv);

            // Check success
            if (!param_set_srv.response.success) {
                if (attempts_ == 3) {
                    ROS_ERROR("Param set of param %s failed after 3 attempts", it->first);
                    drone_init_timer_.stop();
                }
                ROS_WARN("Param %s set failed, trying again in 1s", it->first);
                attempts_++;
                return;
            }
        }

        param_set_ok_ = true;
        attempts_ = 0;
    }

    ROS_INFO("Drone initialization successful!");
    drone_initialized_ = true;
    drone_init_timer_.stop();

}

void DroneStateManager::requestMavlinkStreams() {

    ROS_INFO_THROTTLE(5, "Requesting MAVLink streams from autopilot");

    // Request all streams
    auto streamrate_client = nh_.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    streamrate_client.waitForExistence();
    mavros_msgs::StreamRate streamrate_srv;
    streamrate_srv.request.stream_id = 0;
    streamrate_srv.request.message_rate = all_stream_rate_ * stream_rate_modifier_;
    streamrate_srv.request.on_off = 1;
    streamrate_client.call(streamrate_srv);

    // Request specific streams at particular rates
    auto msg_interval_client = nh_.serviceClient<mavros_msgs::MessageInterval>("/mavros/set_message_interval");
    msg_interval_client.waitForExistence();
    mavros_msgs::MessageInterval msg_interval_srv;

    msg_interval_srv.request.message_id = 30; // ATTITUDE
    msg_interval_srv.request.message_rate = imu_rate_ * stream_rate_modifier_;
    msg_interval_client.call(msg_interval_srv);

    msg_interval_srv.request.message_id = 32; // LOCAL_POSITION_NED
    msg_interval_srv.request.message_rate = local_pos_rate_ * stream_rate_modifier_;
    msg_interval_client.call(msg_interval_srv);

    msg_interval_srv.request.message_id = 147; // BATTERY_STATUS
    msg_interval_srv.request.message_rate = battery_rate_ * stream_rate_modifier_;
    msg_interval_client.call(msg_interval_srv);
}

void DroneStateManager::initUTM(double &utm_x, double &utm_y) {
    GeographicLib::GeoCoords c(current_ll_.latitude, current_ll_.longitude);
    utm_x = c.Easting();
    utm_y = c.Northing();
}

void DroneStateManager::checkMsgRates(const ros::TimerEvent &event) {

    // Don't run until param fetch is complete (avoids annoying warnings)
    if (!param_fetch_complete_) {
        return;
    }

    check_msg_rates_counter_++;

    if (imu_count_ / msg_rate_timer_dt_ < imu_rate_ * 0.8) {
        ROS_WARN("Warning, IMU only sending at %f / %f hz", (imu_count_ / msg_rate_timer_dt_), imu_rate_);
        imu_rate_ok_ = false;
    }
    else {
        imu_rate_ok_ = true;
    }

    // Use battery message as proxy for all generic message streams
    if (battery_count_ / msg_rate_timer_dt_ < battery_rate_ * 0.8) {
        ROS_WARN("Warning, battery only sending at %f / %f hz", (battery_count_ / msg_rate_timer_dt_), battery_rate_);
        battery_rate_ok_ = false;
    }
    else {
        battery_rate_ok_ = true;
    }

    // Also check local pos rate, but don't use this for initialization check b/c drone needs to initialize before
    // vision pose starts publishing (which ultimately, local position comes from)
    if (local_pos_count_ / msg_rate_timer_dt_ <  8) {
        ROS_WARN("Warning, local position only sending at %f / 10 hz", (local_pos_count_ / msg_rate_timer_dt_));
    }

    // Reset counters
    imu_count_ = 0;
    local_pos_count_ = 0;
    battery_count_ = 0;
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

geometry_msgs::PoseStamped DroneStateManager::getCurrentSlamPosition() {
    return current_slam_pose_;
}

geometry_msgs::PoseStamped DroneStateManager::getCurrentLocalPosition() {
    return current_pose_;
}

sensor_msgs::NavSatFix DroneStateManager::getCurrentGlobalPosition() {
    return current_ll_;
}

void DroneStateManager::waitForGlobal() {
    ros::topic::waitForMessage<sensor_msgs::NavSatFix>(mavros_global_pos_topic_, nh_);
}

int DroneStateManager::getUTMZone() {
    return detected_utm_zone_;
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

bool DroneStateManager::getIsArmed() {
    return armed_;
}

bool DroneStateManager::getMapYaw(double &yaw) {
    if (compass_init_ok_) {
        yaw = home_compass_hdg_;
    }
    return compass_init_ok_;
}

double DroneStateManager::getCompass() {
    return compass_hdg_;
}

void DroneStateManager::slamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_slam_pose_ = *msg;
}

void DroneStateManager::globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_ll_ = *msg;
    // Check UTM zone
    double lat = msg->latitude, lon = msg->longitude;
    detected_utm_zone_ = GeographicLib::UTMUPS::StandardZone(lat, lon);
}

void DroneStateManager::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose_ = *msg;
    local_pos_count_++;
}

void DroneStateManager::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    current_imu_ = *msg;
    imu_count_++;
}

void DroneStateManager::compassCallback(const std_msgs::Float64::ConstPtr & msg) {
    compass_hdg_ = msg->data;
    compass_received_ = true;
}

void DroneStateManager::batteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg) {
    current_battery_ = *msg;
    battery_count_++;

    float current = -msg->current; // Mavros current is negative

    // If current is low, we can estimate battery percentage from voltage. 
    // We use this 'last resting percent' as the starting point for calculations for 
    // remaining battery life. 
    if (current < 1.f) {
        last_resting_percent_ = calculateBatteryPercentage(msg->voltage);
        last_resting_percent_time_ = ros::Time::now();
        current_drawn_since_resting_percent_ = 0.f;
    }

    current_drawn_since_resting_percent_ += current * (ros::Time::now() - last_battery_measurement_).toSec() / 3600;
    last_battery_measurement_ = ros::Time::now();

    float battery_percent_drawn_since_resting_ = current_drawn_since_resting_percent_ / battery_size_ * 100.f;
    battery_percentage_ = last_resting_percent_ - battery_percent_drawn_since_resting_;
    float amp_hours_left = battery_size_ * battery_percentage_ / 100.f;


    // Use current estimate and remaining amp hours to determine how many seconds of flight time we have left
    // If current is below 5A, it is likely not accurate in reflecting hover current, so don't update current calculation
    if (current > 5.0) {
        recent_currents_.push_back(current);
        recent_currents_.erase(recent_currents_.begin());
    }

    // Calculate estimated current based on average of recent current measurements
    if (recent_currents_.size() > 0) {
        float sum = 0.f;
        for (auto i : recent_currents_) {
            sum += i;
        }

        estimated_current_ = sum / recent_currents_.size();
    }

    estimated_flight_time_remaining_ = amp_hours_left / estimated_current_ * 3600;

    // Publish custom battery message
    messages_88::Battery batt_msg;
    batt_msg.header.stamp = ros::Time::now();
    batt_msg.percentage = battery_percentage_;
    batt_msg.estimated_current = estimated_current_;
    batt_msg.amp_hours_left = amp_hours_left;
    batt_msg.flight_time_remaining = estimated_flight_time_remaining_;

    battery_pub_.publish(batt_msg);
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


float DroneStateManager::calculateBatteryPercentage(float voltage) {

    float battery_percentage = 0.f;

    // Use voltage per cell as that is more flexible. Currently we can just hardcode 6 cell
    voltage = voltage / 6.f;

    // Battery voltage to battery percentage conversion is not linear. 
    // Use this piecewise function based on gathered data to estimate battery percentage.
    if (voltage >= 4.2) {
        battery_percentage = 100.0;
    }
    else if (voltage >= 3.85) {
        // For this voltage range:
        // voltage ~= 0.7*(percentage) + 3.5, so
        battery_percentage = (voltage - 3.5) / 0.7 * 100;
    }
    else if (voltage >= 3.7) {
        // For this voltage range:
        // voltage ~= 0.333*(percentage) + 3.683
        battery_percentage = (voltage - 3.683) / 0.333 * 100;
    }
    else if (voltage >= 3.5) {
        // For this voltage range:
        // voltage ~= 4*(percentage) + 3.5
        battery_percentage = (voltage - 3.5) / 4 * 100;
    }
    else {
        battery_percentage = 0.0;
    }

    return battery_percentage;
}


// These are old methods of getting battery percentage. They aren't currently used but I don't want to delete them just yet.

// void DroneStateManager::calculateBatteryPercentage(float voltage_adj) {

//     // Battery voltage to battery percentage conversion is not linear. 
//     // Use this piecewise function based on gathered data to estimate battery percentage.
//     if (voltage_adj >= 25.2) {
//         battery_percentage_ = 100.0;
//     }
//     else if (voltage_adj >= 22.95) {
//         // For this voltage range:
//         // voltage ~= 2.9*(1-percentage)^2 - 5.95*(1-percentage) + 25.2
//         // To get percentage, use quadratic formula. 
//         // 0 = 2.9*(1-percentage)^2 - 5.95*(1-percentage) + 25.2 - voltage
//         float a = 2.9;
//         float b = -5.95;
//         float c = 25.2 - voltage_adj;
//         battery_percentage_ = (1 - findValidRoot(a, b, c)) * 100;
//     }
//     else if (voltage_adj >= 22.213) {
//         // For this voltage range:
//         // voltage ~= -1.733*(1-percentage)+23.816
//         battery_percentage_ = (voltage_adj - 22.083) / 1.733 * 100;
//     }
//     else if (voltage_adj >= 21) {
//         // For this voltage range:
//         // voltage ~= -169.068 * (1-percentage)2 + 309.282(1-percentage)-119.214
//         // Solve as before
//         float a = -169.068;
//         float b = 309.282;
//         float c = -119.214 - voltage_adj;
//         battery_percentage_ = (1 - findValidRoot(a, b, c)) * 100;
//     }
//     else {
//         battery_percentage_ = 0.0;
//     }
// }

// float DroneStateManager::findValidRoot(float a, float b, float c) {
//     float discriminant = b*b - 4*a*c;
//     float x1, x2;
    
//     if (discriminant < 0) {
//         ROS_WARN("Invalid battery calculation");
//         return 0.0;
//     }
//     else if (discriminant > 0) {
//         // Note: this is only one of the 2 real roots.
//         // However, this one is the correct one for the possible input values
//         return (-b - sqrt(discriminant)) / (2*a);
//     }
//     else if (discriminant == 0) {
//         return -b/(2*a);
//     }
// }

}