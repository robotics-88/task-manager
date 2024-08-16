/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/flight_controller_interface.h"
#include "messages_88/action/explore.hpp"
#include "messages_88/msg/battery.hpp"

#include "mavros_msgs/srv/message_interval.hpp"
#include "mavros_msgs/srv/param_set.hpp"
#include "mavros_msgs/srv/stream_rate.hpp"
#include "mavros_msgs/srv/waypoint_clear.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "mavconn/mavlink_dialect.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace flight_controller_interface
{
FlightControllerInterface::FlightControllerInterface(const std::shared_ptr<rclcpp::Node> nh)
  : nh_(nh)
  , offline_(false)
  , simulate_(false)
  , do_slam_(false)
  , enable_autonomy_(false)
  , ardupilot_(true)
  , connected_(false)
  , armed_(false)
  , in_air_(false)
  , in_guided_mode_(false)
  , compass_received_(false)
  , current_altitude_(-1.0)
  , target_altitude_(2.0)
  , detected_utm_zone_(-1)
  , utm_set_(false)
  , battery_percentage_(0.0)
  , battery_voltage_(0.0)
  , battery_size_(5.2)
  , estimated_current_(20.0)
  , estimated_flight_time_remaining_(0.0)
  , imu_averaging_n_(50)
  , compass_count_(0)
  , imu_count_(0)
  , local_pos_count_(0)
  , battery_count_(0)
  , imu_rate_ok_(false)
  , battery_rate_ok_(false)
  , check_msg_rates_counter_(0)
  , compass_wait_counter_(0)
  , msg_rate_timer_dt_(5.0)
  , imu_rate_(120.0)
  , local_pos_rate_(60.0)
  , battery_rate_(10.0)
  , all_stream_rate_(5.0)
  , param_fetch_complete_(false)
  , heading_src_ok_(false)
  , stream_rates_ok_(false)
  , geofence_clear_ok_(false)
  , mission_clear_ok_(false)
  , compass_init_ok_(false)
  , param_set_ok_(false)
  , drone_initialized_(false)
{
    // Set params from launch file 
    nh_->declare_parameter("default_alt", target_altitude_);
    nh_->declare_parameter("battery_size", battery_size_);
    nh_->declare_parameter("estimated_current", estimated_current_);
    nh_->declare_parameter("ardupilot", ardupilot_);
    nh_->declare_parameter("imu_rate", imu_rate_);
    nh_->declare_parameter("local_pos_rate", local_pos_rate_);
    nh_->declare_parameter("all_stream_rate", all_stream_rate_);
    nh_->declare_parameter("offline", offline_);

    // Change arducopter param map if using slam pos src
    nh_->declare_parameter("do_slam", do_slam_);
    if (do_slam_) {
        param_map_[ "EK3_SRC1_POSXY" ] = 6;
        param_map_[ "EK3_SRC1_VELXY" ] = 6;
        param_map_[ "EK3_SRC1_POSZ" ] = 6;
        param_map_[ "EK3_SRC1_VELZ" ] = 6;
        param_map_[ "EK3_SRC1_YAW" ] = 6;
        param_map_[ "VISO_TYPE" ] = 1;
    }
    
    // Add a stream rate modifier in simulation b/c arducopter loop rate is slow
    nh_->declare_parameter("simulate", simulate_);
    if (simulate_)
        stream_rate_modifier_ = 300.f / 222.f;
    else
        stream_rate_modifier_ = 1.f;

    // Set subscribers for Mavros
    mavros_global_pos_subscriber_ = nh_->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", 10, std::bind(&FlightControllerInterface::globalPositionCallback, this, _1));
    mavros_local_pos_subscriber_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", 10, std::bind(&FlightControllerInterface::localPositionCallback, this, _1));
    mavros_state_subscriber_ = nh_->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&FlightControllerInterface::statusCallback, this, _1));
    mavros_alt_subscriber_ = nh_->create_subscription<std_msgs::msg::Float64>("/mavros/global_position/rel_alt", 10, std::bind(&FlightControllerInterface::altitudeCallback, this, _1));
    mavros_imu_subscriber_ = nh_->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data", 10, std::bind(&FlightControllerInterface::imuCallback, this, _1));
    mavros_compass_subscriber_ = nh_->create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg", 10, std::bind(&FlightControllerInterface::compassCallback, this, _1));
    mavros_battery_subscriber_ = nh_->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", 10, std::bind(&FlightControllerInterface::batteryCallback, this, _1));
    // mavros_sys_status_subscriber_ = nh_->create_subscription<mavros_msgs::msg::SysStatus>("/mavros/sys_status", 10, std::bind(&FlightControllerInterface::sysStatusCallback, this, _1));
    mavros_status_text_subscriber_ = nh_->create_subscription<mavros_msgs::msg::StatusText>("/mavros/statustext/recv", 10, std::bind(&FlightControllerInterface::statusTextCallback, this, _1));

    // Slam pose subscriber
    slam_pose_subscriber_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>("/decco/pose", 10, std::bind(&FlightControllerInterface::slamPoseCallback, this, _1));

    battery_pub_ = nh_->create_publisher<messages_88::msg::Battery>("/decco/battery", 10);

    if (!offline_) {
        // Run initial mavlink stream request, just so we can get drone data immediately
        requestMavlinkStreams();

        attempts_ = 0;
        drone_init_timer_ = nh_->create_wall_timer(1s, std::bind(&FlightControllerInterface::initializeDrone, this));
        msg_rate_timer_ = nh_->create_wall_timer(std::chrono::duration<float>(msg_rate_timer_dt_), std::bind(&FlightControllerInterface::checkMsgRates, this));
    }

    land_mode_ = "LAND";
    brake_mode_ = "BRAKE";
    rtl_mode_ = "RTL";
    guided_mode_ = "GUIDED";

    // fill recent current vector with starting estimated current from param
    for (unsigned i = 0; i < 10; i++) {
        recent_currents_.push_back(estimated_current_);
    }

    // Initialize mavros IMU to 0s
    mavros_imu_init_.orientation.x = 0;
    mavros_imu_init_.orientation.y = 0;
    mavros_imu_init_.orientation.z = 0;
    mavros_imu_init_.orientation.w = 0;
}

FlightControllerInterface::~FlightControllerInterface() {
    arming_client_.reset();
    set_mode_client_.reset();
    takeoff_client_.reset();
}

void FlightControllerInterface::initializeDrone() {

    // Try setting a dummy param to check if param fetch has completed
    if (!param_fetch_complete_) {

        
        // Wait an extra long on first run
        if (attempts_ == 0) {
            RCLCPP_INFO(nh_->get_logger(), "Drone state manager waiting for param fetch to complete");
            // Param fetch takes about 50 seconds in sim, 15 seconds on drone
            double approx_time_to_fetch = simulate_ ? 50 : 15;
            rclcpp::Rate(approx_time_to_fetch).sleep();
        }

        // Run the action
        auto param_set_client = nh_->create_client<mavros_msgs::srv::ParamSet>("/mavros/param/set");
        auto param_set_req = std::make_shared<mavros_msgs::srv::ParamSet::Request>();
        param_set_req->param_id = "ACRO_RP_RATE"; // Use nh_ as we don't care about acro mode stuff.
        param_set_req->value.real = 360.0;
        auto result = param_set_client->async_send_request(param_set_req);

        // Check success
        if (rclcpp::spin_until_future_complete(nh_, result) !=
            rclcpp::FutureReturnCode::SUCCESS
            || !result.get()->success) {
            if (attempts_ == 10) {
                RCLCPP_ERROR(nh_->get_logger(), "Setting parameter failed after 10 attempts");
                drone_init_timer_.reset();
            }
            RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000, "Param set failed (param fetch may still be occuring)");

            attempts_++;
            return;
        }
        else {
            param_fetch_complete_ = true;
            attempts_ = 0;
            RCLCPP_INFO(nh_->get_logger(), "Param fetch complete");
            RCLCPP_INFO(nh_->get_logger(), "Drone state manager waiting for message rate checker to run");
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
                RCLCPP_ERROR(nh_->get_logger(), "MAVlink stream rates failed after 5 attempts");
                drone_init_timer_.reset();
            }

            requestMavlinkStreams();
            attempts_++;
            return;
        }
        else {
            stream_rates_ok_ = true;
            attempts_ = 0;
            RCLCPP_INFO(nh_->get_logger(), "Mavlink streaming rates OK");
            RCLCPP_INFO(nh_->get_logger(), "Clearing Geofence");
        }
    }

    // Clear previous geofence
    if (!geofence_clear_ok_) {

        // Run the action
        auto geofence_clear_client = nh_->create_client<mavros_msgs::srv::WaypointClear>("/mavros/geofence/clear");
        auto waypoint_clear_req = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
        auto result = geofence_clear_client->async_send_request(waypoint_clear_req);

        // Check success
        if (rclcpp::spin_until_future_complete(nh_, result) !=
            rclcpp::FutureReturnCode::SUCCESS
            || !result.get()->success) {
            if (attempts_ == 3) {
                RCLCPP_ERROR(nh_->get_logger(), "Geofence clear failed after 3 attempts");
                drone_init_timer_.reset();
            }
            RCLCPP_WARN(nh_->get_logger(), "Geofence clear failed, trying again in 1s");
            attempts_++;
            return;
        }
        else {
            geofence_clear_ok_ = true;
            attempts_ = 0;
            RCLCPP_INFO(nh_->get_logger(), "Clearing mission");
        }
    }

    // Clear any existing mission (we don't use missions, nh_ is just for safety)
    if (!mission_clear_ok_) {

        // Run the action
        auto mission_clear_client = nh_->create_client<mavros_msgs::srv::WaypointClear>("/mavros/mission/clear");
        auto waypoint_clear_req = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
        auto result = mission_clear_client->async_send_request(waypoint_clear_req);

        // Check success
        if (rclcpp::spin_until_future_complete(nh_, result) !=
            rclcpp::FutureReturnCode::SUCCESS
            || !result.get()->success) {
            if (attempts_ == 3) {
                RCLCPP_ERROR(nh_->get_logger(), "Mission clear failed after 3 attempts");
                drone_init_timer_.reset();
            }
            RCLCPP_WARN(nh_->get_logger(), "Mission clear failed, trying again in 1s");
            attempts_++;
            return;
        }
        else {
            mission_clear_ok_ = true;
            attempts_ = 0;
            RCLCPP_INFO(nh_->get_logger(), "Setting Arducopter heading source to Compass");
        }
    }

    // Set heading source to compass. Also acts as check on whether parameter fetch is complete
    if (!heading_src_ok_) {
            
        // Run the action
        auto param_set_client = nh_->create_client<mavros_msgs::srv::ParamSet>("/mavros/param/set");
        auto param_set_req = std::make_shared<mavros_msgs::srv::ParamSet::Request>(); 
        param_set_req->param_id = "EK3_SRC1_YAW";
        param_set_req->value.integer = 1; // 1 = Compass, 6 = ExternalNav
        auto result = param_set_client->async_send_request(param_set_req);

        // Check success
        if (rclcpp::spin_until_future_complete(nh_, result) !=
            rclcpp::FutureReturnCode::SUCCESS
            || !result.get()->success) {
            if (attempts_ == 3) {
                RCLCPP_ERROR(nh_->get_logger(), "EKF heading source param set failed after 3 attempts");
                drone_init_timer_.reset();
            }
            RCLCPP_WARN(nh_->get_logger(), "EKF heading source param set failed");
            attempts_++;
            return;
        }
        else {
            heading_src_ok_ = true;
            attempts_ = 0;
            RCLCPP_INFO(nh_->get_logger(), "Getting initial compass heading");
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
                RCLCPP_ERROR(nh_->get_logger(), "Compass orientation not received after 3 attempts");
                drone_init_timer_.reset();
            }
            RCLCPP_WARN(nh_->get_logger(), "Compass orientation not yet received, trying again in 1s");
            attempts_++;
            return;
        }
        else {
            compass_init_ok_ = true;
            home_compass_hdg_ = compass_hdg_;
            attempts_ = 0;
            RCLCPP_INFO(nh_->get_logger(), "Setting Arducopter params");
        }
    }

    // Set arducopter params
    if (!param_set_ok_) {

        // Run the action
        auto param_set_client = nh_->create_client<mavros_msgs::srv::ParamSet>("/mavros/param/set");
        auto param_set_req = std::make_shared<mavros_msgs::srv::ParamSet::Request>();

        std::map<std::string, int>::iterator it;

        for (it = param_map_.begin(); it != param_map_.end(); it++) {
            param_set_req->param_id = it->first;
            param_set_req->value.integer = it->second;
            auto result = param_set_client->async_send_request(param_set_req);

            // Check success
            if (rclcpp::spin_until_future_complete(nh_, result) !=
                rclcpp::FutureReturnCode::SUCCESS
                || !result.get()->success) {
                if (attempts_ == 3) {
                    RCLCPP_ERROR(nh_->get_logger(), "Param set of param %s failed after 3 attempts", it->first.c_str());
                    drone_init_timer_.reset();
                }
                RCLCPP_WARN(nh_->get_logger(), "Param %s set failed, trying again in 1s", it->first.c_str());
                attempts_++;
                return;
            }
        }

        param_set_ok_ = true;
        attempts_ = 0;
    }

    RCLCPP_INFO(nh_->get_logger(), "Drone initialization successful!");
    drone_initialized_ = true;
    drone_init_timer_.reset();

}

void FlightControllerInterface::requestMavlinkStreams() {

    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000, "Requesting MAVLink streams from autopilot");

    // Request all streams
    auto streamrate_client = nh_->create_client<mavros_msgs::srv::StreamRate>("/mavros/set_stream_rate");
    auto streamrate_req = std::make_shared<mavros_msgs::srv::StreamRate::Request>(); 
    streamrate_req->stream_id = 0;
    streamrate_req->message_rate = all_stream_rate_ * stream_rate_modifier_;
    streamrate_req->on_off = 1;
    auto streamrate_res = streamrate_client->async_send_request(streamrate_req);

    if (rclcpp::spin_until_future_complete(nh_, streamrate_res) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(nh_->get_logger(), "Seting general stream rate failed");
    }

    // Request specific streams at particular rates
    auto msg_interval_client = nh_->create_client<mavros_msgs::srv::MessageInterval>("/mavros/set_message_interval");
    auto msg_interval_req = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();

    msg_interval_req->message_id = 30; // ATTITUDE
    msg_interval_req->message_rate = imu_rate_ * stream_rate_modifier_;
    auto attitude_res = msg_interval_client->async_send_request(msg_interval_req);

    if (rclcpp::spin_until_future_complete(nh_, attitude_res) !=
        rclcpp::FutureReturnCode::SUCCESS
        || !attitude_res.get()->success) {
        RCLCPP_WARN(nh_->get_logger(), "Seting attitude stream rate failed");
    }

    msg_interval_req->message_id = 32; // LOCAL_POSITION_NED
    msg_interval_req->message_rate = local_pos_rate_ * stream_rate_modifier_;
    auto local_pos_res = msg_interval_client->async_send_request(msg_interval_req);

    if (rclcpp::spin_until_future_complete(nh_, local_pos_res) !=
        rclcpp::FutureReturnCode::SUCCESS
        || !local_pos_res.get()->success) {
        RCLCPP_WARN(nh_->get_logger(), "Setting local pos stream rate failed");
    }

    msg_interval_req->message_id = 147; // BATTERY_STATUS
    msg_interval_req->message_rate = battery_rate_ * stream_rate_modifier_;
    auto battery_res = msg_interval_client->async_send_request(msg_interval_req);

    if (rclcpp::spin_until_future_complete(nh_, battery_res) !=
        rclcpp::FutureReturnCode::SUCCESS
        || !battery_res.get()->success) {
        RCLCPP_WARN(nh_->get_logger(), "Setting battery stream rate failed");
    }
}

void FlightControllerInterface::initUTM(double &utm_x, double &utm_y) {
    GeographicLib::GeoCoords c(current_ll_.latitude, current_ll_.longitude);
    utm_x = c.Easting();
    utm_y = c.Northing();
}

void FlightControllerInterface::checkMsgRates() {

    // Don't run until param fetch is complete (avoids annoying warnings)
    if (!param_fetch_complete_) {
        return;
    }

    check_msg_rates_counter_++;

    if (imu_count_ / msg_rate_timer_dt_ < imu_rate_ * 0.8) {
        RCLCPP_WARN(nh_->get_logger(), "Warning, IMU only sending at %f / %f hz", (imu_count_ / msg_rate_timer_dt_), imu_rate_);
        imu_rate_ok_ = false;
    }
    else {
        imu_rate_ok_ = true;
    }

    // Use battery message as proxy for all generic message streams
    if (battery_count_ / msg_rate_timer_dt_ < battery_rate_ * 0.8) {
        RCLCPP_WARN(nh_->get_logger(), "Warning, battery only sending at %f / %f hz", (battery_count_ / msg_rate_timer_dt_), battery_rate_);
        battery_rate_ok_ = false;
    }
    else {
        battery_rate_ok_ = true;
    }

    // Also check local pos rate, but don't use nh_ for initialization check b/c drone needs to initialize before
    // vision pose starts publishing (which ultimately, local position comes from)
    if (local_pos_count_ / msg_rate_timer_dt_ <  8) {
        RCLCPP_WARN(nh_->get_logger(), "Warning, local position only sending at %f / 10 hz", (local_pos_count_ / msg_rate_timer_dt_));
    }

    // Reset counters
    imu_count_ = 0;
    local_pos_count_ = 0;
    battery_count_ = 0;
}

void FlightControllerInterface::setAutonomyEnabled(bool enabled) {
    enable_autonomy_ = enabled;
    if (enable_autonomy_) {
        arming_client_ = nh_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        takeoff_client_ = nh_->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    }
    else {
        arming_client_.reset();
        set_mode_client_.reset();
        takeoff_client_.reset();
    }
}

bool FlightControllerInterface::getMapYaw(double &yaw) {
    if (compass_init_ok_) {
        yaw = home_compass_hdg_;
    }
    return compass_init_ok_;
}

bool FlightControllerInterface::getAveragedOrientation(geometry_msgs::msg::Quaternion &orientation) {

    if (imu_averaging_vec_.size() < imu_averaging_n_) {
        RCLCPP_WARN(nh_->get_logger(), "IMU does not have enough samples for proper averaging");
        return false;
    }

    // Calculate average of vector of recent measurements
    for (auto &meas : imu_averaging_vec_) {
        orientation.x += meas.orientation.x;
        orientation.y += meas.orientation.y;
        orientation.z += meas.orientation.z;
        orientation.w += meas.orientation.w;
    }

    orientation.x /= imu_averaging_n_;
    orientation.y /= imu_averaging_n_;
    orientation.z /= imu_averaging_n_;
    orientation.w /= imu_averaging_n_;

    return true;
}

void FlightControllerInterface::slamPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_slam_pose_ = *msg;
}

void FlightControllerInterface::globalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    current_ll_ = *msg;
    // Check UTM zone
    double lat = msg->latitude, lon = msg->longitude;
    detected_utm_zone_ = GeographicLib::UTMUPS::StandardZone(lat, lon);
}

void FlightControllerInterface::localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
    local_pos_count_++;
}

void FlightControllerInterface::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    current_imu_ = *msg;
    imu_count_++;

    // Add to IMU array for averaging
    if (imu_averaging_vec_.size() == imu_averaging_n_) {
        imu_averaging_vec_.pop_front();
    }
    imu_averaging_vec_.push_back(current_imu_);
}

void FlightControllerInterface::compassCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    compass_hdg_ = msg->data;
    compass_received_ = true;
}

void FlightControllerInterface::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    current_battery_ = *msg;
    battery_count_++;

    float current = -msg->current; // Mavros current is negative
    battery_voltage_ = msg->voltage;

    // If current is low, we can estimate battery percentage from voltage. 
    // We use nh_ 'last resting percent' as the starting point for calculations for 
    // remaining battery life. 
    if (current < 1.f) {
        last_resting_percent_ = calculateBatteryPercentage(battery_voltage_);
        last_resting_percent_time_ = nh_->get_clock()->now();
        current_drawn_since_resting_percent_ = 0.f;
    }

    current_drawn_since_resting_percent_ += current * (nh_->get_clock()->now() - last_battery_measurement_).seconds() / 3600;
    last_battery_measurement_ = nh_->get_clock()->now();

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
    messages_88::msg::Battery batt_msg;
    batt_msg.header.stamp = nh_->get_clock()->now();
    batt_msg.percentage = battery_percentage_;
    batt_msg.estimated_current = estimated_current_;
    batt_msg.amp_hours_left = amp_hours_left;
    batt_msg.flight_time_remaining = estimated_flight_time_remaining_;

    battery_pub_->publish(batt_msg);
}

void FlightControllerInterface::statusCallback(const mavros_msgs::msg::State::SharedPtr msg) {
    // TODO add system status with enum matching MAV_STATE defined by Mavlink
    connected_ = msg->connected;
    armed_ = msg->armed;
    current_mode_ = msg->mode;
    in_guided_mode_ = current_mode_ == guided_mode_;
}

void FlightControllerInterface::altitudeCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    in_air_ = msg->data > 1.0 && armed_;
    current_altitude_ = msg->data;
}

// void FlightControllerInterface::sysStatusCallback(const mavros_msgs::msg::SysStatus::SharedPtr msg) {
//     // Bitwise check of healthy sensors vs enabled sensors
//     ready_to_arm_ = (msg->sensors_health & msg->sensors_enabled) == msg->sensors_enabled;

//     using STS = mavlink::common::MAV_SYS_STATUS_SENSOR;

//     std::string reasons = "";

//     // If not ready to arm, determine reasons
//     if (ready_to_arm_)
//         return;

//     if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_GYRO &&
//         !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_GYRO))
//         reasons += "gyro, ";

//     if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_ACCEL &&
//         !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_ACCEL))
//         reasons += "accelerometer, ";

//     if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_MAG &&
//         !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_MAG))
//         reasons += "magnetometer, ";

//     if (msg->sensors_enabled & (uint32_t)STS::ABSOLUTE_PRESSURE &&
//         !(msg->sensors_health & (uint32_t)STS::ABSOLUTE_PRESSURE))
//         reasons += "absolute pressure, ";

//     if (msg->sensors_enabled & (uint32_t)STS::DIFFERENTIAL_PRESSURE &&
//         !(msg->sensors_health & (uint32_t)STS::DIFFERENTIAL_PRESSURE))
//         reasons += "differential pressure, ";

//     if (msg->sensors_enabled & (uint32_t)STS::GPS &&
//         !(msg->sensors_health & (uint32_t)STS::GPS))
//         reasons += "GPS, ";

//     if (msg->sensors_enabled & (uint32_t)STS::OPTICAL_FLOW &&
//         !(msg->sensors_health & (uint32_t)STS::OPTICAL_FLOW))
//         reasons += "optical flow, ";

//     if (msg->sensors_enabled & (uint32_t)STS::VISION_POSITION &&
//         !(msg->sensors_health & (uint32_t)STS::VISION_POSITION))
//         reasons += "computer vision position, ";

//     if (msg->sensors_enabled & (uint32_t)STS::LASER_POSITION &&
//         !(msg->sensors_health & (uint32_t)STS::LASER_POSITION))
//         reasons += "laser based position, ";

//     if (msg->sensors_enabled & (uint32_t)STS::EXTERNAL_GROUND_TRUTH &&
//         !(msg->sensors_health & (uint32_t)STS::EXTERNAL_GROUND_TRUTH))
//         reasons += "external ground truth, ";

//     if (msg->sensors_enabled & (uint32_t)STS::ANGULAR_RATE_CONTROL &&
//         !(msg->sensors_health & (uint32_t)STS::ANGULAR_RATE_CONTROL))
//         reasons += "angular rate control, ";

//     if (msg->sensors_enabled & (uint32_t)STS::ATTITUDE_STABILIZATION &&
//         !(msg->sensors_health & (uint32_t)STS::ATTITUDE_STABILIZATION))
//         reasons += "attitude stabilization, ";

//     if (msg->sensors_enabled & (uint32_t)STS::YAW_POSITION &&
//         !(msg->sensors_health & (uint32_t)STS::YAW_POSITION))
//         reasons += "yaw position, ";

// 	if (msg->sensors_enabled & (uint32_t)STS::Z_ALTITUDE_CONTROL &&
//         !(msg->sensors_health & (uint32_t)STS::Z_ALTITUDE_CONTROL))
//         reasons += "z altitude control, ";

//     if (msg->sensors_enabled & (uint32_t)STS::XY_POSITION_CONTROL &&
//         !(msg->sensors_health & (uint32_t)STS::XY_POSITION_CONTROL))
//         reasons += "x/y position control, ";

//     if (msg->sensors_enabled & (uint32_t)STS::MOTOR_OUTPUTS &&
//         !(msg->sensors_health & (uint32_t)STS::MOTOR_OUTPUTS))
//         reasons += "motor outputs, ";

//     if (msg->sensors_enabled & (uint32_t)STS::RC_RECEIVER &&
//         !(msg->sensors_health & (uint32_t)STS::RC_RECEIVER))
//         reasons += "rc receiver, ";

//     if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_GYRO2 &&
//         !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_GYRO2))
//         reasons += "gyro 2, ";

//     if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_ACCEL2 &&
//         !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_ACCEL2))
//         reasons += "accelerometer 2, ";

//     if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_MAG2 &&
//         !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_MAG2))
//         reasons += "magnetometer 2, ";

// 	if (msg->sensors_enabled & (uint32_t)STS::GEOFENCE &&
//         !(msg->sensors_health & (uint32_t)STS::GEOFENCE))
//         reasons += "geofence, ";

//     if (msg->sensors_enabled & (uint32_t)STS::AHRS &&
//         !(msg->sensors_health & (uint32_t)STS::AHRS))
//         reasons += "AHRS, ";

//     if (msg->sensors_enabled & (uint32_t)STS::TERRAIN &&
//         !(msg->sensors_health & (uint32_t)STS::TERRAIN))
//         reasons += "terrain, ";

//     if (msg->sensors_enabled & (uint32_t)STS::REVERSE_MOTOR &&
//         !(msg->sensors_health & (uint32_t)STS::REVERSE_MOTOR))
//         reasons += "motors reversed, ";

//     if (msg->sensors_enabled & (uint32_t)STS::LOGGING &&
//         !(msg->sensors_health & (uint32_t)STS::LOGGING))
//         reasons += "logging, ";

//     if (msg->sensors_enabled & (uint32_t)STS::BATTERY &&
//         !(msg->sensors_health & (uint32_t)STS::BATTERY))
//         reasons += "battery, ";

//     if (msg->sensors_enabled & (uint32_t)STS::PROXIMITY &&
//         !(msg->sensors_health & (uint32_t)STS::PROXIMITY))
//         reasons += "proximity, ";

//     if (msg->sensors_enabled & (uint32_t)STS::SATCOM &&
//         !(msg->sensors_health & (uint32_t)STS::SATCOM))
//         reasons += "satellite communication, ";

//     if (msg->sensors_enabled & (uint32_t)STS::PREARM_CHECK &&
//         !(msg->sensors_health & (uint32_t)STS::PREARM_CHECK)) {
//         // Add prearm text from statustext message
//         reasons += "prearm checks: [";
//         reasons += prearm_text_ + "], ";
//     }

//     if (msg->sensors_enabled & (uint32_t)STS::OBSTACLE_AVOIDANCE &&
//         !(msg->sensors_health & (uint32_t)STS::OBSTACLE_AVOIDANCE))
//         reasons += "obstacle avoidance / collision prevention, ";

//     if (msg->sensors_enabled & (uint32_t)STS::PROPULSION &&
//         !(msg->sensors_health & (uint32_t)STS::PROPULSION))
//         reasons += "propulsion, ";

//     preflight_check_reasons_ = reasons;
// }

void FlightControllerInterface::statusTextCallback(const mavros_msgs::msg::StatusText::SharedPtr  msg) {
    std::string text = msg->text;
    std::string prefix = "PreArm: ";

    // Prearm text gets updated every 30 seconds, so clear it out just before then for a reset
    if ((nh_->get_clock()->now() - last_prearm_text_).seconds() > 29.0) {
        prearm_text_ = "";
    }

    // Add to prearm text if statustext starts with "PreArm"
    if (text.compare(0, prefix.size(), prefix) == 0) {
        std::string reason = text.substr(prefix.size(), text.size());
        prearm_text_ += reason + ", ";
        last_prearm_text_ = nh_->get_clock()->now();
    }
}

bool FlightControllerInterface::setMode(std::string mode) {
    auto offb_set_mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    offb_set_mode_req->custom_mode = mode;
    auto result = set_mode_client_->async_send_request(offb_set_mode_req);
    if (rclcpp::spin_until_future_complete(nh_, result) ==
            rclcpp::FutureReturnCode::SUCCESS
            && result.get()->mode_sent) {
        RCLCPP_INFO(nh_->get_logger(), "Mode set to: %s", mode.c_str());
        return true;
    }
    else {
        RCLCPP_WARN(nh_->get_logger(), "Mode set failed: %s", mode.c_str());
        return false;
    }
}

bool FlightControllerInterface::arm() {
    if (armed_) {
        RCLCPP_WARN(nh_->get_logger(), "Not arming, already armed");
        return false;
    }
    if (!enable_autonomy_) {
        RCLCPP_WARN(nh_->get_logger(), "Autonomy disabled in arming.");
        return false;
    }
    auto arm_cmd_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_cmd_req->value = true;
    auto result = arming_client_->async_send_request(arm_cmd_req);
    if (rclcpp::spin_until_future_complete(nh_, result) ==
            rclcpp::FutureReturnCode::SUCCESS
            && result.get()->success){
        RCLCPP_INFO(nh_->get_logger(), "Vehicle armed");
        return true;
    }
    else {
        RCLCPP_WARN(nh_->get_logger(), "Arming failed");
        return false;
    }

    return false;
}

bool FlightControllerInterface::takeOff() {

    RCLCPP_INFO(nh_->get_logger(), "Requesting takeoff");

    if (!in_guided_mode_) {
        RCLCPP_WARN(nh_->get_logger(), "Not taking off. Not in guided mode");
        setMode(guided_mode_);
        return false;
    }
    if (in_air_) {
        RCLCPP_WARN(nh_->get_logger(), "Not taking off. Command received while in air.");
        return false;
    }
    if (!enable_autonomy_) {
        RCLCPP_WARN(nh_->get_logger(), "Not taking off. Autonomy disabled");
        return false;
    }
    if (!armed_) {
        if (!arm()) {
            RCLCPP_WARN(nh_->get_logger(), "Not taking off. Arming failed");
            return false;
        }
    }

    auto takeoff_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

    if (!nh_->get_parameter("/task_manager/default_alt", target_altitude_))
        RCLCPP_WARN(nh_->get_logger(), "Drone state manager cannot get default altitude param");

    RCLCPP_INFO(nh_->get_logger(), "Requesting takeoff to %fm", target_altitude_);
    takeoff_req->altitude = target_altitude_;
    auto result = takeoff_client_->async_send_request(takeoff_req);
    if (rclcpp::spin_until_future_complete(nh_, result) ==
            rclcpp::FutureReturnCode::SUCCESS
            && result.get()->success) {
        RCLCPP_INFO(nh_->get_logger(), "Vehicle takeoff succeeded");
        return true;
    }
    else {
        RCLCPP_WARN(nh_->get_logger(), "Vehicle takeoff failed");
        return false;
    }

    return false;
}

float FlightControllerInterface::calculateBatteryPercentage(float voltage) {

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

// void FlightControllerInterface::calculateBatteryPercentage(float voltage_adj) {

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

// float FlightControllerInterface::findValidRoot(float a, float b, float c) {
//     float discriminant = b*b - 4*a*c;
//     float x1, x2;
    
//     if (discriminant < 0) {
//         RCLCPP_WARN(nh_->get_logger(), "Invalid battery calculation");
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