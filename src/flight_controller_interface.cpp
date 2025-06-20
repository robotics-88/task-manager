/*
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include "task_manager/flight_controller_interface.h"
#include "messages_88/msg/battery.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "mavconn/mavlink_dialect.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace flight_controller_interface {
FlightControllerInterface::FlightControllerInterface()
    : Node("flight_controller_interface"),
      offline_(false),
      simulate_(false),
      do_slam_(false),
      mavros_map_frame_("map"),
      slam_map_frame_("slam_map"),
      enable_autonomy_(false),
      connected_(false),
      armed_(false),
      in_air_(false),
      in_guided_mode_(false),
      imu_init_(false),
      current_altitude_(-1.0),
      detected_utm_zone_(-1),
      map_tf_init_(false),
      battery_percentage_(0.0),
      battery_voltage_(0.0),
      battery_size_(5.2),
      estimated_current_(20.0),
      estimated_flight_time_remaining_(0.0),
      imu_averaging_n_(50),
      imu_count_(0),
      local_pos_count_(0),
      battery_count_(0),
      imu_rate_ok_(false),
      battery_rate_ok_(false),
      msg_rate_timer_dt_(2.0),
      imu_rate_(120.0),
      local_pos_rate_(60.0),
      battery_rate_(10.0),
      all_stream_rate_(5.0),
      drone_initialized_(false),
      last_prearm_text_(0, 0, RCL_ROS_TIME),
      last_resting_percent_time_(0, 0, RCL_ROS_TIME),
      last_battery_measurement_(0, 0, RCL_ROS_TIME),
      last_vision_pose_pub_stamp_(0, 0, RCL_ROS_TIME),
      px4_(false) {
    // FCI specific params
    this->declare_parameter("battery_size", battery_size_);
    this->declare_parameter("estimated_current", estimated_current_);
    this->declare_parameter("imu_rate", imu_rate_);
    this->declare_parameter("local_pos_rate", local_pos_rate_);
    this->declare_parameter("all_stream_rate", all_stream_rate_);
    this->declare_parameter("px4", px4_);

    this->get_parameter("battery_size", battery_size_);
    this->get_parameter("estimated_current", estimated_current_);
    this->get_parameter("imu_rate", imu_rate_);
    this->get_parameter("local_pos_rate", local_pos_rate_);
    this->get_parameter("all_stream_rate", all_stream_rate_);
    this->get_parameter("px4", px4_);

    // Params common w task manager
    this->declare_parameter("offline", offline_);
    this->declare_parameter("do_slam", do_slam_);
    this->declare_parameter("simulate", simulate_);
    this->declare_parameter("mavros_map_frame", mavros_map_frame_);
    this->declare_parameter("slam_map_frame", slam_map_frame_);

    this->get_parameter("offline", offline_);
    this->get_parameter("do_slam", do_slam_);
    this->get_parameter("simulate", simulate_);
    this->get_parameter("mavros_map_frame", mavros_map_frame_);
    this->get_parameter("slam_map_frame", slam_map_frame_);

    if (do_slam_) {
        param_map_["EK3_SRC1_POSXY"] = 6;
        param_map_["EK3_SRC1_VELXY"] = 6;
        param_map_["EK3_SRC1_POSZ"] = 6;
        param_map_["EK3_SRC1_VELZ"] = 6;
        param_map_["EK3_SRC1_YAW"] = 1;
        param_map_["VISO_TYPE"] = 1;
    }

    // Set subscribers for Mavros
    auto sensor_qos = rclcpp::SensorDataQoS();
    auto state_qos = rclcpp::QoS(10).transient_local();

    mavros_global_pos_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/global", sensor_qos,
        std::bind(&FlightControllerInterface::globalPositionCallback, this, _1));
    mavros_local_pos_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", sensor_qos,
        std::bind(&FlightControllerInterface::localPositionCallback, this, _1));
    mavros_alt_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mavros/global_position/rel_alt", sensor_qos,
        std::bind(&FlightControllerInterface::altitudeCallback, this, _1));
    mavros_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", sensor_qos,
        std::bind(&FlightControllerInterface::imuCallback, this, _1));
    mavros_compass_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mavros/global_position/compass_hdg", sensor_qos,
        std::bind(&FlightControllerInterface::compassCallback, this, _1));
    mavros_battery_subscriber_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/mavros/battery", sensor_qos,
        std::bind(&FlightControllerInterface::batteryCallback, this, _1));
    mavros_status_text_subscriber_ = this->create_subscription<mavros_msgs::msg::StatusText>(
        "/mavros/statustext/recv", sensor_qos,
        std::bind(&FlightControllerInterface::statusTextCallback, this, _1));
    mavros_state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", state_qos,
        std::bind(&FlightControllerInterface::statusCallback, this, _1));
    mavros_sys_status_subscriber_ = this->create_subscription<mavros_msgs::msg::SysStatus>(
        "/mavros/sys_status", state_qos,
        std::bind(&FlightControllerInterface::sysStatusCallback, this, _1));
    mavros_speed_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/mavros/global_position/raw/gps_vel", sensor_qos,
        std::bind(&FlightControllerInterface::speedCallback, this, _1));

    vision_pose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);

    // Decco pub/subs
    slam_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/decco/pose", 10, std::bind(&FlightControllerInterface::slamPoseCallback, this, _1));

    if (!offline_) {

        msg_rate_timer_ =
            this->create_wall_timer(std::chrono::duration<float>(msg_rate_timer_dt_),
                                    std::bind(&FlightControllerInterface::checkMsgRates, this));

        // Todo: allow some time for stream rates to settle
        // If stream rates not okay, request them and sleep, param fetch probably needs to complete
        if (!battery_rate_ok_ || !imu_rate_ok_) {
            std::thread([this]() { requestMavlinkStreams(); }).detach();
        }

        // Now we can initialize
        if (px4_) {
            initializePX4();
        } else {
            // Run this in a different thread so it doesn't block other important subscription
            // callbacks.
            std::thread([this]() {
                RCLCPP_INFO(this->get_logger(),
                            "Waiting 15s for Arducopter param fetch to complete");
                rclcpp::sleep_for(15s);
                initializeArducopter();
            }).detach();
        }
    } else {
        // Declare that drone is initalized for offline mode
        drone_initialized_ = true;
    }

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

FlightControllerInterface::~FlightControllerInterface() {}

void FlightControllerInterface::initializePX4() {
    RCLCPP_INFO(this->get_logger(), "PX4 initialization successful");
    drone_initialized_ = true;
}

void FlightControllerInterface::initializeArducopter() {

    RCLCPP_INFO(this->get_logger(), "Initializing Arducopter");

    // Set heading source (also checks) that param fetch is complete
    std::shared_ptr<rclcpp::Node> service_call_node =
        rclcpp::Node::make_shared("service_call_node");
    auto param_set_client = service_call_node->create_client<rcl_interfaces::srv::SetParameters>(
        "/mavros/param/set_parameters");

    auto heading_src_req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rclcpp::Parameter param("EK3_SRC1_YAW", 1);
    heading_src_req->parameters.push_back(param.to_parameter_msg());

    int attempts = 5;
    for (unsigned i = 0; i < attempts; i++) {
        auto result = param_set_client->async_send_request(heading_src_req);
        if (rclcpp::spin_until_future_complete(service_call_node, result, 1s) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            bool success = true;
            auto res = result.get();
            for (unsigned i = 0; i < res->results.size(); i++) {
                if (!res->results.at(i).successful) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Heading src param set unsuccessful, param fetch likely incomplete");
                    success = false;
                }
            }
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Heading src param set complete");
                break;
            }

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service /mavros/param/set_parameters");
        }

        if (i == attempts - 1) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Heading src param set unsuccessful after 5 attempts, not initializing drone");
            return;
        }

        rclcpp::sleep_for(1s);
    }

    // If param fetch is done, MAVLink streams should be clear, so request streams again
    requestMavlinkStreams();

    // Set params from param map
    auto param_set_req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (std::map<std::string, int>::iterator it = param_map_.begin(); it != param_map_.end();
         it++) {
        rclcpp::Parameter param(it->first, it->second);
        param_set_req->parameters.push_back(param.to_parameter_msg());
    }

    attempts = 3;
    for (unsigned i = 0; i < attempts; i++) {
        auto result = param_set_client->async_send_request(param_set_req);
        if (rclcpp::spin_until_future_complete(service_call_node, result, 1s) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            bool success = true;
            auto res = result.get();
            for (unsigned i = 0; i < res->results.size(); i++) {
                if (!res->results.at(i).successful) {
                    RCLCPP_INFO(this->get_logger(),
                                "Param set unsuccessful, param fetch likely incomplete");
                    success = false;
                }
            }
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Param set complete");
                break;
            }

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service /mavros/param/set_parameters");
        }

        if (i == attempts - 1) {
            RCLCPP_ERROR(this->get_logger(),
                         "Param set unsuccessful after 3 attempts, not initializing drone");
            return;
        }
    }

    // Check message rates, after waiting between param fetch and time for check message loop
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<float>(msg_rate_timer_dt_));
    rclcpp::sleep_for(nanoseconds);
    attempts = 3;
    for (unsigned i = 0; i < attempts; i++) {
        {
            std::lock_guard<std::mutex> lock(init_mutex_);
            if (battery_rate_ok_ && imu_rate_ok_) {
                RCLCPP_INFO(this->get_logger(), "Mavlink streaming rates OK");
                break;
            } else {
                RCLCPP_INFO(this->get_logger(), "Mavlink streaming rates not OK, trying again");
                if (i == 0)
                    requestMavlinkStreams();
            }
        }

        if (i == attempts - 1) {
            RCLCPP_ERROR(this->get_logger(),
                         "Mavlink stream rates not OK after 5 attempts, not initializing drone");
            return;
        }

        rclcpp::sleep_for(nanoseconds);
    }

    std::lock_guard<std::mutex> lock(init_mutex_);
    drone_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Arducopter initialization successful");
}

void FlightControllerInterface::requestMavlinkStreams() {

    RCLCPP_INFO(this->get_logger(), "Requesting MAVLink streams from autopilot");

    std::shared_ptr<rclcpp::Node> streamrate_node = rclcpp::Node::make_shared("streamrate_node");
    rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedPtr streamrate_client =
        streamrate_node->create_client<mavros_msgs::srv::StreamRate>("/mavros/set_stream_rate");

    // Request all streams
    while (!streamrate_client->wait_for_service(5s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for mavros streamrate "
                                             "service. Not requesting mavlink streams.");
            return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Mavros streamrate service not available, waiting again...");
    }

    auto streamrate_req = std::make_shared<mavros_msgs::srv::StreamRate::Request>();
    streamrate_req->stream_id = 0;
    streamrate_req->message_rate = all_stream_rate_;
    streamrate_req->on_off = true;
    auto streamrate_res = streamrate_client->async_send_request(streamrate_req);
    if (rclcpp::spin_until_future_complete(streamrate_node, streamrate_res, 1s) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "General stream rate set");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service /mavros/set_stream_rate");
    }

    // Request specific streams at particular rates
    std::shared_ptr<rclcpp::Node> msg_interval_node =
        rclcpp::Node::make_shared("msg_interval_node");
    rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr msg_interval_client =
        msg_interval_node->create_client<mavros_msgs::srv::MessageInterval>(
            "/mavros/set_message_interval");

    // ATTITUDE
    auto msg_interval_req = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
    msg_interval_req->message_id = mavlink::common::msg::ATTITUDE::MSG_ID;
    msg_interval_req->message_rate = imu_rate_;
    auto attitude_res = msg_interval_client->async_send_request(msg_interval_req);
    if (rclcpp::spin_until_future_complete(msg_interval_node, attitude_res, 1s) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        if (attitude_res.get()->success) {
            RCLCPP_INFO(this->get_logger(), "ATTITUDE message interval set");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set message interval for ATTITUDE message");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service /mavros/set_message_interval");
    }

    // LOCAL_POSITION_NED
    msg_interval_req->message_id = mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID;
    msg_interval_req->message_rate = local_pos_rate_;
    auto local_pos_res = msg_interval_client->async_send_request(msg_interval_req);
    if (rclcpp::spin_until_future_complete(msg_interval_node, local_pos_res, 1s) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        if (local_pos_res.get()->success) {
            RCLCPP_INFO(this->get_logger(), "LOCAL_POSITION_NED message interval set");
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set message interval for LOCAL_POSITION_NED message");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service /mavros/set_message_interval");
    }

    // BATTERY_STATUS
    msg_interval_req->message_id = mavlink::common::msg::BATTERY_STATUS::MSG_ID;
    msg_interval_req->message_rate = battery_rate_;
    auto battery_res = msg_interval_client->async_send_request(msg_interval_req);
    if (rclcpp::spin_until_future_complete(msg_interval_node, battery_res, 1s) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        if (battery_res.get()->success) {
            RCLCPP_INFO(this->get_logger(), "BATTERY_STATUS message interval set");
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set message interval for BATTERY_STATUS message");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service /mavros/set_message_interval");
    }
}

void FlightControllerInterface::initUTM(double &utm_x, double &utm_y) {
    GeographicLib::GeoCoords c(current_ll_.latitude, current_ll_.longitude);
    utm_x = c.Easting();
    utm_y = c.Northing();
}

void FlightControllerInterface::checkMsgRates() {

    std::lock_guard<std::mutex> lock(init_mutex_);

    if (imu_count_ / msg_rate_timer_dt_ < imu_rate_ * 0.8) {
        if (drone_initialized_)
            RCLCPP_WARN(this->get_logger(), "Warning, IMU only sending at %f / %f hz",
                        (imu_count_ / msg_rate_timer_dt_), imu_rate_);
        imu_rate_ok_ = false;
    } else {
        imu_rate_ok_ = true;
    }

    // Use battery message as proxy for all generic message streams
    if (battery_count_ / msg_rate_timer_dt_ < battery_rate_ * 0.8) {
        if (drone_initialized_)
            RCLCPP_WARN(this->get_logger(), "Warning, battery only sending at %f / %f hz",
                        (battery_count_ / msg_rate_timer_dt_), battery_rate_);
        battery_rate_ok_ = false;
    } else {
        battery_rate_ok_ = true;
    }

    // Also check local pos rate, but don't use this for initialization check b/c drone needs to
    // initialize before vision pose starts publishing (which ultimately, local position comes from)
    if (local_pos_count_ / msg_rate_timer_dt_ < 8) {
        if (drone_initialized_)
            RCLCPP_WARN(this->get_logger(), "Warning, local position only sending at %f / 10 hz",
                        (local_pos_count_ / msg_rate_timer_dt_));
    }

    // Reset counters
    imu_count_ = 0;
    local_pos_count_ = 0;
    battery_count_ = 0;
}

bool FlightControllerInterface::getAveragedOrientation(
    geometry_msgs::msg::Quaternion &orientation) {

    if (imu_averaging_vec_.size() < imu_averaging_n_) {
        RCLCPP_WARN(this->get_logger(), "IMU does not have enough samples for proper averaging");
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

void FlightControllerInterface::slamPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_slam_pose_ = *msg;

    // Transform decco pose (in slam_map frame) and publish it in mavros_map frame as
    // /mavros/vision_pose/pose
    if (!map_tf_init_) {
        return;
    }

    // Apply the transform to the drone pose
    geometry_msgs::msg::PoseStamped vision_pose;

    // Only transform based on rotation, not translation.
    // This effectively removes the initial offset between the lidar frame and base_link. So if
    // initial /decco/pose is 0,0,0, then initial vision_pose will be 0,0,0.
    geometry_msgs::msg::TransformStamped map_to_slam_tf_no_trans = map_to_slam_tf_;
    map_to_slam_tf_no_trans.transform.translation.x = 0;
    map_to_slam_tf_no_trans.transform.translation.y = 0;
    map_to_slam_tf_no_trans.transform.translation.z = 0;

    tf2::doTransform(current_slam_pose_, vision_pose, map_to_slam_tf_no_trans);
    vision_pose.header.frame_id = mavros_map_frame_;
    vision_pose.header.stamp = current_slam_pose_.header.stamp;

    vision_pose_publisher_->publish(vision_pose);

    double time_since_last_pub = (this->get_clock()->now() - last_vision_pose_pub_stamp_).seconds();
    if (time_since_last_pub > 0.25 && last_vision_pose_pub_stamp_.nanoseconds() > 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Vision pose delay, time since last pub: %f", time_since_last_pub);
    }

    last_vision_pose_pub_stamp_ = this->get_clock()->now();
}

void FlightControllerInterface::globalPositionCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    current_ll_ = *msg;
    // Check UTM zone
    double lat = msg->latitude, lon = msg->longitude;
    detected_utm_zone_ = GeographicLib::UTMUPS::StandardZone(lat, lon);
}

void FlightControllerInterface::localPositionCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
    local_pos_count_++;
}

void FlightControllerInterface::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    current_imu_ = *msg;
    imu_count_++;

    // Add to IMU array for averaging
    if (imu_averaging_vec_.size() < imu_averaging_n_) {
        imu_averaging_vec_.push_back(current_imu_);
    } else if (!imu_init_) {
        // Determine initial orientation when code starts for slam map correction
        imu_init_ = getAveragedOrientation(mavros_imu_init_.orientation);
    }
}

void FlightControllerInterface::compassCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(init_mutex_);
    compass_hdg_ = msg->data;
}

void FlightControllerInterface::batteryCallback(
    const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    current_battery_ = *msg;
    battery_count_++;

    float current = -msg->current; // Mavros current is negative
    battery_voltage_ = msg->voltage;

    // If current is low, we can estimate battery percentage from voltage.
    // We use this 'last resting percent' as the starting point for calculations for
    // remaining battery life.
    if (current < 1.f) {
        last_resting_percent_ = calculateBatteryPercentage(battery_voltage_);
        last_resting_percent_time_ = this->get_clock()->now();
        current_drawn_since_resting_percent_ = 0.f;
    }

    current_drawn_since_resting_percent_ +=
        current * (this->get_clock()->now() - last_battery_measurement_).seconds() / 3600;
    last_battery_measurement_ = this->get_clock()->now();

    float battery_percent_drawn_since_resting_ =
        current_drawn_since_resting_percent_ / battery_size_ * 100.f;
    battery_percentage_ = last_resting_percent_ - battery_percent_drawn_since_resting_;
    float amp_hours_left = battery_size_ * battery_percentage_ / 100.f;

    // Use current estimate and remaining amp hours to determine how many seconds of flight time we
    // have left If current is below 5A, it is likely not accurate in reflecting hover current, so
    // don't update current calculation
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

void FlightControllerInterface::sysStatusCallback(
    const mavros_msgs::msg::SysStatus::SharedPtr msg) {
    // Bitwise check of healthy sensors vs enabled sensors
    ready_to_arm_ = (msg->sensors_health & msg->sensors_enabled) == msg->sensors_enabled;

    using STS = mavlink::common::MAV_SYS_STATUS_SENSOR;

    std::string reasons = "";

    // If not ready to arm, determine reasons
    if (ready_to_arm_)
        return;

    if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_GYRO &&
        !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_GYRO))
        reasons += "gyro, ";

    if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_ACCEL &&
        !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_ACCEL))
        reasons += "accelerometer, ";

    if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_MAG &&
        !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_MAG))
        reasons += "magnetometer, ";

    if (msg->sensors_enabled & (uint32_t)STS::ABSOLUTE_PRESSURE &&
        !(msg->sensors_health & (uint32_t)STS::ABSOLUTE_PRESSURE))
        reasons += "absolute pressure, ";

    if (msg->sensors_enabled & (uint32_t)STS::DIFFERENTIAL_PRESSURE &&
        !(msg->sensors_health & (uint32_t)STS::DIFFERENTIAL_PRESSURE))
        reasons += "differential pressure, ";

    if (msg->sensors_enabled & (uint32_t)STS::GPS && !(msg->sensors_health & (uint32_t)STS::GPS))
        reasons += "GPS, ";

    if (msg->sensors_enabled & (uint32_t)STS::OPTICAL_FLOW &&
        !(msg->sensors_health & (uint32_t)STS::OPTICAL_FLOW))
        reasons += "optical flow, ";

    if (msg->sensors_enabled & (uint32_t)STS::VISION_POSITION &&
        !(msg->sensors_health & (uint32_t)STS::VISION_POSITION))
        reasons += "computer vision position, ";

    if (msg->sensors_enabled & (uint32_t)STS::LASER_POSITION &&
        !(msg->sensors_health & (uint32_t)STS::LASER_POSITION))
        reasons += "laser based position, ";

    if (msg->sensors_enabled & (uint32_t)STS::EXTERNAL_GROUND_TRUTH &&
        !(msg->sensors_health & (uint32_t)STS::EXTERNAL_GROUND_TRUTH))
        reasons += "external ground truth, ";

    if (msg->sensors_enabled & (uint32_t)STS::ANGULAR_RATE_CONTROL &&
        !(msg->sensors_health & (uint32_t)STS::ANGULAR_RATE_CONTROL))
        reasons += "angular rate control, ";

    if (msg->sensors_enabled & (uint32_t)STS::ATTITUDE_STABILIZATION &&
        !(msg->sensors_health & (uint32_t)STS::ATTITUDE_STABILIZATION))
        reasons += "attitude stabilization, ";

    if (msg->sensors_enabled & (uint32_t)STS::YAW_POSITION &&
        !(msg->sensors_health & (uint32_t)STS::YAW_POSITION))
        reasons += "yaw position, ";

    if (msg->sensors_enabled & (uint32_t)STS::Z_ALTITUDE_CONTROL &&
        !(msg->sensors_health & (uint32_t)STS::Z_ALTITUDE_CONTROL))
        reasons += "z altitude control, ";

    if (msg->sensors_enabled & (uint32_t)STS::XY_POSITION_CONTROL &&
        !(msg->sensors_health & (uint32_t)STS::XY_POSITION_CONTROL))
        reasons += "x/y position control, ";

    if (msg->sensors_enabled & (uint32_t)STS::MOTOR_OUTPUTS &&
        !(msg->sensors_health & (uint32_t)STS::MOTOR_OUTPUTS))
        reasons += "motor outputs, ";

    if (msg->sensors_enabled & (uint32_t)STS::RC_RECEIVER &&
        !(msg->sensors_health & (uint32_t)STS::RC_RECEIVER))
        reasons += "rc receiver, ";

    if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_GYRO2 &&
        !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_GYRO2))
        reasons += "gyro 2, ";

    if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_ACCEL2 &&
        !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_ACCEL2))
        reasons += "accelerometer 2, ";

    if (msg->sensors_enabled & (uint32_t)STS::SENSOR_3D_MAG2 &&
        !(msg->sensors_health & (uint32_t)STS::SENSOR_3D_MAG2))
        reasons += "magnetometer 2, ";

    if (msg->sensors_enabled & (uint32_t)STS::GEOFENCE &&
        !(msg->sensors_health & (uint32_t)STS::GEOFENCE))
        reasons += "geofence, ";

    if (msg->sensors_enabled & (uint32_t)STS::AHRS && !(msg->sensors_health & (uint32_t)STS::AHRS))
        reasons += "AHRS, ";

    if (msg->sensors_enabled & (uint32_t)STS::TERRAIN &&
        !(msg->sensors_health & (uint32_t)STS::TERRAIN))
        reasons += "terrain, ";

    if (msg->sensors_enabled & (uint32_t)STS::REVERSE_MOTOR &&
        !(msg->sensors_health & (uint32_t)STS::REVERSE_MOTOR))
        reasons += "motors reversed, ";

    if (msg->sensors_enabled & (uint32_t)STS::LOGGING &&
        !(msg->sensors_health & (uint32_t)STS::LOGGING))
        reasons += "logging, ";

    if (msg->sensors_enabled & (uint32_t)STS::BATTERY &&
        !(msg->sensors_health & (uint32_t)STS::BATTERY))
        reasons += "battery, ";

    if (msg->sensors_enabled & (uint32_t)STS::PROXIMITY &&
        !(msg->sensors_health & (uint32_t)STS::PROXIMITY))
        reasons += "proximity, ";

    if (msg->sensors_enabled & (uint32_t)STS::SATCOM &&
        !(msg->sensors_health & (uint32_t)STS::SATCOM))
        reasons += "satellite communication, ";

    if (msg->sensors_enabled & (uint32_t)STS::PREARM_CHECK &&
        !(msg->sensors_health & (uint32_t)STS::PREARM_CHECK)) {
        // Add prearm text from statustext message
        reasons += "prearm checks: [";
        reasons += prearm_text_ + "], ";
    }

    if (msg->sensors_enabled & (uint32_t)STS::OBSTACLE_AVOIDANCE &&
        !(msg->sensors_health & (uint32_t)STS::OBSTACLE_AVOIDANCE))
        reasons += "obstacle avoidance / collision prevention, ";

    if (msg->sensors_enabled & (uint32_t)STS::PROPULSION &&
        !(msg->sensors_health & (uint32_t)STS::PROPULSION))
        reasons += "propulsion, ";

    preflight_check_reasons_ = reasons;
}

void FlightControllerInterface::statusTextCallback(
    const mavros_msgs::msg::StatusText::SharedPtr msg) {
    std::string text = msg->text;
    std::string prefix = "PreArm: ";

    // Prearm text gets updated every 30 seconds, so clear it out just before then for a reset
    if ((this->get_clock()->now() - last_prearm_text_).seconds() > 29.0) {
        prearm_text_ = "";
    }

    // Add to prearm text if statustext starts with "PreArm"
    if (text.compare(0, prefix.size(), prefix) == 0) {
        std::string reason = text.substr(prefix.size(), text.size());
        prearm_text_ += reason + ", ";
        last_prearm_text_ = this->get_clock()->now();
    }
}

void FlightControllerInterface::speedCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    current_speed_ = *msg;
}

bool FlightControllerInterface::setMode(std::string mode) {
    std::shared_ptr<rclcpp::Node> set_mode_node = rclcpp::Node::make_shared("set_mode_client");
    auto set_mode_client =
        set_mode_node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    auto set_mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_req->custom_mode = mode;
    auto result = set_mode_client->async_send_request(set_mode_req);
    if (rclcpp::spin_until_future_complete(set_mode_node, result, 1s) ==
            rclcpp::FutureReturnCode::SUCCESS &&
        result.get()->mode_sent) {
        RCLCPP_INFO(this->get_logger(), "Mode set to: %s", mode.c_str());
        return true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Mode set failed: %s", mode.c_str());
        return false;
    }
}

bool FlightControllerInterface::arm() {
    if (armed_) {
        RCLCPP_WARN(this->get_logger(), "Not arming, already armed");
        return false;
    }
    if (!enable_autonomy_) {
        RCLCPP_WARN(this->get_logger(), "Autonomy disabled in arming.");
        return false;
    }

    std::shared_ptr<rclcpp::Node> arm_node = rclcpp::Node::make_shared("arm_client");
    auto arm_client = arm_node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    auto arm_cmd_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_cmd_req->value = true;
    auto result = arm_client->async_send_request(arm_cmd_req);

    if (rclcpp::spin_until_future_complete(arm_node, result) == rclcpp::FutureReturnCode::SUCCESS &&
        result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Vehicle armed");
        armed_ = true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Arming failed");
        armed_ = false;
    }

    return armed_;
}

bool FlightControllerInterface::takeOff(const double takeoff_altitude) {

    if (!in_guided_mode_) {
        RCLCPP_WARN(this->get_logger(), "Not taking off. Not in guided mode");
        setMode(guided_mode_);
        return false;
    }
    if (in_air_) {
        RCLCPP_WARN(this->get_logger(), "Not taking off. Command received while in air.");
        return false;
    }
    if (!enable_autonomy_) {
        RCLCPP_WARN(this->get_logger(), "Not taking off. Autonomy disabled");
        return false;
    }
    if (!armed_) {
        if (!arm()) {
            RCLCPP_WARN(this->get_logger(), "Not taking off. Arming failed");
            return false;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Requesting takeoff to %fm", takeoff_altitude);

    std::shared_ptr<rclcpp::Node> takeoff_node = rclcpp::Node::make_shared("takeoff_client");
    auto takeoff_client =
        takeoff_node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    auto takeoff_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

    takeoff_req->altitude = takeoff_altitude;
    auto result = takeoff_client->async_send_request(takeoff_req);
    if (rclcpp::spin_until_future_complete(takeoff_node, result) ==
            rclcpp::FutureReturnCode::SUCCESS &&
        result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Vehicle takeoff succeeded");
        return true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Vehicle takeoff failed");
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
    } else if (voltage >= 3.85) {
        // For this voltage range:
        // voltage ~= 0.7*(percentage) + 3.5, so
        battery_percentage = (voltage - 3.5) / 0.7 * 100;
    } else if (voltage >= 3.7) {
        // For this voltage range:
        // voltage ~= 0.333*(percentage) + 3.683
        battery_percentage = (voltage - 3.683) / 0.333 * 100;
    } else if (voltage >= 3.5) {
        // For this voltage range:
        // voltage ~= 4*(percentage) + 3.5
        battery_percentage = (voltage - 3.5) / 4 * 100;
    } else {
        battery_percentage = 0.0;
    }

    return battery_percentage;
}

float FlightControllerInterface::getHomeDistance() {
    return std::sqrt(
        std::pow(current_pose_.pose.position.x, 2) +
        std::pow(current_pose_.pose.position.y, 2));
}

float FlightControllerInterface::getGroundSpeed() {
    return std::sqrt(
        std::pow(current_speed_.twist.linear.x, 2) +
        std::pow(current_speed_.twist.linear.y, 2));
}

} // namespace flight_controller_interface