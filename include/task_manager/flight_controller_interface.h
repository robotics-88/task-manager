/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef FLIGHT_CONTROLLER_INTERFACE_H_
#define FLIGHT_CONTROLLER_INTERFACE_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <map>
#include <deque>

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "mavros_msgs/msg/home_position.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/msg/sys_status.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/message_interval.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/stream_rate.hpp"
#include "mavros_msgs/srv/waypoint_clear.hpp"

#include "messages_88/action/explore.hpp"
#include "messages_88/action/nav_to_point.hpp"
#include "messages_88/msg/battery.hpp"
#include "messages_88/srv/geopoint.hpp"

#include "rcl_interfaces/srv/set_parameters.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

namespace flight_controller_interface {
/**
 * @class FlightControllerInterface
 * @brief Manages task and flight state of drone.
 */
class FlightControllerInterface : public rclcpp::Node
{
    public:
        FlightControllerInterface();
        ~FlightControllerInterface();

        // Getters and setters
        void setAutonomyEnabled(bool enabled) {enable_autonomy_ = enabled;}
        void setMapSlamTf(geometry_msgs::msg::TransformStamped tf) {map_tf_init_ = true; map_to_slam_tf_ = tf;}
        geometry_msgs::msg::PoseStamped getCurrentSlamPosition() {return current_slam_pose_;}
        geometry_msgs::msg::PoseStamped getCurrentLocalPosition() {return current_pose_;}
        sensor_msgs::msg::NavSatFix getCurrentGlobalPosition() {return current_ll_;}
        double getAltitudeAGL() {return current_altitude_;}
        int getUTMZone() {return detected_utm_zone_;}
        std::string getFlightMode() {return current_mode_;}
        bool getIsInAir() {return in_air_;}
        bool getIsArmed() {return armed_;}
        double getCompass() {return compass_hdg_;}
        bool getDroneInitalized() {return drone_initialized_;}
        float getFlightTimeRemaining() {return estimated_flight_time_remaining_;}
        float getBatteryPercentage() {return battery_percentage_;}
        float getBatteryVoltage() {return battery_voltage_;}
        bool getDroneReadyToArm() {return ready_to_arm_;}
        unsigned getImuAveragingN() {return imu_averaging_n_;}
        std::string getPreflightCheckReasons() {return preflight_check_reasons_;}
        bool getMapYaw(double &yaw);
        bool getAveragedOrientation(geometry_msgs::msg::Quaternion &orientation);
        rclcpp::Time getLastVisionPosePubStamp() {return last_vision_pose_pub_stamp_;}

        // Subscriber callbacks
        void slamPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void globalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void altitudeCallback(const std_msgs::msg::Float64::SharedPtr  msg);
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void compassCallback(const std_msgs::msg::Float64::SharedPtr  msg);
        void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
        void sysStatusCallback(const mavros_msgs::msg::SysStatus::SharedPtr msg);
        void statusCallback(const mavros_msgs::msg::State::SharedPtr  msg);
        void statusTextCallback(const mavros_msgs::msg::StatusText::SharedPtr  msg);

        // General public methods
        void initUTM(double &utm_x, double &utm_y);
        bool setMode(std::string mode);
        bool takeOff(const double takeoff_altitude);

        std::string land_mode_ = "LAND";
        std::string brake_mode_ = "BRAKE";
        std::string guided_mode_ = "GUIDED";
        std::string rtl_mode_ = "RTL";

    private:

        bool offline_;
        bool simulate_;
        bool do_slam_;
        std::string mavros_map_frame_;
        std::string slam_map_frame_;
        bool px4_;

        // Safety for enabling control
        bool enable_autonomy_;

        // Control defaults
        bool compass_received_;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr        mavros_global_pos_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    mavros_local_pos_subscriber_;
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr            mavros_state_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             mavros_alt_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr              mavros_imu_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             mavros_compass_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr     mavros_battery_subscriber_;
        rclcpp::Subscription<mavros_msgs::msg::SysStatus>::SharedPtr        mavros_sys_status_subscriber_;
        rclcpp::Subscription<mavros_msgs::msg::StatusText>::SharedPtr       mavros_status_text_subscriber_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    slam_pose_subscriber_;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       vision_pose_publisher_;
        rclcpp::Publisher<messages_88::msg::Battery>::SharedPtr             battery_pub_; // Publisher mostly for debug

        // Param sync client
        std::shared_ptr<rclcpp::SyncParametersClient> sync_params_client_;

        // General private data
        sensor_msgs::msg::NavSatFix current_ll_;
        geometry_msgs::msg::PoseStamped current_pose_;
        sensor_msgs::msg::Imu current_imu_;
        std::deque<sensor_msgs::msg::Imu> imu_averaging_vec_;
        sensor_msgs::msg::Imu mavros_imu_init_;
        unsigned imu_averaging_n_;
        float flight_time_remaining_;
        double home_compass_hdg_;
        double compass_hdg_;
        unsigned compass_count_;
        double current_altitude_;
        std::string current_mode_;
        bool connected_;
        bool ready_to_arm_;
        bool armed_;
        bool in_air_;
        bool in_guided_mode_;
        int detected_utm_zone_;
        bool utm_set_;
        std::string preflight_check_reasons_;
        std::string prearm_text_;
        rclcpp::Time last_prearm_text_;
        bool map_tf_init_;
        geometry_msgs::msg::TransformStamped map_to_slam_tf_;
        rclcpp::Time last_vision_pose_pub_stamp_;

        // Battery estimation stuff
        sensor_msgs::msg::BatteryState current_battery_;
        float last_resting_percent_;
        rclcpp::Time last_resting_percent_time_;
        rclcpp::Time last_battery_measurement_;
        float current_drawn_since_resting_percent_;
        std::vector<float> recent_currents_;
        float battery_percentage_;
        float battery_voltage_;
        float battery_size_;
        float estimated_current_;
        float estimated_flight_time_remaining_;
        

        // Slam pose
        geometry_msgs::msg::PoseStamped current_slam_pose_;

        // Message rate check stuff
        float all_stream_rate_;
        float msg_rate_timer_dt_;
        rclcpp::TimerBase::SharedPtr msg_rate_timer_;
        float imu_rate_;
        unsigned imu_count_;
        float local_pos_rate_;
        unsigned local_pos_count_;
        float battery_rate_;
        unsigned battery_count_;

        bool imu_rate_ok_;
        bool battery_rate_ok_;

        int init_count_;

        // Initialization check stuff
        bool drone_initialized_;
        unsigned compass_wait_counter_;
        unsigned attempts_;
        rclcpp::TimerBase::SharedPtr drone_init_timer_;
        bool param_fetch_complete_;
        bool heading_src_ok_;
        bool stream_rates_ok_;
        bool geofence_clear_ok_;
        bool mission_clear_ok_;
        bool compass_init_ok_;
        bool param_set_ok_;

        std::map<std::string, int> param_map_ = {
            { "EK3_SRC1_POSXY", 3 },
            { "EK3_SRC1_VELXY", 3 },
            { "EK3_SRC1_POSZ", 1 },
            { "EK3_SRC1_VELZ", 3 },
            { "EK3_SRC1_YAW", 1 },
            { "VISO_TYPE", 0 },
        };

        // FCI private methods
        bool arm();
        void initializePX4();
        void initializeArducopter();
        void checkMsgRates();
        void requestMavlinkStreams();
        float calculateBatteryPercentage(float voltage);
        // float findValidRoot(float a, float b, float c);
};

}

#endif