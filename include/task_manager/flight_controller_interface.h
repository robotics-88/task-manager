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
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/msg/home_position.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "messages_88/action/explore.hpp"
#include "messages_88/action/nav_to_point.hpp"
#include "messages_88/msg/battery.hpp"
#include "messages_88/srv/geopoint.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

namespace flight_controller_interface {
/**
 * @class FlightControllerInterface
 * @brief Manages task and flight state of drone.
 */
class FlightControllerInterface
{
    public:
        explicit FlightControllerInterface(const std::shared_ptr<rclcpp::Node> nh);
        ~FlightControllerInterface();

        // State access methods
        void setAutonomyEnabled(bool enabled);
        
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
        int getImuAveragingN() {return imu_averaging_n_;}
        std::string getPreflightCheckReasons() {return preflight_check_reasons_;}

        bool getMapYaw(double &yaw);
        bool getAveragedOrientation(geometry_msgs::msg::Quaternion &orientation);

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

        // Mavros state control
        bool setMode(std::string mode);
        bool arm();
        bool takeOff();

        // safety/validity checking
        void initializeDrone();
        void initUTM(double &utm_x, double &utm_y);
        void checkMsgRates();
        void requestMavlinkStreams();

        // Other methods
        float calculateBatteryPercentage(float voltage);
        // float findValidRoot(float a, float b, float c);


    private:

        const std::shared_ptr<rclcpp::Node> nh_;

        bool offline_;
        bool simulate_;
        bool do_slam_;

        // Safety for enabling control
        bool enable_autonomy_;

        // Control defaults
        float target_altitude_;
        bool ardupilot_;
        bool compass_received_;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr mavros_global_pos_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_local_pos_subscriber_;
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr mavros_alt_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mavros_imu_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr mavros_compass_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr mavros_battery_subscriber_;
        rclcpp::Subscription<mavros_msgs::msg::SysStatus>::SharedPtr mavros_sys_status_subscriber_;
        rclcpp::Subscription<mavros_msgs::msg::StatusText>::SharedPtr mavros_status_text_subscriber_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_subscriber_;

        // Publishers
        rclcpp::Publisher<messages_88::msg::Battery>::SharedPtr battery_pub_; // Publisher mostly for debug

        // Mavros service clients
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

        // Mavros modes
        std::string land_mode_;
        std::string brake_mode_;
        std::string guided_mode_;
        std::string rtl_mode_;

        // General private data
        sensor_msgs::msg::NavSatFix current_ll_;
        geometry_msgs::msg::PoseStamped current_pose_;
        sensor_msgs::msg::Imu current_imu_;
        std::deque<sensor_msgs::msg::Imu> imu_averaging_vec_;
        sensor_msgs::msg::Imu mavros_imu_init_;
        int imu_averaging_n_;
        float flight_time_remaining_;
        double home_compass_hdg_;
        double compass_hdg_;
        int compass_count_;
        double current_altitude_;
        std::string current_mode_;
        bool connected_;
        bool ready_to_arm_;
        bool armed_;
        bool in_air_;
        bool in_guided_mode_;
        rclcpp::Duration service_wait_duration_;
        int detected_utm_zone_;
        bool utm_set_;
        std::string preflight_check_reasons_;
        std::string prearm_text_;
        rclcpp::Time last_prearm_text_;

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
        int imu_count_;
        float local_pos_rate_;
        int local_pos_count_;
        float stream_rate_modifier_;
        float battery_rate_;
        int battery_count_;

        bool imu_rate_ok_;
        bool battery_rate_ok_;

        // Initialization check stuff
        bool drone_initialized_;
        int check_msg_rates_counter_;
        int compass_wait_counter_;
        int attempts_;
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

};

}

#endif