/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef DRONE_STATE_MANAGER_H_
#define DRONE_STATE_MANAGER_H_

#include <ros/ros.h>

#include <map>

#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "messages_88/ExploreAction.h"
#include "messages_88/NavToPointAction.h"

namespace drone_state_manager {
/**
 * @class DroneStateManager
 * @brief Manages task and flight state of drone.
 */
class DroneStateManager {

    public:
        DroneStateManager(ros::NodeHandle& node);
        ~DroneStateManager();

        // State access methods
        void setAutonomyEnabled(bool enabled);
        void setExplorationEnabled(bool enabled);
        geometry_msgs::PoseStamped getCurrentSlamPosition();
        geometry_msgs::PoseStamped getCurrentLocalPosition();
        sensor_msgs::NavSatFix getCurrentGlobalPosition();
        void waitForGlobal();
        int getUTMZone();
        double getAltitudeAGL();
        std::string getFlightMode();
        bool getIsInAir();
        bool getAutonomyActive();
        bool getIsArmed();
        bool getMapYaw(double &yaw);
        double getCompass();
        bool getDroneInitalized() {return drone_initialized_;}
        float getBatteryPercentage() {return battery_percentage_;}
        float getFlightTimeRemaining() {return estimated_flight_time_remaining_;}

        // Mavros subscriber callbacks
        void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void statusCallback(const mavros_msgs::State::ConstPtr & msg);
        void altitudeCallback(const std_msgs::Float64::ConstPtr & msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void compassCallback(const std_msgs::Float64::ConstPtr & msg);
        void batteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg);
        void slamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        // Mavros state control
        bool setGuided();
        bool setMode(std::string mode);
        bool arm();
        bool takeOff();

        // safety/validity checking
        bool readyForAction();
        bool getReadyForAction();
        void initializeDrone(const ros::TimerEvent &event);
        void initUTM(double &utm_x, double &utm_y);
        void checkMsgRates(const ros::TimerEvent &event);
        void requestMavlinkStreams();

        // Other methods
        void calculateBatteryPercentage(float voltage, float current);
        float findValidRoot(float a, float b, float c);


    private:

        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        bool offline_;
        bool simulate_;
        bool do_slam_;
        bool autonomy_active_;

        // Safety for enabling control
        bool enable_autonomy_;
        bool enable_exploration_;

        // Control defaults
        float target_altitude_;
        float min_altitude_;
        float max_altitude_;
        float max_distance_;
        ros::Publisher safety_area_viz_;
        bool ardupilot_;
        bool compass_received_ = false;

        // Mavros subscribers and topics
        std::string mavros_global_pos_topic_;
        ros::Subscriber mavros_global_pos_subscriber_;
        ros::Subscriber mavros_local_pos_subscriber_;
        std::string mavros_state_topic_;
        ros::Subscriber mavros_state_subscriber_;
        std::string mavros_alt_topic_;
        ros::Subscriber mavros_alt_subscriber_;
        ros::Subscriber mavros_home_subscriber_;
        ros::Publisher local_pos_pub_;
        ros::Subscriber mavros_imu_subscriber_;
        ros::Subscriber mavros_compass_subscriber_;
        ros::Subscriber mavros_battery_subscriber_;
        ros::Subscriber slam_pose_subscriber_;

        // Mavros service clients
        std::string arming_topic_;
        ros::ServiceClient arming_client_;
        std::string set_mode_topic_;
        ros::ServiceClient set_mode_client_;
        std::string takeoff_topic_;
        ros::ServiceClient takeoff_client_;

        // Mavros modes
        std::string land_mode_;
        std::string loiter_mode_;
        std::string guided_mode_;
        std::string rtl_mode_;

        // Mavros position and status
        sensor_msgs::NavSatFix current_ll_;
        geometry_msgs::PoseStamped current_pose_;
        sensor_msgs::Imu current_imu_;
        float flight_time_remaining_;
        double home_compass_hdg_;
        double compass_hdg_;
        int compass_count_;
        double current_altitude_;
        std::string current_mode_;
        bool altitude_set_;
        bool connected_;
        bool armed_;
        bool in_air_;
        bool in_guided_mode_;
        ros::Duration service_wait_duration_;
        int detected_utm_zone_;
        bool utm_set_;

        // Battery estimation stuff
        sensor_msgs::BatteryState current_battery_;
        float battery_percentage_ = -1.f;
        std::vector<float> recent_currents_;
        float battery_resistance_;
        float battery_size_;
        float estimated_current_;
        float estimated_flight_time_remaining_;
        // Slam pose
        geometry_msgs::PoseStamped current_slam_pose_;

        // Message rate check stuff
        float all_stream_rate_;        
        float msg_rate_timer_dt_;
        ros::Timer msg_rate_timer_;
        float imu_rate_;
        int imu_count_;
        float local_pos_rate_;
        int local_pos_count_;
        float stream_rate_modifier_;
        int battery_count_;

        bool imu_rate_ok_ = false;
        bool all_stream_rate_ok_ = false;

        // Initialization check stuff
        bool drone_initialized_ = false;
        int check_msg_rates_counter_ = 0;
        int compass_wait_counter_ = 0;
        int attempts_;
        ros::Timer drone_init_timer_;
        bool param_fetch_complete_ = false;
        bool heading_src_ok_ = false;
        bool stream_rates_ok_ = false;
        bool geofence_clear_ok_ = false;
        bool mission_clear_ok_ = false;
        bool compass_init_ok_ = false;
        bool param_set_ok_ = false;

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