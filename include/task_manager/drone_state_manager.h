/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef DRONE_STATE_MANAGER_H_
#define DRONE_STATE_MANAGER_H_

#include <ros/ros.h>

#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
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
        geometry_msgs::Point getCurrentLocalPosition();
        sensor_msgs::NavSatFix getCurrentGlobalPosition();
        double getAltitudeAGL();
        std::string getFlightMode();
        bool getIsInAir();
        bool getAutonomyActive();

        // Mavros subscriber callbacks
        void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void statusCallback(const mavros_msgs::State::ConstPtr & msg);
        void altitudeCallback(const std_msgs::Float64::ConstPtr & msg);

        // Mavros state control
        bool setGuided();
        bool setMode(std::string mode);
        bool arm();
        bool takeOff();

        // safety/validity checking
        bool readyForAction();
        bool getReadyForAction();
        bool setSafetyArea();

    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

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
        double current_altitude_;
        std::string current_mode_;
        bool altitude_set_;
        bool connected_;
        bool armed_;
        bool in_air_;
        bool in_guided_mode_;
        ros::Duration service_wait_duration_;
};

}

#endif