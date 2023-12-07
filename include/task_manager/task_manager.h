/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "messages_88/Emergency.h"
#include "messages_88/ExploreAction.h"
#include "messages_88/InitDroneState.h"
#include "messages_88/GetPosition.h"
#include "messages_88/PrepareDrone.h"
#include "messages_88/PrepareExplore.h"
#include "messages_88/NavToPointAction.h"
#include "messages_88/Save.h"
#include "messages_88/TaskStatus.h"
#include "task_manager/drone_state_manager.h"

namespace task_manager {
/**
 * @class TaskManager
 * @brief The TaskManager manages the task queue (e.g., navigate to polygon, explore, handle emergency)
 */
class TaskManager {

    public:
        TaskManager(ros::NodeHandle& node);
        ~TaskManager();

        bool initDroneStateManager(messages_88::InitDroneState::Request& req, messages_88::InitDroneState::Response& resp);
        bool getReadyForAction(messages_88::PrepareDrone::Request& req, messages_88::PrepareDrone::Response& resp);
        bool getReadyForExplore(messages_88::PrepareExplore::Request& req, messages_88::PrepareExplore::Response& resp);
        bool getDronePosition(messages_88::GetPosition::Request& req, messages_88::GetPosition::Response& resp);
        bool emergencyResponse(messages_88::Emergency::Request& req, messages_88::Emergency::Response& resp);

        void startNav2PointTask(messages_88::NavToPointGoal &nav_goal);
        void sendExploreTask(messages_88::ExploreGoal &goal);
        void startExploreTask();
        void stop();
        void modeMonitor();

        void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        bool pauseOperations();

    private:
        enum CurrentStatus
        {
            ON_START,
            EXPLORING,
            WAITING_TO_EXPLORE,
            HOVERING,
            NAVIGATING,
            TAKING_OFF,
            LANDING
        };

        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
        bool map_tf_init_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Frames
        std::string mavros_map_frame_;
        std::string slam_map_frame_;

        // Pose republisher
        std::string slam_pose_topic_;
        ros::Publisher slam_pose_pub_;

        // Drone state and services
        drone_state_manager::DroneStateManager drone_state_manager_;
        ros::ServiceServer drone_state_service_;
        ros::ServiceServer drone_ready_service_;
        ros::ServiceServer drone_explore_service_;
        ros::ServiceServer drone_position_service_;
        ros::ServiceServer emergency_service_;

        // Record
        bool do_record_;
        bool bag_active_;
        std::string record_config_name_;
        ros::Publisher start_record_pub_;
        ros::Publisher stop_record_pub_;

        // Drone state params
        double max_dist_to_polygon_;
        geometry_msgs::Polygon current_polygon_;
        ros::Timer mode_monitor_timer_;
        std::string cmd_history_;
        messages_88::TaskStatus task_msg_;
        ros::Publisher task_pub_;

        actionlib::SimpleActionClient<messages_88::ExploreAction> explore_action_client_;

        // Saving
        std::string directory_;
        ros::ServiceClient vegetation_save_client_;
        ros::ServiceClient tree_save_client_;
        bool did_save_;
        bool did_takeoff_;

        ros::Publisher local_pos_pub_;
        ros::Publisher local_vel_pub_;
        ros::Subscriber mavros_local_pos_subscriber_;

        // Goal details
        geometry_msgs::Point current_target_;
        messages_88::ExploreGoal current_explore_goal_;

        // Mavros modes
        std::string land_mode_;
        std::string loiter_mode_;
        std::string guided_mode_;
        std::string rtl_mode_;

        CurrentStatus current_status_ = CurrentStatus::ON_START;
        ros::Timer status_timer_;

        void startBag();
        void stopBag();
        void getDroneReady();
        void readyToExplore();
        bool isInside(const geometry_msgs::Polygon& polygon, const geometry_msgs::Point& point);
        bool polygonDistanceOk(double &min_dist, geometry_msgs::PoseStamped &target, geometry_msgs::Polygon &map_region);
        std::string getStatusString();

};

}

#endif