/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "messages_88/Emergency.h"
#include "messages_88/ExploreAction.h"
#include "messages_88/InitDroneState.h"
#include "messages_88/Geopoint.h"
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
 * @brief The TaskManager manages the task queue (e.g., navigate to polygon, explore, handle emergency). Handles comms to/from UI, including task assignment, starting capabilities, and safety features based on drone status.
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
        bool convert2Geo(messages_88::Geopoint::Request& req, messages_88::Geopoint::Response& resp);

        // Heartbeat
        void uiHeartbeatCallback(const std_msgs::String::ConstPtr &msg);
        void heartbeatTimerCallback(const ros::TimerEvent&);

        void startNav2PointTask(messages_88::NavToPointGoal &nav_goal);
        void sendExploreTask(messages_88::ExploreGoal &goal);
        void startExploreTask();
        void stop();
        void modeMonitor();

        void syncedPoseCallback(const geometry_msgs::PoseStampedConstPtr &mavros_pose, const geometry_msgs::PoseStampedConstPtr &slam_pose);
        void deccoPoseCallback(const geometry_msgs::PoseStampedConstPtr &slam_pose);
        bool pauseOperations();

        // Health subscribers, unused except to verify publishing
        void costmapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr &msg);
        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void mapirCallback(const sensor_msgs::ImageConstPtr &msg);
        void rosbagCallback(const std_msgs::StringConstPtr &msg);

    private:
        enum CurrentStatus
        {
            ON_START,
            EXPLORING,
            WAITING_TO_EXPLORE,
            HOVERING,
            NAVIGATING,
            RTL_88,
            TAKING_OFF,
            LANDING
        };

        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        // Map params
        tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        bool map_tf_init_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        double utm_x_offset_;
        double utm_y_offset_;
        double map_yaw_;
        int home_utm_zone_;

        // Frames
        std::string mavros_map_frame_;
        std::string slam_map_frame_;
        geometry_msgs::TransformStamped map_to_slam_tf_;
        geometry_msgs::TransformStamped slam_to_map_tf_;
        ros::Subscriber decco_pose_sub_;
        ros::Publisher vision_pose_publisher_;

        // TF publisher
        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        std::string slam_pose_topic_;
        message_filters::Subscriber<geometry_msgs::PoseStamped> mavros_pose_sub_;
        message_filters::Subscriber<geometry_msgs::PoseStamped> slam_pose_sub_;

        // Drone state and services
        drone_state_manager::DroneStateManager drone_state_manager_;
        ros::ServiceServer drone_state_service_;
        ros::ServiceServer drone_ready_service_;
        ros::ServiceServer drone_explore_service_;
        ros::ServiceServer drone_position_service_;
        ros::ServiceServer emergency_service_;
        ros::ServiceServer geopoint_service_;

        // Heartbeat
        ros::Publisher heartbeat_pub_;
        ros::Subscriber ui_heartbeat_subscriber_;
        ros::Time last_ui_heartbeat_stamp_;
        float ui_hb_threshold_;
        ros::Timer heartbeat_timer_;

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

        // Health params and subscribers (for topics not already present)
        ros::Duration health_check_s_;
        ros::Time last_mavros_pos_stamp_;
        ros::Time last_slam_pos_stamp_;
        ros::Time last_costmap_stamp_;
        ros::Time last_lidar_stamp_;
        ros::Time last_mapir_stamp_;
        ros::Time last_rosbag_stamp_;
        ros::Subscriber costmap_sub_;
        ros::Subscriber lidar_sub_;
        ros::Subscriber mapir_sub_;
        ros::Subscriber rosbag_sub_;
        ros::Publisher health_pub_;
        std::string costmap_topic_;
        std::string lidar_topic_;
        std::string mapir_topic_;
        std::string rosbag_topic_;
        ros::Timer health_pub_timer_;

        // Saving
        std::string directory_;
        ros::ServiceClient vegetation_save_client_;
        ros::ServiceClient tree_save_client_;
        bool did_save_;
        bool did_takeoff_;

        ros::Publisher local_pos_pub_;
        ros::Publisher local_vel_pub_;

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
        geometry_msgs::Polygon transformPolygon(const geometry_msgs::Polygon &map_poly);
        bool isInside(const geometry_msgs::Polygon& polygon, const geometry_msgs::Point& point);
        bool polygonDistanceOk(double &min_dist, geometry_msgs::PoseStamped &target, geometry_msgs::Polygon &map_region);
        void padNavTarget(geometry_msgs::PoseStamped &target);
        std::string getStatusString();
        void publishHealth();

};

}

#endif