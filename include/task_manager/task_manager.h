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
#include <livox_ros_driver/CustomMsg.h>
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
#include "task_manager/hello_decco_manager.h"

#include <task_manager/json.hpp>
using json = nlohmann::json;

namespace task_manager {
/**
 * @class TaskManager
 * @brief The TaskManager manages the task queue (e.g., navigate to polygon, explore, handle emergency). Handles comms to/from UI, including task assignment, starting capabilities, and safety features based on drone status.
 */
class TaskManager {

    public:
        TaskManager(ros::NodeHandle& node);
        ~TaskManager();

        void initDroneStateManager();
        bool getReadyForAction();
        void getReadyForExplore();
        bool convert2Geo(messages_88::Geopoint::Request& req, messages_88::Geopoint::Response& resp);

        // void targetPolygonCallback(const geometry_msgs::Polygon::ConstPtr &msg);
        void setpointResponse(json &json_msg);
        void emergencyResponse(const std::string severity);

        void mapTfTimerCallback(const ros::TimerEvent&);
        void mapTfTimerCallbackNoGlobal(const ros::TimerEvent&);
        void failsafe();
        void mapYawCallback(const std_msgs::Float64::ConstPtr &msg);

        // Heartbeat
        void uiHeartbeatCallback(const json &msg);
        void heartbeatTimerCallback(const ros::TimerEvent&);

        void odidTimerCallback(const ros::TimerEvent &);

        void startNav2PointTask(messages_88::NavToPointGoal &nav_goal);
        void sendExploreTask(messages_88::ExploreGoal &goal);
        void startExploreTask();
        void stop();
        void modeMonitor();

        void deccoPoseCallback(const geometry_msgs::PoseStampedConstPtr &slam_pose);
        bool pauseOperations();

        // Pointcloud republisher
        void registeredPclCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

        // Health subscribers, unused except to verify publishing
        void pathPlannerCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void costmapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr &msg);
        void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg);
        void mapirCallback(const sensor_msgs::ImageConstPtr &msg);
        void attolloCallback(const sensor_msgs::ImageConstPtr &msg);
        void thermalCallback(const sensor_msgs::ImageConstPtr &msg);
        void rosbagCallback(const std_msgs::StringConstPtr &msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    private:
        enum CurrentStatus
        {
            ON_START,
            INITIALIZED,
            EXPLORING,
            WAITING_TO_EXPLORE,
            NAVIGATING,
            RTL_88,
            TAKING_OFF,
            LANDING
        };

        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        bool simulate_;

        // Offline handling
        bool offline_;
        ros::Publisher map_yaw_pub_;
        ros::Subscriber map_yaw_sub_;

        // Hello Decco comms
        hello_decco_manager::HelloDeccoManager hello_decco_manager_;
        ros::Subscriber mapver_sub_;
        // ros::Subscriber target_polygon_subscriber_;
        // ros::Subscriber target_setpoint_subscriber_;
        // ros::Subscriber emergency_subscriber_;

        // Safety for enabling control
        bool do_slam_;
        bool enable_autonomy_;
        bool enable_exploration_;
        bool ardupilot_;
        bool use_failsafes_;

        // Control defaults
        float target_altitude_;
        float min_altitude_;
        float max_altitude_;
        double max_dist_to_polygon_;

        // Map params
        tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        bool map_tf_init_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        double map_yaw_;
        int home_utm_zone_;

        // Frames
        std::string mavros_map_frame_;
        std::string mavros_base_frame_;
        std::string slam_map_frame_;
        geometry_msgs::TransformStamped map_to_slam_tf_;
        ros::Subscriber decco_pose_sub_;
        ros::Publisher vision_pose_publisher_;

        // TF publisher
        ros::Timer map_tf_timer_;
        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        std::string slam_pose_topic_;
        message_filters::Subscriber<geometry_msgs::PoseStamped> mavros_pose_sub_;
        message_filters::Subscriber<geometry_msgs::PoseStamped> slam_pose_sub_;

        // PCL republisher
        ros::Subscriber registered_cloud_sub_;
        ros::Publisher pointcloud_repub_;
        geometry_msgs::TransformStamped slam_to_map_tf_;

        // Drone state and services
        drone_state_manager::DroneStateManager drone_state_manager_;
        ros::ServiceServer geopoint_service_;

        // Heartbeat
        // ros::Publisher heartbeat_pub_;
        // ros::Subscriber ui_heartbeat_subscriber_;
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
        geometry_msgs::PoseStamped home_pos_;
        geometry_msgs::Polygon current_polygon_;
        ros::Timer mode_monitor_timer_;
        std::string cmd_history_;
        messages_88::TaskStatus task_msg_;
        ros::Publisher task_pub_;
        // ros::Publisher task_json_pub_;
        geometry_msgs::PoseStamped goal_;
        ros::Subscriber goal_sub_;
        double estimated_drone_speed_;
        double battery_failsafe_safety_factor_;

        actionlib::SimpleActionClient<messages_88::ExploreAction> explore_action_client_;

        // Health params, bools, and subscribers (for topics not already present)
        bool do_attollo_;
        bool do_mapir_;
        bool do_mapir_rgb_;
        bool do_thermal_;
        ros::Duration health_check_s_;
        ros::Time last_path_planner_stamp_;
        ros::Time last_slam_pos_stamp_;
        ros::Time last_costmap_stamp_;
        ros::Time last_lidar_stamp_;
        ros::Time last_mapir_stamp_;
        ros::Time last_thermal_stamp_;
        ros::Time last_attollo_stamp_;
        ros::Time last_rosbag_stamp_;
        ros::Subscriber path_planner_sub_;
        ros::Subscriber costmap_sub_;
        ros::Subscriber lidar_sub_;
        ros::Subscriber mapir_sub_;
        ros::Subscriber attollo_sub_;
        ros::Subscriber rosbag_sub_;
        ros::Subscriber thermal_sub_;
        ros::Publisher health_pub_;
        std::string path_planner_topic_;
        std::string costmap_topic_;
        std::string lidar_topic_;
        std::string attollo_topic_;
        std::string mapir_rgb_topic_;
        std::string mapir_topic_;
        std::string thermal_topic_;
        std::string rosbag_topic_;
        ros::Timer health_pub_timer_;

        // State
        bool did_takeoff_;
        bool is_armed_;

        ros::Publisher local_pos_pub_;
        ros::Publisher local_vel_pub_;

        // Remote ID
        ros::Publisher odid_basic_id_pub_;
        ros::Publisher odid_operator_id_pub_;
        ros::Publisher odid_self_id_pub_;
        ros::Publisher odid_system_pub_;
        ros::Publisher odid_system_update_pub_;
        ros::Timer odid_timer_;

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

        // Explicit UTM param handling
        bool explicit_global_params_;
        ros::Publisher global_pose_pub_;
        geometry_msgs::TransformStamped utm2map_tf_;
        double home_lat_;
        double home_lon_;

        // Burn unit handling
        ros::Subscriber burn_unit_sub_;
        int current_index_;
        std::string burn_dir_prefix_;
        // void makeBurnUnitJson(const std_msgs::String::ConstPtr &msg);
        void makeBurnUnitJson(json burn_unit);

        void packageFromMapversation(const std_msgs::String::ConstPtr &msg);

        void startBag();
        void stopBag();
        void getDroneReady();
        void readyToExplore();
        // geometry_msgs::Polygon transformPolygon(const geometry_msgs::Polygon &map_poly);
        bool isInside(const geometry_msgs::Polygon& polygon, const geometry_msgs::Point& point);
        bool polygonDistanceOk(double &min_dist, geometry_msgs::PoseStamped &target, geometry_msgs::Polygon &map_region);
        void padNavTarget(geometry_msgs::PoseStamped &target);
        std::string getStatusString();
        void publishHealth();
        json makeTaskJson();
        bool isBatteryOk();
        void doRtl88();
};

}

#endif