/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


#include "bag_recorder_2/srv/record.hpp"

// #include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/polygon.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "visualization_msgs/msg/marker_array.hpp"


#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "map_msgs/msg/occupancy_grid_update.hpp"

#include "mavros_msgs/msg/basic_id.hpp"
#include "mavros_msgs/msg/operator_id.hpp"
#include "mavros_msgs/msg/self_id.hpp"
#include "mavros_msgs/msg/system.hpp"
#include "mavros_msgs/msg/system_update.hpp"

#include "messages_88/action/explore.hpp"
#include "messages_88/action/nav_to_point.hpp"
#include "messages_88/msg/frontier.hpp"
#include "messages_88/msg/task_status.hpp"
#include "messages_88/srv/emergency.hpp"
#include "messages_88/srv/geopoint.hpp"
#include "messages_88/srv/save.hpp"
#include "messages_88/srv/get_map_data.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "flight_controller_interface.h"
#include "hello_decco_manager.h"
#include "json.hpp"
using json = nlohmann::json;

namespace task_manager {
/**
 * @class TaskManager
 * @brief The TaskManager manages the task queue (e.g., navigate to polygon, explore, handle emergency). Handles comms to/from UI, including task assignment, starting capabilities, and safety features based on drone status.
 */
class TaskManager : public rclcpp::Node
{
    public:

        TaskManager(std::shared_ptr<flight_controller_interface::FlightControllerInterface> fci);
        ~TaskManager();

        enum Task
        {
            INITIALIZING,
            PREFLIGHT_CHECK,
            READY,
            MANUAL_FLIGHT,
            PAUSE,
            EXPLORING,
            LAWNMOWER,
            IN_TRANSIT,
            RTL_88,
            TAKING_OFF,
            LANDING,
            FAILSAFE_LANDING,
            COMPLETE
        };

        enum EventType {
            TASK_STATUS,
            STATE_MACHINE,
            FLIGHT_CONTROL,
            FAILSAFE,
            INFO
        };

        enum SurveyType {
            SUB,
            SUPER
        };

        enum Severity {
            LOW,
            MEDIUM,
            HIGH
        };

        void runTaskManager();

        Task getCurrentTask();

        // Timer callbacks
        void uiHeartbeatCallback(const json &msg);
        void heartbeatTimerCallback();
        void odidTimerCallback();

        // Subscriber callbacks
        void slamPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr slam_pose);
        void registeredPclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void pathPlannerCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void costmapCallback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);
        void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void livoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
        void mapirCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void attolloCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void thermalCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rosbagCallback(const std_msgs::msg::String::SharedPtr msg);
        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void mapYawCallback(const std_msgs::msg::Float64::SharedPtr msg);

    private:
        const std::shared_ptr<rclcpp::Node> nh_;
        rclcpp::TimerBase::SharedPtr task_manager_timer_;
        rclcpp::TimerBase::SharedPtr health_check_timer_;

        // Subclasses
        std::shared_ptr<hello_decco_manager::HelloDeccoManager> hello_decco_manager_;
        std::shared_ptr<flight_controller_interface::FlightControllerInterface> flight_controller_interface_;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 health_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                map_yaw_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       vision_pose_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         pointcloud_repub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       position_setpoint_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             local_vel_pub_;
        rclcpp::Publisher<mavros_msgs::msg::BasicID>::SharedPtr             odid_basic_id_pub_;
        rclcpp::Publisher<mavros_msgs::msg::OperatorID>::SharedPtr          odid_operator_id_pub_;
        rclcpp::Publisher<mavros_msgs::msg::SelfID>::SharedPtr              odid_self_id_pub_;
        rclcpp::Publisher<mavros_msgs::msg::System>::SharedPtr              odid_system_pub_;
        rclcpp::Publisher<mavros_msgs::msg::SystemUpdate>::SharedPtr        odid_system_update_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 stop_record_pub_;
        rclcpp::Publisher<messages_88::msg::TaskStatus>::SharedPtr          task_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr           global_pose_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  marker_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 tymbal_hd_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 tymbal_puddle_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr       map_region_pub_;

        // Subscriptions
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      path_planner_sub_;
        rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      lidar_sub_;
        rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr  livox_lidar_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr            mapir_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr            attollo_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr            thermal_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              rosbag_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             map_yaw_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              tymbal_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    slam_pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      registered_cloud_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    goal_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              burn_unit_sub_;

        struct HealthChecks
        {
            bool battery_ok;
            bool lidar_ok;
            bool slam_ok;
            bool path_ok;
            bool costmap_ok;
            bool explore_ok;
            bool mapir_ok;
            bool attollo_ok;
            bool thermal_ok;
            bool rosbag_ok;
        } health_checks_;
        
        std::string path_planner_topic_;
        std::string costmap_topic_;
        std::string lidar_topic_;
        std::string attollo_topic_;
        std::string mapir_rgb_topic_;
        std::string mapir_topic_;
        std::string thermal_topic_;
        std::string rosbag_topic_;

        rclcpp::Duration lidar_timeout_;
        rclcpp::Duration slam_timeout_;
        rclcpp::Duration path_timeout_;
        rclcpp::Duration costmap_timeout_;
        rclcpp::Duration mapir_timeout_;
        rclcpp::Duration attollo_timeout_;
        rclcpp::Duration thermal_timeout_;
        rclcpp::Duration rosbag_timeout_;
        std::chrono::seconds explore_timeout_;
        rclcpp::Time last_lidar_stamp_;
        rclcpp::Time last_slam_pos_stamp_;
        rclcpp::Time last_path_planner_stamp_;
        rclcpp::Time last_costmap_stamp_;
        rclcpp::Time last_mapir_stamp_;
        rclcpp::Time last_thermal_stamp_;
        rclcpp::Time last_attollo_stamp_;
        rclcpp::Time last_rosbag_stamp_;

        float task_manager_loop_duration_;
        rclcpp::TimerBase::SharedPtr health_pub_timer_;
        rclcpp::Duration health_check_pub_duration_;
        rclcpp::Time last_health_pub_stamp_;
        rclcpp::Time last_preflight_check_log_stamp_;

        // Flags to control various behavior
        bool simulate_;
        bool do_slam_;
        bool enable_autonomy_;
        bool use_failsafes_;

        bool do_attollo_;
        bool do_mapir_;
        bool do_mapir_rgb_;
        bool do_thermal_;

        // Offline handling
        bool offline_;
        
        // UTM PCD saving
        bool save_pcd_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_save_;
        bool utm_tf_init_;    

        // Control defaults
        float target_altitude_;
        float target_agl_;
        float min_altitude_;
        float min_agl_;
        float max_altitude_;
        float max_agl_;
        float altitude_offset_;
        double home_elevation_;
        double max_dist_to_polygon_;
        double flightleg_area_acres_;

        // TF
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        bool map_tf_init_;
        double map_yaw_;
        int home_utm_zone_;

        // Frames
        std::string mavros_map_frame_;
        std::string mavros_base_frame_;
        std::string slam_map_frame_;
        geometry_msgs::msg::TransformStamped map_to_slam_tf_;
    
        rclcpp::TimerBase::SharedPtr map_tf_timer_;
        std::string slam_pose_topic_;
        double lidar_pitch_;
        double lidar_x_;
        double lidar_z_;

        // PCL republisher
        geometry_msgs::msg::TransformStamped slam_to_map_tf_;

        // Heartbeat
        rclcpp::Time last_ui_heartbeat_stamp_;
        float ui_hb_threshold_;
        rclcpp::TimerBase::SharedPtr heartbeat_timer_;

        // Record
        bool do_record_;
        bool bag_active_;
        std::string record_config_file_;

        // Drone state params
        geometry_msgs::msg::PoseStamped slam_pose_;
        geometry_msgs::msg::PoseStamped home_pos_;
        geometry_msgs::msg::Polygon map_polygon_;
        geometry_msgs::msg::PoseStamped initial_transit_point_;
        rclcpp::TimerBase::SharedPtr mode_monitor_timer_;
        std::string cmd_history_;
        messages_88::msg::TaskStatus task_msg_;

        std::vector<geometry_msgs::msg::PoseStamped> lawnmower_points_;
        bool lawnmower_started_;
        SurveyType survey_type_;
        
        geometry_msgs::msg::PoseStamped goal_;
        double estimated_drone_speed_;
        double battery_failsafe_safety_factor_;
        bool needs_takeoff_;
        int takeoff_attempts_;

        // Explore action
        using Explore = messages_88::action::Explore;
        using ExploreGoalHandle = rclcpp_action::ClientGoalHandle<Explore>;
        messages_88::action::Explore_Goal current_explore_goal_;
        rclcpp_action::Client<Explore>::SharedPtr explore_action_client_;
        rclcpp_action::ClientGoalHandle<Explore>::SharedPtr explore_action_goal_handle_;
        rclcpp_action::ResultCode explore_action_result_;
        void explore_goal_response_callback(const ExploreGoalHandle::SharedPtr & goal_handle);
        void explore_feedback_callback(ExploreGoalHandle::SharedPtr, const std::shared_ptr<const Explore::Feedback> feedback);
        void explore_result_callback(const ExploreGoalHandle::WrappedResult & result);

        // State
        bool is_armed_;
        bool in_autonomous_flight_;

        rclcpp::TimerBase::SharedPtr odid_timer_;
        bool init_remote_id_message_sent_;
        int last_rid_updated_timestamp_;
        std::string operator_id_;

        Task current_task_;
        rclcpp::TimerBase::SharedPtr status_timer_;
        int hd_drone_id_;
        unsigned long start_time_;

        // Explicit UTM param handling
        bool explicit_global_params_;
        geometry_msgs::msg::TransformStamped utm2map_tf_;
        double home_lat_;
        double home_lon_;

        // Burn unit handling
        int current_index_;
        std::string data_directory_;
        std::string burn_unit_name_;

        rclcpp::Service<messages_88::srv::Geopoint>::SharedPtr geopoint_service_;
        rclcpp::Service<messages_88::srv::GetMapData>::SharedPtr elevation_map_service_;

        // MAVROS geofence publisher
        rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr mavros_geofence_client_;

        // Task methods
        void updateCurrentTask(Task task);
        void startTakeoff();
        void startTransit();
        void startExploration();
        void startRtl88();
        void startLanding();
        void startFailsafeLanding();
        void startPause();
        
        // Other methods
        bool isBatteryOk();
        void checkHealth();
        void checkFailsafes();
        bool initialized();
        void checkArmStatus();
        bool pauseOperations();
        void startBag();
        void stopBag();
        bool polygonDistanceOk(geometry_msgs::msg::PoseStamped &target, geometry_msgs::msg::Polygon &map_region);
        void padNavTarget(geometry_msgs::msg::PoseStamped &target);
        std::string getTaskString(Task task);
        std::string getEventTypeString(EventType type);
        std::string getSeverityString(Severity sev);
        void logEvent(EventType type, Severity sev, std::string description);
        void getLawnmowerPattern(const geometry_msgs::msg::Polygon &polygon, std::vector<geometry_msgs::msg::PoseStamped> &lawnmower_points);
        void getLawnmowerGoal();
        bool lawnmowerGoalComplete();
        void visualizeLawnmower();

        // Service server callbacks and helpers
        bool convert2Geo(const std::shared_ptr<rmw_request_id_t>/*request_header*/,
                         const std::shared_ptr<messages_88::srv::Geopoint::Request> req,
                         const std::shared_ptr<messages_88::srv::Geopoint::Response> resp);
        void map2UtmPoint(geometry_msgs::msg::PointStamped &in, geometry_msgs::msg::PointStamped &out);
        bool getMapData(const std::shared_ptr<rmw_request_id_t>/*request_header*/,
                        const std::shared_ptr<messages_88::srv::GetMapData::Request> req,
                        const std::shared_ptr<messages_88::srv::GetMapData::Response> resp);
        void setAltitudeParams(const double max, const double min, const double target);

        // mapversation methods
        void setpointResponse(json &json_msg);
        void emergencyResponse(const std::string severity);
        void altitudesResponse(json &json_msg);
        void remoteIDResponse(json &json);
        void publishHealth();
        json makeTaskJson();
        void acceptFlight(json flight);
        void packageFromTymbal(const std_msgs::msg::String::SharedPtr msg);

};

}

#endif