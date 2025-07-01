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
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "mavros_msgs/msg/open_drone_id_basic_id.hpp"
#include "mavros_msgs/msg/open_drone_id_operator_id.hpp"
#include "mavros_msgs/msg/open_drone_id_self_id.hpp"
#include "mavros_msgs/msg/open_drone_id_system.hpp"
#include "mavros_msgs/msg/open_drone_id_system_update.hpp"

#include "messages_88/action/nav_to_point.hpp"
#include "messages_88/msg/frontier.hpp"
#include "messages_88/msg/task_status.hpp"
#include "messages_88/srv/emergency.hpp"
#include "messages_88/srv/geopoint.hpp"
#include "messages_88/srv/get_map_data.hpp"
#include "messages_88/srv/save.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "elevation_manager.h"
#include "flight_controller_interface.h"
#include "json.hpp"
using json = nlohmann::json;

namespace task_manager {
/**
 * @class TaskManager
 * @brief The TaskManager manages the task queue (e.g., navigate to polygon, handle
 * emergency). Handles comms to/from UI, including task assignment, starting capabilities, and
 * safety features based on drone status.
 */
class TaskManager : public rclcpp::Node {
  public:
    TaskManager(std::shared_ptr<flight_controller_interface::FlightControllerInterface> fci);
    ~TaskManager();

    enum MissionType {
        NONE,
        LAWNMOWER,
        TRAIL_FOLLOW,
        SETPOINT
    };

    struct Mission {
        MissionType type;
        bool completed;
        geometry_msgs::msg::Polygon polygon;
        geometry_msgs::msg::Point setpoint;
    };

    enum Task {
        INITIALIZING,
        PREFLIGHT_CHECK,
        READY,
        MANUAL_FLIGHT,
        PAUSE,
        MISSION,
        RTL_88,
        TAKING_OFF,
        LANDING,
        FAILSAFE_LANDING,
        COMPLETE
    };

    enum LogLevel {
        NORMAL,
        INFO,
        WARN,
        ERROR
    };

    void runTaskManager();

    Task getCurrentTask();

    void odidTimerCallback();

    // Subscriber callbacks
    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void slamPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr slam_pose);
    void registeredPclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pathPlannerCallback(const std_msgs::msg::Header::SharedPtr msg);
    void rosbagCallback(const std_msgs::msg::String::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void missionCallback(const std_msgs::msg::String::SharedPtr msg);

  private:
    const std::shared_ptr<rclcpp::Node> nh_;
    rclcpp::TimerBase::SharedPtr task_manager_timer_;
    rclcpp::TimerBase::SharedPtr health_check_timer_;

    // Subclasses
    std::shared_ptr<elevation_manager::ElevationManager> elevation_manager_;
    std::shared_ptr<flight_controller_interface::FlightControllerInterface>
        flight_controller_interface_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_repub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr local_vel_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OpenDroneIDBasicID>::SharedPtr odid_basic_id_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OpenDroneIDOperatorID>::SharedPtr odid_operator_id_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OpenDroneIDSelfID>::SharedPtr odid_self_id_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OpenDroneIDSystem>::SharedPtr odid_system_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OpenDroneIDSystemUpdate>::SharedPtr odid_system_update_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stop_record_pub_;
    rclcpp::Publisher<messages_88::msg::TaskStatus>::SharedPtr task_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tymbal_hd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tymbal_puddle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_region_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr tif_grid_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tif_pcl_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trigger_recording_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rest_capabilities_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rest_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rest_log_pub_;

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr path_planner_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rosbag_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tymbal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr registered_cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr burn_unit_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rest_mission_sub_;

    struct HealthChecks {
        bool battery_ok;
        bool slam_ok;
        bool path_ok;
        bool rosbag_ok;
    } health_checks_;

    std::string path_planner_topic_;
    std::string lidar_topic_;
    std::string rosbag_topic_;

    rclcpp::Duration lidar_timeout_;
    rclcpp::Duration vision_pose_timeout_;
    rclcpp::Duration path_timeout_;
    rclcpp::Duration rosbag_timeout_;
    rclcpp::Time last_path_planner_stamp_;
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
    bool do_trail_;

    // Offline handling
    bool offline_;

    // UTM PCD saving
    bool utm_tf_init_;

    // Control defaults
    double target_altitude_;
    double target_agl_;
    double min_altitude_;
    double min_agl_;
    double max_altitude_;
    double max_agl_;
    double altitude_offset_;
    double home_elevation_;
    double max_dist_to_polygon_;
    double flightleg_area_acres_;

    // TF
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    bool map_tf_init_;
    int home_utm_zone_;
    double home_utm_x_;
    double home_utm_y_;
    rclcpp::TimerBase::SharedPtr utm_tf_update_timer_;
    double largest_distance_;
    double avg_angle_diff_;
    int angle_diff_count_;

    // Frames
    std::string mavros_map_frame_;
    std::string mavros_base_frame_;
    std::string slam_map_frame_;
    geometry_msgs::msg::TransformStamped map_to_slam_tf_;

    std::string slam_pose_topic_;
    double lidar_pitch_;
    double lidar_x_;
    double lidar_z_;

    // Heartbeat
    rclcpp::Time last_ui_heartbeat_stamp_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // Record
    bool do_record_;
    bool recording_;
    std::string record_config_file_;

    // Drone state params
    geometry_msgs::msg::PoseStamped slam_pose_;
    geometry_msgs::msg::PoseStamped home_pos_;
    geometry_msgs::msg::Polygon map_polygon_;
    rclcpp::TimerBase::SharedPtr mode_monitor_timer_;
    std::string cmd_history_;
    messages_88::msg::TaskStatus task_msg_;
    std::vector<geometry_msgs::msg::PoseStamped> lawnmower_points_;
    bool lawnmower_started_;
    bool setpoint_started_;

    geometry_msgs::msg::PoseStamped goal_;
    double estimated_drone_speed_;
    double battery_failsafe_safety_factor_;
    bool needs_takeoff_;
    int takeoff_attempts_;

    // State
    bool initialized_;
    bool is_armed_;
    bool in_autonomous_flight_;
    bool has_setpoint_;

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

    // Burn unit handling
    std::string data_directory_;
    std::string burn_unit_name_;

    rclcpp::Service<messages_88::srv::Geopoint>::SharedPtr geopoint_service_;
    rclcpp::Service<messages_88::srv::GetMapData>::SharedPtr elevation_map_service_;

    // MAVROS geofence publisher
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr mavros_geofence_client_;

    // Missions
    Mission current_mission_;
    std::string perception_path_;
    bool has_mission_;
    Mission mission_;

    // Capabilities
    std::map<std::string,bool> perception_status_;
    std::map<std::string,std::vector<std::string>> perception_hardware_;

    // Status
    int num_cameras_;

    // Task methods
    void updateCurrentTask(Task task);
    void startTakeoff();
    void startRtl88();
    void startLanding();
    void startFailsafeLanding();
    void startPause();
    void startTrailFollowing(bool start);

    // Capabilities
    void checkMissions();
    void loadPerceptionRegistry();
    MissionType getMissionType(std::string mission_type);
    void startMission();
    bool parseMission(json mission_json);

    // Other methods
    void updateStatus();
    void publishLog(LogLevel level, const std::string &message);
    bool isBatteryOk();
    void checkHealth();
    void checkFailsafes();
    void initialize();
    void checkArmStatus();
    bool pauseOperations();
    void startRecording();
    void stopRecording();
    std::string getTaskString(Task task);
    bool getLawnmowerPattern(const geometry_msgs::msg::Polygon &polygon,
                             std::vector<geometry_msgs::msg::PoseStamped> &lawnmower_points);
    bool getLawnmowerGoal();
    bool lawnmowerGoalComplete();
    void visualizeLawnmower();
    void publishTif();
    void updateUTMTF();

    // Service server callbacks and helpers
    bool convert2Geo(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                     const std::shared_ptr<messages_88::srv::Geopoint::Request> req,
                     const std::shared_ptr<messages_88::srv::Geopoint::Response> resp);
    void map2UtmPoint(geometry_msgs::msg::PointStamped &in, geometry_msgs::msg::PointStamped &out);
    bool getElevationAtPoint(geometry_msgs::msg::PointStamped &point, double &elevation);
    bool getMapData(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                    const std::shared_ptr<messages_88::srv::GetMapData::Request> req,
                    const std::shared_ptr<messages_88::srv::GetMapData::Response> resp);

};

} // namespace task_manager

#endif