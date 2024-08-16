/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef HELLO_DECCO_MANAGER_H_
#define HELLO_DECCO_MANAGER_H_

#include "rclcpp/rclcpp.hpp"

#include "mavros_msgs/srv/waypoint_push.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "ConcavePolygon.h"
#include "CentroidSplitter.h"

#include "task_manager/json.hpp"
using json = nlohmann::json;

namespace hello_decco_manager {
/**
 * @class HelloDeccoManager
 * @brief Manages interactions with Hello Decco
 */
class HelloDeccoManager
{
    public:
        explicit HelloDeccoManager(const std::shared_ptr<rclcpp::Node> nh);
        ~HelloDeccoManager();

        void acceptFlight(json msgJson, bool &geofence_ok);
        void updateFlightStatus(std::string flight_status);
        void setUtm(double utm_x, double utm_y, int zone) {
            utm_x_offset_ = -utm_x;
            utm_y_offset_ = -utm_y;
            utm_zone_ = zone;
        }
        geometry_msgs::msg::Polygon polygonFromJson(json jsonPolygon);
        geometry_msgs::msg::Polygon polygonToMap(const geometry_msgs::msg::Polygon &polygon);
        void packageToTymbalHD(std::string topic, json gossip);
        void packageToTymbalPuddle(std::string topic, json gossip);

        void setDroneLocationLocal(geometry_msgs::msg::PoseStamped location) {
            drone_location_ = location;
        }

        geometry_msgs::msg::Polygon getMapPolygon() { 
            return map_region_;
        }

    private:
        const std::shared_ptr<rclcpp::Node> nh_;

        enum FlightStatus {
            NOT_STARTED,
            ACTIVE,
            COMPLETED
        };

        enum HttpMethod {
            GET,
            POST,
            PUT
        };

        // Frames/TF
        std::string mavros_map_frame_;
        std::string slam_map_frame_;
        double utm_x_offset_;
        double utm_y_offset_;
        int utm_zone_;
        geometry_msgs::msg::PoseStamped drone_location_;

        // tymbal
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tymbal_hd_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tymbal_puddle_pub_;

        json flight_json_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_region_pub_;
        unsigned long start_time_;
        unsigned long end_time_;

        // MAVROS geofence publisher
        rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr mavros_geofence_client_;

        // Subpolygon creation variables
        std::vector<cxd::Vertex > vertices_;
        geometry_msgs::msg::Polygon map_region_; // Entire unit
        std::vector<geometry_msgs::msg::Polygon> local_subpolygons_; // Flight units
        double flightleg_area_m2_;

        void flightReceipt();
        void polygonInitializer(const geometry_msgs::msg::Polygon &msg, bool make_legs, bool &geofence_ok);

        // Polygon mgmt
        bool polygonToGeofence(const geometry_msgs::msg::Polygon &polygon);
        int polygonNumFlights(const geometry_msgs::msg::Polygon &polygon);
        int concaveToMinimalConvexPolygons();
        void visualizeLegs();
        void visualizePolygon();
};

}

#endif