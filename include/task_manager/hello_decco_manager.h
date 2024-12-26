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
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <cv_bridge/cv_bridge.h>               // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <opencv2/opencv.hpp>                  // We include everything about OpenCV as we don't care much about compilation time at the moment.

#include "ConcavePolygon.h"
#include "CentroidSplitter.h"

#include "task_manager/Elevation2Ros.h"

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
        HelloDeccoManager(const double flightleg_acres, const std::string mavros_map_frame);
        ~HelloDeccoManager();

        std_msgs::msg::String rejectFlight(json msgJson, const rclcpp::Time timestamp);
        std_msgs::msg::String acceptFlight(json msgJson, geometry_msgs::msg::Polygon &polygon, bool &poly_valid, double &home_elevation, const rclcpp::Time timestamp, bool &has_elevation);
        std_msgs::msg::String updateFlightStatus(std::string flight_status, const rclcpp::Time timestamp);
        void setUtm(double utm_x, double utm_y, int zone) {
            utm_x_offset_ = -utm_x;
            utm_y_offset_ = -utm_y;
            utm_zone_ = zone;
        }
        geometry_msgs::msg::Polygon polygonFromJson(json jsonPolygon);
        geometry_msgs::msg::Polygon polygonToMap(const geometry_msgs::msg::Polygon &polygon);
        std_msgs::msg::String packageToTymbalHD(std::string topic, json gossip, const rclcpp::Time timestamp);
        std_msgs::msg::String packageToTymbalPuddle(std::string topic, json gossip);
        void llToMap(const double lat, const double lon, double &px, double &py);
        void mapToLl(const double px, const double py, double &lat, double &lon);

        void setDroneLocationLocal(geometry_msgs::msg::PoseStamped location) {
            drone_location_ = location;
        }

        std_msgs::msg::String flightReceipt(json msgJson, const rclcpp::Time timestamp);

        visualization_msgs::msg::Marker visualizePolygon(const rclcpp::Time timestamp);

        bool polygonToGeofence(const geometry_msgs::msg::Polygon &polygon, std::shared_ptr<mavros_msgs::srv::WaypointPush::Request> &req);

        geometry_msgs::msg::Polygon getMapPolygon() { 
            return map_region_;
        }

        bool getElevationChunk(const double utm_x, const double utm_y, const int width, const int height, sensor_msgs::msg::Image &chunk, double &max, double &min);
        bool getElevationValue(const double utm_x, const double utm_y, double &value);
        bool getHomeElevation(double &value, nav_msgs::msg::OccupancyGrid::SharedPtr tif_grid);
        bool getElevationInit() {
            return elevation_init_;
        }

    private:
        const std::weak_ptr<rclcpp::Node> node_;

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
        double utm_x_offset_;
        double utm_y_offset_;
        int utm_zone_;
        geometry_msgs::msg::PoseStamped drone_location_;

        json flight_json_;
        unsigned long start_time_;
        unsigned long end_time_;

        // Elevation
        elevation2ros::Elevation2Ros elevation_source_;
        bool elevation_init_;

        // Subpolygon creation variables
        std::vector<cxd::Vertex > vertices_;
        geometry_msgs::msg::Polygon map_region_; // Entire unit
        std::vector<geometry_msgs::msg::Polygon> local_subpolygons_; // Flight units
        double flightleg_area_m2_;

        void elevationInitializer(nav_msgs::msg::OccupancyGrid::SharedPtr tif_grid);
};

}

#endif