/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef HELLO_DECCO_MANAGER_H_
#define HELLO_DECCO_MANAGER_H_

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <ConcavePolygon.h>
#include <CentroidSplitter.h>

#include <task_manager/json.hpp>
using json = nlohmann::json;

namespace hello_decco_manager {
/**
 * @class HelloDeccoManager
 * @brief Manages interactions with Hello Decco
 */
class HelloDeccoManager {

    public:
        HelloDeccoManager(ros::NodeHandle& node);
        ~HelloDeccoManager();

        void makeBurnUnitJson(json msgJson, int utm_zone);
        json polygonToBurnUnit(const json &polygon);
        // json polygonToBurnUnit(const geometry_msgs::Polygon &polygon);
        int initBurnUnit(geometry_msgs::Polygon &polygon);
        void updateBurnUnit(int index, std::string flight_status);
        void setFrames(std::string map_frame, std::string slam_frame);
        void mapToGeopoint(const geometry_msgs::PointStamped &point_in, geometry_msgs::PointStamped &point_out, double yaw);
        void utmToLL(const double utm_x, const double utm_y, const int zone, double &lat, double &lon);
        void setUtmOffsets(double utm_x, double utm_y) {
            utm_x_offset_ = -utm_x;
            utm_y_offset_ = -utm_y;
        }
        void packageToMapversation(std::string topic, json gossip);

    private:
        enum FlightStatus {
            NOT_STARTED,
            ACTIVE,
            COMPLETED
        };

        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        // Frames/TF
        std::string mavros_map_frame_;
        std::string slam_map_frame_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        double utm_x_offset_;
        double utm_y_offset_;

        // Mapversation
        ros::Publisher mapver_pub_;

        json burn_unit_json_;
        ros::Publisher map_region_pub_;
        int start_time_;
        int end_time_;

        // MAVROS geofence publisher
        ros::ServiceClient mavros_geofence_client_;

        // Subpolygon creation variables
        std::vector<cxd::Vertex > vertices_;
        geometry_msgs::Polygon map_region_; // Entire unit
        std::vector<geometry_msgs::Polygon> subpolygons_; // Flight units
        double flightleg_area_m2_;

        void polygonInitializer(const geometry_msgs::Polygon &msg, bool make_legs);
        geometry_msgs::Polygon polygonFromJson(json jsonPolygon);

        // Polygon mgmt
        geometry_msgs::Polygon polygonToMap(const geometry_msgs::Polygon &polygon);
        bool polygonToGeofence(const geometry_msgs::Polygon &polygon);
        int polygonNumFlights(const geometry_msgs::Polygon &polygon);
        int concaveToMinimalConvexPolygons();
        void visualizeLegs();
        void visualizePolygon();
        geometry_msgs::Polygon transformPolygon(const geometry_msgs::Polygon &map_poly);
};

}

#endif