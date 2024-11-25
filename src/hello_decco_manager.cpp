/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/hello_decco_manager.h"

#include "mavros_msgs/msg/command_code.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "task_manager/decco_utilities.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/PolygonArea.hpp>

#include <boost/filesystem.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> BgPoint;
typedef bg::model::polygon<BgPoint> BgPolygon;

namespace hello_decco_manager
{
HelloDeccoManager::HelloDeccoManager(const double flightleg_acres, const std::string mavros_map_frame)
    : mavros_map_frame_(mavros_map_frame)
    , flightleg_area_m2_(2023.0)
    , elevation_init_(false)
{

    flightleg_area_m2_ = 4046.86 * flightleg_acres;
}

HelloDeccoManager::~HelloDeccoManager() {
}

std_msgs::msg::String HelloDeccoManager::packageToTymbalHD(std::string topic, json gossip, const rclcpp::Time timestamp) {
    json msg_json;
    msg_json["topic"] = topic;
    json stamped_gossip = gossip;
    stamped_gossip["stamp"] = decco_utilities::rosTimeToMilliseconds(timestamp);
    msg_json["gossip"] = stamped_gossip;
    std::string s = msg_json.dump();
    auto msg_string = std_msgs::msg::String();
    msg_string.data = s;

    return msg_string;
}

std_msgs::msg::String HelloDeccoManager::packageToTymbalPuddle(std::string topic, json gossip) {
    json msg_json;
    msg_json["endpoint"] = topic;
    msg_json["method"] = "POST";
    msg_json["body"] = gossip;
    std::string s = msg_json.dump();
    std_msgs::msg::String msg_string;
    msg_string.data = s;
    return msg_string;
}

std_msgs::msg::String HelloDeccoManager::flightReceipt(json msgJson, const rclcpp::Time timestamp) {
    return packageToTymbalHD("confirmation", msgJson, timestamp);
}

std_msgs::msg::String HelloDeccoManager::rejectFlight(json msgJson, const rclcpp::Time timestamp) {
    return flightReceipt(msgJson, timestamp);
    // Parse data
    RCLCPP_INFO(rclcpp::get_logger("hello_decco_manager"), "Flight received was rejected.");
}

std_msgs::msg::String HelloDeccoManager::acceptFlight(json msgJson, geometry_msgs::msg::Polygon &poly, bool &poly_valid, double &home_elevation, const rclcpp::Time timestamp, bool &has_elevation) {
    flightReceipt(msgJson, timestamp);
    // Parse data
    flight_json_ = msgJson["gossip"];
    RCLCPP_INFO(rclcpp::get_logger("hello_decco_manager"), "Flight received");
    poly = polygonFromJson(flight_json_["subpolygon"]["coordinates"][0]);
    poly_valid = poly.points.size() > 2;

    // Convert polygon to map coordinates and visualize
    map_region_ = polygonToMap(poly);
    has_elevation = getHomeElevation(home_elevation);

    return packageToTymbalHD("burn_unit_receive", flight_json_, timestamp);
}

bool HelloDeccoManager::elevationInitializer() {
    // TODO get tif from HD, for now assumes stored in dem/<burn unit name>
    std::string tif_name;
    try {
        std::string burn_unit_name = static_cast<std::string>(flight_json_["burnUnitName"]);
        tif_name = ament_index_cpp::get_package_share_directory("task_manager") + "/dem/" + burn_unit_name + ".tif";
    }
    catch (...) {
        tif_name = ament_index_cpp::get_package_share_directory("task_manager") + "/dem/bigilly2.tif";
    }

    if (!boost::filesystem::exists(tif_name)){
        return false;
    }
    elevation_source_.init(tif_name);
    elevation_init_ = true;
    return true;
}

bool HelloDeccoManager::getElevationChunk(const double utm_x, const double utm_y, const int width, const int height, sensor_msgs::msg::Image &chunk, double &max, double &min) {
    cv::Mat mat;
    if (!elevation_init_) {
        bool has_elev = elevationInitializer();
        if (!has_elev) return false;
    }
    bool worked = elevation_source_.getElevationChunk(utm_x, utm_y, width, height, mat);
    if (worked) {
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(mat, &min, &max, &minLoc, &maxLoc);
        std_msgs::msg::Header header;
        header.frame_id = "world";
        sensor_msgs::msg::Image::SharedPtr chunk = cv_bridge::CvImage(header, "bgr8", mat).toImageMsg();
    }
    return worked;
}

bool HelloDeccoManager::getHomeElevation(double &value) {
    if (!elevation_init_) {
        bool has_elev = elevationInitializer();
        if (!has_elev) return false;
    }
    value = elevation_source_.getElevation(-1 * utm_x_offset_, -1 * utm_y_offset_);
    return true;
}

bool HelloDeccoManager::getElevationValue(const double utm_x, const double utm_y, double &value) {
    if (!elevation_init_) {
        bool has_elev = elevationInitializer();
        if (!has_elev) return false;
    }
    value = elevation_source_.getElevation(utm_x, utm_y);
    return true;
}

std_msgs::msg::String HelloDeccoManager::updateFlightStatus(std::string flight_status, const rclcpp::Time timestamp) {
    flight_json_["status"] = flight_status;
    if (flight_status == "ACTIVE") {
        start_time_ = decco_utilities::rosTimeToMilliseconds(timestamp);
        flight_json_["startTime"] = std::to_string(start_time_);
    }
    else if (flight_status == "COMPLETED") {
        end_time_ = decco_utilities::rosTimeToMilliseconds(timestamp);
        flight_json_["endTime"] = std::to_string(end_time_);
        flight_json_["duration"] = std::to_string(end_time_ - start_time_);
    }
    RCLCPP_INFO(rclcpp::get_logger("hello_decco_manager"), "Burn json filled in");
    return packageToTymbalHD("flight_receive", flight_json_, timestamp);
}

geometry_msgs::msg::Polygon HelloDeccoManager::polygonFromJson(json jsonPolygon) {
    geometry_msgs::msg::Polygon polygon;
    
    if (jsonPolygon.size() < 3) {
        RCLCPP_WARN(rclcpp::get_logger("hello_decco_manager"), "Invalid JSON polygon, returning empty");
        return polygon;
    }

    for (auto& element : jsonPolygon) {
        geometry_msgs::msg::Point32 pt;
        pt.y = element[0];
        pt.x = element[1];
        polygon.points.push_back(pt);
    }
    return polygon;
}

visualization_msgs::msg::Marker HelloDeccoManager::visualizePolygon(const rclcpp::Time timestamp) {
    visualization_msgs::msg::Marker m;

    // Repost first marker at end to close the loop
    if (map_region_.points.size() < 3) {
        RCLCPP_WARN(rclcpp::get_logger("hello_decco_manager"), "Invalid polygon for visualization, returning empty marker");
        return m;
    }

    m.scale.x = 2.0;
    m.header.frame_id = mavros_map_frame_;

    m.header.stamp = timestamp;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.id = 0;

    for (unsigned ii = 0; ii < map_region_.points.size(); ii++) {
        // Add marker
        geometry_msgs::msg::Point p;
        p.x = map_region_.points.at(ii).x;
        p.y = map_region_.points.at(ii).y;
        m.points.push_back(p);
    }

    geometry_msgs::msg::Point p = m.points.at(0);
    m.points.push_back(p);

    return m;
}

geometry_msgs::msg::Polygon HelloDeccoManager::polygonToMap(const geometry_msgs::msg::Polygon &polygon) {
    // Converts lat/lon polygon to local map polygon
    geometry_msgs::msg::Polygon map_polygon;
    for (unsigned ii = 0; ii < polygon.points.size(); ii++) {
        geometry_msgs::msg::Point32 poly_point;
        double px, py;
        llToMap(polygon.points.at(ii).x, polygon.points.at(ii).y, px, py);
        poly_point.x = px;
        poly_point.y = py;
        map_polygon.points.push_back(poly_point);
    }
    return map_polygon;
}

bool HelloDeccoManager::polygonToGeofence(const geometry_msgs::msg::Polygon &polygon, std::shared_ptr<mavros_msgs::srv::WaypointPush::Request> &request) {

    if (polygon.points.size() < 3) {
        RCLCPP_WARN(rclcpp::get_logger("hello_decco_manager"), "Invalid polygon, not creating geofence");
        return false;
    }

    request->start_index = 0;

    // Convert drone location to point32 instead of posestamped
    geometry_msgs::msg::Point32 drone_location;
    drone_location.x = drone_location_.pose.position.x;
    drone_location.y = drone_location_.pose.position.y;
    drone_location.z = drone_location_.pose.position.z;

    // Convert to map first so that we can calculate distances
    geometry_msgs::msg::Polygon polygon_map;
    for (const auto &point : polygon.points) {
        double px, py;
        llToMap(point.x, point.y, px, py);
        geometry_msgs::msg::Point32 point_map;
        point_map.x = px;
        point_map.y = py;
        polygon_map.points.push_back(point_map);
    }

    // If decco is not inside polygon, adjust polygon to include drone location
    if (!decco_utilities::isInside(polygon_map, drone_location)) {

        RCLCPP_INFO(rclcpp::get_logger("hello_decco_manager"), "Drone not inside polygon, adjusting polygon geofence");

        unsigned n1 = polygon_map.points.size();
        double minDist = std::numeric_limits<double>::max();
        unsigned closest_point_ind = -1;

        // Find closest point in polygon to drone
        for (unsigned i = 0; i < n1; ++i) {
            double d = decco_utilities::distance_xy(polygon_map.points[i], drone_location);
            if (d < minDist) {
                minDist = d;
                closest_point_ind = i;
            }
        }

        // Find line intersection with each segment connecting to closest point
        geometry_msgs::msg::Point32 point1, point2, closest_point = polygon_map.points.at(closest_point_ind);
        unsigned ind1, ind2;
        if (closest_point_ind == 0) {
            ind1 = polygon_map.points.size() - 1;
        }
        else {
            ind1 = closest_point_ind - 1;
        }
        if (closest_point_ind == polygon_map.points.size() - 1) {
            ind2 = 0;
        }
        else {
            ind2 = closest_point_ind + 1;
        }
        point1 = polygon_map.points.at(ind1);
        point2 = polygon_map.points.at(ind2);

        // Compute intersections
        
        geometry_msgs::msg::Point32 intersection_point_1;
        bool intersection1 = decco_utilities::intersectsOrthogonal(closest_point, point1, drone_location, intersection_point_1);

        geometry_msgs::msg::Point32 intersection_point_2;
        bool intersection2 = decco_utilities::intersectsOrthogonal(closest_point, point2, drone_location, intersection_point_2);

        // Determine where to put the geofence, in meters away from the drone
        double buffer_dist = 5.0;

        // If both lines have intersection, choose to act on the closer one by setting the other 'intersection' flag to false
        if (intersection1 && intersection2) {

            // Add point 'behind' drone
            double length_1 = decco_utilities::distance_xy(intersection_point_1, drone_location);
            double length_2 = decco_utilities::distance_xy(intersection_point_2, drone_location);

            if (length_1 <= length_2) {
                intersection2 = false;
            }
            else {
                intersection1 = false;
            }
        }
        
        if (intersection1) {
            // Put geofence to point 1

            // Add point 'behind' drone
            geometry_msgs::msg::Point32 point_corr;
            point_corr.x = intersection_point_1.x;
            point_corr.y = intersection_point_1.y;

            double length = decco_utilities::distance_xy(point_corr, drone_location);

            geometry_msgs::msg::Vector3 unit_v;
        
            unit_v.x = (point_corr.x - drone_location.x) / length;
            unit_v.y = (point_corr.y - drone_location.y) / length;

            geometry_msgs::msg::Point32 buffer_point;
            buffer_point.x = drone_location.x - buffer_dist * unit_v.x;
            buffer_point.y = drone_location.y - buffer_dist * unit_v.y;

            // Insert into geofence polygon
            polygon_map.points.insert(polygon_map.points.begin() + closest_point_ind, buffer_point);
        }
        else if (intersection2) {
            // Put geo fence to point 2

            // Add point 'behind' drone
            geometry_msgs::msg::Point32 point_corr;
            point_corr.x = intersection_point_2.x;
            point_corr.y = intersection_point_2.y;

            double length = decco_utilities::distance_xy(point_corr, drone_location);

            geometry_msgs::msg::Vector3 unit_v;

            unit_v.x = (point_corr.x - drone_location.x) / length;
            unit_v.y = (point_corr.y - drone_location.y) / length;

            geometry_msgs::msg::Point32 buffer_point;
            buffer_point.x = drone_location.x - buffer_dist * unit_v.x;
            buffer_point.y = drone_location.y - buffer_dist * unit_v.y;

            // Insert into geofence polygon
            polygon_map.points.insert(polygon_map.points.begin() + ind2, buffer_point);
        }
        else {
            // Add point 'behind' drone
            double length = decco_utilities::distance_xy(closest_point, drone_location);

            geometry_msgs::msg::Vector3 unit_v;

            unit_v.x = (closest_point.x - drone_location.x) / length;
            unit_v.y = (closest_point.y - drone_location.y) / length;

            double buffer_dist = 5;
            geometry_msgs::msg::Point32 buffer_point;
            buffer_point.x = drone_location.x - buffer_dist * unit_v.x;
            buffer_point.y = drone_location.y - buffer_dist * unit_v.y;

            // Replace closest point in fence with drone buffer point
            polygon_map.points[closest_point_ind] = buffer_point;
        }
    }

    // Check if polygon is valid using boost library
    BgPolygon bg_poly;
    for (const auto &point : polygon_map.points) {
        BgPoint bgpoint(point.x, point.y);
        bg_poly.outer().push_back(bgpoint);
    }

    // Push back first point to close the polygon for validity checking
    bg_poly.outer().push_back(bg_poly.outer().front());
    bg::correct(bg_poly);

    std::string reason;
    if (!bg::is_valid(bg_poly, reason)) {
        RCLCPP_ERROR(rclcpp::get_logger("hello_decco_manager"), "Geofence polgyon invalid due to %s, not setting geofence", reason.c_str());
        return false;
    }

    // Now convert polygon to LL to use as geofence
    geometry_msgs::msg::Polygon geofence_polygon;
    for (unsigned i = 0; i < polygon_map.points.size(); i++) {
            
        // Convert map frame polygon to lat/lon
        double lat, lon;
        decco_utilities::mapToLl(polygon_map.points[i].x, polygon_map.points[i].y, lat, lon,
                                    utm_x_offset_, utm_y_offset_, utm_zone_);

        geometry_msgs::msg::Point32 geofence_point;
        geofence_point.x = lat;
        geofence_point.y = lon;
        geofence_polygon.points.push_back(geofence_point);

    }

    // Now push all points to waypoint struct
    for (unsigned i = 0; i < geofence_polygon.points.size(); i++) {
        mavros_msgs::msg::Waypoint wp;
        wp.command = mavros_msgs::msg::CommandCode::NAV_FENCE_POLYGON_VERTEX_INCLUSION;

        // Param1 is for number of fence points
        wp.param1 = geofence_polygon.points.size();

        wp.x_lat = geofence_polygon.points[i].x;
        wp.y_long = geofence_polygon.points[i].y;

        request->waypoints.push_back(wp);
    }

    return true;
}

void HelloDeccoManager::llToMap(const double lat, const double lon, double &px, double &py) {
    decco_utilities::llToMap(lat, lon, px, py, utm_x_offset_, utm_y_offset_);
}

void HelloDeccoManager::mapToLl(const double px, const double py, double &lat, double &lon) {
    decco_utilities::mapToLl(px, py, lat, lon, utm_x_offset_, utm_y_offset_, utm_zone_);
}

}