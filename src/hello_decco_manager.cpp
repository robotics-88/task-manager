/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/hello_decco_manager.h"

#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointPush.h>
#include <visualization_msgs/Marker.h>

#include <decco_utilities.h>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/PolygonArea.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> BgPoint;
typedef bg::model::polygon<BgPoint> BgPolygon;

namespace hello_decco_manager
{
HelloDeccoManager::HelloDeccoManager(ros::NodeHandle& node)
    : private_nh_("~")
    , nh_(node)
    , mavros_map_frame_("map")
    , slam_map_frame_("slam_map")
    , tf_listener_(tf_buffer_)
    , flightleg_area_m2_(2023.0)
{
    double flightleg_acres;
    private_nh_.param<double>("flightleg_area_acres", flightleg_acres, flightleg_acres);
    flightleg_area_m2_ = 4046.86 * flightleg_acres;
    private_nh_.param<std::string>("mavros_map_frame", mavros_map_frame_, mavros_map_frame_);
    private_nh_.param<std::string>("slam_map_frame", slam_map_frame_, slam_map_frame_);

    mapver_pub_ = nh_.advertise<std_msgs::String>("/mapversation/to_hello_decco", 10);
    mavros_geofence_client_ = nh_.serviceClient<mavros_msgs::WaypointPush>("/mavros/geofence/push");
    map_region_pub_ = nh_.advertise<visualization_msgs::Marker>("/map_region", 10, true);
}

HelloDeccoManager::~HelloDeccoManager() {
}

void HelloDeccoManager::packageToMapversation(std::string topic, json gossip) {
    json msg_json;
    msg_json["topic"] = topic;
    json stamped_gossip = gossip;
    stamped_gossip["stamp"] = ros::Time::now().toSec();
    msg_json["gossip"] = stamped_gossip;
    std::string s = msg_json.dump();
    std_msgs::String msg_string;
    msg_string.data = s;
    mapver_pub_.publish(msg_string); // Mapversation to Hello Decco
}

void HelloDeccoManager::flightReceipt() {
    json msg;
    msg["data"] = "received";
    std::cout << msg.dump(4) << std::endl;
    packageToMapversation("flight_confirm", msg);
}

void HelloDeccoManager::acceptFlight(json msgJson, int utm_zone, bool &geofence_ok) {
    flightReceipt();
    // Parse data
    flight_json_ = msgJson;
    ROS_INFO("Flight received");
    geometry_msgs::Polygon poly = polygonFromJson(flight_json_["subpolygon"]);
    polygonInitializer(poly, false, geofence_ok);

    packageToMapversation("burn_unit_receive", flight_json_);

}

void HelloDeccoManager::makeBurnUnitJson(json msgJson, int utm_zone, bool &geofence_ok) {
    // TODO check trip type to decide what data is recording? Or do that on the HD side?
    flightReceipt();
    // Parse data
    flight_json_ = msgJson;
    ROS_INFO("Burn unit received");
    std_msgs::String burn_string;
    geometry_msgs::Polygon poly = polygonFromJson(flight_json_["polygon"]);
    // Check if already filled in
    int num_flights = flight_json_["trips"][0]["flights"].size();

    if (num_flights >= 1 && flight_json_["trips"][0]["flights"][0]["subpolygon"].size() >= 3) {
        // Already filled in, pass directly to TM
        std::string s = flight_json_.dump();
        burn_string.data = s;
        polygonInitializer(poly, false, geofence_ok);
        // Fill in subpolygon array
        local_subpolygons_.clear();
        geometry_msgs::Polygon ll_poly;
        for (auto subpoly_point: flight_json_["trips"][0]["flights"][0]["subpolygon"]) {
            geometry_msgs::Point32 ll_point;
            ll_point.x = subpoly_point[0];
            ll_point.y = subpoly_point[1];
            ll_poly.points.push_back(ll_point);
        }
        geometry_msgs::Polygon poly = polygonToMap(ll_poly);
        local_subpolygons_.push_back(poly);
    }
    else {
        // Need to fill in
        polygonInitializer(poly, true, geofence_ok);
        json flightLegArray;
        for (int ii = 0; ii < local_subpolygons_.size(); ii++) {
            json flight_leg;
            json ll_json;
            for (int jj = 0; jj < local_subpolygons_.at(ii).points.size(); jj++) {
                json ll;
                double lat, lon;
                decco_utilities::mapToLl(local_subpolygons_.at(ii).points.at(jj).x, local_subpolygons_.at(ii).points.at(jj).y, lat, lon,
                                         utm_x_offset_, utm_y_offset_, utm_zone_);
                ll.push_back(lat);
                ll.push_back(lon);
                ll_json.push_back(ll);
            }
            flight_leg["subpolygon"] = ll_json;
            flight_leg["status"] = "NOT_STARTED";
            flight_leg["startTime"] = 0;
            flight_leg["endTime"] = 0;
            flight_leg["duration"] = 0;
            flightLegArray.push_back(flight_leg);
        }
        flight_json_["trips"][0]["flights"] = flightLegArray;
        packageToMapversation("burn_unit_receive", flight_json_);
    }
}

void HelloDeccoManager::polygonInitializer(const geometry_msgs::Polygon &msg, bool make_legs, bool &geofence_ok) {
    // Convert polygon to map coordinates and visualize
    map_region_ = polygonToMap(msg);
    visualizePolygon();

    if (!polygonToGeofence(msg)) {
        ROS_WARN("Failed to convert polygon to geofence");
        geofence_ok = false;
    }
    else {
        geofence_ok = true;
    }

    if (make_legs) {
        int num_legs = polygonNumFlights(msg);
        ROS_INFO("Polygon for exploration will take %d flights to complete.", num_legs);
    }
}

int HelloDeccoManager::initFlightArea(geometry_msgs::Polygon &polygon) {
    // Iterate through subpolygons to get first not done
    // TODO not needed anymore? HD only sends the next flight?
    int ind = 0;
    geometry_msgs::Polygon poly;
    bool found = false;
    for (auto& flight : flight_json_["trips"][0]["flights"]) {
        if (flight["status"] == "NOT_STARTED") {
            poly = local_subpolygons_.at(ind);
            ROS_INFO("will do polygon %d", ind);
            found = true;
            break;
        }
        ind++;
    }
    if (found) {
        polygon = poly;
    }
    else {
        ind = -1;
    }
    return ind;
}

void HelloDeccoManager::updateFlightStatus(int index, std::string flight_status) {
    flight_json_["status"] = flight_status;
    if (flight_status == "ACTIVE") {
        start_time_ = static_cast<int>(ros::Time::now().toSec());
        flight_json_["startTime"] = std::to_string(start_time_);
    }
    else if (flight_status == "COMPLETED") {
        end_time_ = static_cast<int>(ros::Time::now().toSec());
        flight_json_["endTime"] = std::to_string(end_time_);
        flight_json_["duration"] = std::to_string(end_time_ - start_time_);
    }
    packageToMapversation("flight_receive", flight_json_);
    ROS_INFO("Burn json filled in");
}

geometry_msgs::Polygon HelloDeccoManager::polygonFromJson(json jsonPolygon) {
    geometry_msgs::Polygon polygon;
    for (auto& element : jsonPolygon) {
        geometry_msgs::Point32 pt;
        pt.x = element[0];
        pt.y = element[1];
        polygon.points.push_back(pt);
    }
    return polygon;
}

void HelloDeccoManager::visualizePolygon() {
    visualization_msgs::Marker m;
    m.scale.x = 2.0;
    m.header.frame_id = mavros_map_frame_;
    m.header.stamp = ros::Time::now();
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.id = 0;

    for (int ii = 0; ii < map_region_.points.size(); ii++) {
        // Add marker
        geometry_msgs::Point p;
        p.x = map_region_.points.at(ii).x;
        p.y = map_region_.points.at(ii).y;
        m.points.push_back(p);
    }
    // Repost first marker at end to close the loop
    geometry_msgs::Point p = m.points.at(0);
    m.points.push_back(p);
    map_region_pub_.publish(m);
}

geometry_msgs::Polygon HelloDeccoManager::polygonToMap(const geometry_msgs::Polygon &polygon) {
    // Converts lat/lon polygon to local map polygon
    geometry_msgs::Polygon map_polygon;
    for (int ii = 0; ii < polygon.points.size(); ii++) {
        geometry_msgs::Point32 poly_point;
        double px, py;
        decco_utilities::llToMap(polygon.points.at(ii).x, polygon.points.at(ii).y, px, py, utm_x_offset_, utm_y_offset_);
        poly_point.x = px;
        poly_point.y = py;
        map_polygon.points.push_back(poly_point);
    }
    return map_polygon;
}

bool HelloDeccoManager::polygonToGeofence(const geometry_msgs::Polygon &polygon) {
    mavros_msgs::WaypointPush srv;
    srv.request.start_index = 0;

    // Convert drone location to point32 instead of posestamped
    geometry_msgs::Point32 drone_location;
    drone_location.x = drone_location_.pose.position.x;
    drone_location.y = drone_location_.pose.position.y;
    drone_location.z = drone_location_.pose.position.z;

    // Convert to map first so that we can calculate distances
    geometry_msgs::Polygon polygon_map;
    for (const auto &point : polygon.points) {
        double px, py;
        decco_utilities::llToMap(point.x, point.y, px, py, utm_x_offset_, utm_y_offset_);
        geometry_msgs::Point32 point_map;
        point_map.x = px;
        point_map.y = py;
        polygon_map.points.push_back(point_map);
    }

    // If decco is not inside polygon, adjust polygon to include drone location
    if (!decco_utilities::isInside(polygon_map, drone_location)) {

        ROS_INFO("Drone not inside polygon, adjusting polygon geofence");

        int n1 = polygon_map.points.size();
        double minDist = std::numeric_limits<double>::max();
        int closest_point_ind = -1;

        // Find closest point in polygon to drone
        for (int i = 0; i < n1; ++i) {
            double d = decco_utilities::distance_xy(polygon_map.points[i], drone_location);
            if (d < minDist) {
                minDist = d;
                closest_point_ind = i;
            }
        }

        // Find line intersection with each segment connecting to closest point
        geometry_msgs::Point32 point1, point2, closest_point = polygon_map.points.at(closest_point_ind);
        int ind1, ind2;
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
        
        geometry_msgs::Point32 intersection_point_1;
        bool intersection1 = decco_utilities::intersectsOrthogonal(closest_point, point1, drone_location, intersection_point_1);

        geometry_msgs::Point32 intersection_point_2;
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
            geometry_msgs::Point32 point_corr;
            point_corr.x = intersection_point_1.x;
            point_corr.y = intersection_point_1.y;

            double length = decco_utilities::distance_xy(point_corr, drone_location);

            geometry_msgs::Vector3 unit_v;
        
            unit_v.x = (point_corr.x - drone_location.x) / length;
            unit_v.y = (point_corr.y - drone_location.y) / length;

            geometry_msgs::Point32 buffer_point;
            buffer_point.x = drone_location.x - buffer_dist * unit_v.x;
            buffer_point.y = drone_location.y - buffer_dist * unit_v.y;

            // Insert into geofence polygon
            polygon_map.points.insert(polygon_map.points.begin() + closest_point_ind, buffer_point);
        }
        else if (intersection2) {
            // Put geo fence to point 2

            // Add point 'behind' drone
            geometry_msgs::Point32 point_corr;
            point_corr.x = intersection_point_2.x;
            point_corr.y = intersection_point_2.y;

            double length = decco_utilities::distance_xy(point_corr, drone_location);

            geometry_msgs::Vector3 unit_v;

            unit_v.x = (point_corr.x - drone_location.x) / length;
            unit_v.y = (point_corr.y - drone_location.y) / length;

            geometry_msgs::Point32 buffer_point;
            buffer_point.x = drone_location.x - buffer_dist * unit_v.x;
            buffer_point.y = drone_location.y - buffer_dist * unit_v.y;

            // Insert into geofence polygon
            polygon_map.points.insert(polygon_map.points.begin() + ind2, buffer_point);
        }
        else {
            // Add point 'behind' drone
            double length = decco_utilities::distance_xy(closest_point, drone_location);

            geometry_msgs::Vector3 unit_v;

            unit_v.x = (closest_point.x - drone_location.x) / length;
            unit_v.y = (closest_point.y - drone_location.y) / length;

            double buffer_dist = 5;
            geometry_msgs::Point32 buffer_point;
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
        ROS_ERROR("Geofence polgyon invalid due to %s, not setting geofence", reason.c_str());
        return false;
    }

    // Now convert polygon to LL to use as geofence
    geometry_msgs::Polygon geofence_polygon;
    for (int i = 0; i < polygon_map.points.size(); i++) {
            
        // Convert map frame polygon to lat/lon
        double lat, lon;
        decco_utilities::mapToLl(polygon_map.points[i].x, polygon_map.points[i].y, lat, lon,
                                    utm_x_offset_, utm_y_offset_, utm_zone_);

        geometry_msgs::Point32 geofence_point;
        geofence_point.x = lat;
        geofence_point.y = lon;
        geofence_polygon.points.push_back(geofence_point);

    }

    // Now push all points to waypoint struct
    for (int i = 0; i < geofence_polygon.points.size(); i++) {
        mavros_msgs::Waypoint wp;
        wp.command = mavros_msgs::CommandCode::NAV_FENCE_POLYGON_VERTEX_INCLUSION;

        // Param1 is for number of fence points
        wp.param1 = geofence_polygon.points.size();

        wp.x_lat = geofence_polygon.points[i].x;
        wp.y_long = geofence_polygon.points[i].y;

        srv.request.waypoints.push_back(wp);
    }

    ROS_INFO("Uploading polygon to geofence");
    // Run geofence rosservice call for mavros to upload geofence wps to autopilot
    if (!mavros_geofence_client_.call(srv)) {
        ROS_WARN("Geofence push service call failed");
        return false;
    }

    return srv.response.success;
}

int HelloDeccoManager::polygonNumFlights(const geometry_msgs::Polygon &polygon) {
    GeographicLib::Geodesic geod(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    // Alternatively: const Geodesic& geod = Geodesic::WGS84();
    GeographicLib::PolygonArea poly(geod);
    for (int ii = 0; ii<polygon.points.size(); ii++) {
        poly.AddPoint(polygon.points.at(ii).x, polygon.points.at(ii).y);
    }
    double perimeter, area;
    poly.Compute(false, true, perimeter, area);
    int num_legs = 1;
    local_subpolygons_.clear();
    if (std::abs(area) > flightleg_area_m2_) {
        centroid_splitter::CentroidSplitter splitter(map_region_, flightleg_area_m2_);
        local_subpolygons_ = splitter.slicePolygon();
        num_legs = local_subpolygons_.size();
        visualizeLegs();
    }
    else {
        local_subpolygons_.push_back(map_region_);
    }
    return num_legs;
}

void HelloDeccoManager::visualizeLegs() {
    for (int nn = 0; nn < local_subpolygons_.size(); nn++) {
        // Set up subpolygon viz
        visualization_msgs::Marker m;
        m.scale.x = 2.0;
        m.header.frame_id = mavros_map_frame_;
        m.header.stamp = ros::Time::now();
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.color.a = 1.0;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.id = nn + 1;

        // Compute subpolys
        geometry_msgs::Polygon geom_polygon = local_subpolygons_.at(nn);
        for (int vv = 0; vv < geom_polygon.points.size(); vv++) {
            // Add marker
            geometry_msgs::Point p;
            p.x = geom_polygon.points.at(vv).x;
            p.y = geom_polygon.points.at(vv).y;
            m.points.push_back(p);
        }
        // Repost first marker at end to close the loop
        geometry_msgs::Point p = m.points.at(0);
        m.points.push_back(p);
        map_region_pub_.publish(m);
    }
}

// TODO decide whether to hang on to this polygon splitter
// int HelloDeccoManager::concaveToMinimalConvexPolygons() {
//     cxd::ConcavePolygon concavePoly(vertices_);
//     concavePoly.convexDecomp();
//     std::vector<cxd::ConcavePolygon > subPolygonList;
//     concavePoly.returnLowestLevelPolys(subPolygonList);
//     int num_legs = concavePoly.getNumberSubPolys();
//     for (int nn = 0; nn < subPolygonList.size(); nn++) {
//         // Set up subpolygon viz
//         visualization_msgs::Marker m;
//         m.scale.x = 2.0;
//         m.header.frame_id = mavros_map_frame_;
//         m.header.stamp = ros::Time::now();
//         m.type = visualization_msgs::Marker::LINE_STRIP;
//         m.action = visualization_msgs::Marker::ADD;
//         m.color.a = 1.0;
//         m.color.r = 0.0;
//         m.color.g = 1.0;
//         m.color.b = 0.0;
//         m.id = nn + 1;

//         // Compute subpolys
//         std::vector<cxd::Vertex > subPolyVerts = subPolygonList.at(nn).getVertices();
//         geometry_msgs::Polygon geom_polygon;
//         std::vector<geometry_msgs::Point32> geom_points;
//         for (int vv = 0; vv < subPolyVerts.size(); vv++) {
//             geometry_msgs::Point32 pt;
//             pt.x = subPolyVerts.at(vv).position.x;
//             pt.y = subPolyVerts.at(vv).position.y;
//             pt.z = 0;
//             geom_points.push_back(pt);

//             // Add marker
//             geometry_msgs::Point p;
//             p.x = pt.x;
//             p.y = pt.y;
//             m.points.push_back(p);
//         }
//         geom_polygon.points = geom_points;
//         subpolygons_.push_back(geom_polygon);
//         // Repost first marker at end to close the loop
//         geometry_msgs::Point p = m.points.at(0);
//         m.points.push_back(p);
//         map_region_pub_.publish(m);
//     }
//     return num_legs;
// }

json HelloDeccoManager::polygonToBurnUnit(const json &polygon) {
    json burn_unit = R"({
        "id" : 1,
        "orgId" : 1,
        "createdBy" : 1,
        "name" : "Super Great Burn Unit",
        "polygon" : [],
        "trips": [
            {
            "id" : 1,
            "type" : "PRE",
            "startTime" : 0,
            "endTime" : 0, 
            "flights" : [ 
                {
                    "startTime" : 0,
                    "endTime" : 0, 
                    "duration" : 0,
                    "subpolygon" : [],
                    "flightLogUrl" : "hellodecco.com/my-url",
                    "status" : "NOT_STARTED"
                }
            ]
            }
        ]
        })"_json;
    burn_unit["polygon"] = polygon;
    return burn_unit;
}
}