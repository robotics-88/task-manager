/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/hello_decco_manager.h"

#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointPush.h>
#include <visualization_msgs/Marker.h>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/PolygonArea.hpp>

namespace hello_decco_manager
{
HelloDeccoManager::HelloDeccoManager(ros::NodeHandle& node)
    : private_nh_("~")
    , nh_(node)
    , mavros_map_frame_("map")
    , slam_map_frame_("slam_map")
    , tf_listener_(tf_buffer_)
    , flightleg_area_(2023.0)
{
    nh_.param<double>("flightleg_area_m2", flightleg_area_, flightleg_area_);

    burn_unit_pub_ = nh_.advertise<std_msgs::String>("/mapversation/burn_unit_receive", 10);
    mavros_geofence_client_ = nh_.serviceClient<mavros_msgs::WaypointPush>("/mavros/geofence/push");
    map_region_pub_ = nh_.advertise<visualization_msgs::Marker>("/map_region", 10);
}

HelloDeccoManager::~HelloDeccoManager() {
}

void HelloDeccoManager::setFrames(std::string map_frame, std::string slam_frame) {
    mavros_map_frame_ = map_frame;
    slam_map_frame_ = slam_frame;
}

void HelloDeccoManager::makeBurnUnitJson(json msgJson, int utm_zone) {
    // TODO check trip type to decide what data is recording? Or do that on the HD side?
    // Parse data
    burn_unit_json_ = msgJson;
    std_msgs::String burn_string;
    // Check if already filled in
    int num_flights = burn_unit_json_["trips"][0]["flights"].size();
    if (num_flights > 1 || (num_flights == 1 && burn_unit_json_["trips"][0]["flights"][0]["subpolygon"].size() > 3)) {
        // Already filled in, pass directly to TM
        std::cout << "burn json was: \n" << burn_unit_json_.dump(4) << std::endl;
        std::string s = burn_unit_json_.dump();
        burn_string.data = s;
    }
    else {
        // Need to fill in
        geometry_msgs::Polygon poly = polygonFromJson(burn_unit_json_["polygon"]);
        polygonInitializer(poly); // TODO use index to decide which flight leg to send to explore
        json flightLegArray;
        for (int ii = 0; ii < subpolygons_.size(); ii++) {
            json flight_leg = burn_unit_json_["trips"][0]["flights"][0]; // Should be received with 1 empty flight leg;
            flight_leg["index"] = ii;
            json ll_json;
            for (int jj = 0; jj < subpolygons_.at(ii).points.size(); jj++) {
                json ll;
                double xval = subpolygons_.at(ii).points.at(jj).x - utm_x_offset_;
                double yval = subpolygons_.at(ii).points.at(jj).y - utm_y_offset_;
                double lat, lon, gamma, k;
                GeographicLib::UTMUPS::Reverse(utm_zone, true, xval, yval, lat, lon, gamma, k);
                // GeographicLib::GeoCoords c(utm_zone, xval, yval, lat, lon, gamma, k);
                ll.push_back(lat);
                ll.push_back(lon);
                ll_json.push_back(ll);
            }
            flight_leg["subpolygon"] = ll_json;
            flightLegArray.push_back(flight_leg);
            // GOnna end up with a blank first leg, fix it
        }
        burn_unit_json_["trips"][0]["flights"] = flightLegArray;
        std::string s = burn_unit_json_.dump();
        burn_string.data = s;
        burn_unit_pub_.publish(burn_string); // Hello Decco
        std::cout << "burn json filled in: \n" << burn_unit_json_.dump(4) << std::endl;
    }
}

void HelloDeccoManager::polygonInitializer(const geometry_msgs::Polygon &msg) {
    // Convert polygon to map coordinates and store
    if (!polygonToMap(msg)) {
        ROS_WARN("Failed to convert polygon to map, polygon ignored.");
        return;
    }

    if (!polygonToGeofence(msg)) {
        ROS_WARN("Failed to convert polygon to geofence");
    }

    int num_legs = polygonNumFlights(msg);
    ROS_INFO("Polygon for exploration will take %d flights to complete.", num_legs);
}

int HelloDeccoManager::initBurnUnit(geometry_msgs::Polygon &polygon) {
    // Iterate through subpolygons to get first not done
    int ind = 0;
    geometry_msgs::Polygon poly;
    std::cout << "init burn unit, has json: " << burn_unit_json_.dump(4) << std::endl;
    for (auto& flight : burn_unit_json_["trips"][0]["flights"]) {
        if (flight["status"] == "NOT_STARTED") {
            poly = subpolygons_.at(ind);
            ROS_INFO("will do polygon %d", ind);
            break;
        }
        ind++;
    }
    polygon = transformPolygon(poly);
    return ind;
}

void HelloDeccoManager::updateBurnUnit(int index, std::string flight_status) {
    burn_unit_json_["trips"][0]["flights"][index]["status"] = flight_status;
    if (flight_status == "ACTIVE") {
        start_time_ = ros::Time::now();
        burn_unit_json_["trips"][0]["flights"][index]["startTime"] = std::to_string(round(start_time_.toSec()));
    }
    else if (flight_status == "COMPLETED") {
        end_time_ = ros::Time::now();
        burn_unit_json_["trips"][0]["flights"][index]["endTime"] = std::to_string(round(end_time_.toSec()));
        burn_unit_json_["trips"][0]["flights"][index]["duration"] = std::to_string(round((start_time_ - end_time_).toSec()));
    }
    std::string s = burn_unit_json_.dump();
    std_msgs::String burn_string;
    burn_string.data = s;
    burn_unit_pub_.publish(burn_string);
    std::cout << "burn json filled in: \n" << burn_unit_json_.dump(4) << std::endl;
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


bool HelloDeccoManager::polygonToMap(const geometry_msgs::Polygon &polygon) {
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

    for (int ii = 0; ii < polygon.points.size(); ii++) {
        GeographicLib::GeoCoords coord(polygon.points.at(ii).x, polygon.points.at(ii).y);
        geometry_msgs::Point32 poly_point;
        poly_point.x = coord.Easting() + utm_x_offset_;
        poly_point.y = coord.Northing() + utm_y_offset_;
        map_region_.points.push_back(poly_point);

        // Add marker
        geometry_msgs::Point p;
        p.x = coord.Easting() + utm_x_offset_;
        p.y = coord.Northing() + utm_y_offset_;
        m.points.push_back(p);

        ROS_INFO("map point was (%f, %f)", poly_point.x, poly_point.y);

        // Add cvx polygon point
        vertices_.push_back(cxd::Vec2({p.x, p.y}));
    }
    // Repost first marker at end to close the loop
    geometry_msgs::Point p = m.points.at(0);
    m.points.push_back(p);
    map_region_pub_.publish(m);
    return true;
}

bool HelloDeccoManager::polygonToGeofence(const geometry_msgs::Polygon &polygon) {

    mavros_msgs::WaypointPush srv;

    srv.request.start_index = 0;

    // Convert polygon point to geofence waypoint and push to list
    for (int i = 0; i < polygon.points.size(); i++) {
        mavros_msgs::Waypoint wp;

        wp.command = mavros_msgs::CommandCode::NAV_FENCE_POLYGON_VERTEX_INCLUSION;

        // Param1 is for number of fence points
        wp.param1 = polygon.points.size();

        wp.x_lat = polygon.points[i].x;
        wp.y_long = polygon.points[i].y;

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
    // TODO: add num flights based on fixed flight leg area
    GeographicLib::Geodesic geod(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    // Alternatively: const Geodesic& geod = Geodesic::WGS84();
    GeographicLib::PolygonArea poly(geod);
    for (int ii = 0; ii<polygon.points.size(); ii++) {
        // TODO fix on UI side: x,y currently are lat, long. should be long, lat to match Cartesian (also UTM) x/y understanding.
        poly.AddPoint(polygon.points.at(ii).x, polygon.points.at(ii).y);
    }
    double perimeter, area;
    poly.Compute(false, true, perimeter, area);
    int num_legs = 1;
    subpolygons_.clear();
    if (std::abs(area) > flightleg_area_) {
        // num_legs = concaveToMinimalConvexPolygons();
        centroid_splitter::CentroidSplitter splitter(map_region_, flightleg_area_);
        subpolygons_ = splitter.slicePolygon();
        num_legs = subpolygons_.size();
        visualizeLegs();
    }
    else {
        subpolygons_.push_back(map_region_);
    }
    std::cout << "legs " << num_legs << ", perimeter " << perimeter << " m, area " << area << "m2 \n";
    return num_legs;
}

void HelloDeccoManager::visualizeLegs() {
    for (int nn = 0; nn < subpolygons_.size(); nn++) {
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
        geometry_msgs::Polygon geom_polygon = subpolygons_.at(nn);
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

int HelloDeccoManager::concaveToMinimalConvexPolygons() {
    cxd::ConcavePolygon concavePoly(vertices_);
    concavePoly.convexDecomp();
    std::vector<cxd::ConcavePolygon > subPolygonList;
    concavePoly.returnLowestLevelPolys(subPolygonList);
    int num_legs = concavePoly.getNumberSubPolys();
    for (int nn = 0; nn < subPolygonList.size(); nn++) {
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
        std::vector<cxd::Vertex > subPolyVerts = subPolygonList.at(nn).getVertices();
        geometry_msgs::Polygon geom_polygon;
        std::vector<geometry_msgs::Point32> geom_points;
        for (int vv = 0; vv < subPolyVerts.size(); vv++) {
            geometry_msgs::Point32 pt;
            pt.x = subPolyVerts.at(vv).position.x;
            pt.y = subPolyVerts.at(vv).position.y;
            pt.z = 0;
            geom_points.push_back(pt);

            // Add marker
            geometry_msgs::Point p;
            p.x = pt.x;
            p.y = pt.y;
            m.points.push_back(p);
        }
        geom_polygon.points = geom_points;
        subpolygons_.push_back(geom_polygon);
        // Repost first marker at end to close the loop
        geometry_msgs::Point p = m.points.at(0);
        m.points.push_back(p);
        map_region_pub_.publish(m);
    }
    return num_legs;
}

geometry_msgs::Polygon HelloDeccoManager::transformPolygon(const geometry_msgs::Polygon &map_poly) {
    geometry_msgs::Polygon slam_map_poly;
    geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(slam_map_frame_, mavros_map_frame_, ros::Time(0));
    for (int nn = 0; nn < map_poly.points.size(); nn++) {
        geometry_msgs::PointStamped point_tf;
        geometry_msgs::PointStamped map_pt;
        map_pt.point.x = map_poly.points.at(nn).x;
        map_pt.point.y = map_poly.points.at(nn).y;
        map_pt.header.frame_id = mavros_map_frame_;
        tf2::doTransform(map_pt, point_tf, tf);
        geometry_msgs::Point32 point;
        point.x = point_tf.point.x;
        point.y = point_tf.point.y;
        slam_map_poly.points.push_back(point);
    }
    return slam_map_poly;
}

void HelloDeccoManager::mapToGeopoint(const geometry_msgs::PointStamped &point_in, geometry_msgs::PointStamped &point_out, double yaw) {
    // TODO init tf at the start, doesn't change
    tf2::Vector3 translate(utm_x_offset_, utm_y_offset_, 0.0);
    tf2::Transform utm2slam_tf;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    utm2slam_tf.setRotation(quat);
    utm2slam_tf.setOrigin(translate);
    geometry_msgs::Transform slam2utm = tf2::toMsg(utm2slam_tf);
    // Apply tf
    geometry_msgs::TransformStamped geom_stamped_tf;
    geom_stamped_tf.header.frame_id = slam_map_frame_;
    geom_stamped_tf.header.stamp = ros::Time(0);
    geom_stamped_tf.child_frame_id = "utm";
    geom_stamped_tf.transform = slam2utm;
    tf2::doTransform(point_in, point_out, geom_stamped_tf);
}

json HelloDeccoManager::polygonToBurnUnit(const geometry_msgs::Polygon &polygon) {
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
                    "index" : 0,
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

    json polygon_json;
    for (int jj = 0; jj < polygon.points.size(); jj++) {
        json ll;
        ll.push_back(polygon.points.at(jj).x);
        ll.push_back(polygon.points.at(jj).y);
        polygon_json.push_back(ll);
    }
    burn_unit["polygon"] = polygon_json;
    return burn_unit;
}

}