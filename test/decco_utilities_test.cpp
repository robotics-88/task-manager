#include <ros/ros.h>
#include <gtest/gtest.h>
#include <decco_utilities.h>

TEST(Methods, distance) 
{
    geometry_msgs::Point32 point_a, point_b;
    point_a.x = 2.0;
    point_a.y = 2.0;
    point_a.z = 2.0;

    point_b.x = 3.0;
    point_b.y = 3.0;
    point_b.z = 3.0;

    double distance = decco_utilities::distance(point_a, point_b);
    ASSERT_FLOAT_EQ(distance, sqrt(3));
}

TEST(Methods, distance_xy) 
{
    geometry_msgs::Point32 point_a, point_b;
    point_a.x = 2.0;
    point_a.y = 2.0;
    point_a.z = 2.0;

    point_b.x = 3.0;
    point_b.y = 3.0;
    point_b.z = 3.0;

    double distance = decco_utilities::distance_xy(point_a, point_b);
    ASSERT_FLOAT_EQ(distance, sqrt(2));
}

TEST(Methods, intersectsOrthogonal) 
{
    geometry_msgs::Point32 point_a, point_b, origin;

    point_a.x = 3.0;
    point_a.y = 1.0;

    point_b.x = 1.0;
    point_b.y = 3.0;

    origin.x = 1.0;
    origin.y = 1.0;

    geometry_msgs::Point32 intersection;
    bool intersects = decco_utilities::intersectsOrthogonal(point_a, point_b, origin, intersection);

    ASSERT_TRUE(intersects);

    ASSERT_FLOAT_EQ(intersection.x, 2.0);
    ASSERT_FLOAT_EQ(intersection.y, 2.0);
}

TEST(Methods, isInside) 
{
    geometry_msgs::Polygon polygon;

    geometry_msgs::Point32 point;

    point.x = 10;
    point.y = 10;
    polygon.points.push_back(point);
    point.x = 20;
    point.y = 10;
    polygon.points.push_back(point);
    point.x = 20;
    point.y = 20;
    polygon.points.push_back(point);
    point.x = 10;
    point.y = 20;
    polygon.points.push_back(point);

    geometry_msgs::Point32 test_point;
    test_point.x = 15.0;
    test_point.y = 15.0;

    ASSERT_TRUE(decco_utilities::isInside(polygon, test_point));

    test_point.x = 5.0;

    ASSERT_FALSE(decco_utilities::isInside(polygon, test_point));
}

TEST(Methods, llToUtm) 
{   
    double lat = 41.496334075927905;
    double lon = -71.30898284911267;

    double utm_x, utm_y;
    int zone;
    decco_utilities::llToUtm(lat, lon, zone, utm_x, utm_y);
    double new_lat, new_lon;
    decco_utilities::utmToLL(utm_x, utm_y, zone, new_lat, new_lon);

    ASSERT_DOUBLE_EQ(lat, new_lat);
    ASSERT_DOUBLE_EQ(lon, new_lon);
}



TEST(Methods, llToUtm2) 
{
    double utm_x = 307268.564434;
    double utm_y = 4596431.061994;
    int zone = 19;
    
    double lat, lon;
    decco_utilities::utmToLL(utm_x, utm_y, zone, lat, lon);
    double new_utm_x, new_utm_y;
    int new_zone;
    decco_utilities::llToUtm(lat, lon, new_zone, new_utm_x, new_utm_y);

    ASSERT_DOUBLE_EQ(utm_x, new_utm_x);
    ASSERT_DOUBLE_EQ(utm_y, new_utm_y);
    ASSERT_EQ(zone, new_zone);
}

// TODO
TEST(Methods, llToMap) 
{
    ASSERT_TRUE(true);
}

TEST(Methods, mapToLL) 
{
    ASSERT_TRUE(true);
}


// Set up node for service servers
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "decco_utilities_test");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}