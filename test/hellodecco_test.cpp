#include "task_manager/hello_decco_manager.h"
#include "decco_utilities.h"

#include <gtest/gtest.h>
#include <ros/connection_manager.h>

/**************************                 TEST FIXTURES                   ********************************/

struct MapverHelper
{
    MapverHelper()
    : count(0)
    , msg_received(false)
    , burn_unit_received(false)
    {
    }

    void cb(const std_msgs::StringConstPtr &msg)
    {
        msg_received = true;
        count++;
        json mapver_json = json::parse(msg->data);
        std::string topic = mapver_json["topic"];
        json gossip_json = mapver_json["gossip"];
        if (topic == "burn_unit_receive") {
            burn_unit = gossip_json;
            burn_unit_received = true;
        }
    }

    int count;
    bool msg_received;
    bool burn_unit_received;
    json burn_unit;
};

struct BurnUnitHelper
{
    public: 
        json burn_unit = R"({
            "id" : 1,
            "orgId" : 1,
            "createdBy" : 1,
            "name" : "Super Great Burn Unit",
            "polygon" : [[41.49633579471772,-71.30898170628119],[41.49672955153177,-71.30774763443078],[41.49609069998187,-71.3075008200607],[41.49609069998187,-71.3085973996545]],
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
        json burn_unit_filled = R"({
            "id" : 1,
            "orgId" : 1,
            "createdBy" : 1,
            "name" : "Super Great Burn Unit",
            "polygon" : [[41.49633579471772,-71.30898170628119],[41.49672955153177,-71.30774763443078],[41.49609069998187,-71.3075008200607],[41.49609069998187,-71.3085973996545]],
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
                        "subpolygon" : [[41.496334075927905,-71.30898284911267],[41.49673080444696,-71.30774688722171],[41.49608993530461,-71.30750274657066],[41.496089935302145,-71.30859375000817]],
                        "flightLogUrl" : "hellodecco.com/my-url",
                        "status" : "NOT_STARTED"
                    }
                ]
                }
            ]
            })"_json;
        
        geometry_msgs::Polygon map_polygon;

        int utm_zone = 19;
        double utm_x_offset = 307330.376282;
        double utm_y_offset = 4596431.058002;

    BurnUnitHelper()
    {
    }

};

/**************************                 TEST CLASSES                   ********************************/

TEST(MapversationPackage, mapversationPackage)
{
    ros::NodeHandle node;
    hello_decco_manager::HelloDeccoManager hello_decco_manager(node);
    MapverHelper mapver;

    std::string topic = "test";
    json gossip;
    gossip["testfield"] = "teststring";
    ros::Subscriber sub = node.subscribe("/mapversation/to_hello_decco", 10, &MapverHelper::cb, &mapver);
    EXPECT_EQ(sub.getNumPublishers(), 1U);

    ros::Rate r(10);
    int counter = 0;
    // Not sure why but it needs to run twice for the mock subscriber to catch
    while (counter <=1) {
        hello_decco_manager.packageToMapversation(topic, gossip);
        ros::spinOnce();
        counter++;
        if (mapver.count >= 1) {
            EXPECT_TRUE(mapver.msg_received);
            break;
        }
        r.sleep();
    }
}

TEST(ReceiveBurnUnit, receiveBurnUnit)
{
    ros::NodeHandle node;
    hello_decco_manager::HelloDeccoManager hello_decco_manager(node);
    BurnUnitHelper burner;

    hello_decco_manager.setUtm(burner.utm_x_offset, burner.utm_y_offset, burner.utm_zone);
    hello_decco_manager.makeBurnUnitJson(burner.burn_unit, burner.utm_zone);
    int ind = hello_decco_manager.initFlightArea(burner.map_polygon);
    // Confirm that map polygon was initialized
    ASSERT_TRUE(ind >= 0);

    geometry_msgs::Polygon geo_poly = hello_decco_manager.polygonFromJson(burner.burn_unit_filled["trips"][0]["flights"][0]["subpolygon"]);
    geometry_msgs::Polygon manual_poly = hello_decco_manager.polygonToMap(geo_poly);
    // Confirm that polygons match in size
    ASSERT_EQ(burner.map_polygon.points.size(), manual_poly.points.size());

    // Confirm that polygons match in data (comparing in map frame)
    for (int ii = 0; ii < burner.map_polygon.points.size(); ii++) {
        ASSERT_EQ(burner.map_polygon.points.at(ii), manual_poly.points.at(ii));
    }
    
    // Confirm that when reverting map polygon to GPS, matches points
    for (int ii = 0; ii < burner.map_polygon.points.size(); ii++) {
        double lat, lon;
        decco_utilities::mapToLl(burner.map_polygon.points.at(ii).x, burner.map_polygon.points.at(ii).y, lat, lon,
                                 burner.utm_x_offset, burner.utm_y_offset, burner.utm_zone);
        geometry_msgs::Point32 ll_point;
        ll_point.x = lat;
        ll_point.y = lon;
        EXPECT_EQ(ll_point, geo_poly.points.at(ii));
    }

}

TEST(UpdateBurnUnit, updateBurnUnit)
{
    ros::NodeHandle node;
    hello_decco_manager::HelloDeccoManager hello_decco_manager(node);
    BurnUnitHelper burner;
    MapverHelper mapver;
    ros::Rate r(10);

    hello_decco_manager.setUtm(burner.utm_x_offset, burner.utm_y_offset, burner.utm_zone);
    ros::Subscriber sub = node.subscribe("/mapversation/to_hello_decco", 10, &MapverHelper::cb, &mapver);

    while (!mapver.msg_received) {
        hello_decco_manager.makeBurnUnitJson(burner.burn_unit, burner.utm_zone);
        ros::spinOnce();
        r.sleep();
    }

    // Check if update works
    EXPECT_TRUE(mapver.burn_unit["trips"][0]["flights"][0]["status"] == "NOT_STARTED");
    int counter = 0;
    mapver.count = 0;
    // Not sure why but it needs to run twice for the mock subscriber to catch
    while (counter < 2) {
        hello_decco_manager.updateFlightStatus(0, "ACTIVE");
        ros::spinOnce();
        counter++;
        r.sleep();
    }
    
    EXPECT_TRUE(mapver.burn_unit["trips"][0]["flights"][0]["status"] == "ACTIVE");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}