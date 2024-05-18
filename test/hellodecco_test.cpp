#include "task_manager/hello_decco_manager.h"

#include <gtest/gtest.h>
#include <ros/connection_manager.h>

/**************************                 TEST FIXTURES                   ********************************/

struct MapverHelper
{
    public: 
        bool msg_received;

    MapverHelper()
    : msg_received(false)
    , count(0)
    {
    }

    void cb(const std_msgs::StringConstPtr &gossip)
    {
        msg_received = true;
        count++;
    }

    int count;
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

    ros::Rate r(1);
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

    hello_decco_manager.setUtmOffsets(burner.utm_x_offset, burner.utm_y_offset);
    hello_decco_manager.makeBurnUnitJson(burner.burn_unit, burner.utm_zone);
    int ind = hello_decco_manager.initBurnUnit(burner.map_polygon);
    // Confirm that map polygon was initialized
    ASSERT_TRUE(ind >= 0);

    geometry_msgs::Polygon manual_poly = hello_decco_manager.polygonToMap(hello_decco_manager.polygonFromJson(burner.burn_unit_filled["trips"][0]["flights"][0]["subpolygon"]));
    // Confirm that polygons match in size
    ASSERT_EQ(burner.map_polygon.points.size(), manual_poly.points.size());

    // Confirm that polygons match in data (comparing in map frame)
    for (int ii = 0; ii < burner.map_polygon.points.size(); ii++) {
        EXPECT_EQ(burner.map_polygon.points.at(ii), manual_poly.points.at(ii));
    }
    
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}