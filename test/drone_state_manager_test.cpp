#include "task_manager/drone_state_manager.h"

#include <actionlib/server/simple_action_server.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>  

#include <gtest/gtest.h>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dz = one.z - two.z;
  return dx==0 && dy==0 && dz==0;
}

TEST(Accessors, autonomy)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    drone_state_manager.setAutonomyEnabled(true);
    drone_state_manager.setAutonomyEnabled(false);
}

TEST(Accessors, exploration)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    drone_state_manager.setExplorationEnabled(true);
    drone_state_manager.setExplorationEnabled(false);
}

TEST(Accessors, localPosition)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = 0;
    msg.pose.position.z = 0;
    msg.pose.position.y = 0;
    geometry_msgs::PoseStamped::ConstPtr msg_p(new geometry_msgs::PoseStamped(msg));
    drone_state_manager.localPositionCallback(msg_p);
    geometry_msgs::Point position = drone_state_manager.getCurrentLocalPosition();
    ASSERT_EQ(position, msg.pose.position);
}

TEST(Accessors, altitudeAGL)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    std_msgs::Float64 msg;
    msg.data = 10.0;
    std_msgs::Float64::ConstPtr msg_p(new std_msgs::Float64(msg));
    drone_state_manager.altitudeCallback(msg_p);
    double altitude = drone_state_manager.getAltitudeAGL();
    ASSERT_GE(altitude, 0);
}

TEST(MavrosCallbacks, globalPose)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    sensor_msgs::NavSatFix msg;
    msg.latitude = 0;
    msg.longitude = 0;
    sensor_msgs::NavSatFix::ConstPtr msg_p(new sensor_msgs::NavSatFix(msg));
    drone_state_manager.globalPositionCallback(msg_p);
}

TEST(MavrosCallbacks, localPose)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = 0;
    msg.pose.position.z = 0;
    msg.pose.position.y = 0;
    geometry_msgs::PoseStamped::ConstPtr msg_p(new geometry_msgs::PoseStamped(msg));
    drone_state_manager.localPositionCallback(msg_p);
}

TEST(MavrosCallbacks, status)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    mavros_msgs::State msg;
    msg.armed = false;
    mavros_msgs::State::ConstPtr msg_p(new mavros_msgs::State(msg));
    drone_state_manager.statusCallback(msg_p);
}

TEST(MavrosCallbacks, altitude)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    std_msgs::Float64 msg;
    msg.data = 2.0;
    std_msgs::Float64::ConstPtr msg_p(new std_msgs::Float64(msg));
    drone_state_manager.altitudeCallback(msg_p);
}

TEST(MavrosStateControl, setGuided)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    drone_state_manager.setAutonomyEnabled(false);
    ASSERT_FALSE(drone_state_manager.setGuided());

    drone_state_manager.setAutonomyEnabled(true);
    ASSERT_TRUE(drone_state_manager.setGuided());
}

TEST(MavrosStateControl, setMode)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    drone_state_manager.setAutonomyEnabled(true);
    drone_state_manager.setGuided();
    std::string mode = "fake_mode";
    ASSERT_FALSE(drone_state_manager.setMode(mode));
    mode = "LOITER";
    ASSERT_TRUE(drone_state_manager.setMode(mode));
}

TEST(MavrosStateControl, arm)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    drone_state_manager.setAutonomyEnabled(false);
    ASSERT_FALSE(drone_state_manager.arm());
    drone_state_manager.setAutonomyEnabled(true);
    ASSERT_TRUE(drone_state_manager.arm());
}

TEST(Safety, readiness)
{
    // isReady and getReady tests are combined by necessity
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    drone_state_manager.setAutonomyEnabled(false);
    ASSERT_FALSE(drone_state_manager.getReadyForAction());
    ASSERT_FALSE(drone_state_manager.readyForAction());

    drone_state_manager.setAutonomyEnabled(true);
    ros::spinOnce();
    ASSERT_TRUE(drone_state_manager.getReadyForAction());
    ros::spinOnce();
    ASSERT_TRUE(drone_state_manager.readyForAction());
}

TEST(Safety, safetyArea)
{
    // TODO (not yet implemented)
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}