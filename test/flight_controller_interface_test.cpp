#include "task_manager/flight_controller_interface.h"

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


TEST(Accessors, slamPose) 
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = 1.0;
    msg.pose.position.z = 2.0;
    msg.pose.position.y = 3.0;
    geometry_msgs::PoseStamped::ConstPtr msg_p(new geometry_msgs::PoseStamped(msg));
    flight_controller_interface.slamPoseCallback(msg_p);

    geometry_msgs::PoseStamped position = flight_controller_interface.getCurrentSlamPosition();
    ASSERT_EQ(position.pose.position, msg.pose.position);
}

TEST(Accessors, localPosition)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = 4.0;
    msg.pose.position.z = 5.0;
    msg.pose.position.y = 6.0;
    geometry_msgs::PoseStamped::ConstPtr msg_p(new geometry_msgs::PoseStamped(msg));
    flight_controller_interface.localPositionCallback(msg_p);

    geometry_msgs::PoseStamped position = flight_controller_interface.getCurrentLocalPosition();
    ASSERT_EQ(position.pose.position, msg.pose.position);
}

TEST(Accessors, globalPose)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);
    sensor_msgs::NavSatFix msg;
    msg.latitude = 1.234;
    msg.longitude = -1.234;
    msg.altitude = 50.0;
    sensor_msgs::NavSatFix::ConstPtr msg_p(new sensor_msgs::NavSatFix(msg));
    flight_controller_interface.globalPositionCallback(msg_p);

    sensor_msgs::NavSatFix ll = flight_controller_interface.getCurrentGlobalPosition();

    ASSERT_EQ(ll.latitude, msg.latitude);
    ASSERT_EQ(ll.longitude, msg.longitude);
    ASSERT_EQ(ll.altitude, msg.altitude);

    int utm_1 = GeographicLib::UTMUPS::StandardZone(msg.latitude, msg.longitude);
    int utm_2 = flight_controller_interface.getUTMZone();

    ASSERT_EQ(utm_1, utm_2);

}

TEST(Accessors, altitude)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);
    std_msgs::Float64 msg;
    msg.data = 10.0;
    std_msgs::Float64::ConstPtr msg_p(new std_msgs::Float64(msg));
    flight_controller_interface.altitudeCallback(msg_p);

    double altitude = flight_controller_interface.getAltitudeAGL();
    ASSERT_EQ(msg.data, altitude);
}

TEST(Accessors, compass)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);
    std_msgs::Float64 msg;
    msg.data = 145.0;
    std_msgs::Float64::ConstPtr msg_p(new std_msgs::Float64(msg));
    flight_controller_interface.compassCallback(msg_p);

    double compass = flight_controller_interface.getCompass();
    ASSERT_EQ(msg.data, compass);
}

TEST(Accessors, battery)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);
    sensor_msgs::BatteryState msg;
    msg.voltage = 26.0;
    msg.current = 0.0;

    sensor_msgs::BatteryState::ConstPtr msg_p(new sensor_msgs::BatteryState(msg));
    flight_controller_interface.batteryCallback(msg_p);

    double voltage = flight_controller_interface.getBatteryVoltage();
    ASSERT_EQ(voltage, msg.voltage);
    double batt_percent = flight_controller_interface.getBatteryPercentage();
    ASSERT_GE(batt_percent, 99.0);

    msg.voltage = 22.5;

    sensor_msgs::BatteryState::ConstPtr msg_p_2(new sensor_msgs::BatteryState(msg));
    flight_controller_interface.batteryCallback(msg_p_2);

    voltage = flight_controller_interface.getBatteryVoltage();
    ASSERT_EQ(voltage, msg.voltage);
    batt_percent = flight_controller_interface.getBatteryPercentage();
    ASSERT_GE(batt_percent, 1.0);
    ASSERT_LE(batt_percent, 99.0);


    msg.voltage = 19.0;

    sensor_msgs::BatteryState::ConstPtr msg_p_3(new sensor_msgs::BatteryState(msg));
    flight_controller_interface.batteryCallback(msg_p_3);

    voltage = flight_controller_interface.getBatteryVoltage();
    ASSERT_EQ(voltage, msg.voltage);
    batt_percent = flight_controller_interface.getBatteryPercentage();
    ASSERT_LE(batt_percent, 1.0);
}

TEST(Accessors, state)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);

    mavros_msgs::State msg;
    msg.armed = false;
    msg.mode = "STABILIZED";

    mavros_msgs::State::ConstPtr msg_p(new mavros_msgs::State(msg));
    flight_controller_interface.statusCallback(msg_p);

    ASSERT_EQ(flight_controller_interface.getFlightMode(), msg.mode);
    ASSERT_FALSE(flight_controller_interface.getIsArmed());
    ASSERT_FALSE(flight_controller_interface.getIsInAir());

    // Arm and takeoff
    msg.mode = "GUIDED";
    msg.armed = true;

    mavros_msgs::State::ConstPtr msg_p_2(new mavros_msgs::State(msg));
    flight_controller_interface.statusCallback(msg_p_2);

    flight_controller_interface.setAutonomyEnabled(true);

    ASSERT_TRUE(flight_controller_interface.takeOff());

    std_msgs::Float64 alt_msg;
    alt_msg.data = 10.0;
    std_msgs::Float64::ConstPtr alt_msg_p(new std_msgs::Float64(alt_msg));
    flight_controller_interface.altitudeCallback(alt_msg_p);

    ASSERT_TRUE(flight_controller_interface.getIsInAir());

}

TEST(Accessors, sysStatus) 
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);

    mavros_msgs::SysStatus status;
    status.sensors_enabled = 1;
    status.sensors_health = 1;

    mavros_msgs::SysStatusConstPtr msg_p(new mavros_msgs::SysStatus(status));
    flight_controller_interface.sysStatusCallback(msg_p);

    ASSERT_TRUE(flight_controller_interface.getDroneReadyToArm());

    status.sensors_enabled = 1;
    status.sensors_health = 0;

    mavros_msgs::SysStatusConstPtr msg_p2(new mavros_msgs::SysStatus(status));
    flight_controller_interface.sysStatusCallback(msg_p2);

    ASSERT_FALSE(flight_controller_interface.getDroneReadyToArm());
}

TEST(Utility, initializeImu)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);

    sensor_msgs::Imu imu;
    imu.orientation.w = 0.01;
    imu.orientation.x = 0.02;
    imu.orientation.y = 0.03;
    imu.orientation.z = 0.04;

    sensor_msgs::ImuConstPtr msg_p(new sensor_msgs::Imu(imu));

    geometry_msgs::Quaternion avg_orientation;

    // Add fewer than required number of samples to pass
    int averaging_n = flight_controller_interface.getImuAveragingN();
    for (int i = 0; i < averaging_n - 1; i++) {
        flight_controller_interface.imuCallback(msg_p);
    }

    ASSERT_FALSE(flight_controller_interface.getAveragedOrientation(avg_orientation));

    // Add one more sample to pass
    flight_controller_interface.imuCallback(msg_p);

    ASSERT_TRUE(flight_controller_interface.getAveragedOrientation(avg_orientation));

    ASSERT_FLOAT_EQ(imu.orientation.x, avg_orientation.x);
    ASSERT_FLOAT_EQ(imu.orientation.y, avg_orientation.y);
    ASSERT_FLOAT_EQ(imu.orientation.z, avg_orientation.z);
    ASSERT_FLOAT_EQ(imu.orientation.w, avg_orientation.w);
}

TEST(Utility, initUTM)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);

    sensor_msgs::NavSatFix msg;
    msg.latitude = 1.234;
    msg.longitude = -1.234;
    msg.altitude = 50.0;

    GeographicLib::GeoCoords coords(msg.latitude, msg.longitude);

    sensor_msgs::NavSatFix::ConstPtr msg_p(new sensor_msgs::NavSatFix(msg));
    flight_controller_interface.globalPositionCallback(msg_p);

    double utm_x, utm_y;
    flight_controller_interface.initUTM(utm_x, utm_y);

    ASSERT_EQ(utm_x, coords.Easting());
    ASSERT_EQ(utm_y, coords.Northing());

}

TEST(Utility, getUTMZone)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);
    sensor_msgs::NavSatFix msg;
    msg.latitude = 1.234;
    msg.longitude = -1.234;
    msg.altitude = 50.0;
    sensor_msgs::NavSatFix::ConstPtr msg_p(new sensor_msgs::NavSatFix(msg));
    flight_controller_interface.globalPositionCallback(msg_p);

    int utm_1 = GeographicLib::UTMUPS::StandardZone(msg.latitude, msg.longitude);
    int utm_2 = flight_controller_interface.getUTMZone();

    ASSERT_EQ(utm_1, utm_2);
}

TEST(StateControl, setMode)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);
    flight_controller_interface.setAutonomyEnabled(true);

    std::string mode = "fake_mode";
    ASSERT_FALSE(flight_controller_interface.setMode(mode));

    mode = "LOITER";
    ASSERT_TRUE(flight_controller_interface.setMode(mode));
}

TEST(StateControl, arm)
{
    ros::NodeHandle node;
    flight_controller_interface::FlightControllerInterface flight_controller_interface(node);
    flight_controller_interface.setAutonomyEnabled(false);
    ASSERT_FALSE(flight_controller_interface.arm());
    flight_controller_interface.setAutonomyEnabled(true);
    ASSERT_TRUE(flight_controller_interface.arm());
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