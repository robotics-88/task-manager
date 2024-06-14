#include "task_manager/task_manager.h"


#include <gtest/gtest.h>


TEST(Accessors, slamPose) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = 1.0;
    msg.pose.position.z = 2.0;
    msg.pose.position.y = 3.0;
    geometry_msgs::PoseStamped::ConstPtr msg_p(new geometry_msgs::PoseStamped(msg));
    task_manager.slamPoseCallback(msg_p);
}

TEST(Accessors, registeredPcl) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    sensor_msgs::PointCloud2 msg;
    msg.data.push_back(UINT8_MAX);
    sensor_msgs::PointCloud2::ConstPtr msg_p(new sensor_msgs::PointCloud2(msg));
    task_manager.registeredPclCallback(msg_p);
}

TEST(Accessors, pathPlanner) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    sensor_msgs::PointCloud2 msg;
    msg.data.push_back(UINT8_MAX);
    sensor_msgs::PointCloud2::ConstPtr msg_p(new sensor_msgs::PointCloud2(msg));
    task_manager.pathPlannerCallback(msg_p);
}

TEST(Accessors, costmap) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    map_msgs::OccupancyGridUpdate msg;
    msg.data.push_back(UINT8_MAX);
    map_msgs::OccupancyGridUpdate::ConstPtr msg_p(new map_msgs::OccupancyGridUpdate(msg));
    task_manager.costmapCallback(msg_p);
}

TEST(Accessors, pointcloud) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    sensor_msgs::PointCloud2 msg;
    msg.data.push_back(UINT8_MAX);
    sensor_msgs::PointCloud2::ConstPtr msg_p(new sensor_msgs::PointCloud2(msg));
    task_manager.pointcloudCallback(msg_p);
}

TEST(Accessors, livox) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    livox_ros_driver::CustomMsg msg;
    livox_ros_driver::CustomPoint pt;
    pt.x = 1.0;
    pt.y = 2.0;
    pt.z = 3.0;
    msg.points.push_back(pt);
    livox_ros_driver::CustomMsg::ConstPtr msg_p(new livox_ros_driver::CustomMsg(msg));
    task_manager.livoxCallback(msg_p);
}

TEST(Accessors, mapir) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    sensor_msgs::Image msg;
    msg.data.push_back(UINT8_MAX);
    sensor_msgs::Image::ConstPtr msg_p(new sensor_msgs::Image(msg));
    task_manager.mapirCallback(msg_p);
}

TEST(Accessors, attollo) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    sensor_msgs::Image msg;
    msg.data.push_back(UINT8_MAX);
    sensor_msgs::Image::ConstPtr msg_p(new sensor_msgs::Image(msg));
    task_manager.attolloCallback(msg_p);
}

TEST(Accessors, thermal) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    sensor_msgs::Image msg;
    msg.data.push_back(UINT8_MAX);
    sensor_msgs::Image::ConstPtr msg_p(new sensor_msgs::Image(msg));
    task_manager.thermalCallback(msg_p);
}

TEST(Accessors, rosbag) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    std_msgs::String msg;
    msg.data = "test";
    std_msgs::String::ConstPtr msg_p(new std_msgs::String(msg));
    task_manager.rosbagCallback(msg_p);
}

TEST(Accessors, goal) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = 1.0;
    msg.pose.position.z = 2.0;
    msg.pose.position.y = 3.0;
    geometry_msgs::PoseStamped::ConstPtr msg_p(new geometry_msgs::PoseStamped(msg));
    task_manager.goalCallback(msg_p);
}

TEST(Accessors, mapYaw) 
{
    ros::NodeHandle node;
    task_manager::TaskManager task_manager(node);

    std_msgs::Float64 msg;
    msg.data = 1.234;
    std_msgs::Float64::ConstPtr msg_p(new std_msgs::Float64(msg));
    task_manager.mapYawCallback(msg_p);
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

  std::cout << "Whattup" << std::endl;

  return RUN_ALL_TESTS();
}