/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/task_manager.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<task_manager::TaskManager>(options);

  exec.add_node(node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}