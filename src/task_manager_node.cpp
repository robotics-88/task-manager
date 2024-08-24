/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/task_manager.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto tm_node = std::make_shared<task_manager::TaskManager>();
  tm_node->initialize();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tm_node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}