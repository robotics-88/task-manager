/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/task_manager.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);


  auto node = std::make_shared<task_manager::TaskManager>();
  node->initialize();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}