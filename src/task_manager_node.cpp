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
  try {
    executor.spin();
  } catch (const std::exception &e) {
      RCLCPP_ERROR(tm_node->get_logger(), "Exception in executor spin: %s", e.what());
  } catch (...) {
      RCLCPP_ERROR(tm_node->get_logger(), "Unknown exception in executor spin");
  }
  
  rclcpp::shutdown();
  RCLCPP_INFO(tm_node->get_logger(), "Shutting down task manager node");
  return 0;
}