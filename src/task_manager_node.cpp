/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/task_manager.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto fci_node = std::make_shared<flight_controller_interface::FlightControllerInterface>();
  auto tm_node = std::make_shared<task_manager::TaskManager>(fci_node);
  tm_node->initialize();
  fci_node->initialize();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(fci_node);
  executor.add_node(tm_node);
  executor.spin();
  try {
    executor.spin();
  } catch (const std::exception &e) {
      RCLCPP_ERROR(tm_node->get_logger(), "Exception in executor spin: %s", e.what());
  } catch (...) {
      RCLCPP_ERROR(tm_node->get_logger(), "Unknown exception in executor spin");
  }
  
  std::cout << "Reference count 1: " << tm_node.use_count() << std::endl;
  executor.remove_node(tm_node);
  std::cout << "Reference count 2: " << tm_node.use_count() << std::endl;

  rclcpp::shutdown();

  std::cout << "Reference count 3: " << tm_node.use_count() << std::endl;
  RCLCPP_INFO(tm_node->get_logger(), "Shutting down task manager node");
  return 0;
}