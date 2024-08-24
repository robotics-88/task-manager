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

  // Careful with ordering here, this has to be correct for proper initialization order
  tm_node->initialize();
  fci_node->initialize();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tm_node);
  executor.add_node(fci_node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}