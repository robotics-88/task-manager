/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/task_manager.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);

  rclcpp::NodeOptions node_options;
  std::shared_ptr <rclcpp::Node> nh = rclcpp::Node::make_shared("task_manager_node", node_options);
  task_manager::TaskManager task_manager(nh);
  exec.add_node(nh);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}