/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/task_manager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_manager");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle node;
  task_manager::TaskManager task_manager(node);

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}