#include "task_manager/drone_state_manager.h"
#include <gtest/gtest.h>

TEST(MavrosStateControl, takeOff)
{
    ros::NodeHandle node;
    drone_state_manager::DroneStateManager drone_state_manager(node);
    drone_state_manager.setAutonomyEnabled(false);
    ASSERT_FALSE(drone_state_manager.takeOff());

    drone_state_manager.setAutonomyEnabled(true);
    ASSERT_FALSE(drone_state_manager.takeOff());

    drone_state_manager.setAutonomyEnabled(true);
    drone_state_manager.getReadyForAction();
    ASSERT_TRUE(drone_state_manager.takeOff());
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