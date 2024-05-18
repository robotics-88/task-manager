# task-manager

Each UAS has a task manager keeping track of all queued tasks. Perception is not a task at this time, because it does not conflict with other processes, so all perception begins at startup. In contrast, actions need to be ordered and deconflicted. Actions include:
* Takeoff and land 
* Record on takeoff, save on land
* Navigate to waypoint
* Explore polygon 
* Emergency behaviors (e.g., land, RTL, hover)

The task manager tracks UAS position and flight status to determine when actions are complete and then update the queue.

# testing

To build with tests, run

`catkin build --catkin-make-args run_tests`

To run the tests, launch the respective test file with, e.g.,

`rostest task_manager tests_hellodecco.launch`

To see print statements at levels lower than WARN, use:

`rostest task_manager tests_hellodecco.launch --text`

Note that the drone state tests require ArduPilot to first be running in SITL. This can either be through Unreal/AirSim, or the simpler ArduPilot SITL. For the latter, in the ArduCopter directory run

`../Tools/autotest/sim_vehicle.py -v ArduCopter`

This must be restarted between tests. Wait for the message to print, ~"EKF using GPS", before launching the rostest.