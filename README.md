# task-manager

Each UAS has a task manager keeping track of all queued tasks. Perception is not a task at this time, because it does not conflict with other processes, so all perception begins at startup. In contrast, actions need to be ordered and deconflicted. Actions include:
* Takeoff and land *(needs remove from Monarch)*
* Navigate to waypoint
* Explore polygon *(needs remove from Monarch)*
* Emergency behaviors (e.g., land, RTL, hover) *(needs remove from Monarch)*

The task manager tracks UAS position and flight status to determine when actions are complete and then update the queue.