/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PoseStamped.h>

#include "messages_88/ExploreAction.h"
#include "messages_88/NavToPointAction.h"

namespace task_manager {
/**
 * @class TaskManager
 * @brief The TaskManager manages the task queue (e.g., navigate to polygon, explore, handle emergency)
 */
class TaskManager {

    public:
        TaskManager(ros::NodeHandle& node);
        ~TaskManager();

        void startNav2PointTask();
        void receivedExploreTask();
        void startExploreTask();
        void stop();

        void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    private:
        enum CurrentStatus
        {
            ON_START,
            EXPLORING,
            WAITING_TO_EXPLORE,
            HOVERING,
            NAVIGATING,
            TAKING_OFF,
            LANDING
        };

        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<messages_88::NavToPointAction> nav2point_action_server_;
        actionlib::SimpleActionServer<messages_88::ExploreAction> explore_action_server_;
        actionlib::SimpleActionClient<messages_88::ExploreAction> explore_action_client_;

        ros::Publisher local_pos_pub_;
        ros::Subscriber mavros_local_pos_subscriber_;

        // Goal details
        geometry_msgs::Point current_target_;
        messages_88::ExploreGoal current_explore_goal_;

        CurrentStatus current_status_ = CurrentStatus::ON_START;
        ros::Timer status_timer_;

        void readyToExplore();
        bool isInside(const geometry_msgs::Polygon& polygon, const geometry_msgs::Point& point);

};

}

#endif