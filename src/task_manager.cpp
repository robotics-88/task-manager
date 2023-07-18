/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "task_manager/task_manager.h"

#include <float.h>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 1.0; // Goal reached if within a meter
}

namespace task_manager
{
TaskManager::TaskManager(ros::NodeHandle& node)
    : private_nh_("~")
    , nh_(node)
    , nav2point_action_server_(private_nh_, "nav2point", false)
    , explore_action_server_(private_nh_, "explore", false)
    , explore_action_client_("explore", true)
{
    std::string goal_topic = "/mavros/setpoint_position/local";
    private_nh_.param<std::string>("goal_topic", goal_topic, goal_topic);

    // Create action server (called by UI/Monarch)
    nav2point_action_server_.registerGoalCallback(boost::bind(&TaskManager::startNav2PointTask, this));
    nav2point_action_server_.registerPreemptCallback(boost::bind(&TaskManager::stop, this));
    nav2point_action_server_.start();

    // Create action server (called by UI/Monarch)
    explore_action_server_.registerGoalCallback(boost::bind(&TaskManager::receivedExploreTask, this));
    explore_action_server_.registerPreemptCallback(boost::bind(&TaskManager::stop, this));
    explore_action_server_.start();

    // MAVROS
    mavros_local_pos_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &TaskManager::localPositionCallback, this);
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
}

TaskManager::~TaskManager(){}

void TaskManager::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (current_status_ == CurrentStatus::NAVIGATING) {
        if (msg->pose.position == current_target_) {
            // Within a meter of target (TODO, add or already inside polygon: || isInside(current_explore_goal_.polygon, msg->pose.position)
            current_status_ = CurrentStatus::WAITING_TO_EXPLORE;
        }
    }
}

void TaskManager::startNav2PointTask() {
    messages_88::NavToPointGoalConstPtr goal = nav2point_action_server_.acceptNewGoal();
    current_status_ = CurrentStatus::NAVIGATING;
    geometry_msgs::Point point = goal->point;
    current_target_ = point;
    geometry_msgs::PoseStamped target;
    target.pose.position = point;
    local_pos_pub_.publish(target);
}

void TaskManager::receivedExploreTask() {
    messages_88::ExploreGoalConstPtr goal = explore_action_server_.acceptNewGoal();
    current_explore_goal_ = *goal;
    std::string uuid = current_explore_goal_.uuid.data; // If no transit, uuid=NO_TRANSIT, otherwise should match transit uuid
    // TODO Add validation. Eg: Check if inside, and if no, check if nav goal received, if no, reject

    if (uuid == "NO_TRANSIT" || current_status_ == CurrentStatus::WAITING_TO_EXPLORE) {
        startExploreTask();
    }
    else {
        status_timer_ = private_nh_.createTimer(ros::Duration(1.0),
                               [this](const ros::TimerEvent&) { readyToExplore(); });
    }

}

void TaskManager::readyToExplore() {
    if (current_status_ == CurrentStatus::WAITING_TO_EXPLORE) {
        status_timer_.stop();
        startExploreTask();
    }
}

void TaskManager::startExploreTask() {
    current_status_ = CurrentStatus::EXPLORING;
    explore_action_client_.sendGoal(current_explore_goal_);
}

void TaskManager::stop() {

}

bool TaskManager::isInside(const geometry_msgs::Polygon& polygon, const geometry_msgs::Point& point)
{
  // TODO should make a map_util class to copy into packages that need it (eg, monarch)

  // Determine if the given point is inside the polygon using the number of crossings method
  // https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
  int n = polygon.points.size();
  int cross = 0;
  // Loop from i = [0 ... n - 1] and j = [n - 1, 0 ... n - 2]
  // Ensures first point connects to last point
  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    // Check if the line to x,y crosses this edge
    if ( ((polygon.points[i].y > point.y) != (polygon.points[j].y > point.y))
           && (point.x < (polygon.points[j].x - polygon.points[i].x) * (point.y - polygon.points[i].y) /
            (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x) )
    {
      cross++;
    }
  }
  // Return true if the number of crossings is odd
  return cross % 2 > 0;
}

}