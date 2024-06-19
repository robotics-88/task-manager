#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/StreamRate.h>

#include <unordered_map>

typedef std::unordered_map<uint32_t, const std::string> cmode_map;
static const cmode_map arducopter_cmode_map{{
	{ 0, "STABILIZE" },
	{ 1, "ACRO" },
	{ 2, "ALT_HOLD" },
	{ 3, "AUTO" },
	{ 4, "GUIDED" },
	{ 5, "LOITER" },
	{ 6, "RTL" },
	{ 7, "CIRCLE" },
	{ 8, "POSITION" },	// not in list
	{ 9, "LAND" },
	{ 10, "OF_LOITER" },
	{ 11, "DRIFT" },	// renamed, prev name: APPROACH
	{ 13, "SPORT" },
	{ 14, "FLIP" },
	{ 15, "AUTOTUNE" },
	{ 16, "POSHOLD" },
	{ 17, "BRAKE" },
	{ 18, "THROW" },
	{ 19, "AVOID_ADSB" },
	{ 20, "GUIDED_NOGPS" }
}};

bool setStreamRate(mavros_msgs::StreamRate::Request &req, mavros_msgs::StreamRate::Response &res) {
    return true;
}

bool setMessageInterval(mavros_msgs::MessageInterval::Request &req, mavros_msgs::MessageInterval::Response &res) {
    return true;
}

bool arm(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res) {
    res.success = true;
    return true;
}

bool takeoff(mavros_msgs::CommandTOL::Request &req, mavros_msgs::CommandTOL::Response &res) {
    res.success = true;
    return true;
}

bool setMode(mavros_msgs::SetMode::Request &req, mavros_msgs::SetMode::Response &res) {
    for (auto &mode : arducopter_cmode_map) {
        if (req.custom_mode == mode.second) {
            res.mode_sent = true;
        }
    }
    return true;
}

// Set up node for service servers
int main(int argc, char **argv){
  ros::init(argc, argv, "test_services");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::NodeHandle nh;

  ros::ServiceServer srv1, srv2, srv3, srv4, srv5;

  srv1 = nh.advertiseService("/mavros/set_stream_rate", setStreamRate);
  srv2 = nh.advertiseService("/mavros/set_message_interval", setMessageInterval);
  srv3 = nh.advertiseService("/mavros/cmd/arming", arm);
  srv4 = nh.advertiseService("/mavros/cmd/takeoff", takeoff);
  srv5 = nh.advertiseService("/mavros/set_mode", setMode);

  ros::spin();

  return 0;
}