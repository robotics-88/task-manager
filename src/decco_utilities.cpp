/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> , Gus Meyer <gus@robotics88.com>
*/

#include "task_manager/decco_utilities.h"

namespace decco_utilities {

unsigned long rosTimeToMilliseconds(const rclcpp::Time ros_time) {
    return std::round(ros_time.nanoseconds() * 1E-6);
}

void llToUtm(const double lat, const double lon, int &zone, double &utm_x, double &utm_y) {
    double k, gamma;
    bool north;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, north, utm_x, utm_y, gamma, k);
}

void utmToLL(const double utm_x, const double utm_y, const int zone, double &lat, double &lon) {
    double k, gamma;
    GeographicLib::UTMUPS::Reverse(zone, true, utm_x, utm_y, lat, lon, gamma, k);
}

void mapToLl(const double px, const double py, double &lat, double &lon, const double utm_x_offset,
             const double utm_y_offset, const double utm_zone) {
    double xval = px - utm_x_offset;
    double yval = py - utm_y_offset;
    utmToLL(xval, yval, utm_zone, lat, lon);
}

void llToMap(const double lat, const double lon, double &px, double &py, const double utm_x_offset,
             const double utm_y_offset) {
    double utm_x, utm_y;
    int zone;
    llToUtm(lat, lon, zone, utm_x, utm_y);
    px = utm_x + utm_x_offset;
    py = utm_y + utm_y_offset;
}

std::string get_time_str() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::gmtime(&now_time);
    std::stringstream ss;
    ss << std::put_time(&now_tm, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

} // namespace decco_utilities