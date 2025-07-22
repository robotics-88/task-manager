/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#ifndef ELEVATION_MANAGER_H_
#define ELEVATION_MANAGER_H_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"

#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/gdalwarper.h>
#include <gdal/ogr_p.h>
#include <gdal/ogr_spatialref.h>
namespace elevation_manager {
/**
 * @class ElevationManager
 * @brief Manages elevation dem map
 */
class ElevationManager {
  public:
    ElevationManager(const double flightleg_acres, const std::string mavros_map_frame);
    ~ElevationManager();

    std_msgs::msg::String updateFlightStatus(std::string flight_status,
                                             const rclcpp::Time timestamp);
    void setUtm(double utm_x, double utm_y, int zone) {
        utm_x_offset_ = -utm_x;
        utm_y_offset_ = -utm_y;
        utm_zone_ = zone;
    }

    bool elevationInitializer(std::string tif_name = "");
    void llToMap(const double lat, const double lon, double &px, double &py);
    void mapToLl(const double px, const double py, double &lat, double &lon);

    bool getElevationChunk(const double utm_x, const double utm_y, const int width,
                           const int height, sensor_msgs::msg::Image &chunk, double &max,
                           double &min);
    bool getElevationValue(const double utm_x, const double utm_y, double &value);
    bool getHomeElevation(double &value);
    bool getElevationInit() { return elevation_init_; }
    nav_msgs::msg::OccupancyGrid getTifGrid() { return tif_grid_; }
    pcl::PointCloud<pcl::PointXYZ> getTifCloud() { return tif_cloud_; }

  private:
    cv::Mat dem_cv_;
    cv::Mat slope_cv_;
    double ul_y_utm_;
    double ul_x_utm_;
    nav_msgs::msg::OccupancyGrid tif_grid_;
    pcl::PointCloud<pcl::PointXYZ> tif_cloud_;

    // Frames/TF
    std::string mavros_map_frame_;
    double utm_x_offset_;
    double utm_y_offset_;
    int utm_zone_;

    bool elevation_init_;

    // Tif handler
    bool initElevation(const std::string &tif_name, const double utm_x, const double utm_y);
    bool getElevationChunk(const double utm_x, const double utm_y, const int width,
                           const int height, cv::Mat &chunk);
    bool getElevation(const double utm_x, const double utm_y, double &value);
    double getSlope(const double utm_x, const double utm_y);
    void makeOutputFile(GDALDataset *hSrcDS, std::string filename);
    bool getMapValue(const double utm_x, const double utm_y, const cv::Mat &mat, double &value);
    void initializeValleys();
};

} // namespace elevation_manager

#endif