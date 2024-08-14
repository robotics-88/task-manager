/* 
© 2024 Robotics 88
Author: Gus Meyer <gus@robotics88.com>
*/

#ifndef DECCO_UTILITIES_H_
#define DECCO_UTILITIES_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include <GeographicLib/GeoCoords.hpp>

namespace decco_utilities
{

    template<typename T>
    double distance(const T point_a, const T point_b) {
        double dist_x = point_b.x - point_a.x;
        double dist_y = point_b.y - point_a.y;
        double dist_z = point_b.z - point_a.z;

        return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
    }

    template<typename T>
    double distance_xy(const T point_a, const T point_b) {
        double dist_x = point_b.x - point_a.x;
        double dist_y = point_b.y - point_a.y;

        return sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    // Determines if line segment between point_a and point_b has an orthogonal line segment that intersects the "origin" point
    template<typename T>
    bool intersectsOrthogonal(const T point_a, const T point_b, const T origin, T &intersection) {
        // Adjust points by origin
        double x1 = point_a.x - origin.x;
        double y1 = point_a.y - origin.y;
        double x2 = point_b.x - origin.x;
        double y2 = point_b.y - origin.y;
        
        // Parameterized line segment formula is x(t) = (1-t)*x1 + t*x2, y(t) = (1-t)y1 + t*y2
        // For 0 <= t <= 1. So, check if there is a valid t which satisfies the conditions of desired line
        // segment which intersects the origin and is orthogonal to the first line segment
        
        // Slope of line segment is (y2 - y1) / (x2 - x1);
        
        // Slope of orthogonal line is -1/m = -(x2 - x1) / (y2 - y1)
        
        // Plug in parameterized points for x and y in equation y = -(x2 - x1) / (y2 - y1) * x
        
        // (1-t)y1 + t*y2 = -(x2 - x1) / (y2 - y1) * ((1-t)*x1 + t*x2)
        
        // Solving for t, we find equation
        double numerator = x1 * x1 - x1*x2 + y1 * y1 - y1*y2;
        double denomenator = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
        if (abs(denomenator) <= DBL_EPSILON) {
            return false;
        }

        double t = numerator / denomenator;
        
        if (t >= 0 && t <= 1) {
            // Determine x and y based on parameterized line
            double x_t = (1 - t) * x1 + t * x2;
            double y_t = (1 - t) * y1 + t * y2;

            // Add back in origin
            intersection.x = x_t + origin.x;
            intersection.y = y_t + origin.y;
            
            return true;
        }
        else {
            return false;
        }
    }

    template<typename T>
    bool isInside(const geometry_msgs::msg::Polygon polygon, const T point)
    {
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

    unsigned long rosTimeToMilliseconds(const rclcpp::Time ros_time);
    
    void llToUtm(const double lat, const double lon, int &zone, double &utm_x, double &utm_y);
    void utmToLL(const double utm_x, const double utm_y, const int zone, double &lat, double &lon);
    void mapToLl(const double px, const double py, double &lat, double &lon,
                 const double utm_x_offset, const double utm_y_offset, const double utm_zone);
    void llToMap(const double lat, const double lon, double &px, double &py,
                 const double utm_x_offset, const double utm_y_offset);

} // namespace decco_utilities

#endif // DECCO_UTILITIES_H_
