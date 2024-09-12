#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

namespace lawnmower {

struct Point {
    double x, y;
};

bool isPointInPolygon(const std::vector<Point>& poly, const Point& p) {
    int n = poly.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((poly[i].y > p.y) != (poly[j].y > p.y)) &&
            (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

std::vector<Point> generateLawnmowerPattern(const std::vector<Point>& polygon, double legSpacing) {
    // Determine the bounding box of the polygon
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    
    for (const auto& p : polygon) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
    }
    
    std::vector<Point> pattern;
    bool direction = true;  // true for left-to-right, false for right-to-left
    
    for (double y = minY; y <= maxY; y += legSpacing) {
        bool found_first_in = false;
        Point last_point;
        if (direction) {
            for (double x = minX; x <= maxX; x += legSpacing) {
                Point p = {x, y};
                if (isPointInPolygon(polygon, p) && !found_first_in) {
                    pattern.push_back(p);
                    found_first_in = true;
                }
                else if ((!isPointInPolygon(polygon, p) && found_first_in) || (x + legSpacing > maxX)) {
                    if (isPointInPolygon(polygon, last_point)) {
                        pattern.push_back(last_point);
                    }
                }
                last_point = p;
            }
        } else {
            for (double x = maxX; x >= minX; x -= legSpacing) {
                Point p = {x, y};
                if (isPointInPolygon(polygon, p) && !found_first_in) {
                    pattern.push_back(p);
                    found_first_in = true;
                }
                else if ((!isPointInPolygon(polygon, p) && found_first_in) || (x - legSpacing < minX)) {
                    if (isPointInPolygon(polygon, last_point)) {
                        pattern.push_back(last_point);
                    }
                    break;
                }
                last_point = p;
            }
        }
        direction = !direction;  // Alternate direction for the next row
    }
    
    return pattern;
}
}