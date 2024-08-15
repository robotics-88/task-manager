#ifndef CENTROID_SPLITTER_H_
#define CENTROID_SPLITTER_H_

#include <vector>
#include <cmath>
#include <map>
#include <iostream>

#include <geometry_msgs/msg/polygon.hpp>

namespace centroid_splitter
{

struct Vec2
{
    double x;
    double y;

    Vec2() {}
    Vec2(const double _x, const double _y) : x(_x), y(_y) {}

    static float length(Vec2 const & v)
    {
        return sqrtf(v.x*v.x + v.y*v.y);
    }

    static Vec2 norm(Vec2 const & v)
    {
        if(length(v) < 1e-30)
            return {0.0f, 0.0f};
    }

    static float dot(Vec2 const & v1, Vec2 const & v2)
    {
        return v1.x * v2.x + v1.y * v2.y;
    }

    static float square(Vec2 const & v)
    {
        return dot(v,v);
    }

    static float cross(Vec2 const & v1, Vec2 const & v2)
    {
        return v1.x*v2.y - v1.y*v2.x;
    }

    Vec2 operator - (Vec2 const & v1) const
    {
        return {x - v1.x, y - v1.y};
    }

    Vec2 operator + (Vec2 const & v1) const
    {
        return {x + v1.x, y + v1.y};
    }

    Vec2 operator * (float const & f) const
    {
        return {f*x, f*y};
    }

    Vec2 operator / (float const & f) const
    {
        return {x/f, y/f};
    }

    static float getSignedArea(Vec2 const & v1,
                               Vec2 const & v2)
    {
        return (v2.x - v1.x) * (v2.y + v1.y);
    }
};

struct Vertex
{
    Vec2 position;

    Vertex() {}
    Vertex(Vec2 const & _position) : position{_position} {}

    static float getHandedness(Vertex const & v1,
                               Vertex const & v2,
                               Vertex const & v3)
    {
        Vec2 edge1 = v2.position-v1.position;
        Vec2 edge2 = v3.position-v2.position;

        return Vec2::cross(edge1, edge2);
    }
};

struct LineSegment
{
    Vec2 startPos;
    Vec2 finalPos;

    LineSegment() {}
    LineSegment(Vec2 const & _startPos,
                Vec2 const & _finalPos) : startPos{_startPos},
                finalPos{_finalPos} {}

    Vec2 direction() const
    {
        return finalPos-startPos;
    }

    Vec2 normalisedDirection()
    {
        return Vec2::norm(finalPos-startPos);
    }

    LineSegment operator + (LineSegment const & ls)
    {
        Vec2 newStartPos = (startPos + ls.startPos) / 2.0f;
        Vec2 newFinalPos = (finalPos + ls.finalPos) / 2.0f;

        return LineSegment(newStartPos, newFinalPos);
    }

    static std::pair<bool, Vec2> intersects(LineSegment s1, LineSegment s2)
    {
        const float TOLERANCE = 1e-2;

        Vec2 p1 = s1.startPos;
        Vec2 p2 = s2.startPos;
        Vec2 d1 = s1.direction();
        Vec2 d2 = s2.direction();

        if(std::abs(Vec2::cross(d1, d2)) < 1e-30)
           return {false, {0.0f, 0.0f}};

        float t1 = Vec2::cross(p2 - p1, d2) / Vec2::cross(d1, d2);

        if((t1 < (0.0f - TOLERANCE)) || (t1 > (1.0f + TOLERANCE)))
            return {false, {0.0f, 0.0f}};

        Vec2 pIntersect = p1 + d1 * t1;

        float t2 = Vec2::dot(pIntersect - p2,
                             s2.finalPos - p2);

        if(t2 < (0.0f-TOLERANCE) || t2 / Vec2::square(s2.finalPos - p2) >= 1.0f - TOLERANCE)
            return {false, {0.0f, 0.0f}};

        return {true, pIntersect};
    }
};

struct Polygon2Split
{
    Polygon2Split() {}
    Polygon2Split(geometry_msgs::msg::Polygon const &polygon) : original(polygon) {
        // Get area, polygon is in meters/local map coordinates so no need for geodesic
        double x_sum = 0, y_sum = 0, xy_sum = 0, sum_x_square = 0;
        int N = polygon.points.size();
        for (int ii = 0; ii<N; ii++) {
            // Add to avgs
            x_sum += polygon.points.at(ii).x;
            y_sum += polygon.points.at(ii).y;
            xy_sum += polygon.points.at(ii).x * polygon.points.at(ii).y;
            sum_x_square += polygon.points.at(ii).x * polygon.points.at(ii).x;
            // Make edge
            Vec2 start_pos(polygon.points.at(ii).x, polygon.points.at(ii).y);
            int next_ind = ii + 1;
            if (next_ind == polygon.points.size()) {
                next_ind = 0;
            }
            Vec2 end_pos(polygon.points.at(next_ind).x, end_pos.y = polygon.points.at(next_ind).y);
            LineSegment ls(start_pos, end_pos);
            edges.push_back(ls);
            Vertex vert(start_pos);
            vertices.push_back(vert);
        }
        // Centroid is avg x,y
        centroid.x = x_sum / N;
        centroid.y = y_sum / N;

        // Get best fit line
        float slope_numerator = (N * xy_sum - x_sum * y_sum);
        float slope_denominator = (N * sum_x_square - x_sum * x_sum);
        blfs_params.push_back(slope_numerator);
        blfs_params.push_back(slope_denominator);
    }

    double area;
    Vec2 centroid;
    std::vector<double> blfs_params;
    LineSegment opposite_line;
    geometry_msgs::msg::Polygon original;
    std::vector<Vertex> vertices;
    std::vector<LineSegment> edges;
};

class CentroidSplitter
{
    typedef std::vector<Polygon2Split > PolygonArray;
    typedef std::vector<geometry_msgs::msg::Polygon > OutputArray;

    Polygon2Split original_polygon;
    OutputArray outputPolygons;
    double area_threshold;



public:
    CentroidSplitter(geometry_msgs::msg::Polygon const & _polygon, double _threshold) : original_polygon{_polygon}, area_threshold(_threshold)
    {
        assignArea();
        assignOppositeLine();
    }

    int mod(int x, int m)
    {
        int r = x%m;
        return r<0 ? r+m : r;
    }

    void assignArea() {
        // Area
        float signedArea = 0.0f;
        for(unsigned int i=0; i<original_polygon.vertices.size(); ++i)
        {
            signedArea += Vec2::getSignedArea(original_polygon.vertices[i].position,
                                                   original_polygon.vertices[mod(i+1, original_polygon.vertices.size())].position);
        }
        original_polygon.area = abs(signedArea);
    }

    void assignOppositeLine() {
        // For comparison with opposite line, the best fit line would have been:
        // line.slope = cv_line[1] / cv_line[0];
        // line.intercept = cv_line[3] - line.slope * cv_line[2];
        // Create a line segment from the best fit line with large padding, so can compute line segment intersections
        // Compute opposite line
        double opposite_slope = -1 * original_polygon.blfs_params.at(1) / original_polygon.blfs_params.at(0);
        double opposite_intercept = original_polygon.centroid.y - opposite_slope * original_polygon.centroid.x;
        double x1 = original_polygon.centroid.x - original_polygon.area;
        double y1 = opposite_slope * x1 + opposite_intercept;
        double x2 = original_polygon.centroid.x + original_polygon.area;
        double y2 = opposite_slope * x2 + opposite_intercept;
        LineSegment bfls_opposite(Vec2(x1, y1), Vec2(x2, y2));
        original_polygon.opposite_line = bfls_opposite;

    }

    std::vector<geometry_msgs::msg::Polygon> slicePolygon()
    {
        std::vector<geometry_msgs::msg::Polygon> polys;
        // fill in recursive alg, and produce outputPolygons on finish
        if (original_polygon.area < area_threshold) {
            polys.push_back(original_polygon.original);
        }
        else {
            geometry_msgs::msg::Polygon poly1, poly2;
            halvePolygon(poly1, poly2);
            CentroidSplitter centroid1 = CentroidSplitter(poly1, area_threshold);
            CentroidSplitter centroid2 = CentroidSplitter(poly2, area_threshold);
            std::vector<geometry_msgs::msg::Polygon> result1 = centroid1.slicePolygon();
            std::vector<geometry_msgs::msg::Polygon> result2 = centroid2.slicePolygon();
            if (polys.empty()) {
                polys = result1;
            }
            else {
                polys.insert(polys.end(), result1.begin(), result1.end());
            }
            polys.insert(polys.end(), result2.begin(), result2.end());
        }
        return polys;
    }

    void halvePolygon(geometry_msgs::msg::Polygon &poly1, geometry_msgs::msg::Polygon &poly2) {
        // Only guaranteed for convex shapes
        std::vector<int> indices;
        int intersect_count = 0;
        std::vector<Vec2> intersection_vertex;
        for (int nn = 0; nn < original_polygon.edges.size(); nn++) {
            std::pair<bool, Vec2 > intersectionResult = LineSegment::intersects(original_polygon.edges.at(nn), original_polygon.opposite_line);
            if (intersectionResult.first) {
                intersect_count++;
                indices.push_back(nn);
                intersection_vertex.push_back(intersectionResult.second);
                if (intersect_count == 2) {
                    // Collect vertices for each half and generate output polys
                    // Group 1
                    std::vector<Vec2> new_vertices1;
                    new_vertices1.push_back(intersection_vertex.at(0));
                    addVerticesBetweenIndices(new_vertices1, indices.at(0), indices.at(1));
                    new_vertices1.push_back(intersection_vertex.at(1));
                    // Group 2
                    std::vector<Vec2> new_vertices2;
                    new_vertices2.push_back(intersection_vertex.at(1));
                    addVerticesBetweenIndices(new_vertices2, indices.at(1), indices.at(0));
                    new_vertices2.push_back(intersection_vertex.at(0));
                    // Make geom polygons from each
                    geometryFromVertices(new_vertices1, poly1);
                    geometryFromVertices(new_vertices2, poly2);
                    return;
                }
            }
        }
    }

    void addVerticesBetweenIndices(std::vector<Vec2> &vertices, const int start_ind, const int end_ind) {
        int len;
        if (end_ind > start_ind) {
            len = end_ind - start_ind;
        }
        else {
            len = end_ind + (original_polygon.edges.size() - start_ind);
        }
        int index = start_ind;
        for (int nn = 0; nn < len; nn++) {
            index++; // Skip the first line segment
            if (index >= original_polygon.edges.size()) {
                index -= original_polygon.edges.size();
            }
            vertices.push_back(original_polygon.edges.at(index).startPos);
        }

    }

    void geometryFromVertices(const std::vector<Vec2> vertices, geometry_msgs::msg::Polygon &polygon) {
        for (int nn = 0; nn < vertices.size(); nn++) {
            geometry_msgs::msg::Point32 pt;
            pt.x = vertices.at(nn).x;
            pt.y = vertices.at(nn).y;
            polygon.points.push_back(pt);
        }
    }

    std::vector<geometry_msgs::msg::Polygon> getSubpolygons() {
        return outputPolygons;
    }

};

}

#endif // CENTROID_SPLITTER_H_
