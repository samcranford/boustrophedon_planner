#include <float.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <list>
#include <string>
#include <thread>
#include <vector>

#include "boustrophedon_algorithms.hpp"

namespace boustrophedon_algorithms
{
    double euclideanDistance(double ax, double ay, double bx, double by)
    {
        double dx = ax - bx;
        double dy = ay - by;
        return std::pow(std::pow(dx, 2.0) + std::pow(dy, 2.0), 0.5);
    }

    bool isPointInsidePolygon(std::vector<Location> points, Location robot_location)
    {
        // Raycasting Algorithm to determine if inside or outside geofence
        double epsilon = 0.0000001; // Small epsilon to shift location if it falls on vertex point
        bool inside = false;
        Location lower_coord;
        Location upper_coord;

        for (size_t i = 0; i < points.size() - 1; i++)
        {
            auto current_coord = points[i];
            auto next_coord = points[i + 1];
            if (current_coord.y > next_coord.y)
            {
                lower_coord = next_coord;
                upper_coord = current_coord;
            }
            else
            {
                lower_coord = current_coord;
                upper_coord = next_coord;
            }
            if (robot_location.y == upper_coord.y || robot_location.y == lower_coord.y)
            {
                robot_location.y += epsilon;
            }
            if (robot_location.y < upper_coord.y && robot_location.y > lower_coord.y)
            {
                if (robot_location.x < std::min(lower_coord.x, upper_coord.x))
                {
                    inside = !inside;
                }
                else if (robot_location.x < std::max(lower_coord.x, upper_coord.x))
                {
                    double slope = (upper_coord.y - lower_coord.y) / (upper_coord.x - lower_coord.x);
                    double current_slope = (robot_location.y - lower_coord.y) / (robot_location.x - lower_coord.x);
                    if (current_slope > slope)
                    {
                        inside = !inside;
                    }
                }
            }
        }
        return inside;
    }

    bool getIntersection(const std::vector<Location> points, Location vertex, Segment &upper_segment, Segment &lower_segment, Location &upper_intersect,
                         Location &lower_intersect)
    {
        if (points.size() < 1)
        {
            return false;
        }
        double upper_splice_closest_distance = DBL_MAX;
        double lower_splice_closest_distance = DBL_MAX;
        bool valid_upper = false;
        bool valid_lower = false;
        // Loop through the polygon and find available upper and lower segments that the sweep line intercepts
        for (size_t i = 0; i < points.size() - 1; i++)
        {
            if ((vertex.x > points[i].x && vertex.x < points[i + 1].x) || (vertex.x < points[i].x && vertex.x > points[i + 1].x))
            {
                double slope = (points[i + 1].y - points[i].y) / (points[i + 1].x - points[i].x);
                double b = points[i + 1].y - slope * points[i + 1].x;
                double intercept = slope * vertex.x + b;
                // Check if intersect point exists
                Location viable_point;
                viable_point.y = intercept;
                viable_point.x = vertex.x;
                // Ensure intersection is not going outside of polygon
                Location mid_point;
                mid_point.y = (viable_point.y + vertex.y) / 2;
                mid_point.x = vertex.x;
                // If intersection is closest to vertex, make the point the new intersection point for upper and lower respectively
                if (isPointInsidePolygon(points, mid_point))
                {
                    if (viable_point.y > vertex.y)
                    {
                        double distance = euclideanDistance(vertex.x, vertex.y, viable_point.x, viable_point.y);
                        if (distance < upper_splice_closest_distance)
                        {
                            upper_splice_closest_distance = distance;
                            upper_intersect.y = viable_point.y;
                            upper_intersect.x = viable_point.x;
                            upper_segment.start = points[i];
                            upper_segment.end = points[i + 1];
                            valid_upper = true;
                        }
                    }
                    else if (viable_point.y < vertex.y)
                    {
                        double distance = euclideanDistance(vertex.x, vertex.y, viable_point.x, viable_point.y);
                        if (distance < lower_splice_closest_distance)
                        {
                            lower_splice_closest_distance = distance;
                            lower_intersect.y = viable_point.y;
                            lower_intersect.x = viable_point.x;
                            lower_segment.start = points[i];
                            lower_segment.end = points[i + 1];
                            valid_lower = true;
                        }
                    }
                }
            }
        }
        return valid_lower && valid_upper;
    }

    PointEvent spliceEvent(Location vertex, Location upper_edge_vertex, Location lower_edge_vertex)
    {
        if (lower_edge_vertex.x < vertex.x && upper_edge_vertex.x < vertex.x)
        {
            return PointEvent::Merge;
        }
        else if (lower_edge_vertex.x > vertex.x && upper_edge_vertex.x > vertex.x)
        {
            return PointEvent::Split;
        }
        else
        {
            return PointEvent::Middle;
        }
    }

    Location findNextVertex(std::vector<Location> points, Location vertex)
    {
        points.pop_back();
        for (size_t i = 0; i < points.size(); i++)
        {
            if (points[i].x == vertex.x && points[i].y == vertex.y)
            {
                if (i == points.size() - 1)
                {
                    return points.front();
                }
                else
                {
                    return points[i + 1];
                }
            }
        }
        return points.front();
    }

    Location findPreviousVertex(std::vector<Location> points, Location vertex)
    {
        points.pop_back();
        for (size_t i = 0; i < points.size(); i++)
        {
            if (points[i].x == vertex.x && points[i].y == vertex.y)
            {
                if (i == 0)
                {
                    return points.back();
                }
                else
                {
                    return points[i - 1];
                }
            }
        }
        return points.front();
    }

    bool replaceSegment(Segment edge_current, Segment edge_replacement, std::vector<Cell> &cells, size_t &cell_index)
    {
        for (size_t i = 0; i < cells.size(); i++)
        {
            for (size_t j = 0; j < cells[i].lower.size(); j++)
            {
                if (edge_current.start.x == cells[i].lower[j].start.x && edge_current.start.y == cells[i].lower[j].start.y)
                {
                    cells[i].lower[j] = edge_replacement;
                    cell_index = i;
                    return true;
                }
            }
            for (size_t j = 0; j < cells[i].upper.size(); j++)
            {
                if (edge_current.start.x == cells[i].upper[j].start.x && edge_current.start.y == cells[i].upper[j].start.y)
                {
                    cells[i].upper[j] = edge_replacement;
                    cell_index = i;
                    return true;
                }
            }
        }
        return false;
    }

    bool addSegment(Segment edge_find, Segment edge_add, std::vector<Cell> &cells, size_t &cell_index)
    {
        for (size_t i = 0; i < cells.size(); i++)
        {
            if (cells[i].lower.back().end.x == edge_find.end.x && cells[i].lower.back().end.y == edge_find.end.y)
            {
                cells[i].lower.emplace_back(edge_add);
                cell_index = i;
                return true;
            }
            else if (cells[i].upper.back().start.x == edge_find.start.x && cells[i].upper.back().start.y == edge_find.start.y)
            {
                cells[i].upper.emplace_back(edge_add);
                cell_index = i;
                return true;
            }
        }
        return false;
    }

    bool findCell(const std::vector<Cell> cells, Segment edge_find, size_t &cell_index)
    {
        for (size_t i = 0; i < cells.size(); i++)
        {
            if (cells[i].lower.back().end.x == edge_find.end.x && cells[i].lower.back().end.y == edge_find.end.y)
            {
                cell_index = i;
                return true;
            }
            else if (cells[i].upper.back().start.x == edge_find.start.x && cells[i].upper.back().start.y == edge_find.start.y)
            {
                cell_index = i;
                return true;
            }
        }
        return false;
    }

    void appendNewLocation(Location new_point, std::vector<Location> &points)
    {
        if (points.size() == 0)
        {
            points.emplace_back(new_point);
        }
        else if (points.back().x != new_point.x || points.back().y != new_point.y)
        {
            points.emplace_back(new_point);
        }
        return;
    }

    Location rotatePoint(Location point, Location anchor_point, double rotation_angle)
    {
        double radian_angle = degreesToRadians(rotation_angle);
        double dx = point.x - anchor_point.x;
        double dy = point.y - anchor_point.y;
        Location rotated_point;
        rotated_point.x = anchor_point.x + ((std::cos(radian_angle) * dx) - (std::sin(radian_angle) * dy));
        rotated_point.y = anchor_point.y + ((std::sin(radian_angle) * dx) + (std::cos(radian_angle) * dy));
        return rotated_point;
    }

    std::vector<Location> rotatePoints(std::vector<Location> points, Location anchor_point, double rotation_angle)
    {
        std::vector<Location> rotated_points;
        for (size_t i = 0; i < points.size(); i++)
        {
            rotated_points.emplace_back(rotatePoint(points[i], anchor_point, rotation_angle));
        }
        return rotated_points;
    }

    Orient orientation(Location p, Location q, Location r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) {
        return Orient::Collinear;
    }
    return (val > 0) ? Orient::Clockwise : Orient::Counterclockwise;
}

    std::vector<std::vector<Location>> boustrophedonDecompositionPlanner(const std::vector<Location> points, double waypoint_angle)

    {
        double epsilon = .000001;                    // Cannot have critical vertices on same sweep line
        double rotation_angle = 90 - waypoint_angle; // Angle to make waypoint angle 90, for vertical sweep line
        Location anchor_point{0, 0};                 // Anchor point not inside the polygon
        std::vector<Location> ccw_points = rotatePoints(points, anchor_point, rotation_angle);

        // Remove closure to check for points on same sweep line
        // Ensure no vertices have the same vertical sweep values for lexigraphical sorting
        // Re-close polygon after check
        if (ccw_points.front().x == ccw_points.back().x && ccw_points.front().y == ccw_points.back().y)
        {
            ccw_points.pop_back();
        }
        for (size_t i = 0; i < ccw_points.size(); i++)
        {
            for (size_t j = 0; j < ccw_points.size(); j++)
            {
                if (ccw_points[i].x == ccw_points[j].x && i != j)
                {
                    ccw_points[j].x += epsilon;
                }
            }
        }

        std::vector<Location> sorted_points = ccw_points;
        std::sort(sorted_points.begin(), sorted_points.end(), CompareLocations());

        if (ccw_points.front().x != ccw_points.back().x || ccw_points.front().y != ccw_points.back().y)
        {
            ccw_points.emplace_back(ccw_points.front());
        }

        std::vector<Cell> closed_polygons;
        std::vector<Cell> open_polygons;

        // Process vertices and decompose as needed
        for (size_t i = 0; i < sorted_points.size(); i++)
        {
            // Determine the edges which the current vertex belongs to
            Location vertex = sorted_points[i];
            Location next_ccw_vertex = findNextVertex(ccw_points, vertex);
            Location prev_ccw_vertex = findPreviousVertex(ccw_points, vertex);
            Location upper_edge_vertex = next_ccw_vertex;
            Location lower_edge_vertex = prev_ccw_vertex;
            Segment e_upper;
            Segment e_lower;
            PointEvent event = spliceEvent(vertex, upper_edge_vertex, lower_edge_vertex);
            switch (event)
            {
            case PointEvent::Merge:
            {
                if(orientation(lower_edge_vertex, vertex, upper_edge_vertex) != Orient::Counterclockwise){
                    upper_edge_vertex = prev_ccw_vertex;
                    lower_edge_vertex = next_ccw_vertex;
                }
                e_upper.start = vertex;
                e_upper.end = upper_edge_vertex;
                e_lower.start = lower_edge_vertex;
                e_lower.end = vertex;
                Location upper_intersect;
                Location lower_intersect;
                Segment upper_segment;
                Segment lower_segment;
                bool intersection_exists = getIntersection(ccw_points, vertex, upper_segment, lower_segment, upper_intersect, lower_intersect);
                // If intersection exists replace the intersected segments, then close two, open one, else close one
                if (intersection_exists)
                {
                    std::cout << "Merge Close 2 Open 1" << std::endl;
                    Segment upper_replacement = upper_segment;
                    upper_replacement.start = upper_intersect;
                    size_t open_upper_cell_index;
                    Segment lower_replacement = lower_segment;
                    lower_replacement.end = lower_intersect;
                    size_t open_lower_cell_index;
                    bool upper_found = replaceSegment(upper_segment, upper_replacement, open_polygons, open_upper_cell_index);
                    bool lower_found = replaceSegment(lower_segment, lower_replacement, open_polygons, open_lower_cell_index);
                    if (upper_found && lower_found)
                    {
                        if (open_upper_cell_index > open_lower_cell_index)
                        {
                            closed_polygons.emplace_back(open_polygons[open_upper_cell_index]);
                            closed_polygons.emplace_back(open_polygons[open_lower_cell_index]);
                            open_polygons.erase(open_polygons.begin() + open_upper_cell_index);
                            open_polygons.erase(open_polygons.begin() + open_lower_cell_index);
                        }
                        else
                        {
                            closed_polygons.emplace_back(open_polygons[open_upper_cell_index]);
                            closed_polygons.emplace_back(open_polygons[open_lower_cell_index]);
                            open_polygons.erase(open_polygons.begin() + open_lower_cell_index);
                            open_polygons.erase(open_polygons.begin() + open_upper_cell_index);
                        }
                    }
                    Cell new_polygon;
                    Segment new_upper;
                    new_upper.end = upper_intersect;
                    new_upper.start = upper_segment.start;
                    Segment new_lower;
                    new_lower.end = lower_segment.end;
                    new_lower.start = lower_intersect;
                    new_polygon.lower.emplace_back(new_lower);
                    new_polygon.upper.emplace_back(new_upper);
                    open_polygons.emplace_back(new_polygon);
                }
                else
                {
                    std::cout << "Merge Close one" << std::endl;
                    size_t open_cell_index;
                    bool cell_found = findCell(open_polygons, e_upper, open_cell_index);
                    if (cell_found)
                    {
                        closed_polygons.emplace_back(open_polygons[open_cell_index]);
                        open_polygons.erase(open_polygons.begin() + open_cell_index);
                    }
                }
            }
            break;
            case PointEvent::Split:
            {
                if(orientation(lower_edge_vertex, vertex, upper_edge_vertex) != Orient::Clockwise){
                    upper_edge_vertex = prev_ccw_vertex;
                    lower_edge_vertex = next_ccw_vertex;
                }
                e_upper.start = upper_edge_vertex;
                e_upper.end = vertex;
                e_lower.start = vertex;
                e_lower.end = lower_edge_vertex;
                Location upper_intersect;
                Location lower_intersect;
                Segment upper_segment;
                Segment lower_segment;
                bool intersection_exists = getIntersection(ccw_points, vertex, upper_segment, lower_segment, upper_intersect, lower_intersect);
                // If intersection exists replace the intersected segments, then close one, open two, else open one
                if (intersection_exists)
                {
                    std::cout << "Split Close One Open Two" << std::endl;
                    // Replace Segments and close the open polygon
                    Segment upper_replacement = upper_segment;
                    upper_replacement.start = upper_intersect;
                    size_t open_upper_cell_index;
                    Segment lower_replacement = lower_segment;
                    lower_replacement.end = lower_intersect;
                    size_t open_lower_cell_index;
                    bool upper_found = replaceSegment(upper_segment, upper_replacement, open_polygons, open_upper_cell_index);
                    bool lower_found = replaceSegment(lower_segment, lower_replacement, open_polygons, open_lower_cell_index);
                    if (upper_found && lower_found)
                    {
                        closed_polygons.emplace_back(open_polygons[open_upper_cell_index]);
                        open_polygons.erase(open_polygons.begin() + open_upper_cell_index);
                    }
                    // Open the new upper polygon
                    Cell new_upper_polygon;
                    Segment new_upper;
                    new_upper.end = upper_intersect;
                    new_upper.start = upper_segment.start;
                    Segment new_lower;
                    new_lower.start = e_upper.end;
                    new_lower.end = e_upper.start;
                    new_upper_polygon.lower.emplace_back(new_lower);
                    new_upper_polygon.upper.emplace_back(new_upper);
                    open_polygons.emplace_back(new_upper_polygon);
                    // Open the new lower polygon
                    Cell new_lower_polygon;
                    new_upper.start = e_lower.end;
                    new_upper.end = e_lower.start;
                    new_lower.end = lower_segment.end;
                    new_lower.start = lower_intersect;
                    new_lower_polygon.lower.emplace_back(new_lower);
                    new_lower_polygon.upper.emplace_back(new_upper);
                    open_polygons.emplace_back(new_lower_polygon);
                }
                else
                {
                    std::cout << "Open One" << std::endl;
                    Cell new_polygon;
                    new_polygon.lower.emplace_back(e_lower);
                    new_polygon.upper.emplace_back(e_upper);
                    open_polygons.emplace_back(new_polygon);
                }
            }
            break;
            case PointEvent::Middle:
            {
                // Add Segment
                Segment e_right;
                Segment e_left;
                size_t cell_index;
                // Floor Segment
                if (next_ccw_vertex.x > prev_ccw_vertex.x)
                {
                    e_right.start = vertex;
                    e_right.end = next_ccw_vertex;
                    e_left.start = prev_ccw_vertex;
                    e_left.end = vertex;
                }
                else
                { // Ceiling Segment
                    e_right.start = prev_ccw_vertex;
                    e_right.end = vertex;
                    e_left.start = vertex;
                    e_left.end = next_ccw_vertex;
                }
                addSegment(e_left, e_right, open_polygons, cell_index);
            }
            break;
            default:
            {
                std::cout << "Undefined Event Ignored" << std::endl;
                // Default does not decompose
            }
            }
        }
        std::vector<std::vector<Location>> closed_polygons_vector;
        for (size_t i = 0; i < closed_polygons.size(); i++)
        {
            std::vector<Location> sub_polygon;
            for (size_t j = 0; j < closed_polygons[i].lower.size(); j++)
            {
                appendNewLocation(closed_polygons[i].lower[j].start, sub_polygon);
                appendNewLocation(closed_polygons[i].lower[j].end, sub_polygon);
            }
            if (closed_polygons[i].upper.size() > 0)
            {
                for (int j = closed_polygons[i].upper.size() - 1; j >= 0; j--)
                {
                    appendNewLocation(closed_polygons[i].upper[j].start, sub_polygon);
                    appendNewLocation(closed_polygons[i].upper[j].end, sub_polygon);
                }
            }
            closed_polygons_vector.emplace_back(sub_polygon);
        }
        // Rotate the polygons back to the original orientation
        for (size_t i = 0; i < closed_polygons_vector.size(); i++)
        {
            closed_polygons_vector[i] = rotatePoints(closed_polygons_vector[i], anchor_point, -rotation_angle);
        }
        return closed_polygons_vector;
    }

} // namespace boustrophedon_algorithms