#pragma once

#include <list>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace boustrophedon_algorithms
{
    struct Location
    {
        double y;
        double x;
    };

    enum PointEvent
    {
        Split,
        Merge,
        Middle
    }; // Point event for decomposing polgyons

    struct Segment
    { // Segment structure used for polygon segmentation
        Location start;
        Location end;
    };

    struct Cell
    { // Cell structure used for polygon segmentation
        std::vector<Segment> upper;
        std::vector<Segment> lower;
    };

    struct CompareLocations
    { // Compares Location values based on x
        bool operator()(const Location &a, const Location &b) const
        {
            return (a.x < b.x);
        }
    };

    /**
     * @brief determine the intersected segments from a vertical sweep line during polygon decomposition
     *
     * @param points polygon points
     * @param vertex the vertex that the sweep line is on
     * @param[output] upper_segment the upper segment intercepted by the sweep line if found
     * @param[output] lower_segment the lower segment intercepted by the sweep line if found
     * @param[output] upper_intersect the upper intersect point of the sweep line if found
     * @param[output] lower_intersect the lower intersect point of the sweep line if found
     * @return true if both lower and upper intersection points were found
     * @return false otherwise
     */
    bool getIntersection(const std::vector<Location> points, Location vertex, Segment &upper_segment, Segment &lower_segment, Location &upper_intersect,
                         Location &lower_intersect);

    /**
     * @brief determines the point event based on the current vertex, the previous vertex, and the next vertex
     *
     * @param vertex the current vertex to determine splice event
     * @param upper_edge_vertex the vertex, previous or next, with the higher y value
     * @param lower_edge_vertex the vertex, previous or next, with the lower y value
     * @return PointEvent the point event occurring at the vertex
     */
    PointEvent spliceEvent(Location vertex, Location upper_edge_vertex, Location lower_edge_vertex);

    /**
     * @brief finds the next vertex in the polygon
     *
     * @param points polygon
     * @param vertex current vertex
     * @return Location next vertex
     */
    Location findNextVertex(const std::vector<Location> points, Location vertex);

    /**
     * @brief finds the previous vertex in the polygon
     *
     * @param points polygon
     * @param vertex current vertex
     * @return Location previous vertex
     */
    Location findPreviousVertex(const std::vector<Location> points, Location vertex);

    /**
     * @brief replaces a segment with another segment for the cell containing it
     *
     * @param edge_current current segment
     * @param edge_replacement segment to replace current segment
     * @param[output] cells vector of cells to look for the current segment in
     * @param[output] cell_index the index of the cell which contained the segment
     * @return true if a cell was found to contain the segment and it was replaced
     * @return false otherwise
     */
    bool replaceSegment(Segment edge_current, Segment edge_replacement, std::vector<Cell> &cells, size_t &cell_index);

    /**
     * @brief add a segment to a cell following another segment
     *
     * @param edge_find the segment to find
     * @param edge_add  the segment to add after the found segment
     * @param[output] cells vector of cells
     * @param[output] cell_index index to cell which segment was added to
     * @return true if segment was added
     * @return false otherwise
     */
    bool addSegment(Segment edge_find, Segment edge_add, std::vector<Cell> &cells, size_t &cell_index);

    /**
     * @brief finds a cell which contains a segment
     *
     * @param cells vector of cells
     * @param edge_find segment to find
     * @param[output] cell_index index to cell that contained segment
     * @return true if found
     * @return false otherwise
     */
    bool findCell(const std::vector<Cell> cells, Segment edge_find, size_t &cell_index);

    /**
     * @brief adds a location to a vector if the location does not equal the previously added location
     *
     * @param new_point point to add
     * @param[output] points vector of locations to add to
     */
    void appendNewLocation(Location new_point, std::vector<Location> &points);

    /**
     * @brief Boustrophedon Decomposition Planner is a trapezoidal based decomposition with middle events not decomposed. See link for description.
     * \warning Planner works only for counter clockwise polygon points
     * https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.69.2482&rep=rep1&type=pdf
     * Takes in a polygon as a list of locations, calculates the sub polygons which are useable for survey like planning, and outputs a waypoint list
     * for waypoints in all sub polygons.
     * Survey sweep conducted vertically and therefore corn rows should also be make in the same direction.
     *
     * @param points polygon to be broken down
     * @return a list sub polygons found
     */
    std::vector<std::vector<Location>> boustrophedonDecompositionPlanner(const std::vector<Location> points);

} // namespace boustrophedon_algorithms