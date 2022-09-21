#include "boustrophedon_algorithms.hpp"
#include "iostream"

int main()
{
    std::vector<boustrophedon_algorithms::Location> input_polygon;
    double waypoint_angle = 0.0;
    // Location (y,x)
    boustrophedon_algorithms::Location a{2.0, 13.0};
    boustrophedon_algorithms::Location b{15.0, 15.0};
    boustrophedon_algorithms::Location c{10.0, 7.0};
    boustrophedon_algorithms::Location d{15.0, 0};
    boustrophedon_algorithms::Location e{2.0, 2.0};
    input_polygon.emplace_back(a);
    input_polygon.emplace_back(b);
    input_polygon.emplace_back(c);
    input_polygon.emplace_back(d);
    input_polygon.emplace_back(e);
    input_polygon.emplace_back(a);
    std::vector<std::vector<boustrophedon_algorithms::Location>> output_polygons = boustrophedon_algorithms::boustrophedonDecompositionPlanner(input_polygon, waypoint_angle);
    for (size_t i = 0; i < output_polygons.size(); i++)
    {
        std::cout << "Polygon" << std::endl;
        for (size_t j = 0; j < output_polygons[i].size(); j++)
        {
            std::cout << output_polygons[i][j].x << " , " << output_polygons[i][j].y << std::endl;
        }
    }
    return 0;
}