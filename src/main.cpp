#include "boustrophedon_algorithms.hpp"
#include "iostream"

int main()
{
    std::vector<boustrophedon_algorithms::Location> input_polygon;
    double waypoint_angle = 90.0;
    // Location (y,x)
    boustrophedon_algorithms::Location a{68.34799267041292,71.16124776575906};
    boustrophedon_algorithms::Location b{58.16932204985284,47.631891814741024};
    boustrophedon_algorithms::Location c{83.79529767744064,41.232334659101944};
    boustrophedon_algorithms::Location d{90.85692282440469,62.58255856841241};
    boustrophedon_algorithms::Location e{68.01697899189179,72.76113075899136};
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