#include "planner/voronoi_boost.h"

void VoronoiBuilder::create_voronoi_from_WTK(string filename)
{
    // opens the file
    std::ifstream file(filename);

    // checks if the file is open successfully
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        exit(1); // Return an error code
    }

    // reads the content of the file into a string
    std::string wkt_string((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());

    // closes the file
    file.close();

    std::cout << "WKT: " << wkt_string << "\n";
    boost::geometry::read_wkt(wkt_string, polygon);
    std::string reason;
    // checks for validity of the polygon otherwise tries to correct it
    bool ok = boost::geometry::is_valid(polygon, reason);
    std::cout << "Expected: " << boost::geometry::dsv(polygon) << (ok ? "\nand got: VALID" : "\nand got: INVALID -> '" + reason + "'") << "\n";
    if (!ok)
    {
        correct_geometry(polygon);
    }

    // prepares data structure for voronoi diagram
    std::vector<Point> points;
    std::vector<Segment> segments;

    // Iterates over exterior ring segments
    auto &exterior_ring = boost::geometry::exterior_ring(polygon);
    for (std::size_t i = 0; i < exterior_ring.size() - 1; ++i)
    {
        const auto &vertex1 = exterior_ring[i];
        const auto &vertex2 = exterior_ring[i + 1];
        std::cout << "Exterior Segment: (" << boost::geometry::get<0>(vertex1) << ", " << boost::geometry::get<1>(vertex1) << ") - ("
                  << boost::geometry::get<0>(vertex2) << ", " << boost::geometry::get<1>(vertex2) << ")\n";
        segments.push_back(Segment(boost::geometry::get<0>(vertex1), boost::geometry::get<1>(vertex1), boost::geometry::get<0>(vertex2), boost::geometry::get<1>(vertex2)));
    }

    // Iterate over interior rings segments
    auto &interior_rings = boost::geometry::interior_rings(polygon);
    for (const auto &interior_ring : interior_rings)
    {
        for (std::size_t i = 0; i < interior_ring.size() - 1; ++i)
        {
            const auto &vertex1 = interior_ring[i];
            const auto &vertex2 = interior_ring[i + 1];
            std::cout << "Interior Segment: (" << boost::geometry::get<0>(vertex1) << ", " << boost::geometry::get<1>(vertex1) << ") - ("
                      << boost::geometry::get<0>(vertex2) << ", " << boost::geometry::get<1>(vertex2) << ")\n";
            segments.push_back(Segment(boost::geometry::get<0>(vertex1), boost::geometry::get<1>(vertex1), boost::geometry::get<0>(vertex2), boost::geometry::get<1>(vertex2)));
        }
    }

    for (auto &segment : segments)
    {
        std::cout << "Segment: (" << segment.p0.a << ", " << segment.p0.b << ") - ("
                  << segment.p1.a << ", " << segment.p1.b << ")\n";
    }
    construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &voronoi_diagram_);
}
void VoronoiBuilder::save_voronoi(string path)
{
    std::ofstream fout(path);
    for (voronoi_diagram<double>::const_edge_iterator edge = voronoi_diagram_.edges().begin(); edge != voronoi_diagram_.edges().end(); ++edge)
    {
        // if edges are infinite, we do not want to save them
        if (edge->is_finite() && !is_edge_connected_to_polygon(edge, polygon))
        {
            std::cout << "Adding edge: " << edge->vertex0()->x() << ", " << edge->vertex0()->y() << " - " << edge->vertex1()->x() << ", " << edge->vertex1()->y() << std::endl;
            fout << edge->vertex0()->x() << "," << edge->vertex0()->y() << "," << edge->vertex1()->x() << "," << edge->vertex1()->y() << std::endl;
        }
    }
    return;
}

void VoronoiBuilder::correct_geometry(Polygon &polygon)
{
    boost::geometry::validity_failure_type failure;
    bool could_be_fixed = (failure == boost::geometry::failure_not_closed || boost::geometry::failure_wrong_orientation);

    std::cout << "can boost::geometry::correct remedy invalidity? " << (could_be_fixed ? "possibly yes" : "no") << std::endl;
    if (could_be_fixed)
    {
        boost::geometry::correct(polygon);
        std::cout << "after correction: " << (boost::geometry::is_valid(polygon) ? "valid" : "still invalid") << std::endl;
        std::cout << "corrected geometry: " << boost::geometry::dsv(polygon) << std::endl;
    }
    else
    {
        std::cout << "cannot be fixed" << std::endl;
        exit(1);
    }
}

bool VoronoiBuilder::is_edge_connected_to_polygon(voronoi_diagram<double>::const_edge_iterator edge, Polygon &polygon)
{
    // check the edge ends in a outer polygon's vertex
    for (const auto &point : polygon.outer())
    {
        if (edge->vertex0()->x() == point.x() && edge->vertex0()->y() == point.y())
        {
            return true;
        }
        if (edge->vertex1()->x() == point.x() && edge->vertex1()->y() == point.y())
        {
            return true;
        }
    }
    // check the edge ends in a inner polygon's vertex
    for (const auto &inner_ring : polygon.inners())
    {
        for (const auto &point : inner_ring)
        {
            if (edge->vertex0()->x() == point.x() && edge->vertex0()->y() == point.y())
            {
                return true;
            }
            if (edge->vertex1()->x() == point.x() && edge->vertex1()->y() == point.y())
            {
                return true;
            }
        }
    }

    return false;
}


int main()
{
    // monitor execution time
    auto start = high_resolution_clock::now();
    VoronoiBuilder vb;
    vb.create_voronoi_from_WTK(ament_index_cpp::get_package_share_directory("planner") + "/data/polygon.txt");
    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "Execution time: " << duration.count() << " microseconds. Saving..." << std::endl;
    vb.save_voronoi(ament_index_cpp::get_package_share_directory("planner") + "/data/boost_voronoi_edges.csv");
    std::cout << "Done!" << std::endl;
    return 0;
}
