// Compile with: clang++ -DBOOST_ALL_NO_LIB -DCGAL_USE_GMPXX=1 -O3 -g -Wall -Wextra -pedantic -march=native -frounding-math main.cpp -lgmpxx -lmpfr -lgmp
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/IO/WKT.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Segment_Delaunay_graph_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_policies_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_traits_2.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Voronoi_diagram_2.h>

#include <algorithm>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_set>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Segment_Delaunay_graph_traits_2<K> Gt;
typedef CGAL::Segment_Delaunay_graph_2<Gt> SDG2;
typedef CGAL::Segment_Delaunay_graph_adaptation_traits_2<SDG2> AT;
typedef CGAL::Segment_Delaunay_graph_degeneracy_removal_policy_2<SDG2> AP;
typedef CGAL::Voronoi_diagram_2<SDG2, AT, AP> VoronoiDiagram;
typedef AT::Site_2 Site_2;
typedef AT::Point_2 Point_2;
typedef VoronoiDiagram::Locate_result Locate_result;
typedef VoronoiDiagram::Vertex_handle Vertex_handle;
typedef VoronoiDiagram::Face_handle Face_handle;
typedef VoronoiDiagram::Halfedge_handle Halfedge_handle;
typedef VoronoiDiagram::Ccb_halfedge_circulator Ccb_halfedge_circulator;
typedef VoronoiDiagram::Bounded_halfedges_iterator BHE_Iter;
typedef VoronoiDiagram::Halfedge Halfedge;
typedef VoronoiDiagram::Vertex Vertex;
typedef CGAL::Polygon_with_holes_2<K> Polygon;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef std::deque<Polygon> MultiPolygon;

/// Creates a hash of a Point_2, used for making O(1) point lookups
// struct Point2Hash {
//   size_t operator()(const Point_2 &pt) const {
//     std::hash<double> hasher;
//     auto seed = hasher(pt.x());
//     // boost::hash_combine from https://stackoverflow.com/q/35985960/752843
//     seed ^= hasher(pt.y()) + 0x9e3779b9 + (seed<<6) + (seed>>2);
//     return seed;
//   }
// };
const std::string input_file = "/home/robotics/Desktop/multipolygon_data.txt";
typedef std::set<Point_2> Point2_Set;
typedef std::map<Vertex_handle, int> VH_Int_Map;

/// Holds a more accessible description of the Voronoi diagram
struct MedialData
{
    /// Map of vertices comprising the Voronoi diagram
    VH_Int_Map vertex_handles;
    /// List of edges in the diagram (pairs of the vertices above)
    std::vector<std::pair<int, int>> edges;
    /// Medial axis up governor. 1:1 correspondance with edges above.
    std::vector<VoronoiDiagram::Delaunay_graph::Vertex_handle> ups;
    /// Medial axis down governor. 1:1 correspondance with edges above.
    std::vector<VoronoiDiagram::Delaunay_graph::Vertex_handle> downs;
};

/// Read well-known text from @p filename to obtain shape boundary
MultiPolygon get_wkt_from_file(std::string filename)
{
    std::ifstream fin(filename);
    MultiPolygon mp;
    CGAL::read_multi_polygon_WKT(fin, mp);

    if (mp.empty())
    {
        throw std::runtime_error("WKT file '" + filename + "' was empty!");
    }
    for (const auto &poly : mp)
    {
        if (poly.outer_boundary().size() == 0)
        {
            throw std::runtime_error("WKT file '" + filename + "' contained a polygon without an outer boundary!");
        }
    }

    return mp;
}

/// Converts a MultiPolygon into its corresponding Voronoi diagram
VoronoiDiagram convert_mp_to_voronoi_diagram(const MultiPolygon &mp)
{
    VoronoiDiagram vd;

    const auto add_segments_to_vd = [&](const auto &poly)
    {
        for (std::size_t i = 0; i < poly.size(); i++)
        {
            std::cerr << i << " " << std::fixed << std::setprecision(10) << poly[i] << std::endl;
            // Modulus to close the loop
            vd.insert(
                Site_2::construct_site_2(poly[i], poly[(i + 1) % poly.size()]));
        }
    };

    for (const auto &poly : mp)
    {                                              // For each polygon in MultiPolygon
        std::cout << poly << std::endl;            // Print polygon to screen for debugging
        add_segments_to_vd(poly.outer_boundary()); // Add the outer boundary
        for (const auto &hole : poly.holes())
        { // And any holes
            add_segments_to_vd(hole);
        }
    }

    if (!vd.is_valid())
    {
        throw std::runtime_error("Voronoi Diagram was not valid!");
    }

    return vd;
}

/// Find @p item in collection @p c or add it if not present.
/// Returns the index of `item`'s location
int find_or_add(VH_Int_Map &c, const Vertex_handle &item)
{
    // Map means we can do this in log(N) time
    if (c.count(item) == 0)
    {
        c.emplace(item, c.size());
        return c.size() - 1;
    }

    return c.at(item);
}

/// Convert a map of <T, int> pairs to a vector of `T` ordered by increasing int
std::vector<Vertex_handle> map_to_ordered_vector(const VH_Int_Map &m)
{
    std::vector<std::pair<Vertex_handle, int>> to_sort(m.begin(), m.end());
    to_sort.reserve(m.size());
    std::sort(to_sort.begin(), to_sort.end(), [](const auto &a, const auto &b)
              { return a.second < b.second; });

    std::vector<Vertex_handle> ret;
    ret.reserve(to_sort.size());
    std::transform(begin(to_sort), end(to_sort), std::back_inserter(ret),
                   [](auto const &pair)
                   { return pair.first; });

    return ret;
}

/// Find vertex handles which are in the interior of the MultiPolygon
std::set<Vertex_handle> identify_vertex_handles_inside_mp(
    const VoronoiDiagram &vd,
    const MultiPolygon &mp)
{
    // Used to accelerate interior lookups by avoiding Point-in-Polygon checks for
    // vertices we've already considered
    std::set<Vertex_handle> considered;
    // The set of interior vertices we are building
    std::set<Vertex_handle> interior;

    for (
        auto edge_iter = vd.bounded_halfedges_begin();
        edge_iter != vd.bounded_halfedges_end();
        edge_iter++)
    {
        // Determine if an orientation implies an interior vertex
        const auto inside = [](const auto &orientation)
        {
            return orientation == CGAL::ON_ORIENTED_BOUNDARY || orientation == CGAL::POSITIVE;
        };

        // Determine if a vertex is in the interior of the multipolygon and, if so,
        // add it to `interior`
        const auto vertex_in_mp_interior = [&](const Vertex_handle &vh)
        {
            // Skip vertices which have already been considered, since a vertex may
            // be connected to multiple halfedges
            if (considered.count(vh) != 0)
            {
                return;
            }
            // Ensure we don't look at a vertex twice
            considered.insert(vh);
            // Determine if the vertex is inside of any polygon of the MultiPolygon
            const auto inside_of_a_poly = std::any_of(
                mp.begin(), mp.end(), [&](const auto &poly)
                { return inside(CGAL::oriented_side(vh->point(), poly)); });
            // If the vertex was inside the MultiPolygon make a note of it
            if (inside_of_a_poly)
            {
                interior.insert(vh);
            }
        };

        // Check both vertices of the current halfedge of the Voronoi diagram
        vertex_in_mp_interior(edge_iter->source());
        vertex_in_mp_interior(edge_iter->target());
    }

    return interior;
}

/// The medial axis is formed by building a Voronoi diagram and then removing
/// the edges of the diagram which connect to the concave points of the
/// MultiPolygon. Here, we identify those concave points
Point2_Set identify_concave_points_of_mp(const MultiPolygon &mp)
{
    Point2_Set concave_points;

    // Determine cross-product, given three points. The sign of the cross-product
    // determines whether the point is concave or convex.
    const auto z_cross_product = [](const Point_2 &pt1, const Point_2 &pt2, const Point_2 &pt3)
    {
        const auto dx1 = pt2.x() - pt1.x();
        const auto dy1 = pt2.y() - pt1.y();
        const auto dx2 = pt3.x() - pt2.x();
        const auto dy2 = pt3.y() - pt2.y();
        return dx1 * dy2 - dy1 * dx2;
    };

    // Loop through all the points in a polygon, get their cross products, and
    // add any concave points to the set we're building.
    // `sense` should be `1` for outer boundaries and `-1` for holes, since holes
    // will have points facing outward.
    const auto consider_polygon = [&](const auto &poly, const double sense)
    {
        for (size_t i = 0; i < poly.size() + 1; i++)
        {
            const auto zcp = z_cross_product(
                poly[(i + 0) % poly.size()],
                poly[(i + 1) % poly.size()],
                poly[(i + 2) % poly.size()]);
            if (sense * zcp < 0)
            {
                concave_points.insert(poly[(i + 1) % poly.size()]);
            }
        }
    };

    // Loop over the polygons of the MultiPolygon, as well as their holes
    for (const auto &poly : mp)
    {
        // Outer boundary has positive sense
        consider_polygon(poly.outer_boundary(), 1);
        for (const auto &hole : poly.holes())
        {
            // Inner boundaries (holes) have negative (opposite) sense
            consider_polygon(hole, -1);
        }
    }

    return concave_points;
}

// void print_voronoi(VoronoiDiagram vd, const std::string &filename){
//     std::ofstream fout(filename);
//     // fout << "SourceIdx,TargetIdx,UpGovernorIsPoint,DownGovernorIsPoint" << std::endl;
//     for (std::size_t i = 0; i < vd.edges.size(); i++)
//     {
//         fout << md.edges[i].first << ","
//              << md.edges[i].second << ","
//              << md.ups[i]->is_point() << "," // Is up-governor a point?
//              << md.downs[i]->is_point()      // Is down-governor a point?
//              << std::endl;
//     }
// }

/// Print the points which collectively comprise the medial axis
void print_medial_axis_points(const MedialData &md, const std::string &filename)
{
    std::ofstream fout(filename);
    fout << "x,y" << std::endl;
    for (const auto &vh : map_to_ordered_vector(md.vertex_handles))
    {
        fout << vh->point().x() << "," << vh->point().y() << std::endl;
    }
}

/// Prints the edges of the medial diagram
void print_medial_axis_edges(const MedialData &md, const std::string &filename)
{
    std::ofstream fout(filename);
    fout << "SourceIdx,TargetIdx,UpGovernorIsPoint,DownGovernorIsPoint" << std::endl;
    for (std::size_t i = 0; i < md.edges.size(); i++)
    {
        fout << md.edges[i].first << ","
             << md.edges[i].second << ","
             << md.ups[i]->is_point() << "," // Is up-governor a point?
             << md.downs[i]->is_point()      // Is down-governor a point?
             << std::endl;
    }
}

MedialData filter_voronoi_diagram_to_medial_axis(
    const VoronoiDiagram &vd,
    const MultiPolygon &mp)
{
    MedialData ret;

    const auto interior = identify_vertex_handles_inside_mp(vd, mp);
    const auto concave_points = identify_concave_points_of_mp(mp);

    // Returns true if a point is a concave point of the MultiPolygon
    const auto pconcave = [&](const Point_2 &pt)
    {
        return concave_points.count(pt) != 0;
    };

    // The Voronoi diagram is comprised of a number of vertices connected by lines
    // Here, we go through each edge of the Voronoi diagram and determine which
    // vertices it's incident on. We add these vertices to `ret.vertex_handles`
    // so that they will have unique ids.

    // The `up` and `down` refer to the medial axis governors - that which
    // constrains each edge of the Voronoi diagram
    for (
        auto edge_iter = vd.bounded_halfedges_begin();
        edge_iter != vd.bounded_halfedges_end();
        edge_iter++)
    {
        const Halfedge &halfedge = *edge_iter;
        const Vertex_handle &v1p = halfedge.source();
        const Vertex_handle &v2p = halfedge.target();

        // Filter Voronoi diagram to only the part in the interior of the
        // MultiPolygon
        if (interior.count(v1p) == 0 || interior.count(v2p) == 0)
        {
            continue;
        }

        // Drop those edges of the diagram which are not part of the medial axis
        if (pconcave(v1p->point()) || pconcave(v2p->point()))
        {
            continue;
        }

        // Get unique ids for edge vertex handle that's part of the medial axis
        const auto id1 = find_or_add(ret.vertex_handles, v1p);
        const auto id2 = find_or_add(ret.vertex_handles, v2p);
        ret.edges.emplace_back(id1, id2);

        // Keep track of the medial axis governors
        ret.ups.push_back(halfedge.up());
        ret.downs.push_back(halfedge.down());
    }

    return ret;
}






bool is_point_inside_polygon(const Point_2& point, const Polygon_2& polygon) {
    return CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), point, K()) == CGAL::ON_BOUNDED_SIDE;
}






bool edge_connected_to_hole(const Halfedge_handle &edge_iter, const MultiPolygon &mp)
{
    // For each polygon in MultiPolygon
    for (const auto &poly : mp)
    {
        // ignore edges connected to the external polygon or inside holes
        for (auto vertex_iter = poly.outer_boundary().vertices_begin(); vertex_iter != poly.outer_boundary().vertices_end(); vertex_iter++)
        {
            if ((edge_iter->source()->point() == *vertex_iter) || (edge_iter->target()->point() == *vertex_iter) )
            {
                return true;
            }
        }

        // For each hole in the polygon
        for (const auto &hole : poly.holes())
        { // check for each vertex
            for (auto vertex_iter = hole.vertices_begin(); vertex_iter != hole.vertices_end(); vertex_iter++)
            {
                // ignore edges connected to holes
                if ((edge_iter->source()->point() == *vertex_iter) || (edge_iter->target()->point() == *vertex_iter) ||
                is_point_inside_polygon(edge_iter->source()->point(), hole) || is_point_inside_polygon(edge_iter->target()->point(), hole))
                {
                    return true;
                }
            }
        }
    }
    return false;
}
int main()
{
    // if (argc != 2)
    // {
    //     std::cerr << "Syntax: " << argv[0] << " <Shape Boundary WKT>" << std::endl;
    //     return -1;
    // }

    CGAL::set_pretty_mode(std::cout);

    const auto mp = get_wkt_from_file(input_file);
    const auto voronoi = convert_mp_to_voronoi_diagram(mp);

    // draw_voronoi_diagram(voronoi);
    //  CGAL::draw(voronoi);
    //  constrains each edge of the Voronoi diagram
    Point_2 pt1(0, 0);
    Point_2 pt2(0, 0);
    std::cout << (pt1 == pt2) << std::endl;
    std::ofstream fout1("/home/robotics/Desktop/voronoi_points.csv");
    std::ofstream fout2("/home/robotics/Desktop/voronoi_edges.csv");
    fout1 << "x1,y1,x2,y2" << std::endl;
    fout2 << "SourceIdx,TargetIdx" << std::endl;

    for (
        auto edge_iter = voronoi.bounded_halfedges_begin();
        edge_iter != voronoi.bounded_halfedges_end();
        edge_iter++)
    {
        // ignore edges connected to holes
        if (!edge_connected_to_hole(edge_iter, mp))
        {
            std::cout << "Edge: " << edge_iter->source()->point() << edge_iter->target()->point() << std::endl;
            fout1 << edge_iter->source()->point().x() << "," << edge_iter->source()->point().y() << "," << edge_iter->target()->point().x() << "," << edge_iter->target()->point().y() << std::endl;
        }
    }

    // const auto ma_data = filter_voronoi_diagram_to_medial_axis(voronoi, mp);
    // std::cerr << "Found " << ma_data.vertex_handles.size() << " vertices and "
    //           << ma_data.edges.size() << " edges" << std::endl;
    // print_medial_axis_points(ma_data, "/home/robotics/Desktop/voronoi_points.csv");
    // print_medial_axis_edges(ma_data, "/home/robotics/Desktop/voronoi_edges.csv");

    return EXIT_SUCCESS;
}