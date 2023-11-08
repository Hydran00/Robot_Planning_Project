// #include <CGAL/Simple_cartesian.h>
// #include <CGAL/Surface_mesh.h>
// #include <CGAL/draw_surface_mesh.h>
// #include <fstream>
// typedef CGAL::Simple_cartesian<double>                       Kernel;
// typedef Kernel::Point_3                                      Point;
// typedef CGAL::Surface_mesh<Point>                            Mesh;
// int main(int argc, char* argv[])
// {
//   const std::string filename = (argc>1) ? argv[1] : CGAL::data_file_path("/home/hydran00/Desktop/elephant.off");
//   Mesh sm;
//   if(!CGAL::IO::read_polygon_mesh(filename, sm))
//   {
//     std::cerr << "Invalid input file." << std::endl;
//     return EXIT_FAILURE;
//   }
//   // Internal color property maps are used if they exist and are called "v:color", "e:color" and "f:color".
//   auto vcm = sm.add_property_map<Mesh::Vertex_index, CGAL::IO::Color>("v:color").first;
//   auto ecm = sm.add_property_map<Mesh::Edge_index, CGAL::IO::Color>("e:color").first;
//   auto fcm = sm.add_property_map<Mesh::Face_index>("f:color", CGAL::IO::white() /*default*/).first;
//   for(auto v : vertices(sm))
//   {
//     if(v.idx()%2)
//       put(vcm, v, CGAL::IO::black());
//     else
//       put(vcm, v, CGAL::IO::blue());
//   }
//   for(auto e : edges(sm))
//     put(ecm, e, CGAL::IO::gray());
//   CGAL_USE(fcm);
//   // Draw!
//   CGAL::draw(sm);
//   return EXIT_SUCCESS;
// }

// #include <iostream>
// #include <CGAL/Simple_cartesian.h>
// #include <CGAL/Octree.h>
// // Type Declarations
// typedef CGAL::Simple_cartesian<double> Kernel;
// // typedef Kernel::Point_3 Point;
// typedef Kernel::Point_2 Point;
// typedef std::vector<Point> Point_vector;
// typedef CGAL::Octree<Kernel, Point_vector> Octree;
// int main() {
//   // Here, our point set is a vector
//   Point_vector points;
//   // Add a few points to the vector, most of which are in one region
// //   points.emplace_back(1, 1, 1);
// //   points.emplace_back(2, 1, -11);
// //   points.emplace_back(2, 1, 1);
// //   points.emplace_back(1, -2, 1);
// //   points.emplace_back(1, 1, 1);
// //   points.emplace_back(-1, 1, 1);
// //   points.emplace_back(-1.1, 1, 1);
// //   points.emplace_back(-1.01, 1, 1);
// //   points.emplace_back(-1.001, 1, 1);
// //   points.emplace_back(-1.0001, 1, 1);
// //   points.emplace_back(-1.0001, 1, 1);
//   points.emplace_back(0,0);
//   points.emplace_back(4,0);
//   points.emplace_back(4,4);
//   points.emplace_back(2,2);
//   points.emplace_back(0,4);

//   // Create an octree from the points
//   Octree octree(points);
//   // Build the octree with a small bucket size, so we get a deep node
//   octree.refine(10, 2);
//   // Print out the tree
//   std::cout << "\nUn-graded tree" << std::endl;
//   std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
//   std::cout << octree << std::endl;
//   // Grade the tree to eliminate large jumps in depth
//   octree.grade();
//   // Print out the tree again
//   std::cout << "\nGraded tree" << std::endl;
//   std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
//   std::cout << octree << std::endl;
//   return EXIT_SUCCESS;
// }

#include <iostream>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Quadtree.h>
#include <CGAL/Random.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/draw_polygon_2.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polygon_2<K>                                  Polygon_2;
typedef CGAL::Point_2<K>                                    Point;
// Type Declarations
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef std::vector<Point_2> Point_vector;
typedef CGAL::Quadtree<Kernel, Point_vector> Quadtree;
int main()
{
    CGAL::Random r;
    Point_vector points_2d;
    for (std::size_t i = 0; i < 5; ++i)
        points_2d.emplace_back(r.get_double(-1., 1.),
                               r.get_double(-1., 1.));
    Quadtree quadtree(points_2d);
    quadtree.refine(10, 5);
    // Print out the tree
    std::cout << "\nUn-graded tree" << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << quadtree << std::endl;
    // Grade the tree to eliminate large jumps in depth
    quadtree.grade();
    // Print out the tree again
    std::cout << "\nGraded tree" << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << quadtree << std::endl;

    Polygon_2 p;
    for (int i = 0; i < 5; i++){
        p.push_back(points_2d.at(i));
    }
    CGAL::draw(p);
    return EXIT_SUCCESS;
}
