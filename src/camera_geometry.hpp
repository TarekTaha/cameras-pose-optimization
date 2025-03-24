#ifndef CAMERA_GEOMETRY_HPP
#define CAMERA_GEOMETRY_HPP

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <vector>
#include <utility>
#include <algorithm>
#include "camera.hpp"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;

// Forward declaration of helper functions
double estimate_frustum_volume(const std::vector<Point_3>& vertices);
double tetrahedron_volume(const Point_3& p0, const Point_3& p1, const Point_3& p2, const Point_3& p3);

class CameraGeometry {
public:
    static double compute_frustum_intersection_volume(const Camera& camera1, const Camera& camera2);
    static double compute_total_coverage_volume(const std::vector<Camera>& cameras);
    static std::vector<std::pair<std::vector<size_t>, double>> compute_all_intersection_combinations(const std::vector<Camera>& cameras);
    static void print_intersection_summary(const std::vector<Camera>& cameras);
    
    // Make helper function available to test program
    static double estimate_frustum_volume(const std::vector<Point_3>& vertices) {
        return ::estimate_frustum_volume(vertices);
    }

private:
    template <class HDS>
    struct Frustum_builder;
};

// Template class definition needs to be in header
template <class HDS>
struct CameraGeometry::Frustum_builder : public CGAL::Modifier_base<HDS> {
    std::vector<Point_3> vertices;
    Frustum_builder(const std::vector<Point_3>& v) : vertices(v) {}
    void operator()(HDS& hds) {
        // Create a tetrahedron from the first 4 vertices
        CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
        
        // Use just 4 vertices (making a tetrahedron)
        // We expect at least 4 vertices in the frustum
        size_t num_vertices = std::min(size_t(4), vertices.size());
        if (num_vertices < 4) {
            std::cerr << "Not enough vertices for a tetrahedron, need 4 but got " << num_vertices << std::endl;
            B.rollback();
            return;
        }
        
        B.begin_surface(4, 4, 6);
        
        // Add 4 vertices
        for (size_t i = 0; i < 4; ++i) {
            B.add_vertex(vertices[i]);
        }
        
        // Define the tetrahedron faces with correct orientation
        // Each face defined counter-clockwise when viewed from outside
        
        // Face 0-1-2
        B.begin_facet();
        B.add_vertex_to_facet(0);
        B.add_vertex_to_facet(1);
        B.add_vertex_to_facet(2);
        B.end_facet();
        
        // Face 0-2-3
        B.begin_facet();
        B.add_vertex_to_facet(0);
        B.add_vertex_to_facet(2);
        B.add_vertex_to_facet(3);
        B.end_facet();
        
        // Face 0-3-1
        B.begin_facet();
        B.add_vertex_to_facet(0);
        B.add_vertex_to_facet(3);
        B.add_vertex_to_facet(1);
        B.end_facet();
        
        // Face 1-3-2
        B.begin_facet();
        B.add_vertex_to_facet(1);
        B.add_vertex_to_facet(3);
        B.add_vertex_to_facet(2);
        B.end_facet();
        
        B.end_surface();
    }
};

#endif // CAMERA_GEOMETRY_HPP 