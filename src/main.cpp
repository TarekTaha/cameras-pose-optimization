#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/boost/graph/properties.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/Color.h>

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Surface_mesh<Point> Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;

// Function to draw a mesh with colors
void draw_mesh_with_colors(Mesh& mesh) {
    // Add vertex color property
    auto vertex_color = mesh.add_property_map<vertex_descriptor, CGAL::Color>("v:color", CGAL::Color(0, 0, 0)).first;
    
    // Add face color property
    auto face_color = mesh.add_property_map<face_descriptor, CGAL::Color>("f:color", CGAL::Color(0, 0, 0)).first;

    // Set vertex colors to red
    for(vertex_descriptor vd : mesh.vertices()) {
        put(vertex_color, vd, CGAL::Color(255, 0, 0));
    }

    // Set face colors to blue
    for(face_descriptor fd : mesh.faces()) {
        put(face_color, fd, CGAL::Color(0, 0, 255));
    }

    // Draw the mesh with colors
    CGAL::draw(mesh);
}

// Function to add a cylinder to a mesh
void add_cylinder_to_mesh(Mesh& mesh, const Point& start, const Point& end, double radius, int segments = 32) {
    // Calculate cylinder direction and length
    Vector direction = end - start;
    double length = std::sqrt(direction.squared_length());
    direction = direction / length;

    // Create orthogonal vectors
    Vector v1, v2;
    if (std::abs(direction.x()) > std::abs(direction.y())) {
        v1 = Vector(-direction.z(), 0, direction.x());
    } else {
        v1 = Vector(0, -direction.z(), direction.y());
    }
    v1 = v1 / std::sqrt(v1.squared_length());
    v2 = CGAL::cross_product(direction, v1);

    // Create vertices for both caps
    std::vector<vertex_descriptor> bottom_vertices;
    std::vector<vertex_descriptor> top_vertices;

    for (int i = 0; i < segments; ++i) {
        double angle = 2.0 * M_PI * i / segments;
        Vector offset = v1 * std::cos(angle) * radius + v2 * std::sin(angle) * radius;
        
        Point bottom_point = start + offset;
        Point top_point = end + offset;
        
        bottom_vertices.push_back(mesh.add_vertex(bottom_point));
        top_vertices.push_back(mesh.add_vertex(top_point));
    }

    // Add faces for the cylinder walls
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        mesh.add_face(bottom_vertices[i], bottom_vertices[next], top_vertices[next], top_vertices[i]);
    }

    // Add faces for bottom cap
    vertex_descriptor bottom_center = mesh.add_vertex(start);
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        mesh.add_face(bottom_center, bottom_vertices[i], bottom_vertices[next]);
    }

    // Add faces for top cap
    vertex_descriptor top_center = mesh.add_vertex(end);
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        mesh.add_face(top_center, top_vertices[next], top_vertices[i]);
    }
}

// Function to add a vector visualization to a mesh
void add_vector_to_mesh(Mesh& mesh, const Point& start, const Vector& direction, double length, double thickness) {
    Point end = start + direction * length;
    add_cylinder_to_mesh(mesh, start, end, thickness);
}

// Function to add camera vectors to a mesh
void add_camera_vectors_to_mesh(Mesh& mesh, const Point& position, const Vector& direction, 
                              double length, double thickness) {
    // Add main direction vector
    add_vector_to_mesh(mesh, position, direction, length, thickness);
}

// Function to add a camera frustum to a mesh
void add_camera_frustum_to_mesh(Mesh& mesh, const Point& position, const Vector& direction,
                               double near_width, double near_height, double far_width, double far_height,
                               double near_distance, double far_distance) {
    // Normalize direction vector
    Vector dir = direction / std::sqrt(direction.squared_length());
    
    // Create orthogonal vectors for camera orientation
    Vector up, right;
    if (std::abs(dir.x()) > std::abs(dir.y())) {
        up = Vector(-dir.z(), 0, dir.x());
    } else {
        up = Vector(0, -dir.z(), dir.y());
    }
    up = up / std::sqrt(up.squared_length());
    right = CGAL::cross_product(dir, up);

    // Calculate corners of near plane
    Point near_center = position + dir * near_distance;
    Point near_top_left = near_center + up * (near_height/2) - right * (near_width/2);
    Point near_top_right = near_center + up * (near_height/2) + right * (near_width/2);
    Point near_bottom_left = near_center - up * (near_height/2) - right * (near_width/2);
    Point near_bottom_right = near_center - up * (near_height/2) + right * (near_width/2);

    // Calculate corners of far plane
    Point far_center = position + dir * far_distance;
    Point far_top_left = far_center + up * (far_height/2) - right * (far_width/2);
    Point far_top_right = far_center + up * (far_height/2) + right * (far_width/2);
    Point far_bottom_left = far_center - up * (far_height/2) - right * (far_width/2);
    Point far_bottom_right = far_center - up * (far_height/2) + right * (far_width/2);

    // Add vertices
    vertex_descriptor v_near_top_left = mesh.add_vertex(near_top_left);
    vertex_descriptor v_near_top_right = mesh.add_vertex(near_top_right);
    vertex_descriptor v_near_bottom_left = mesh.add_vertex(near_bottom_left);
    vertex_descriptor v_near_bottom_right = mesh.add_vertex(near_bottom_right);
    vertex_descriptor v_far_top_left = mesh.add_vertex(far_top_left);
    vertex_descriptor v_far_top_right = mesh.add_vertex(far_top_right);
    vertex_descriptor v_far_bottom_left = mesh.add_vertex(far_bottom_left);
    vertex_descriptor v_far_bottom_right = mesh.add_vertex(far_bottom_right);

    // Add faces
    // Near plane
    mesh.add_face(v_near_top_left, v_near_bottom_left, v_near_bottom_right, v_near_top_right);
    // Far plane
    mesh.add_face(v_far_top_right, v_far_bottom_right, v_far_bottom_left, v_far_top_left);
    // Side planes
    mesh.add_face(v_near_top_left, v_far_top_left, v_far_bottom_left, v_near_bottom_left);
    mesh.add_face(v_near_bottom_right, v_far_bottom_right, v_far_top_right, v_near_top_right);
    mesh.add_face(v_near_top_left, v_near_top_right, v_far_top_right, v_far_top_left);
    mesh.add_face(v_near_bottom_left, v_far_bottom_left, v_far_bottom_right, v_near_bottom_right);
}

// Function to compute intersection volume between two meshes
double compute_intersection_volume(const Mesh& mesh1, const Mesh& mesh2) {
    Mesh intersection;
    CGAL::Polygon_mesh_processing::corefine_and_compute_intersection(mesh1, mesh2, intersection);
    return CGAL::Polygon_mesh_processing::volume(intersection);
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file.yaml>" << std::endl;
        return 1;
    }

    // Load configuration from YAML file
    YAML::Node config = YAML::LoadFile(argv[1]);

    // Create meshes for each camera's frustum
    std::vector<Mesh> camera_frustums;
    
    // Read camera configurations
    for (const auto& camera : config["cameras"]) {
        // Create a new mesh for this camera's frustum
        Mesh frustum;
        
        // Get camera parameters
        double x = camera["position"]["x"].as<double>();
        double y = camera["position"]["y"].as<double>();
        double z = camera["position"]["z"].as<double>();
        
        double dx = camera["direction"]["x"].as<double>();
        double dy = camera["direction"]["y"].as<double>();
        double dz = camera["direction"]["z"].as<double>();
        
        double near_width = camera["near_width"].as<double>();
        double near_height = camera["near_height"].as<double>();
        double far_width = camera["far_width"].as<double>();
        double far_height = camera["far_height"].as<double>();
        double near_distance = camera["near_distance"].as<double>();
        double far_distance = camera["far_distance"].as<double>();

        // Add frustum to mesh
        add_camera_frustum_to_mesh(
            frustum,
            Point(x, y, z),
            Vector(dx, dy, dz),
            near_width, near_height,
            far_width, far_height,
            near_distance, far_distance
        );

        camera_frustums.push_back(frustum);
    }

    // Compute intersection volumes between all pairs of cameras
    double largest_overlap = 0.0;
    int overlapping_regions = 0;
    
    std::cout << "Computing intersection volumes for camera combinations..." << std::endl;
    
    for (size_t i = 0; i < camera_frustums.size(); ++i) {
        for (size_t j = i + 1; j < camera_frustums.size(); ++j) {
            double volume = compute_intersection_volume(camera_frustums[i], camera_frustums[j]);
            
            if (volume > 0) {
                std::cout << "Overlap between Camera " << (i+1) << " and Camera " << (j+1) 
                         << ": " << volume << " cubic meters" << std::endl;
                overlapping_regions++;
                largest_overlap = std::max(largest_overlap, volume);
            }
        }
    }
    
    std::cout << "\nSummary:" << std::endl;
    std::cout << "Largest overlap: " << largest_overlap << " cubic meters" << std::endl;
    std::cout << "Number of overlapping regions: " << overlapping_regions << " (for 2 cameras)" << std::endl;

    // Draw the first camera frustum with colors
    if (!camera_frustums.empty()) {
        draw_mesh_with_colors(camera_frustums[0]);
    }

    return 0;
}