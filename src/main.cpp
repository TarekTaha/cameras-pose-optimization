#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "config_reader.hpp"
#include <CGAL/Qt/Basic_viewer_qt.h>
#include <CGAL/draw_polyhedron.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/convex_decomposition_3.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include "camera_geometry.hpp"

typedef Kernel::Vector_3 Vector_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

// Helper function to draw mesh with colors
void draw_mesh_with_colors(const Surface_mesh& mesh, const char* title) {
    // Create a copy of the mesh to ensure we don't modify the original
    Surface_mesh display_mesh = mesh;
    
    // Make sure color properties are available
    if (!display_mesh.property_map<Surface_mesh::Vertex_index, CGAL::Color>("v:color").second) {
        display_mesh.add_property_map<Surface_mesh::Vertex_index, CGAL::Color>("v:color");
    }
    if (!display_mesh.property_map<Surface_mesh::Face_index, CGAL::Color>("f:color").second) {
        display_mesh.add_property_map<Surface_mesh::Face_index, CGAL::Color>("f:color");
    }
    
    // Draw with mono color explicitly disabled
    CGAL::draw(display_mesh, title, false);
}

void add_cylinder_to_mesh(const Point_3& start, const Point_3& end, double radius, const CGAL::Color& color, Surface_mesh& mesh) {
    auto vertex_color = mesh.property_map<Surface_mesh::Vertex_index, CGAL::Color>("v:color").first;
    auto face_color = mesh.property_map<Surface_mesh::Face_index, CGAL::Color>("f:color").first;
    
    // Create a cylinder approximation with 8 sides
    const int num_sides = 8;
    Vector_3 direction(end.x() - start.x(), end.y() - start.y(), end.z() - start.z());
    double length = std::sqrt(direction.squared_length());
    
    // Create basis vectors
    Vector_3 up(0, 0, 1);
    if (std::abs(direction * up) > 0.9 * length) {
        up = Vector_3(0, 1, 0);
    }
    Vector_3 right = CGAL::cross_product(direction, up);
    up = CGAL::cross_product(right, direction);
    
    // Normalize vectors
    right = right / std::sqrt(right.squared_length()) * radius;
    up = up / std::sqrt(up.squared_length()) * radius;
    
    // Create vertices around the start and end points
    std::vector<Surface_mesh::Vertex_index> start_vertices, end_vertices;
    for (int i = 0; i < num_sides; ++i) {
        double angle = 2.0 * M_PI * i / num_sides;
        Vector_3 offset = right * std::cos(angle) + up * std::sin(angle);
        
        auto v_start = mesh.add_vertex(start + offset);
        auto v_end = mesh.add_vertex(end + offset);
        
        vertex_color[v_start] = color;
        vertex_color[v_end] = color;
        
        start_vertices.push_back(v_start);
        end_vertices.push_back(v_end);
    }
    
    // Create faces for the cylinder sides
    for (int i = 0; i < num_sides; ++i) {
        int next = (i + 1) % num_sides;
        auto f1 = mesh.add_face(start_vertices[i], start_vertices[next], end_vertices[next]);
        auto f2 = mesh.add_face(start_vertices[i], end_vertices[next], end_vertices[i]);
        
        face_color[f1] = color;
        face_color[f2] = color;
    }
}

void add_vector_to_mesh(const Point_3& start, const Vector_3& direction, double length, double thickness, const CGAL::Color& color, Surface_mesh& mesh) {
    Point_3 end = start + direction * length;
    add_cylinder_to_mesh(start, end, thickness, color, mesh);
    
    // Add arrow head (cone)
    double head_length = length * 0.2;  // 20% of vector length
    double head_radius = thickness * 2;  // Twice as thick as the shaft
    Point_3 head_start = end - direction * head_length;
    
    // Create cone vertices
    auto vertex_color = mesh.property_map<Surface_mesh::Vertex_index, CGAL::Color>("v:color").first;
    auto face_color = mesh.property_map<Surface_mesh::Face_index, CGAL::Color>("f:color").first;
    
    auto tip = mesh.add_vertex(end);
    vertex_color[tip] = color;
    
    // Create basis vectors for the cone base
    Vector_3 up(0, 0, 1);
    if (std::abs(direction * up) > 0.9) {
        up = Vector_3(0, 1, 0);
    }
    Vector_3 right = CGAL::cross_product(direction, up);
    up = CGAL::cross_product(right, direction);
    
    // Normalize vectors
    right = right / std::sqrt(right.squared_length()) * head_radius;
    up = up / std::sqrt(up.squared_length()) * head_radius;
    
    // Create cone base vertices
    const int num_sides = 8;
    std::vector<Surface_mesh::Vertex_index> base_vertices;
    for (int i = 0; i < num_sides; ++i) {
        double angle = 2.0 * M_PI * i / num_sides;
        Vector_3 offset = right * std::cos(angle) + up * std::sin(angle);
        auto v = mesh.add_vertex(head_start + offset);
        vertex_color[v] = color;
        base_vertices.push_back(v);
    }
    
    // Create cone faces
    for (int i = 0; i < num_sides; ++i) {
        int next = (i + 1) % num_sides;
        auto f = mesh.add_face(base_vertices[i], base_vertices[next], tip);
        face_color[f] = color;
    }
}

void add_camera_vectors_to_mesh(const Camera& camera, Surface_mesh& mesh) {
    Point_3 position = camera.getPosition();
    Vector_3 forward = camera.getForwardVector();    // X-axis (forward)
    Vector_3 left = camera.getRightVector();         // Y-axis (left)
    Vector_3 up = camera.getUpVector();             // Z-axis (up)
    
    double vector_length = 0.5;  // Length of the vectors
    double thickness = 0.02;     // Thickness of the vectors
    
    // Draw position vector from origin to camera position in gray
    Point_3 origin(0, 0, 0);
    Vector_3 pos_vector(position.x(), position.y(), position.z());
    add_vector_to_mesh(origin, pos_vector / std::sqrt(pos_vector.squared_length()), 
                      std::sqrt(pos_vector.squared_length()), 
                      thickness, CGAL::Color(128, 128, 128), mesh);
    
    // Add forward vector (X-axis, red)
    add_vector_to_mesh(position, forward, vector_length, thickness, CGAL::Color(255, 0, 0), mesh);
    
    // Add left vector (Y-axis, green) - negate the vector to point left instead of right
    add_vector_to_mesh(position, -left, vector_length, thickness, CGAL::Color(0, 255, 0), mesh);
    
    // Add up vector (Z-axis, blue)
    add_vector_to_mesh(position, up, vector_length, thickness, CGAL::Color(0, 0, 255), mesh);
}

// Comment out the arc drawing function
/*
void add_arc_to_mesh(const Point_3& center, const Vector_3& normal, const Vector_3& start_dir, 
                    double radius, double angle, const CGAL::Color& color, Surface_mesh& mesh) {
    auto vertex_color = mesh.property_map<Surface_mesh::Vertex_index, CGAL::Color>("v:color").first;
    
    // Create basis vectors for the arc
    Vector_3 right = start_dir / std::sqrt(start_dir.squared_length()) * radius;
    Vector_3 up = CGAL::cross_product(normal, right);
    up = up / std::sqrt(up.squared_length()) * radius;
    
    // Create arc points
    const int num_segments = 32;
    std::vector<Surface_mesh::Vertex_index> arc_vertices;
    
    for (int i = 0; i <= num_segments; ++i) {
        double t = angle * i / num_segments;
        Vector_3 offset = right * std::cos(t) + up * std::sin(t);
        auto v = mesh.add_vertex(center + offset);
        vertex_color[v] = color;
        arc_vertices.push_back(v);
    }
    
    // Create cylinder along the arc
    double thickness = 0.01;
    for (size_t i = 0; i < arc_vertices.size() - 1; ++i) {
        Point_3 start = mesh.point(arc_vertices[i]);
        Point_3 end = mesh.point(arc_vertices[i + 1]);
        add_cylinder_to_mesh(start, end, thickness, color, mesh);
    }
    
    // Add arrow head at the end
    Vector_3 end_dir = right * std::cos(angle) + up * std::sin(angle);
    Vector_3 tangent = -right * std::sin(angle) + up * std::cos(angle);
    tangent = tangent / std::sqrt(tangent.squared_length());
    
    Point_3 arrow_base = center + end_dir;
    Point_3 arrow_tip = arrow_base + tangent * (radius * 0.2);
    add_cylinder_to_mesh(arrow_base, arrow_tip, thickness * 2, color, mesh);
}
*/

// Comment out the position and orientation function
/*
void add_position_and_orientation_to_mesh(const Camera& camera, Surface_mesh& mesh) {
    Point_3 position = camera.getPosition();
    
    // Draw rotation arcs for roll, pitch, and yaw
    double arc_radius = 0.3;
    double thickness = 0.01;
    
    // Yaw (rotation around Z-axis) in yellow
    if (std::abs(camera.getYaw()) > 1e-6) {
        Vector_3 z_axis(0, 0, 1);
        Vector_3 start_dir(1, 0, 0);
        add_arc_to_mesh(position, z_axis, start_dir, arc_radius, camera.getYaw(), CGAL::Color(255, 255, 0), mesh);
    }
    
    // Pitch (rotation around Y-axis) in cyan
    if (std::abs(camera.getPitch()) > 1e-6) {
        Vector_3 y_axis(0, 1, 0);
        Vector_3 start_dir(1, 0, 0);
        add_arc_to_mesh(position, y_axis, start_dir, arc_radius * 0.8, camera.getPitch(), CGAL::Color(0, 255, 255), mesh);
    }
    
    // Roll (rotation around X-axis) in magenta
    if (std::abs(camera.getRoll()) > 1e-6) {
        Vector_3 x_axis(1, 0, 0);
        Vector_3 start_dir(0, 1, 0);
        add_arc_to_mesh(position, x_axis, start_dir, arc_radius * 0.6, camera.getRoll(), CGAL::Color(255, 0, 255), mesh);
    }
}
*/

void add_edge_to_mesh(Surface_mesh& mesh, Surface_mesh::Vertex_index v1, Surface_mesh::Vertex_index v2, 
                     double thickness, const CGAL::Color& color) {
    Point_3 p1 = mesh.point(v1);
    Point_3 p2 = mesh.point(v2);
    add_cylinder_to_mesh(p1, p2, thickness, color, mesh);
}

void add_frustum_to_mesh(const Camera& camera, Surface_mesh& mesh) {
    auto vertices = camera.getFrustumVertices();
    
    // Add vertices to mesh
    std::vector<Surface_mesh::Vertex_index> v_indices;
    auto vertex_color = mesh.property_map<Surface_mesh::Vertex_index, CGAL::Color>("v:color").first;
    
    // Light transparent blue color
    CGAL::Color frustum_color(100, 100, 255);  // RGB: light blue
    double edge_thickness = 0.02;  // Thickness of wireframe edges
    
    for (const auto& point : vertices) {
        auto v = mesh.add_vertex(point);
        vertex_color[v] = frustum_color;
        v_indices.push_back(v);
    }
    
    // Add edges for near plane (square)
    add_edge_to_mesh(mesh, v_indices[0], v_indices[1], edge_thickness, frustum_color);
    add_edge_to_mesh(mesh, v_indices[1], v_indices[2], edge_thickness, frustum_color);
    add_edge_to_mesh(mesh, v_indices[2], v_indices[3], edge_thickness, frustum_color);
    add_edge_to_mesh(mesh, v_indices[3], v_indices[0], edge_thickness, frustum_color);
    
    // Add edges for far plane (square)
    add_edge_to_mesh(mesh, v_indices[4], v_indices[5], edge_thickness, frustum_color);
    add_edge_to_mesh(mesh, v_indices[5], v_indices[6], edge_thickness, frustum_color);
    add_edge_to_mesh(mesh, v_indices[6], v_indices[7], edge_thickness, frustum_color);
    add_edge_to_mesh(mesh, v_indices[7], v_indices[4], edge_thickness, frustum_color);
    
    // Add edges connecting near and far planes (forming the frustum)
    add_edge_to_mesh(mesh, v_indices[0], v_indices[4], edge_thickness, frustum_color);
    add_edge_to_mesh(mesh, v_indices[1], v_indices[5], edge_thickness, frustum_color);
    add_edge_to_mesh(mesh, v_indices[2], v_indices[6], edge_thickness, frustum_color);
    add_edge_to_mesh(mesh, v_indices[3], v_indices[7], edge_thickness, frustum_color);
    
    // Add camera coordinate system and position vectors
    add_camera_vectors_to_mesh(camera, mesh);
}

void add_robot_to_mesh(const Robot& robot, Surface_mesh& mesh) {
    auto vertices = robot.getBoundingBoxVertices();
    
    // Add vertices to mesh
    std::vector<Surface_mesh::Vertex_index> v_indices;
    auto vertex_color = mesh.property_map<Surface_mesh::Vertex_index, CGAL::Color>("v:color").first;
    auto face_color = mesh.property_map<Surface_mesh::Face_index, CGAL::Color>("f:color").first;
    
    // Solid orange color
    CGAL::Color robot_color(255, 140, 0);  // RGB: orange
    
    for (const auto& point : vertices) {
        auto v = mesh.add_vertex(point);
        vertex_color[v] = robot_color;
        v_indices.push_back(v);
    }
    
    // Add faces for bottom
    auto f1 = mesh.add_face(v_indices[0], v_indices[2], v_indices[1]);
    auto f2 = mesh.add_face(v_indices[0], v_indices[3], v_indices[2]);
    face_color[f1] = robot_color;
    face_color[f2] = robot_color;
    
    // Add faces for top
    auto f3 = mesh.add_face(v_indices[4], v_indices[5], v_indices[6]);
    auto f4 = mesh.add_face(v_indices[4], v_indices[6], v_indices[7]);
    face_color[f3] = robot_color;
    face_color[f4] = robot_color;
    
    // Add faces for sides
    auto f5 = mesh.add_face(v_indices[0], v_indices[1], v_indices[5]);
    auto f6 = mesh.add_face(v_indices[0], v_indices[5], v_indices[4]);
    auto f7 = mesh.add_face(v_indices[1], v_indices[2], v_indices[6]);
    auto f8 = mesh.add_face(v_indices[1], v_indices[6], v_indices[5]);
    auto f9 = mesh.add_face(v_indices[2], v_indices[3], v_indices[7]);
    auto f10 = mesh.add_face(v_indices[2], v_indices[7], v_indices[6]);
    auto f11 = mesh.add_face(v_indices[3], v_indices[0], v_indices[4]);
    auto f12 = mesh.add_face(v_indices[3], v_indices[4], v_indices[7]);
    
    face_color[f5] = robot_color;
    face_color[f6] = robot_color;
    face_color[f7] = robot_color;
    face_color[f8] = robot_color;
    face_color[f9] = robot_color;
    face_color[f10] = robot_color;
    face_color[f11] = robot_color;
    face_color[f12] = robot_color;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return 1;
    }
    
    try {
        // Read configuration
        ConfigReader config_reader(argv[1]);
        config_reader.parse();
        
        // Get cameras
        auto cameras = config_reader.getCameras();
        
        // Calculate intersections for all combinations
        if (cameras.size() >= 2) {
            CameraGeometry::print_intersection_summary(cameras);
        }
        
        // Create mesh for visualization
        Surface_mesh mesh;
        
        // Add vertex color property map
        mesh.add_property_map<Surface_mesh::Vertex_index, CGAL::Color>("v:color");
        
        // Add face color property map
        mesh.add_property_map<Surface_mesh::Face_index, CGAL::Color>("f:color");
        
        // Add robot first (so it's rendered behind transparent elements)
        add_robot_to_mesh(config_reader.getRobot(), mesh);
        
        // Add camera frustums
        for (const auto& camera : config_reader.getCameras()) {
            add_frustum_to_mesh(camera, mesh);
        }
        
        // Display the mesh with mono color explicitly set to false
        std::cout << "Opening visualization window with colored elements..." << std::endl;
        
        // Draw the mesh with colors directly
        CGAL::draw(mesh, "Camera Poses Visualization", false);
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 