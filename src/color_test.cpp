#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/IO/Color.h>
#include <iostream>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef Kernel::Point_3 Point_3;

// Helper function to draw mesh with colors
void draw_mesh_with_colors(const Mesh& mesh, const char* title) {
    // Create a copy of the mesh to ensure we don't modify the original
    Mesh display_mesh = mesh;
    
    // Make sure color properties are available
    if (!display_mesh.property_map<Mesh::Vertex_index, CGAL::Color>("v:color").second) {
        display_mesh.add_property_map<Mesh::Vertex_index, CGAL::Color>("v:color");
    }
    if (!display_mesh.property_map<Mesh::Face_index, CGAL::Color>("f:color").second) {
        display_mesh.add_property_map<Mesh::Face_index, CGAL::Color>("f:color");
    }
    
    // Draw with mono color explicitly disabled
    CGAL::draw(display_mesh, title, false);
}

int main(int argc, char* argv[]) {
    // Create a simple colored mesh
    Mesh mesh;
    
    // Add vertex color property map
    auto vertex_color = mesh.add_property_map<Mesh::Vertex_index, CGAL::Color>("v:color").first;
    
    // Add face color property map
    auto face_color = mesh.add_property_map<Mesh::Face_index, CGAL::Color>("f:color").first;
    
    // Create a simple cube with more distinct colors
    auto v0 = mesh.add_vertex(Point_3(-1, -1, -1));
    auto v1 = mesh.add_vertex(Point_3(1, -1, -1));
    auto v2 = mesh.add_vertex(Point_3(1, 1, -1));
    auto v3 = mesh.add_vertex(Point_3(-1, 1, -1));
    auto v4 = mesh.add_vertex(Point_3(-1, -1, 1));
    auto v5 = mesh.add_vertex(Point_3(1, -1, 1));
    auto v6 = mesh.add_vertex(Point_3(1, 1, 1));
    auto v7 = mesh.add_vertex(Point_3(-1, 1, 1));
    
    // Set different colors for vertices with full intensity
    vertex_color[v0] = CGAL::Color(255, 0, 0);     // Pure Red
    vertex_color[v1] = CGAL::Color(0, 255, 0);     // Pure Green
    vertex_color[v2] = CGAL::Color(0, 0, 255);     // Pure Blue
    vertex_color[v3] = CGAL::Color(255, 255, 0);   // Pure Yellow
    vertex_color[v4] = CGAL::Color(255, 0, 255);   // Pure Magenta
    vertex_color[v5] = CGAL::Color(0, 255, 255);   // Pure Cyan
    vertex_color[v6] = CGAL::Color(255, 128, 0);   // Pure Orange
    vertex_color[v7] = CGAL::Color(128, 0, 255);   // Pure Purple
    
    // Create faces with different colors
    auto f1 = mesh.add_face(v0, v1, v2, v3); // Bottom
    auto f2 = mesh.add_face(v4, v7, v6, v5); // Top
    auto f3 = mesh.add_face(v0, v4, v5, v1); // Front
    auto f4 = mesh.add_face(v1, v5, v6, v2); // Right
    auto f5 = mesh.add_face(v2, v6, v7, v3); // Back
    auto f6 = mesh.add_face(v3, v7, v4, v0); // Left
    
    // Set face colors with full intensity
    face_color[f1] = CGAL::Color(255, 0, 0);     // Pure Red
    face_color[f2] = CGAL::Color(0, 255, 0);     // Pure Green
    face_color[f3] = CGAL::Color(0, 0, 255);     // Pure Blue
    face_color[f4] = CGAL::Color(255, 255, 0);   // Pure Yellow
    face_color[f5] = CGAL::Color(255, 0, 255);   // Pure Magenta
    face_color[f6] = CGAL::Color(0, 255, 255);   // Pure Cyan
    
    std::cout << "Testing CGAL color visualization..." << std::endl;
    std::cout << "- Created cube with " << mesh.number_of_vertices() << " vertices and "
              << mesh.number_of_faces() << " faces" << std::endl;
    std::cout << "- Vertex and face colors set to pure RGB values" << std::endl;
    
    // Draw the mesh with colors directly
    std::cout << "Drawing with mono_color=false..." << std::endl;
    CGAL::draw(mesh, "Color Test - Mono Color Disabled", false);
    
    return 0;
} 