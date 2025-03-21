#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/IO/Color.h>
#include <iostream>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;

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

int main() {
    // Create a simple cube mesh
    Mesh mesh;
    
    // Add vertices
    vertex_descriptor v0 = mesh.add_vertex(Point(0, 0, 0));
    vertex_descriptor v1 = mesh.add_vertex(Point(1, 0, 0));
    vertex_descriptor v2 = mesh.add_vertex(Point(1, 1, 0));
    vertex_descriptor v3 = mesh.add_vertex(Point(0, 1, 0));
    vertex_descriptor v4 = mesh.add_vertex(Point(0, 0, 1));
    vertex_descriptor v5 = mesh.add_vertex(Point(1, 0, 1));
    vertex_descriptor v6 = mesh.add_vertex(Point(1, 1, 1));
    vertex_descriptor v7 = mesh.add_vertex(Point(0, 1, 1));

    // Add faces
    mesh.add_face(v0, v1, v2, v3); // bottom
    mesh.add_face(v4, v7, v6, v5); // top
    mesh.add_face(v0, v4, v5, v1); // front
    mesh.add_face(v1, v5, v6, v2); // right
    mesh.add_face(v2, v6, v7, v3); // back
    mesh.add_face(v3, v7, v4, v0); // left

    std::cout << "Created a cube with " << mesh.number_of_vertices() << " vertices and "
              << mesh.number_of_faces() << " faces." << std::endl;
    std::cout << "Setting vertex colors to pure red (255, 0, 0) and face colors to pure blue (0, 0, 255)" << std::endl;

    // Draw the mesh with colors
    draw_mesh_with_colors(mesh);

    return 0;
}