#include "robot.hpp"
#include <array>

Robot::Robot(double length, double width, double height, const Point_3& position)
    : length_(length)
    , width_(width)
    , height_(height)
    , position_(position)
{}

std::array<Point_3, 8> Robot::getBoundingBoxVertices() const {
    std::array<Point_3, 8> vertices;
    
    // Calculate half dimensions
    double half_length = length_ / 2.0;
    double half_width = width_ / 2.0;
    
    // Bottom vertices (counter-clockwise from origin)
    vertices[0] = position_ + Kernel::Vector_3(-half_length, -half_width, 0);
    vertices[1] = position_ + Kernel::Vector_3(half_length, -half_width, 0);
    vertices[2] = position_ + Kernel::Vector_3(half_length, half_width, 0);
    vertices[3] = position_ + Kernel::Vector_3(-half_length, half_width, 0);
    
    // Top vertices (counter-clockwise from origin)
    vertices[4] = position_ + Kernel::Vector_3(-half_length, -half_width, height_);
    vertices[5] = position_ + Kernel::Vector_3(half_length, -half_width, height_);
    vertices[6] = position_ + Kernel::Vector_3(half_length, half_width, height_);
    vertices[7] = position_ + Kernel::Vector_3(-half_length, half_width, height_);
    
    return vertices;
} 