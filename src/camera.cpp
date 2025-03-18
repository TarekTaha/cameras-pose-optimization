#include "camera.hpp"
#include <cmath>

Camera::Camera(const Point_3& position,
               double roll,
               double pitch,
               double yaw,
               double hfov,
               double vfov,
               double near_plane,
               double far_plane)
    : position_(position)
    , roll_(roll)
    , pitch_(pitch)
    , yaw_(yaw)
    , hfov_(hfov)
    , vfov_(vfov)
    , near_plane_(near_plane)
    , far_plane_(far_plane)
{}

std::array<Vector_3, 3> Camera::getRotationMatrix() const {
    // Convert Euler angles to rotation matrix
    // Right-hand rule camera convention:
    // - X-axis: Forward (viewing direction)
    // - Y-axis: Left
    // - Z-axis: Up
    
    double cr = std::cos(roll_);
    double sr = std::sin(roll_);
    double cp = std::cos(pitch_);
    double sp = std::sin(pitch_);
    double cy = std::cos(yaw_);
    double sy = std::sin(yaw_);

    std::array<Vector_3, 3> rot;
    // Forward vector (X-axis)
    rot[2] = Vector_3(cp * cy,
                      cp * sy,
                      -sp);
    
    // Left vector (Y-axis)
    rot[0] = Vector_3(sy,
                      -cy,
                      0);
    
    // Up vector (Z-axis)
    rot[1] = Vector_3(sp * cy,
                      sp * sy,
                      cp);
    
    return rot;
}

std::array<Point_3, 8> Camera::getFrustumVertices() const {
    std::array<Point_3, 8> vertices;
    
    // Calculate frustum dimensions
    double near_height = 2 * near_plane_ * std::tan(vfov_ * M_PI / 360.0);
    double near_width = 2 * near_plane_ * std::tan(hfov_ * M_PI / 360.0);
    double far_height = 2 * far_plane_ * std::tan(vfov_ * M_PI / 360.0);
    double far_width = 2 * far_plane_ * std::tan(hfov_ * M_PI / 360.0);
    
    // Get rotation matrix
    auto rot = getRotationMatrix();
    Vector_3 forward = rot[2];  // Points along X
    Vector_3 right = rot[0];    // Points along Y for hfov
    Vector_3 up = rot[1];       // Points along Z for vfov
    
    // Near plane vertices (counter-clockwise from bottom-left)
    Vector_3 near_center = forward * near_plane_;
    vertices[0] = position_ + near_center + (-right * near_width/2) + (-up * near_height/2);
    vertices[1] = position_ + near_center + (right * near_width/2) + (-up * near_height/2);
    vertices[2] = position_ + near_center + (right * near_width/2) + (up * near_height/2);
    vertices[3] = position_ + near_center + (-right * near_width/2) + (up * near_height/2);
    
    // Far plane vertices (counter-clockwise from bottom-left)
    Vector_3 far_center = forward * far_plane_;
    vertices[4] = position_ + far_center + (-right * far_width/2) + (-up * far_height/2);
    vertices[5] = position_ + far_center + (right * far_width/2) + (-up * far_height/2);
    vertices[6] = position_ + far_center + (right * far_width/2) + (up * far_height/2);
    vertices[7] = position_ + far_center + (-right * far_width/2) + (up * far_height/2);
    
    return vertices;
} 