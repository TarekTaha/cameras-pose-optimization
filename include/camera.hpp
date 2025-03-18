#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <CGAL/Simple_cartesian.h>
#include <array>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;

class Camera {
public:
    Camera(const Point_3& position, 
           double roll,  // Roll angle in radians
           double pitch, // Pitch angle in radians
           double yaw,   // Yaw angle in radians
           double hfov, 
           double vfov, 
           double near_plane, 
           double far_plane);

    // Getters
    Point_3 getPosition() const { return position_; }
    double getRoll() const { return roll_; }
    double getPitch() const { return pitch_; }
    double getYaw() const { return yaw_; }
    double getHFOV() const { return hfov_; }
    double getVFOV() const { return vfov_; }
    double getNearPlane() const { return near_plane_; }
    double getFarPlane() const { return far_plane_; }

    // Calculate frustum vertices in world coordinates
    std::array<Point_3, 8> getFrustumVertices() const;

    // Get camera orientation vectors
    Vector_3 getForwardVector() const {
        auto rot = getRotationMatrix();
        return rot[2]; // X-axis: Forward direction
    }
    
    Vector_3 getRightVector() const {
        auto rot = getRotationMatrix();
        return rot[0]; // Y-axis: Left direction
    }
    
    Vector_3 getUpVector() const {
        auto rot = getRotationMatrix();
        return rot[1]; // Z-axis: Up direction
    }

private:
    Point_3 position_;
    double roll_;   // Rotation around X-axis
    double pitch_;  // Rotation around Y-axis
    double yaw_;    // Rotation around Z-axis
    double hfov_;
    double vfov_;
    double near_plane_;
    double far_plane_;

    // Helper functions
    std::array<Vector_3, 3> getRotationMatrix() const;
};

#endif // CAMERA_HPP 