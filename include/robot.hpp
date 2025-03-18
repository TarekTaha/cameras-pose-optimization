#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <CGAL/Simple_cartesian.h>
#include <array>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;

class Robot {
public:
    Robot(double length, double width, double height, 
          const Point_3& position = Point_3(0, 0, 0));

    // Getters
    double getLength() const { return length_; }
    double getWidth() const { return width_; }
    double getHeight() const { return height_; }
    Point_3 getPosition() const { return position_; }

    // Get the vertices of the robot's bounding box
    std::array<Point_3, 8> getBoundingBoxVertices() const;

private:
    double length_;
    double width_;
    double height_;
    Point_3 position_;
};

#endif // ROBOT_HPP 