#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "camera.hpp"
#include "camera_geometry.hpp"

// Function to calculate exact volume of a frustum
double calculate_exact_frustum_volume(double near_width, double near_height, 
                                     double far_width, double far_height,
                                     double depth) {
    // Volume of a frustum = (1/3) * depth * (A1 + A2 + sqrt(A1*A2))
    // where A1 is the area of the near face and A2 is the area of the far face
    double near_area = near_width * near_height;
    double far_area = far_width * far_height;
    return (depth / 3.0) * (near_area + far_area + std::sqrt(near_area * far_area));
}

// Function to create a camera with a specific configuration
Camera create_test_camera(const Point_3& position, double yaw, double pitch, double roll,
                         double hfov, double vfov, double near_plane, double far_plane) {
    return Camera(position, roll, pitch, yaw, hfov, vfov, near_plane, far_plane);
}

// Test a single camera's volume calculation
void test_single_camera_volume() {
    std::cout << "\n=== Testing Single Camera Volume ===" << std::endl;
    
    // Create a camera with known parameters
    Point_3 position(0, 0, 0);
    double roll = 0.0;        // Radians
    double pitch = 0.0;       // Radians
    double yaw = 0.0;         // Radians
    double hfov = 60.0;       // Degrees
    double vfov = 45.0;       // Degrees
    double near_plane = 1.0;  // Meters
    double far_plane = 10.0;  // Meters
    
    Camera camera = create_test_camera(position, yaw, pitch, roll, 
                                      hfov, vfov, near_plane, far_plane);
    
    // Calculate ground truth volume
    double near_height = 2 * near_plane * std::tan(vfov * M_PI / 360.0);
    double near_width = 2 * near_plane * std::tan(hfov * M_PI / 360.0);
    double far_height = 2 * far_plane * std::tan(vfov * M_PI / 360.0);
    double far_width = 2 * far_plane * std::tan(hfov * M_PI / 360.0);
    double depth = far_plane - near_plane;
    
    double exact_volume = calculate_exact_frustum_volume(near_width, near_height, 
                                                         far_width, far_height, depth);
    
    // Calculate volume using our implementation
    auto vertices = camera.getFrustumVertices();
    double estimated_volume = estimate_frustum_volume(std::vector<Point_3>(vertices.begin(), vertices.end()));
    
    // Print results
    std::cout << "Camera parameters:" << std::endl;
    std::cout << "  Position: (" << position.x() << ", " << position.y() << ", " << position.z() << ")" << std::endl;
    std::cout << "  HFOV: " << hfov << " degrees" << std::endl;
    std::cout << "  VFOV: " << vfov << " degrees" << std::endl;
    std::cout << "  Near plane: " << near_plane << " meters" << std::endl;
    std::cout << "  Far plane: " << far_plane << " meters" << std::endl;
    
    std::cout << "Frustum dimensions:" << std::endl;
    std::cout << "  Near face: " << near_width << "x" << near_height << " meters" << std::endl;
    std::cout << "  Far face: " << far_width << "x" << far_height << " meters" << std::endl;
    std::cout << "  Depth: " << depth << " meters" << std::endl;
    
    std::cout << "Volume calculations:" << std::endl;
    std::cout << "  Exact volume: " << exact_volume << " cubic meters" << std::endl;
    std::cout << "  Estimated volume: " << estimated_volume << " cubic meters" << std::endl;
    std::cout << "  Error: " << (estimated_volume - exact_volume) << " cubic meters" << std::endl;
    std::cout << "  Percent error: " << ((estimated_volume - exact_volume) / exact_volume * 100) << "%" << std::endl;
}

// Test precise overlap configurations
void test_precise_overlap() {
    std::cout << "\n=== Testing Precise Overlap Configurations ===" << std::endl;
    
    // Global parameters for all tests
    double hfov = 60.0;      // Degrees
    double vfov = 45.0;      // Degrees
    double near_plane = 1.0; // Meters
    double far_plane = 10.0; // Meters
    
    // Test case 1: Exactly 50% overlap along X-axis
    {
        std::cout << "\nTest Case 1: Exactly 50% overlap along X-axis" << std::endl;
        
        // Calculate the width of the frustum at the far plane
        double far_width = 2 * far_plane * std::tan(hfov * M_PI / 360.0);
        
        // Create two cameras, with the second camera offset by half the far width
        // This should create a 50% overlap in their frustums
        Point_3 position1(0, 0, 0);
        Point_3 position2(far_width/2, 0, 0);
        
        // Both cameras pointing forward (along Z-axis)
        Camera camera1 = create_test_camera(position1, 0, 0, 0, hfov, vfov, near_plane, far_plane);
        Camera camera2 = create_test_camera(position2, 0, 0, 0, hfov, vfov, near_plane, far_plane);
        
        // Calculate individual volumes
        auto vertices1 = camera1.getFrustumVertices();
        auto vertices2 = camera2.getFrustumVertices();
        double volume1 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices1.begin(), vertices1.end()));
        double volume2 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices2.begin(), vertices2.end()));
        
        // Calculate intersection volume
        double intersection_volume = CameraGeometry::compute_frustum_intersection_volume(camera1, camera2);
        
        // Expected overlap should be close to 50% of the volume
        double expected_overlap = volume1 * 0.5;
        
        std::cout << "Camera 1 volume: " << volume1 << " cubic meters" << std::endl;
        std::cout << "Camera 2 volume: " << volume2 << " cubic meters" << std::endl;
        std::cout << "Intersection volume: " << intersection_volume << " cubic meters" << std::endl;
        std::cout << "Expected intersection: " << expected_overlap << " cubic meters" << std::endl;
        std::cout << "Error: " << (intersection_volume - expected_overlap) << " cubic meters" << std::endl;
        std::cout << "Percent error: " << ((intersection_volume - expected_overlap) / expected_overlap * 100) << "%" << std::endl;
        std::cout << "Overlap percentage: " << (intersection_volume / volume1 * 100) << "%" << std::endl;
    }
    
    // Test case 2: Exactly 100% overlap with different sizes
    {
        std::cout << "\nTest Case 2: Exactly 100% overlap with different sizes" << std::endl;
        
        Point_3 position(0, 0, 0);
        
        // Camera 1 with standard parameters
        Camera camera1 = create_test_camera(position, 0, 0, 0, hfov, vfov, near_plane, far_plane);
        
        // Camera 2 with same orientation but smaller frustum (80% of the size)
        double smaller_hfov = hfov * 0.8;
        double smaller_vfov = vfov * 0.8;
        Camera camera2 = create_test_camera(position, 0, 0, 0, smaller_hfov, smaller_vfov, near_plane, far_plane);
        
        // Calculate individual volumes
        auto vertices1 = camera1.getFrustumVertices();
        auto vertices2 = camera2.getFrustumVertices();
        double volume1 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices1.begin(), vertices1.end()));
        double volume2 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices2.begin(), vertices2.end()));
        
        // Calculate intersection volume
        double intersection_volume = CameraGeometry::compute_frustum_intersection_volume(camera1, camera2);
        
        // Expected overlap should be the smaller volume (100% of the smaller camera)
        double expected_overlap = volume2;
        
        std::cout << "Camera 1 volume: " << volume1 << " cubic meters" << std::endl;
        std::cout << "Camera 2 volume: " << volume2 << " cubic meters" << std::endl;
        std::cout << "Intersection volume: " << intersection_volume << " cubic meters" << std::endl;
        std::cout << "Expected intersection: " << expected_overlap << " cubic meters" << std::endl;
        std::cout << "Error: " << (intersection_volume - expected_overlap) << " cubic meters" << std::endl;
        std::cout << "Percent error: " << ((intersection_volume - expected_overlap) / expected_overlap * 100) << "%" << std::endl;
        std::cout << "Overlap percentage of smaller camera: " << (intersection_volume / volume2 * 100) << "%" << std::endl;
    }
}

// Test two cameras with known overlap volume
void test_two_cameras_overlap() {
    std::cout << "\n=== Testing Two Cameras with Known Overlap ===" << std::endl;
    
    // Test case 1: Two identical cameras, 50% overlap along X-axis
    {
        std::cout << "\nTest Case 1: 50% Overlap along X-axis" << std::endl;
        
        // Create camera at origin, pointing forward (along Z-axis)
        Point_3 position1(0, 0, 0);
        double roll1 = 0.0;
        double pitch1 = 0.0;
        double yaw1 = 0.0;
        double hfov = 60.0;
        double vfov = 45.0;
        double near_plane = 1.0;
        double far_plane = 10.0;
        
        Camera camera1 = create_test_camera(position1, yaw1, pitch1, roll1, hfov, vfov, near_plane, far_plane);
        
        // Create similar camera, shifted 5 units to the right but pointing left
        // This should create partial side-by-side overlap
        Point_3 position2(5, 0, 0);
        double yaw2 = M_PI/2; // 90 degrees, pointing along negative X-axis (left)
        
        Camera camera2 = create_test_camera(position2, yaw2, pitch1, roll1, hfov, vfov, near_plane, far_plane);
        
        // Calculate individual volumes
        auto vertices1 = camera1.getFrustumVertices();
        auto vertices2 = camera2.getFrustumVertices();
        double volume1 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices1.begin(), vertices1.end()));
        double volume2 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices2.begin(), vertices2.end()));
        
        // Calculate intersection volume
        double intersection_volume = CameraGeometry::compute_frustum_intersection_volume(camera1, camera2);
        
        // Calculate expected overlap based on geometry
        // The expected overlap should be meaningful with the cameras in this configuration
        std::cout << "Camera 1 volume: " << volume1 << " cubic meters" << std::endl;
        std::cout << "Camera 2 volume: " << volume2 << " cubic meters" << std::endl;
        std::cout << "Intersection volume: " << intersection_volume << " cubic meters" << std::endl;
        std::cout << "Overlap percentage: " << (intersection_volume / std::min(volume1, volume2) * 100) << "%" << std::endl;
    }
    
    // Test case 2: Two identical cameras with 100% overlap (same position and orientation)
    {
        std::cout << "\nTest Case 2: 100% Overlap (same position and orientation)" << std::endl;
        
        Point_3 position(0, 0, 0);
        double hfov = 60.0;      // Degrees
        double vfov = 45.0;      // Degrees
        double near_plane = 1.0; // Meters
        double far_plane = 10.0; // Meters
        
        Camera camera1 = create_test_camera(position, 0, 0, 0, hfov, vfov, near_plane, far_plane);
        Camera camera2 = create_test_camera(position, 0, 0, 0, hfov, vfov, near_plane, far_plane);
        
        // Calculate individual volumes
        auto vertices1 = camera1.getFrustumVertices();
        double volume1 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices1.begin(), vertices1.end()));
        
        // Calculate intersection volume
        double intersection_volume = CameraGeometry::compute_frustum_intersection_volume(camera1, camera2);
        
        // Expected intersection should be equal to the individual camera volume
        std::cout << "Camera volume: " << volume1 << " cubic meters" << std::endl;
        std::cout << "Intersection volume: " << intersection_volume << " cubic meters" << std::endl;
        std::cout << "Expected intersection: " << volume1 << " cubic meters" << std::endl;
        std::cout << "Error: " << (intersection_volume - volume1) << " cubic meters" << std::endl;
        std::cout << "Percent error: " << ((intersection_volume - volume1) / volume1 * 100) << "%" << std::endl;
        std::cout << "Overlap percentage: " << (intersection_volume / volume1 * 100) << "%" << std::endl;
    }
    
    // Test case 3: Two identical cameras with 100% overlap but facing opposite directions
    {
        std::cout << "\nTest Case 3: Same position, opposite directions" << std::endl;
        
        Point_3 position(0, 0, 0);
        double hfov = 60.0;      // Degrees
        double vfov = 45.0;      // Degrees
        double near_plane = 1.0; // Meters
        double far_plane = 10.0; // Meters
        
        Camera camera1 = create_test_camera(position, 0, 0, 0, hfov, vfov, near_plane, far_plane);   // Forward
        Camera camera2 = create_test_camera(position, M_PI, 0, 0, hfov, vfov, near_plane, far_plane); // Backward
        
        // Calculate individual volumes
        auto vertices1 = camera1.getFrustumVertices();
        auto vertices2 = camera2.getFrustumVertices();
        double volume1 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices1.begin(), vertices1.end()));
        double volume2 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices2.begin(), vertices2.end()));
        
        // Calculate intersection volume
        double intersection_volume = CameraGeometry::compute_frustum_intersection_volume(camera1, camera2);
        
        // Expected intersection should be very small or zero
        std::cout << "Camera 1 volume: " << volume1 << " cubic meters" << std::endl;
        std::cout << "Camera 2 volume: " << volume2 << " cubic meters" << std::endl;
        std::cout << "Intersection volume: " << intersection_volume << " cubic meters" << std::endl;
        std::cout << "Expected intersection: Very small or zero" << std::endl;
        std::cout << "Overlap percentage: " << (intersection_volume / std::min(volume1, volume2) * 100) << "%" << std::endl;
    }
    
    // Test case 4: Perpendicular cameras with overlap
    {
        std::cout << "\nTest Case 4: Perpendicular with overlap" << std::endl;
        
        Point_3 position1(0, 0, 0);
        Point_3 position2(0, 0, 0);
        double hfov = 60.0;      // Degrees
        double vfov = 45.0;      // Degrees
        double near_plane = 1.0; // Meters
        double far_plane = 10.0; // Meters
        
        Camera camera1 = create_test_camera(position1, 0, 0, 0, hfov, vfov, near_plane, far_plane);        // Forward (Z)
        Camera camera2 = create_test_camera(position2, M_PI/2, 0, 0, hfov, vfov, near_plane, far_plane);   // Right (X)
        
        // Calculate individual volumes
        auto vertices1 = camera1.getFrustumVertices();
        auto vertices2 = camera2.getFrustumVertices();
        double volume1 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices1.begin(), vertices1.end()));
        double volume2 = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices2.begin(), vertices2.end()));
        
        // Calculate intersection volume
        double intersection_volume = CameraGeometry::compute_frustum_intersection_volume(camera1, camera2);
        
        // Expected intersection - should be moderate given perpendicular orientation at same location
        std::cout << "Camera 1 volume: " << volume1 << " cubic meters" << std::endl;
        std::cout << "Camera 2 volume: " << volume2 << " cubic meters" << std::endl;
        std::cout << "Intersection volume: " << intersection_volume << " cubic meters" << std::endl;
        std::cout << "Overlap percentage: " << (intersection_volume / std::min(volume1, volume2) * 100) << "%" << std::endl;
    }
}

// Test three cameras with known total coverage
void test_three_cameras_coverage() {
    std::cout << "\n=== Testing Three Cameras Total Coverage ===" << std::endl;
    
    // Test case 1: Three cameras in a triangular formation all pointing inward
    {
        std::cout << "\nTest Case 1: Triangular formation pointing inward" << std::endl;
        
        Point_3 position1(0, 0, 0);
        Point_3 position2(10, 0, 0);
        Point_3 position3(5, 8, 0);
        
        // Calculate the center of the triangle
        Point_3 center((position1.x() + position2.x() + position3.x()) / 3.0,
                       (position1.y() + position2.y() + position3.y()) / 3.0,
                       (position1.z() + position2.z() + position3.z()) / 3.0);
        
        // Calculate yaw angles pointing toward the center
        double yaw1 = std::atan2(center.y() - position1.y(), center.x() - position1.x());
        double yaw2 = std::atan2(center.y() - position2.y(), center.x() - position2.x());
        double yaw3 = std::atan2(center.y() - position3.y(), center.x() - position3.x());
        
        double hfov = 60.0;      // Degrees
        double vfov = 45.0;      // Degrees
        double near_plane = 1.0; // Meters
        double far_plane = 10.0; // Meters
        
        // Create cameras pointing toward the center of the triangle
        Camera camera1 = create_test_camera(position1, yaw1, 0, 0, hfov, vfov, near_plane, far_plane);
        Camera camera2 = create_test_camera(position2, yaw2, 0, 0, hfov, vfov, near_plane, far_plane);
        Camera camera3 = create_test_camera(position3, yaw3, 0, 0, hfov, vfov, near_plane, far_plane);
        
        std::vector<Camera> cameras = {camera1, camera2, camera3};
        
        // Calculate individual volumes
        std::vector<double> individual_volumes;
        double total_individual_volume = 0.0;
        
        for (size_t i = 0; i < cameras.size(); ++i) {
            auto vertices = cameras[i].getFrustumVertices();
            double volume = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices.begin(), vertices.end()));
            individual_volumes.push_back(volume);
            total_individual_volume += volume;
            std::cout << "Camera " << i << " volume: " << volume << " cubic meters" << std::endl;
        }
        
        // Calculate pairwise intersections
        double total_pairwise_intersection = 0.0;
        for (size_t i = 0; i < cameras.size(); ++i) {
            for (size_t j = i + 1; j < cameras.size(); ++j) {
                double intersection = CameraGeometry::compute_frustum_intersection_volume(cameras[i], cameras[j]);
                std::cout << "Intersection of cameras " << i << " and " << j << ": " 
                         << intersection << " cubic meters" << std::endl;
                std::cout << "Overlap percentage: " << (intersection / std::min(individual_volumes[i], individual_volumes[j]) * 100) 
                         << "%" << std::endl;
                total_pairwise_intersection += intersection;
            }
        }
        
        // Calculate three-way intersection
        std::vector<std::pair<std::vector<size_t>, double>> all_intersections = 
            CameraGeometry::compute_all_intersection_combinations(cameras);
            
        double three_way_intersection = 0.0;
        for (const auto& intersection : all_intersections) {
            if (intersection.first.size() == 3) {
                three_way_intersection = intersection.second;
                break;
            }
        }
        
        std::cout << "Three-way intersection: " << three_way_intersection << " cubic meters" << std::endl;
        
        // Calculate total coverage using inclusion-exclusion principle
        double expected_coverage = total_individual_volume - total_pairwise_intersection + three_way_intersection;
        double calculated_coverage = CameraGeometry::compute_total_coverage_volume(cameras);
        
        std::cout << "Total individual volume: " << total_individual_volume << " cubic meters" << std::endl;
        std::cout << "Total pairwise intersection: " << total_pairwise_intersection << " cubic meters" << std::endl;
        std::cout << "Expected coverage (inclusion-exclusion): " << expected_coverage << " cubic meters" << std::endl;
        std::cout << "Calculated coverage: " << calculated_coverage << " cubic meters" << std::endl;
        std::cout << "Error: " << (calculated_coverage - expected_coverage) << " cubic meters" << std::endl;
        if (expected_coverage != 0) {
            std::cout << "Percent error: " << ((calculated_coverage - expected_coverage) / expected_coverage * 100) << "%" << std::endl;
        }
    }
    
    // Test case 2: Sequential line of cameras with partial overlap
    {
        std::cout << "\nTest Case 2: Sequential line of cameras with partial overlap" << std::endl;
        
        // Create cameras in a line, overlapping their frustums
        Point_3 position1(0, 0, 0);
        Point_3 position2(7, 0, 0);  // Placed so far ends overlap with near of next camera
        Point_3 position3(14, 0, 0);
        
        double hfov = 60.0;      // Degrees
        double vfov = 45.0;      // Degrees
        double near_plane = 1.0; // Meters
        double far_plane = 10.0; // Meters
        
        // All cameras facing forward (along X-axis)
        Camera camera1 = create_test_camera(position1, M_PI/2, 0, 0, hfov, vfov, near_plane, far_plane);
        Camera camera2 = create_test_camera(position2, M_PI/2, 0, 0, hfov, vfov, near_plane, far_plane);
        Camera camera3 = create_test_camera(position3, M_PI/2, 0, 0, hfov, vfov, near_plane, far_plane);
        
        std::vector<Camera> cameras = {camera1, camera2, camera3};
        
        // Calculate individual volumes and pairwise intersections
        std::vector<double> individual_volumes;
        double total_individual_volume = 0.0;
        
        for (size_t i = 0; i < cameras.size(); ++i) {
            auto vertices = cameras[i].getFrustumVertices();
            double volume = CameraGeometry::estimate_frustum_volume(std::vector<Point_3>(vertices.begin(), vertices.end()));
            individual_volumes.push_back(volume);
            total_individual_volume += volume;
            std::cout << "Camera " << i << " volume: " << volume << " cubic meters" << std::endl;
        }
        
        // Calculate pairwise intersections
        double total_pairwise_intersection = 0.0;
        for (size_t i = 0; i < cameras.size(); ++i) {
            for (size_t j = i + 1; j < cameras.size(); ++j) {
                double intersection = CameraGeometry::compute_frustum_intersection_volume(cameras[i], cameras[j]);
                std::cout << "Intersection of cameras " << i << " and " << j << ": " 
                         << intersection << " cubic meters" << std::endl;
                std::cout << "Overlap percentage: " << (intersection / std::min(individual_volumes[i], individual_volumes[j]) * 100) 
                         << "%" << std::endl;
                total_pairwise_intersection += intersection;
            }
        }
        
        // Calculate three-way intersection (should be near zero in this case)
        std::vector<std::pair<std::vector<size_t>, double>> all_intersections = 
            CameraGeometry::compute_all_intersection_combinations(cameras);
            
        double three_way_intersection = 0.0;
        for (const auto& intersection : all_intersections) {
            if (intersection.first.size() == 3) {
                three_way_intersection = intersection.second;
                break;
            }
        }
        
        std::cout << "Three-way intersection: " << three_way_intersection << " cubic meters" << std::endl;
        
        // Calculate total coverage
        double expected_coverage = total_individual_volume - total_pairwise_intersection + three_way_intersection;
        double calculated_coverage = CameraGeometry::compute_total_coverage_volume(cameras);
        
        std::cout << "Total individual volume: " << total_individual_volume << " cubic meters" << std::endl;
        std::cout << "Total pairwise intersection: " << total_pairwise_intersection << " cubic meters" << std::endl;
        std::cout << "Expected coverage (inclusion-exclusion): " << expected_coverage << " cubic meters" << std::endl;
        std::cout << "Calculated coverage: " << calculated_coverage << " cubic meters" << std::endl;
        std::cout << "Error: " << (calculated_coverage - expected_coverage) << " cubic meters" << std::endl;
        if (expected_coverage != 0) {
            std::cout << "Percent error: " << ((calculated_coverage - expected_coverage) / expected_coverage * 100) << "%" << std::endl;
        }
    }
}

int main(int argc, char* argv[]) {
    std::cout << std::fixed << std::setprecision(6);
    
    // Process command line arguments to determine which tests to run
    bool run_single = true;
    bool run_two = true;
    bool run_three = true;
    bool run_precise = true;
    
    if (argc > 1) {
        // If arguments provided, only run specified tests
        run_single = false;
        run_two = false;
        run_three = false;
        run_precise = false;
        
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "single" || arg == "1") {
                run_single = true;
            } else if (arg == "two" || arg == "2") {
                run_two = true;
            } else if (arg == "three" || arg == "3") {
                run_three = true;
            } else if (arg == "precise" || arg == "4") {
                run_precise = true;
            } else if (arg == "all") {
                run_single = true;
                run_two = true;
                run_three = true;
                run_precise = true;
            }
        }
    }
    
    // Run the tests based on flags
    if (run_single) {
        test_single_camera_volume();
    }
    
    if (run_precise) {
        test_precise_overlap();
    }
    
    if (run_two) {
        test_two_cameras_overlap();
    }
    
    if (run_three) {
        test_three_cameras_coverage();
    }
    
    return 0;
} 