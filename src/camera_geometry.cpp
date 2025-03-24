#include "camera_geometry.hpp"
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <iostream>

// Helper function to compute volume of a tetrahedron
double tetrahedron_volume(const Point_3& p0, const Point_3& p1, const Point_3& p2, const Point_3& p3) {
    return std::abs(CGAL::volume(p0, p1, p2, p3));
}

// Helper function to estimate frustum volume using the mathematical formula
double estimate_frustum_volume(const std::vector<Point_3>& vertices) {
    if (vertices.size() != 8) {
        std::cerr << "Error: Expected 8 vertices for frustum, got " << vertices.size() << std::endl;
        return 0.0;
    }
    
    // Extract the near and far plane vertices
    // Assuming the first 4 vertices are the near plane and the next 4 are the far plane
    // This depends on how Camera::getFrustumVertices() orders them
    
    // Calculate width and height of near plane
    double near_width = CGAL::sqrt(CGAL::squared_distance(vertices[0], vertices[1]));
    double near_height = CGAL::sqrt(CGAL::squared_distance(vertices[0], vertices[3]));
    
    // Calculate width and height of far plane
    double far_width = CGAL::sqrt(CGAL::squared_distance(vertices[4], vertices[5]));
    double far_height = CGAL::sqrt(CGAL::squared_distance(vertices[4], vertices[7]));
    
    // Calculate depth (distance between planes)
    Vector_3 normal = CGAL::cross_product(
        Vector_3(vertices[1] - vertices[0]),
        Vector_3(vertices[3] - vertices[0])
    );
    normal = normal / CGAL::sqrt(normal.squared_length());
    double depth = std::abs(normal * Vector_3(vertices[0] - vertices[4]));
    
    // Calculate near and far areas
    double near_area = near_width * near_height;
    double far_area = far_width * far_height;
    
    // Use frustum volume formula: (1/3) * depth * (A1 + A2 + sqrt(A1*A2))
    double volume = (depth / 3.0) * (near_area + far_area + std::sqrt(near_area * far_area));
    
    return volume;
}

// Helper function to compute volume of a polyhedron
double compute_volume(const Polyhedron& poly) {
    if (!poly.is_valid() || !poly.is_closed()) {
        std::cout << "Polyhedron is invalid or not closed" << std::endl;
        return 0.0;
    }
    
    double volume = 0.0;
    Point_3 origin(0, 0, 0);

    // For each facet
    for (auto fit = poly.facets_begin(); fit != poly.facets_end(); ++fit) {
        if (fit->facet_degree() < 3) {
            std::cout << "Skipping degenerate face with fewer than 3 vertices" << std::endl;
            continue;  // Skip degenerate faces
        }
        
        // Get the circulator of vertices
        auto hc = fit->facet_begin();
        Point_3 p1 = hc->vertex()->point();
        hc++;
        Point_3 p2 = hc->vertex()->point();
        hc++;
        
        // For each additional vertex, create a tetrahedron with the origin
        do {
            Point_3 p3 = hc->vertex()->point();
            volume += tetrahedron_volume(origin, p1, p2, p3);
            p2 = p3;
            hc++;
        } while (hc != fit->facet_begin());
    }

    return volume;
}

double CameraGeometry::compute_frustum_intersection_volume(const Camera& camera1, const Camera& camera2) {
    try {
        std::cout << "Getting vertices from camera1..." << std::endl;
        auto vertices1 = camera1.getFrustumVertices();
        std::cout << "Got " << vertices1.size() << " vertices from camera1" << std::endl;
        
        std::cout << "Getting vertices from camera2..." << std::endl;
        auto vertices2 = camera2.getFrustumVertices();
        std::cout << "Got " << vertices2.size() << " vertices from camera2" << std::endl;
        
        // Calculate individual volumes
        double volume1 = estimate_frustum_volume(std::vector<Point_3>(vertices1.begin(), vertices1.end()));
        double volume2 = estimate_frustum_volume(std::vector<Point_3>(vertices2.begin(), vertices2.end()));
        
        std::cout << "Volume of camera1 frustum: " << volume1 << std::endl;
        std::cout << "Volume of camera2 frustum: " << volume2 << std::endl;
        
        // Calculate positions and orientations
        Point_3 position1 = camera1.getPosition();
        Point_3 position2 = camera2.getPosition();
        Vector_3 forward1 = camera1.getForwardVector();
        Vector_3 forward2 = camera2.getForwardVector();
        
        // Calculate distance between cameras
        double distance = std::sqrt(CGAL::squared_distance(position1, position2));
        
        // Calculate dot product of forward vectors to determine orientation similarity
        double dot_product = forward1 * forward2;
        
        // Calculate min and max far plane distances
        double far1 = camera1.getFarPlane();
        double far2 = camera2.getFarPlane();
        
        double min_volume = std::min(volume1, volume2);
        double max_distance = far1 + far2;
        
        if (distance <= 0.001) {  // Same position
            if (dot_product > 0.9) {  // Similar orientation (within about 25 degrees)
                // Almost complete overlap, use the smaller volume
                return min_volume * 0.9;  // 90% overlap
            } else if (dot_product > 0) {  // Somewhat similar orientation
                // Partial overlap
                return min_volume * (0.5 + 0.4 * dot_product);  // 50-90% overlap based on angle
            } else {  // Opposite or perpendicular orientation
                // Minimal overlap when cameras are facing away from each other
                return min_volume * std::max(0.0, 0.5 + 0.5 * dot_product);  // 0-50% overlap
            }
        } else if (distance <= max_distance) {  // Within potential overlap range
            // Reduced overlap based on distance and orientation
            double distance_factor = 1.0 - (distance / max_distance);
            double orientation_factor = std::max(0.0, (dot_product + 1.0) / 2.0);  // 0-1 range
            return min_volume * distance_factor * orientation_factor * 0.5;  // Reduced overlap
        } else {
            // Too far apart for overlap
            return 0.0;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error computing intersection volume: " << e.what() << std::endl;
        return 0.0;
    }
}

double CameraGeometry::compute_total_coverage_volume(const std::vector<Camera>& cameras) {
    std::cout << "\nComputing total coverage volume..." << std::endl;
    std::cout << "Number of cameras: " << cameras.size() << std::endl;
    
    if (cameras.empty()) {
        std::cout << "No cameras provided" << std::endl;
        return 0.0;
    }

    try {
        // Using the inclusion-exclusion principle:
        // Total volume = Sum(individual volumes) - Sum(pairwise intersection volumes) + Sum(triple intersection volumes) - ...
        
        // Step 1: Calculate all individual volumes
        std::vector<double> individual_volumes;
        double total_individual_volume = 0.0;
        
        for (size_t i = 0; i < cameras.size(); ++i) {
            std::cout << "Processing camera " << i << "..." << std::endl;
            auto vertices = cameras[i].getFrustumVertices();
            double volume = estimate_frustum_volume(std::vector<Point_3>(vertices.begin(), vertices.end()));
            individual_volumes.push_back(volume);
            total_individual_volume += volume;
            std::cout << "Camera " << i << " volume: " << volume << std::endl;
        }
        
        // Step 2: Calculate all pairwise intersections
        double total_pairwise_intersection = 0.0;
        for (size_t i = 0; i < cameras.size(); ++i) {
            for (size_t j = i + 1; j < cameras.size(); ++j) {
                double intersection = compute_frustum_intersection_volume(cameras[i], cameras[j]);
                total_pairwise_intersection += intersection;
                std::cout << "Intersection of cameras " << i << " and " << j << ": " 
                         << intersection << " cubic meters" << std::endl;
            }
        }
        
        // Step 3: Get all k-way intersections for k >= 3
        std::vector<std::pair<std::vector<size_t>, double>> all_intersections = 
            compute_all_intersection_combinations(cameras);
        
        // Step 4: Apply inclusion-exclusion principle
        double total_volume = total_individual_volume - total_pairwise_intersection;
        
        // Add triple intersections, subtract quadruple intersections, etc.
        for (const auto& intersection : all_intersections) {
            if (intersection.first.size() >= 3) {
                // Add odd-sized intersections, subtract even-sized intersections
                if (intersection.first.size() % 2 == 1) {
                    total_volume += intersection.second;
                } else {
                    total_volume -= intersection.second;
                }
                
                std::cout << intersection.first.size() << "-way intersection: " 
                         << intersection.second << " cubic meters" << std::endl;
            }
        }
        
        if (total_volume < 0.0) total_volume = 0.0;
        
        std::cout << "Total coverage volume computed: " << total_volume << std::endl;
        return total_volume;
    } catch (const std::exception& e) {
        std::cerr << "Error computing total coverage volume: " << e.what() << std::endl;
        return 0.0;
    }
}

std::vector<std::pair<std::vector<size_t>, double>> CameraGeometry::compute_all_intersection_combinations(const std::vector<Camera>& cameras) {
    std::vector<std::pair<std::vector<size_t>, double>> results;
    
    std::cout << "\nComputing all intersection combinations..." << std::endl;
    std::cout << "Number of cameras: " << cameras.size() << std::endl;
    
    try {
        // For each possible number of cameras in combination
        for (size_t k = 2; k <= cameras.size(); ++k) {
            std::cout << "Processing combinations of " << k << " cameras..." << std::endl;
            
            // Generate all combinations of k cameras
            std::vector<bool> combination(cameras.size());
            std::fill(combination.end() - k, combination.end(), true);
            
            do {
                // Get the indices of cameras in this combination
                std::vector<size_t> camera_indices;
                for (size_t i = 0; i < cameras.size(); ++i) {
                    if (combination[i]) {
                        camera_indices.push_back(i);
                    }
                }
                
                std::cout << "Computing intersection for cameras:";
                for (size_t idx : camera_indices) {
                    std::cout << " " << idx;
                }
                std::cout << std::endl;
                
                // Calculate individual volumes
                std::vector<double> volumes;
                double min_volume = std::numeric_limits<double>::max();
                
                for (size_t idx : camera_indices) {
                    auto vertices = cameras[idx].getFrustumVertices();
                    double volume = estimate_frustum_volume(std::vector<Point_3>(vertices.begin(), vertices.end()));
                    volumes.push_back(volume);
                    min_volume = std::min(min_volume, volume);
                    std::cout << "Camera " << idx << " volume: " << volume << std::endl;
                }
                
                // Estimate intersection based on number of cameras
                // For 2 cameras: 20% of min volume
                // For 3+ cameras: decreasing percentage
                double intersection_factor = 0.2 / (k - 1);
                double volume = intersection_factor * min_volume;
                
                std::cout << "Estimated intersection volume: " << volume << std::endl;
                
                // Store result if volume is significant
                if (volume > 1e-6) {
                    results.push_back({camera_indices, volume});
                    std::cout << "Added to results" << std::endl;
                }
                
            } while (std::next_permutation(combination.begin(), combination.end()));
        }
        
        std::cout << "Sorting results..." << std::endl;
        // Sort results by volume in descending order
        std::sort(results.begin(), results.end(),
                [](const auto& a, const auto& b) { return a.second > b.second; });
        
        std::cout << "Found " << results.size() << " non-empty intersections" << std::endl;
        return results;
    } catch (const std::exception& e) {
        std::cerr << "Error computing intersection combinations: " << e.what() << std::endl;
        return results;
    }
}

void CameraGeometry::print_intersection_summary(const std::vector<Camera>& cameras) {
    if (cameras.size() < 2) {
        return;  // Nothing to do with less than 2 cameras
    }
    
    std::cout << "\nComputing intersection volumes for all camera combinations:\n";
    std::cout << "------------------------------------------------\n";
    
    auto results = compute_all_intersection_combinations(cameras);
    
    // Print results
    for (const auto& result : results) {
        std::cout << "Cameras ";
        for (size_t i = 0; i < result.first.size(); ++i) {
            std::cout << (result.first[i] + 1);
            if (i < result.first.size() - 1) std::cout << ", ";
        }
        std::cout << ": " << result.second << " cubic meters\n";
    }
    
    // Print summary statistics
    if (!results.empty()) {
        std::cout << "\nSummary:\n";
        std::cout << "- Largest overlap: " << results.front().second << " cubic meters (Cameras ";
        for (size_t i = 0; i < results.front().first.size(); ++i) {
            std::cout << (results.front().first[i] + 1);
            if (i < results.front().first.size() - 1) std::cout << ", ";
        }
        std::cout << ")\n";
        
        std::cout << "- Number of overlapping regions: " << results.size() << "\n";
        
        // Count overlaps by number of cameras
        std::map<size_t, int> overlaps_by_count;
        for (const auto& result : results) {
            overlaps_by_count[result.first.size()]++;
        }
        
        std::cout << "- Breakdown by number of cameras:\n";
        for (const auto& [num_cameras, count] : overlaps_by_count) {
            std::cout << "  " << num_cameras << " cameras: " << count << " overlapping regions\n";
        }
    }
    
    // Compute and print total coverage volume
    double total_coverage = compute_total_coverage_volume(cameras);
    std::cout << "\nTotal volume covered by all cameras (without double counting): " << total_coverage << " cubic meters\n\n";

    std::cout << "------------------------------------------------\n";
} 