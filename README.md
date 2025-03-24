# Camera Frustum Volume Coverage Optimization

This project provides tools for calculating and optimizing the volume coverage of multiple camera frustums in 3D space. It includes functionality for:

- Computing individual camera frustum volumes
- Computing the intersection volumes between multiple camera frustums
- Calculating total coverage volume using the inclusion-exclusion principle
- Testing and verification tools for volume calculations

## Features

- Accurate frustum volume calculation based on camera parameters
- Estimation of intersection volumes based on camera positions and orientations
- Visualization of camera frustums in 3D space
- Test suite for verifying volume calculations with known ground truth

## Requirements

- C++17 compatible compiler
- CMake 3.1 or higher
- CGAL with Qt5 support
- yaml-cpp library

## Building the Project

```bash
mkdir build
cd build
cmake ..
make
```

## Running the Tests

After building, you can run the volume tests to verify the calculations:

```bash
cd build
./volume_test
```

You can also run specific test cases:

```bash
./volume_test single    # Test single camera volume
./volume_test precise   # Test precise overlap configurations
./volume_test two       # Test two-camera scenarios
./volume_test three     # Test three-camera scenarios
./volume_test all       # Run all tests
```

## Main Application

The main application optimizes camera placements to maximize coverage:

```bash
cd build
./cameras_optimizer ../config/default_config.yaml
```

## Configuration File Format

The configuration file should specify:
- Camera parameters (position, orientation, field of view, etc.)
- Robot dimensions
- Required overlap between cameras

Example configuration:
```yaml
cameras:
  - position: [0.0, 0.0, 0.4]  # x, y, z in meters
    orientation:
      roll: 0.0   # rotation around X-axis in radians
      pitch: 0.0  # rotation around Y-axis in radians
      yaw: 0.0    # rotation around Z-axis in radians
    hfov: 120.0   # horizontal field of view in degrees
    vfov: 60.0    # vertical field of view in degrees
    near_plane: 0.5  # near plane in meters
    far_plane: 4.0   # far plane in meters

robot:
  position: [0.0, 0.0, 0.0]  # x, y, z in meters
  length: 1.0  # meters
  width: 0.5   # meters
  height: 0.4  # meters

required_overlap: 0.2  # 20% overlap requirement
```

### Understanding the Output

The application provides detailed information about camera frustum overlaps, including:
- All possible overlap combinations between cameras
- Volume of each overlapping region in cubic meters
- Summary statistics on overlaps
- Breakdown of overlaps by number of participating cameras

This information is crucial for:
- Ensuring proper coverage of the robot's surroundings
- Maintaining sufficient overlap for stereo vision or redundancy
- Optimizing camera placement for maximum efficiency

## Current Features

- Reads camera and robot specifications from YAML configuration
- Visualizes camera frustums and robot body using CGAL
- Supports multiple cameras with different positions and orientations
- Configurable field of view and overlap requirements
- Calculates and displays intersection volumes between camera frustums
- Interactive 3D visualization with camera coordinate systems

## Future Enhancements

- Optimization of camera positions to maximize coverage
- Collision detection between camera frustums and environment
- Coverage analysis and visualization
- Export of optimized camera positions
- Support for more complex robot geometries
- Thermal mapping of coverage areas

## Troubleshooting

### Common Issues

1. **CGAL Qt5 Not Found**: Ensure you have installed CGAL with Qt5 support
   ```bash
   sudo apt-get install libcgal-qt5-dev qtbase5-dev
   ```

2. **Build Errors**: Make sure you have a C++17 compatible compiler
   ```bash
   g++ --version  # Should be at least version 7.0
   ```

3. **Visualization Not Working**: Check that Qt5 is properly installed and that your system supports OpenGL

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. 