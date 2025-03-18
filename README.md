# Cameras Pose Optimization

This project helps optimize cameras positions on a robot to maximize coverage while maintaining required overlap between cameras' fields of view. It visualizes camera frustums and calculates overlap volumes between multiple cameras to aid in optimal placement.

## Overview

The application provides a 3D visualization of:
- Camera positions and their viewing frustums
- Robot body representation
- Coordinate system with appropriate markers
- Overlap volumes between camera frustums

It also calculates and reports statistics about the overlapping regions, which is useful for ensuring proper coverage and redundancy in multi-camera systems.

## Getting Started

### Prerequisites

- CMake (>= 3.1)
- CGAL library with Qt5 support
- yaml-cpp library
- C++17 compatible compiler

On Ubuntu/Debian, you can install the dependencies with:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake libcgal-dev libcgal-qt5-dev libyaml-cpp-dev qtbase5-dev
```

On macOS with Homebrew:
```bash
brew install cmake cgal yaml-cpp qt@5
```

### Cloning the Repository

```bash
# Clone the repository
git clone https://github.com/TarekTaha/cameras-pose-optimization.git

# Navigate to the project directory
cd cameras-pose-optimization
```

## Building the Project

```bash
# Create a build directory
mkdir build
cd build

# Generate build files
cmake ..

# Compile the project
make -j$(nproc)  # Use multiple cores for faster compilation
```

## Running the Application

1. Ensure you have a configuration file in YAML format (see `config/default_config.yaml` for an example)
2. Run the program:
```bash
# From the build directory
./cameras_optimizer ../config/default_config.yaml

# Or specify a different configuration file
./cameras_optimizer /path/to/your/config.yaml
```

### Interacting with the 3D Visualization

When the application opens the visualization window:
- Left-click and drag to rotate the view
- Right-click and drag to pan
- Scroll to zoom in and out
- Press 'c' to center the view
- Press 'w' to toggle between wireframe and solid mode
- Press 's' to save a screenshot

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