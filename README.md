# Cameras Pose Optimization

This project helps optimize cameras positions on a robot to maximize coverage while maintaining required overlap between cameras' fields of view.

## Prerequisites

- CMake (>= 3.1)
- CGAL library
- yaml-cpp library

On Ubuntu/Debian, you can install the dependencies with:
```bash
sudo apt-get update
sudo apt-get install libcgal-dev libyaml-cpp-dev
```

## Building the Project

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

1. Create a configuration file in YAML format (see `config/default_config.yaml` for an example)
2. Run the program:
```bash
./camera_optimizer ../config/default_config.yaml
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
    orientation: [1.0, 0.0, 0.0, 0.0]  # quaternion (w, x, y, z)
    hfov: 120.0  # horizontal field of view in degrees
    vfov: 60.0   # vertical field of view in degrees
    near_plane: 0.5  # near plane in meters
    far_plane: 4.0   # far plane in meters

robot:
  length: 1.0  # meters
  width: 0.5   # meters
  height: 0.4  # meters

required_overlap: 0.2  # 20% overlap requirement
```

## Current Features

- Reads camera and robot specifications from YAML configuration
- Visualizes camera frustums and robot body using CGAL
- Supports multiple cameras
- Configurable field of view and overlap requirements

## Future Enhancements

- Optimization of camera positions to maximize coverage
- Collision detection between camera frustums
- Coverage analysis and visualization
- Export of optimized camera positions 