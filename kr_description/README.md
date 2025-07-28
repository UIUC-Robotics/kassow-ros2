# Outdoor Manipulator URDFs and Xacro description package

## Installation
```
cd colcon_ws
rosdep install --from-paths src/kr_description --ignore-src -r -y
colcon build --symlink-install --packages-select kr_description
```

## Usage
```
source ~/colcon_ws/install/setup.bash
ros2 launch kr_description display.launch.py

```