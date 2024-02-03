# PX4 Offboard ROS2 Package

## Overview
The PX4 Offboard ROS2 package enables autonomous control of a UAV by generating and sending offboard setpoints based on a polygonal path. This package communicates with the PX4 flight controller using ROS2 topics.

## Features
- **Polygonal Path Generation:** The package generates a polygonal path based on user-defined parameters such as center point, circumradius, number of sides, and rotation angle.
- **Offboard Setpoint Control:** The `offboard_setpoint` node publishes trajectory setpoints to the PX4 flight controller, allowing the UAV to follow the specified polygonal path.
- **Dynamic Parameters:** Users can customize key parameters to define the shape of the polygon and the UAV's behavior, providing flexibility for different mission requirements.
- **Offboard Mode Handling:** The package monitors the vehicle's control mode, ensuring that setpoints are only sent when the UAV is in offboard mode.

## Prerequisites
- ROS2 (Robot Operating System 2)
- PX4 firmware configured for offboard mode

## Installation
1. Clone this repository to your ROS2 workspace:

    ```bash
    git clone https://github.com/hpaul360/px4_offboard.git
    ```

2. Build the package:

    ```bash
    colcon build --symlink-install
    ```

3. Source the ROS2 setup file:

    ```bash
    source install/setup.bash
    ```

## Usage
1. Ensure your PX4 flight controller is configured for offboard mode and is armed.
2. Take off the UAV.
3. Launch the `px4_offboard` node:

    ```bash
    ros2 launch px4_offboard offboard_launch.py
    ```

4. The UAV will follow a polygonal path based on the specified parameters.

## Connection Parameters

Set parameters in the config file accordingly: px4_offboard/config/params.yaml
```bash
## Parameters
polygon_center: [0, 3] # Center of the polygon x,y in m
circum_radius: 0.5 # in m
n_sides: 6 # Number of sides of a polygon
meter_per_sec: 1.0 # Estimated expected speed in m/s(can be close/less than this value). Note this is just a estimate, only used to set number of setpoints in between vertices. This is not actual velocity control with feedback.
polygon_angle: 45 # in deg (Self note: Keep 45 for square.)
uav_height: -0.8 # in m. Should be a negative value.
uav_yaw: 0.0 # in deg
```

## License
This software is released under the MIT License. See the [LICENSE](LICENSE) file for details.

## Author
- [Hannibal Paul](https://github.com/hpaul360)

## Acknowledgments
- This package is based on the PX4 Offboard Control example.

## Issues and Contributions
- Report any issues or contribute to the development of this package on [GitHub](https://github.com/hpaul360/px4_offboard).

## Contact
- For questions or further assistance, feel free to contact the author [Hannibal Paul](https://hannibalpaul.com/).

