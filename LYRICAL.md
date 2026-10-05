# ROS 2 Lyrical port (unofficial)

This branch carries a community port of `fanuc_driver` (upstream `main`, equal to `jazzy`)
to ROS 2 Lyrical Luth on Ubuntu 26.04, which ships ros2_control 6.x. It is not endorsed
or supported by FANUC.

## Build

```bash
sudo apt install git-lfs ros-lyrical-desktop ros-dev-tools \
     ros-lyrical-force-torque-sensor-broadcaster ros-lyrical-ros2controlcli
git lfs install
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/FANUC-CORPORATION/fanuc_description.git
git clone --branch lyrical --single-branch --recurse-submodules \
    https://github.com/botsandus/fanuc_driver.git
cd ~/ros2_ws
rosdep update && rosdep install --ignore-src --from-paths src -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=0
```

## What changed against upstream

| Area | Change |
|---|---|
| `fanuc_hardware_interface` | Distro check accepts `lyrical`; `on_init(const HardwareComponentInterfaceParams&)` |
| `fanuc_controllers` | `ament_target_dependencies` shim (removed from ament_cmake); `get_value()` -> `get_optional<double>()`; `RealtimePublisher` `trylock/unlockAndPublish` -> `try_publish` |
| `ScaledJointTrajectoryController` | Rebuilt as a thin override of the Lyrical JTC, which has native speed scaling. Keeps: abort when motion is not possible, hold after reconnect, topic scale x robot collaborative scaling |
| Launch files | `--param-file` passed to every spawner (6.x controllers no longer inherit the controller manager's parameter files) |
| `slider_publisher` | `COLCON_IGNORE` (needs Qt5, Lyrical uses Qt6); no longer started by the mock launch |

## Status

Verified:
- Full workspace builds with `-DBUILD_TESTING=0` (ROS Lyrical packages: ros2_control 6.10.1, joint_trajectory_controller 6.9.0, MoveIt 2.15.2).
- Mock hardware: controllers activate; a 3 s trajectory completes in 3.00 s and takes 6.00 s at 50% speed scaling.
- Physical CRX-30iA, status only (`motion_control:=0 initial_controller:=none`): driver connects, `/joint_states` arrives at 500 Hz.

Not verified:
- Motion on physical hardware.
- Unit tests: the test code still builds `Loaned*Interface` objects the old way and does not compile with `-DBUILD_TESTING=1`.
- The stock mock launch cannot activate `fanuc_gpio_controller` (the mock URDF lacks `ConnectionStatus/motion_command_type`); this is also true upstream.
