# ROS 2 Lyrical port (unofficial)

This branch carries a community port of `fanuc_driver` (upstream `main`, equal to `jazzy`)
to ROS 2 Lyrical Luth on Ubuntu 26.04, which ships ros2_control 6.x. It is not endorsed
or supported by FANUC.

## Quick setup

With ROS 2 Lyrical (`ros-lyrical-desktop`) installed, clone this branch into `<workspace>/src` and run the
script. It installs the apt extras, fetches `fanuc_description`, runs rosdep and builds
(`SKIP_APT=1` skips the sudo step):

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --branch lyrical --single-branch --recurse-submodules \
    https://github.com/botsandus/fanuc_driver.git
fanuc_driver/scripts/setup-lyrical.sh
```

## Manual build

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

## Connecting to a robot, status only

Robot side and network follow the upstream quick start (J519 + R912 options, AUTO mode,
teach pendant off, driver on a dedicated NIC, e.g. `192.168.1.101/24` for a controller at
`192.168.1.100`). To read state without taking motion control or activating a trajectory
controller:

```bash
source /opt/ros/lyrical/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch fanuc_moveit_config fanuc_moveit.launch.py robot_model:=crx30ia \
    robot_ip:=192.168.1.100 motion_control:=0 initial_controller:=none
```

Then `ros2 topic echo --once /joint_states` and `ros2 topic echo --once /fanuc_gpio_controller/robot_status`.
With `motion_control:=0` the driver does not start RMI or take motion control, and the scaled
controller aborts any goal (for example an RViz "Plan & Execute") with
`Aborted: STMO is inactive (!motion_possible)`. Dropping either option lets the driver take control.

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
