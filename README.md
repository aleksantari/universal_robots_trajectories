# universal_robots_trajectories

ros2 humble cpp package to control universal robotic arm for LSCR eye-snake robot research

## What this package does
- **Capture + interpolate**: Collect 3 user-defined “ideal” poses of the TCP on an arc, then interpolate poses along that arc, oriented toward the target.
- **IK + execute**: Convert sampled poses to joint targets via MoveIt IK and queue them for execution via a `FollowJointTrajectory` controller.

Main nodes:
- `capture_interpolate_node`
  - Services: `/capture_pose`, `/compute_views`
  - Publishes: `/captured_poses` (PoseArray), `/desired_poses` (PoseArray), `visualization_marker_array`
- `ik_queue_executor_node`
  - Subscribes: `/desired_poses` (PoseArray)
  - Service: `/execute_next`
  - Action client: `/<controller_ns>/follow_joint_trajectory`

Helper libraries:
- `pose_utils` (TF2-based TCP pose lookup)
- `interpolation_utils` (arc interpolation between poses)

The simple examples `send_trajectory_action` and `send_trajectory_topic` exist but are not used by the workflow below.

## Prerequisites
- ROS 2 Humble
- UR ROS 2 driver (MoveIt2 integration) running for your robot or simulation. See:
  - UR driver with MoveIt usage: [docs.ros.org (Humble) – UR Driver Using MoveIt](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/usage.html#using-moveit)
  - UR driver repo (Humble): [UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble?tab=readme-ov-file)
- A `FollowJointTrajectory` controller active (typically at `/joint_trajectory_controller/follow_joint_trajectory`).
- TF available for `base_link` → `tool0` (provided by your UR bringup/robot_state_publisher).

Best practices for structuring launch files: [ROS 2 Launch for Large Projects](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)

## Build
```bash
cd ~/ros2_ws
colcon build --packages-select ur3_trajectory_senders
source install/setup.bash
```

## Launch the program
This starts both nodes and provides MoveIt with `robot_description` and kinematics. You can override parameters as needed.

```bash
ros2 launch ur3_trajectory_senders arc_capture.launch.py \
  base_frame:=base_link \
  tcp_frame:=tool0 \
  samples:=10 \
  dt:=0.5 \
  planning_group:=ur_manipulator \
  controller_ns:=/joint_trajectory_controller \
  ur_type:=ur3 \
  kinematics_yaml:=$(ros2 pkg prefix ur_moveit_config)/share/ur_moveit_config/config/kinematics.yaml \
  urdf_xacro:=$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro
```
Notes:
- `robot_description` is built from the UR xacro to ensure `ik_queue_executor_node` has the URDF locally. If your setup uses a customized URDF/calibration, pass the matching xacro via `urdf_xacro:=...`.
- `planning_group` is the MoveIt group used for IK (default `ur_manipulator`).
- `controller_ns` should match your controller namespace (default `/joint_trajectory_controller`).

## Operating procedure
1. Manually jog the robot to an “ideal” view pose on the desired arc (camera facing the object).
2. Capture the pose:
   ```bash
   ros2 service call /capture_pose std_srvs/srv/Trigger {}
   ```
3. Repeat step 1–2 until you have at least 3 poses (start, mid, end) captured.
4. Compute the interpolated views along the arc:
   ```bash
   ros2 service call /compute_views std_srvs/srv/Trigger {}
   ```
   - This publishes: `/desired_poses` (PoseArray), and RViz markers via `visualization_marker_array`.
5. Queue and execute one waypoint at a time:
   ```bash
   ros2 service call /execute_next std_srvs/srv/Trigger {}
   ```
   - Call repeatedly to step through each sampled waypoint. The IK node logs queue size and sends a 1-point trajectory to the controller each time.

## Topics & services summary
- `capture_interpolate_node`
  - Services: `/capture_pose` (Trigger), `/compute_views` (Trigger)
  - Publishes: `/captured_poses` (geometry_msgs/PoseArray), `/desired_poses` (geometry_msgs/PoseArray), `visualization_marker_array` (visualization_msgs/MarkerArray)
- `ik_queue_executor_node`
  - Subscribes: `/desired_poses` (geometry_msgs/PoseArray)
  - Service: `/execute_next` (std_srvs/Trigger)
  - Action: `/<controller_ns>/follow_joint_trajectory` (control_msgs/action/FollowJointTrajectory)

## Parameter reference (launch)
- `base_frame` (string): TF base frame (default `base_link`)
- `tcp_frame` (string): TF tool frame (default `tool0`)
- `samples` (int): number of interpolated poses along the arc (default `10`)
- `dt` (double): seconds per 1-point trajectory (default `0.5`)
- `planning_group` (string): MoveIt group for IK (default `ur_manipulator`)
- `controller_ns` (string): namespace of the joint trajectory controller (default `/joint_trajectory_controller`)
- `ur_type` (string): UR robot type for xacro (default `ur3`)
- `kinematics_yaml` (path): MoveIt kinematics config (defaults to `ur_moveit_config`)
- `urdf_xacro` (path): UR xacro to build `robot_description` (defaults to `ur_description`)

## Troubleshooting
- No action server: ensure your controller is running and discoverable at `/<controller_ns>/follow_joint_trajectory`.
  ```bash
  ros2 action list | grep follow_joint_trajectory
  ```
- No TF from `base_link` to `tool0`: check the UR driver/robot_state_publisher is running, confirm frames in TF.
  ```bash
  ros2 run tf2_tools view_frames  # or use RViz TF display
  ```
- IK fails often: verify `planning_group`, `kinematics_yaml`, and that the provided `robot_description` matches the active robot model and calibration.
- Wrong frames: adjust `base_frame`/`tcp_frame` launch args to your setup.

---

`pose_utils`: helper to query TCP pose via TF2.