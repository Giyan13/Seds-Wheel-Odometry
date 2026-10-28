# wheel_odometry

ROS 2 (Humble) wheel odometry node for the SEDS Pakistan rover.
Task SEDS-TASK-ROV-SRILANKA-008, Pod 3A. Follows
`robojackets-urc-software-training` Lesson 5 (the lesson is C++; this is the Python version).

## Interface
| | |
|---|---|
| Subscribes | `/joint_states` (`sensor_msgs/JointState`), wheel positions in radians |
| Publishes | `/wheel/odometry` (`nav_msgs/Odometry`), `frame_id: odom`, `child_frame_id: base_footprint` |
| TF | **None.** The EKF owns `odom -> base_footprint` |

One odometry message is published per valid `/joint_states` message (same rate as the source),
using that message's timestamp.

Output: `pose.pose.position.x/y` (m); yaw as a quaternion in `pose.pose.orientation`
(only z and w non-zero; `yaw = atan2(2*w*z, 1 - 2*z*z)`); `twist.twist.linear.x` (m/s);
`twist.twist.angular.z` (rad/s). Covariance: 1e-3 for x/y, 1e-2 for yaw.

## Parameters
| Name | Default | Meaning |
|---|---|---|
| `wheel_radius` | 0.075 | wheel radius r (m), match the URDF |
| `wheel_separation` | 0.34 | distance between wheels L (m), match the URDF |
| `left_joint_name` | `left_wheel_joint` | left wheel joint in `/joint_states` |
| `right_joint_name` | `right_wheel_joint` | right wheel joint in `/joint_states` |
| `velocity_smoothing` | 1.0 | optional smoothing of v and omega (1.0 = off). Pose is never smoothed. |

## How it works
1. First valid message: record baseline, publish zero pose/velocity, no integration.
2. Later messages: `d = r * delta_phi` per wheel, `d_center = (d_r + d_l)/2`,
   `d_theta = (d_r - d_l)/L`, pose integrated with the **midpoint heading**, heading
   normalised to [-pi, pi], `v = d_center/dt`, `omega = d_theta/dt`.
3. Messages with `dt <= 0` are ignored (the delta is not consumed).

## Build, run, test
```
cd ~/odom_ws && colcon build --packages-select wheel_odometry && source install/setup.bash
ros2 launch wheel_odometry wheel_odometry.launch.py
# or with overrides:
ros2 run wheel_odometry wheel_odometry --ros-args -p wheel_separation:=0.30
```
Demo without the simulator (straight 5 s, in-place turn 5 s, repeat):
```
ros2 run wheel_odometry fake_joint_states &
ros2 run wheel_odometry wheel_odometry
ros2 topic echo /wheel/odometry --field pose.pose
ros2 topic echo /wheel/odometry --field twist.twist
```
Unit tests (straight, in-place turn, full rotation, arc, smoothness, wrap-around, baseline, bad dt):
```
cd ~/odom_ws/src/wheel_odometry && python3 -m pytest test -v
```

## Assumptions (please read)
- Interface, parameter defaults and frame names come from Lesson 5
  (`robojackets-urc-software-training`). The lesson is C++; this is the Python version
  (the task allows `.py` or `.cpp`).
- Defaults `wheel_radius=0.075`, `wheel_separation=0.34` and joint names
  `left_wheel_joint` / `right_wheel_joint` are the lesson's values. The real competition
  rover may differ (for example, a 4-wheel Leo Rover with different joint names).
  Everything is a launch parameter, so no code change is needed:
  ```
  ros2 run wheel_odometry wheel_odometry --ros-args \
    -p wheel_radius:=<r> -p wheel_separation:=<L> \
    -p left_joint_name:=<name> -p right_joint_name:=<name>
  ```
- Wheel odometry drifts by design (no external correction). It is meant to be fused by the EKF,
  which should use the velocities.

## Verification
- `python3 -m pytest test -v` -> 11 passed (straight driving, in-place turn, full rotation, arc,
  smoothness, angle wrap-around, baseline, bad `dt`, quaternion, smoothing, parameters).
- Demo with `fake_joint_states`: x grows during the straight phase with y and yaw about 0; during
  the turn x and y stay constant while yaw changes.

## Proof
Saved evidence is in the [`proof/`](proof/) folder:
- `test-output.txt`: output of `python3 -m pytest test -v` (11 passed)
- `odometry-demo.mp4`: screen recording of `/wheel/odometry` during the demo (straight driving, then an in-place turn)
- `*.png`: screenshots of the test run and of `ros2 topic echo /wheel/odometry --field pose.pose`

What the demo shows: during straight driving the heading stays constant and position advances in
even steps (about 0.006 m per message at 50 Hz, i.e. 0.3 m/s); during the in-place turn x and y stay
fixed while the heading changes smoothly. No jumps in either phase.

## Interface notes for other pods
- Heading convention: yaw is counter-clockwise from +x, in radians, range [-pi, pi]. For a dashboard
  that wants degrees in 0 to 360: `heading_deg = degrees(yaw) % 360`.
- Position is in a local frame in metres, starting at (0, 0) at the first message.
- This package publishes a ROS 2 topic. It does not talk WebSocket, so a ground-station dashboard
  needs a bridge (for example rosbridge) to receive it.

## Limitations
- Tested with the included fake `/joint_states` publisher and unit tests only. It has **not** been run
  against the Gazebo / Leo Rover simulator or a recorded bag.
- Defaults are the Lesson 5 values. The real rover's wheel radius, wheel separation and joint names
  are not confirmed. They are launch parameters.
- Uses one left and one right wheel joint. A 4-wheel rover would need the front and rear wheel on each
  side combined (not implemented).
- Wheel odometry drifts without bound (slip, wrong dimensions, quantisation). It is meant to be fused,
  using its velocities, by an EKF.
- 2D only. No wheel-slip handling.
- Python version of a lesson written in C++.
