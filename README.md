# ArUco Treasure Hunt (ROS 2 Humble)

ROS 2 implementation of the Experimental Robotics Lab ArUco assignment. The `aruco_navigator` node detects `DICT_ARUCO_ORIGINAL` markers (physical side 0.2 m), aligns and approaches from image features, executes marker turns using odometry yaw feedback, and stops at marker 4. Its node parameters and launch arguments allow the same controller to use the ROSbot camera and odometry topics.

## Architecture

- `aruco_navigator_node`: subscribes to camera image, camera calibration, and odometry; publishes velocity and annotated reached-marker images.
- State machine: `SEARCHING` (initially drive toward marker 1; after each completed turn, drive ahead while holding the new heading; after marker loss, stop and rotate slowly to reacquire), `ALIGNING` (center the expected marker, with slow forward motion for moderate errors while distant), `APPROACHING` (target apparent side), `RECOVERING` (bounded slow forward advance toward the last visual bearing of a distant marker), `EXECUTING` (closed-loop yaw turn), `SUCCESS` (marker 4, stopped).
- The assignment route is enforced in order: IDs 1, 2, 3, then 4. A marker is marked used as soon as it is reached, before its turn, so it cannot be reacquired during a rotation. New detections need five consecutive frames and at least 12 px mean side; an already active marker remains tracked even if its apparent size briefly drops below that threshold. After prolonged loss, the returning marker must pass confirmation again before servoing resumes.
- During alignment, forward speed is capped at 0.08 m/s and tapers to zero at a horizontal error of 15% of image width (96 px at 640 px width). Once the marker reaches 80% of its target size, alignment happens in place. Centered approach still uses the assignment's size-error controller. The four tuning values are launch arguments: `min_detected_marker_side_px`, `alignment_forward_speed`, `alignment_stop_error_fraction`, and `alignment_near_target_ratio`. Setting `alignment_forward_speed:=0.0` restores alignment in place. The ROSbot still needs hardware validation.
- Marker 1 turns right 90 degrees, marker 2 turns 180 degrees, marker 3 turns left 90 degrees. On reaching marker 4, it stops and reports success.

## Detection and recovery

The native ArUco detector is retried at 3x image scale if the expected ID is missing. Corners are divided by three before measurement, so the 166 px target retains its original meaning. Enlarging improves decoding but does not add sensor resolution.

Between decoded frames, a corner tracker uses current image pixels, forward/backward optical-flow consistency, image bounds, and quadrilateral geometry checks. Tracking lasts at most one second without a new ID decode, rejects frame gaps over 0.2 s, and cannot declare arrival. A decoded ID is required to complete a marker. Acquisition and reacquisition still require five consecutive decoded frames.

If a distant target's observation becomes stale, `RECOVERING` holds its last image-derived bearing using odometry and advances at 0.06 m/s. Motion is capped at 0.4 m or eight seconds, and starts only after heading error is below 0.2 rad. The last apparent size must be below half the stopping target. At the limit, the robot holds position and continues detecting. Close-range losses stop forward travel. These bounds are ROS startup parameters (`recovery_speed`, `recovery_max_distance_m`, `recovery_timeout_sec`, `recovery_far_target_ratio`). This is not obstacle avoidance: real deployment requires a clear route and the robot's independent command timeout/emergency stop.

Motion requires valid camera calibration and recent image/odometry delivery (0.5 s timeout). Old camera timestamps are rejected. Sensor subscriptions use sensor-data QoS. Reached snapshots use the image/header associated with the observation, and arrival clears the entire velocity command immediately.

`/aruco_navigator/status` publishes JSON once per second: state, marker ID, used IDs, observed center/size, observation source (`decoded` or `tracked`), age, and calibration readiness. Parameters are read at startup; restart to apply tuning. `SEARCHING` before first acquisition still assumes a clear path along the prescribed initial/post-turn heading. The fixed ID order and physical sensor/network compatibility must be validated for the real route. This project has simulation evidence, not production certification.

Run focused regression tests without moving a robot:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src/aruco_navigator:$PYTHONPATH" /usr/bin/python3 -m pytest -q \
  src/aruco_navigator/test/test_visual_tracking.py \
  src/aruco_navigator/test/test_marker_tracking.py
```

## Build

```bash
git clone https://github.com/albertrichard080/aruco-treasure-hunt.git ~/aruco_treasure_hunt_ws
cd ~/aruco_treasure_hunt_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The host needs ROS 2 Humble packages used by the workspace, including `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`, `robot_localization`, `cv_bridge`, `image_transport`, OpenCV with `cv2.aruco`, and NumPy.

## Gazebo simulation

The simulation entry point is `erl1/launch/spawn_assignment.launch.py`. It adds the installed model directory to Gazebo's resource path. Start the world and controller in separate terminals:

```bash
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$HOME/aruco_treasure_hunt_ws/src/erl1/models"
export IGN_GAZEBO_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH:$HOME/aruco_treasure_hunt_ws/src/erl1/models"
ros2 launch erl1 spawn_assignment.launch.py
```

```bash
source ~/aruco_treasure_hunt_ws/install/setup.bash
ros2 launch aruco_navigator aruco_navigator.launch.py use_sim_time:=true \
  image_topic:=/camera/image camera_info_topic:=/camera/camera_info \
  odom_topic:=/odom cmd_vel_topic:=/cmd_vel
```

The launch file defaults are for the real ROSbot, so the simulation topics and `use_sim_time:=true` are given explicitly here.

The camera image is bridged with `ros_gz_image image_bridge` as described in the course CV notes. `/camera/camera_info`, `/odom`, `/cmd_vel`, and `/clock` use `ros_gz_bridge`. Check actual topics with `ros2 topic list` if the course Gazebo version or robot description differs.

## ROSbot

Use the robot's ROS domain (course sheet: 10/11/12 for rosbot_1/2/3; `ros2 topic list | grep rosbot` must show the robot's topics), the default Fast DDS, and do not set `ROS_LOCALHOST_ONLY=1`.

```bash
cd ~/aruco_treasure_hunt_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aruco_navigator aruco_navigator.launch.py robot:=rosbot_1
```

With no other arguments the launch file uses `use_sim_time:=false` and the ROSbot topic names (`robot:=rosbot_2` or `rosbot_3` changes the namespace):

| Input | Topic on rosbot_1 |
|---|---|
| Camera | `/rosbot_1/camera/rgb/image_raw` (320x240, `rgb8`) |
| Calibration | `/rosbot_1/camera/rgb/camera_info` (fx = 288.02 px) |
| Odometry | `/rosbot_1/odometry/filtered` |
| Command | `/rosbot_1/cmd_vel` (`geometry_msgs/msg/Twist`) |

The desired marker side is scaled from `camera_info`: 166 x 288.02 / 381.36 = 125.4 px on the ROSbot.

Add `image_transport:=compressed` to receive JPEG frames from `/rosbot_1/camera/rgb/image_raw/compressed` instead (about 30 KB per frame instead of 230 KB), which helps on a busy Wi-Fi network.

The robot's clock does not have to match the laptop's. Frame delay is measured relative to the smallest delay seen in the last few seconds, so a constant offset is ignored; the node prints the offset once at startup. Synchronize the clocks anyway for RViz and TF.

If the robot does not move, the node prints `Holding still, waiting for ...` with the missing input (camera_info, camera images or odometry).

Speed changes are ramped (`max_linear_accel` 0.3 m/s^2 and `max_angular_accel` 1.0 rad/s^2, both launch arguments), and short Wi-Fi gaps do not stop the robot: a marker counts as lost after 0.4 s without it, and camera or odometry after 1 s.

## Interface

| Topic | Type | Purpose |
|---|---|---|
| `/camera/image` | `sensor_msgs/msg/Image` | Camera input (changeable) |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Focal length scaling (changeable) |
| `/odom` | `nav_msgs/msg/Odometry` | Closed-loop heading (changeable) |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity output (changeable) |
| `/marker_image` | `sensor_msgs/msg/Image` | Annotated image published at all reached markers, including 4 (image transports such as `/marker_image/compressed` are available) |

For real robots, use the course network/domain setup. Do not set `ROS_LOCALHOST_ONLY=1`; use the standard Fast DDS configuration specified by the course.

## Credits

Simulation assets and the original `erl1` package come from [Carmine Tommaso Recchiuto’s course repository](https://github.com/CarmineD8/erl1). Existing package license declarations are retained.
