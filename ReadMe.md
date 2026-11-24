# Autonomous ArUco Treasure Hunt (ROS 2 Jazzy)

**Memebers:** 
* Hani Bouhraoua        ->       8314923
* Richard Albert King Mechoda  -> 8525970
* Riadh Bahri                  -> 8335614
* Basmala Sherief             -> 8248659


**Course:** Experimental Robotics Laboratory (`erl1`) — M.Sc. in Robotics Engineering, University of Genoa (UniGe)  
**Platforms:** Gazebo Simulation & Physical Husarion ROSbot 3

An autonomous mobile robot navigation and visual-servoing package implemented in **ROS 2 Jazzy** (`rclpy`, `OpenCV`, and `cv_bridge`). The robot explores the environment, detects `DICT_ARUCO_ORIGINAL` fiducial markers (IDs `1` through `4`), approaches each marker using **Image-Based Visual Servoing (IBVS)**, publishes annotated target frames on `/marker_image`, and executes closed-loop odometry turns until reaching the final goal at Marker `4`.

---

## Demonstration Videos

> Replace the placeholder links below with your uploaded GitHub release or YouTube URLs.

* **Gazebo Simulation Run:** [Watch Simulation Demo](#)
* **Physical Husarion ROSbot 3 Run:** [Watch Real-World Lab Demo](#)

---

## System Architecture

The `ArUcoTreasureHunt` node (`scripts/treasure_hunt.py`) is organized into four core functional modules:

### 1. ArUco Perception & Camera
* Subscribes to raw RGB image streams (`image_topic`) and camera intrinsic parameters (`camera_info_topic`), converting ROS `Image` messages to OpenCV frames via `cv_bridge`.
* Detects `DICT_ARUCO_ORIGINAL` markers and filters for unvisited valid IDs (`1`, `2`, `3`, `4`).
* For each detected marker, computes its apparent pixel side length $s$ (average of the four detected edges) and pixel centroid $(u, v)$.
* Reads the horizontal focal length $f_x$ and optical center $c_x$ from `/camera_info` to dynamically scale the desired target pixel size $s_d$, keeping the physical stopping distance ($\approx 60\text{ cm}$) consistent between simulation and hardware:
  $$s_d = s_{d,\text{sim}} \cdot \frac{f_x}{f_{\text{sim}}}$$
* Upon reaching a marker, draws a green circle of radius $r = \frac{s}{\sqrt{2}}$ and an `id <N>` label onto the frame and publishes it to `/marker_image`.

### 2. Image-Based Visual Servoing (IBVS)
* Tracks the closest unvisited marker currently in view (largest apparent side length $s$).
* Computes the size error $e_s = s_d - s$ and horizontal centering error $e_u = u - c_x$.
* Generates proportional linear and angular velocity commands to center the marker in the image plane while closing the distance:
  $$v = k_v \cdot e_s, \qquad \omega = -k_w \cdot e_u$$
* Applies velocity floors and safe lab ceilings so the robot approaches smoothly without stalling or overshooting the target.

### 3. Closed-Loop Odometry Rotation
* Subscribes to filtered odometry (`odom_topic`) and extracts the current heading $\psi$ from the orientation quaternion $(x, y, z, w)$:
  $$\psi = \text{atan2}\big(2(wz + xy),\; 1 - 2(y^2 + z^2)\big)$$
* Runs a **20 Hz** `control_loop` timer that computes the normalized angular error to the target heading $\psi_{\text{target}}$:
  $$e_\psi = \text{atan2}\big(\sin(\psi_{\text{target}} - \psi),\; \cos(\psi_{\text{target}} - \psi)\big)$$
* Commands proportional angular velocity $\omega = k_{\text{yaw}} \cdot e_\psi$ (bounded by a skid-steer friction floor and safe maximum speed) until $|e_\psi| \le \text{yaw\_tolerance}$.

### 4. Treasure Hunt State Machine
The mission logic is governed by a deterministic three-state finite state machine (`SEARCH_AND_SERVO` $\rightarrow$ `ROTATING` $\rightarrow$ `DONE`) backed by a `visited_ids` set to prevent re-triggering already completed markers:

1. **`SEARCH_AND_SERVO`:** Drives straight forward at `search_vel` when no unvisited marker is visible, or engages IBVS when a marker enters the camera frame. Once $s \ge (s_d - s_{\text{tol}})$ and $|e_u| \le u_{\text{tol}}$, the robot halts, publishes the annotated image on `/marker_image`, logs the marker ID in `visited_ids`, and dispatches the marker's navigation rule:
   * **Marker `1`:** Turn right $90^\circ$ ($\psi_{\text{target}} = \psi - \frac{\pi}{2}\text{ rad}$) $\rightarrow$ `ROTATING`
   * **Marker `2`:** Rotate $180^\circ$ ($\psi_{\text{target}} = \psi + \pi\text{ rad}$) $\rightarrow$ `ROTATING`
   * **Marker `3`:** Turn left $90^\circ$ ($\psi_{\text{target}} = \psi + \frac{\pi}{2}\text{ rad}$) $\rightarrow$ `ROTATING`
   * **Marker `4`:** Mission complete $\rightarrow$ `DONE`
2. **`ROTATING`:** Executes the closed-loop odometry turn at 20 Hz. Once aligned within `yaw_tolerance`, stops the wheels and transitions back to `SEARCH_AND_SERVO`.
3. **`DONE`:** Publishes zero velocity (`Twist()`) and terminates the hunt at Marker `4`.

---

## Sim-to-Real Challenges & Fixes

While the baseline implementation completed the mission cleanly in Gazebo, deploying on the physical **Husarion ROSbot 3** over lab Wi-Fi revealed four Sim-to-Real issues that were diagnosed and fixed:

### 1. Motor Friction & 4WD Skid-Steer Deadband
* **Problem:** As the robot approached a target distance or heading, proportional commands ($v = k_v e_s$, $\omega = k_{\text{yaw}} e_\psi$) dropped to low values ($0.02\text{--}0.04\text{ m/s}$ or $0.15\text{ rad/s}$). In Gazebo's frictionless physics, the chassis still moved; on the real 4-wheel skid-steer ROSbot, these commands lacked the torque to overcome static gearbox friction and sideways tire drag, stalling the robot just before Marker `1` and preventing in-place turns.
* **Fix:** Added minimum velocity floors of $0.11\text{ m/s}$ for linear IBVS approach and $0.50\text{ rad/s}$ for in-place rotation so the motors always break static friction until the target threshold is reached.

### 2. Tight Pixel Stopping Tolerances & Momentum Overshoot
* **Problem:** Requiring `abs(err_s) <= 4.0` px and `abs(err_u) <= 12.0` px simultaneously worked in ideal simulation, but real-world camera noise and chassis momentum carried the ROSbot outside the $\pm 4\text{ px}$ window, trapping it in `SEARCH_AND_SERVO` without triggering `ROTATING`.
* **Fix:** Relaxed stopping tolerances to `s_tolerance = 18.0` px and `u_tolerance = 45.0` px, and updated the stopping condition to `s >= (self.s_d - self.s_tol)` so rotation triggers reliably upon reaching target proximity.

### 3. Wi-Fi DDS Buffer Saturation on Raw RGB Streams
* **Problem:** Subscribing to uncompressed raw RGB images (`/rosbot_3/camera/rgb/image_raw`) over shared lab Wi-Fi with queue depth `10` caused stale frames to build up in the Fast-DDS buffer during latency spikes, freezing the terminal and preventing clean `Ctrl+C` shutdowns.
* **Fix:** Switched all sensor subscriptions (`Image`, `CameraInfo`, `Odometry`) to `qos_profile_sensor_data` (**Best-Effort** reliability, **Keep-Last depth `1`**) so outdated frames are dropped immediately and the node stays responsive.

### 4. Camera Intrinsics Scaling & Safe Lab Velocity Ceilings
* **Problem:** The physical ROSbot camera intrinsics ($f_x = 288.4\text{ px}$, $c_x = 159.5\text{ px}$) differed from Gazebo ($f_x = 381.4\text{ px}$, $c_x = 320.0\text{ px}$), and high simulation speeds risked overshooting markers or causing motion blur on the lab floor.
* **Fix:** Enabled automatic focal-length scaling from `/camera_info` (adjusting $s_d$ from $166.0\text{ px}$ in simulation to $125.5\text{ px}$ on `rosbot_3`) and enforced conservative lab speed ceilings (`search_vel = 0.20 m/s`, linear max $0.25\text{ m/s}$, angular max $0.65\text{--}0.75\text{ rad/s}$).

### Parameter Comparison (Gazebo vs. Physical ROSbot 3)

| Parameter | Gazebo Baseline | Physical ROSbot 3 | Purpose |
| :--- | :--- | :--- | :--- |
| `s_tolerance` | `4.0 px` | `18.0 px` | Absorbs camera noise and stopping momentum |
| `u_tolerance` | `12.0 px` | `45.0 px` | Realistic horizontal alignment window |
| `search_vel` | `0.30 m/s` | `0.20 m/s` | Safe forward exploration speed |
| Linear cap / floor | $\pm 0.40\text{ m/s}$ / none | $\pm 0.25\text{ m/s}$ / $0.11\text{ m/s}$ | Prevents lunging and low-speed motor stall |
| Angular cap / floor (`ROTATING`) | $\pm 0.80\text{ rad/s}$ / $0.15\text{ rad/s}$ | $\pm 0.75\text{ rad/s}$ / $0.50\text{ rad/s}$ | Breaks 4WD skid-steer tire friction safely |
| `yaw_tolerance` | `0.03 rad` | `0.04 rad` | Clean turn termination without oscillation |
| Sensor QoS | Default (Reliable, depth `10`) | `qos_profile_sensor_data` (Best-Effort, depth `1`) | Prevents Wi-Fi buffer buildup and freezes |

---

## How to Run

### 1. Build the Package
```bash
cd ~/Desktop/erl1
colcon build
source install/setup.bash
```

### 2. Run in Gazebo Simulation
Launch your Gazebo world first, then run the node with simulation topics and `use_sim_time:=true`:
```bash
ros2 run erl1 treasure_hunt.py --ros-args \
  -p use_sim_time:=true \
  -p image_topic:=/camera/image \
  -p camera_info_topic:=/camera/camera_info \
  -p odom_topic:=/odometry/filtered \
  -p cmd_vel_topic:=/cmd_vel
```

### 3. Run on Physical Husarion ROSbot 3 (`ROS_DOMAIN_ID=13`)
Ensure your laptop is connected to the lab Wi-Fi network and configured for `rosbot_3`:
```bash
export ROS_DOMAIN_ID=13
unset ROS_LOCALHOST_ONLY
cd ~/Desktop/erl1
source install/setup.bash
ros2 run erl1 treasure_hunt.py
```

### 4. Visualize Annotated Marker Detections
In a separate terminal (with `ROS_DOMAIN_ID=13` exported when using the real ROSbot):
```bash
ros2 run rqt_image_view rqt_image_view /marker_image
```