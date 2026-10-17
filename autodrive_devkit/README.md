# AutoDRIVE Devkit

<p align="justify">
This directory hosts ROS 2 API (a meta-package), which supports modular algorithm development targetted towards autonomous driving. It uses client libraries for Python and C++, and can be therefore exploited by the users to develop their algorithms flexibly.
</p>

## SETUP

1. Clone the `AutoDRIVE-RoboRacer-Sim-Racing` repository.
    ```bash
    $ git clone https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing.git
    ```
2. Give executable permissions to the Python scripts.
   ```bash
   $ cd autodrive_devkit
   $ sudo chmod +x *.py
   ```
4. Install the necessary dependencies as mentioned below.
    [AutoDRIVE Devkit's ROS 2 API](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/tree/main/autodrive_devkit) has the following dependencies (tested with Python 3.8, 3.9 and 3.10):
    
    - Websocket-related dependencies for communication bridge between [AutoDRIVE Simulator](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/tree/main/autodrive_simulator) and [AutoDRIVE Devkit](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/tree/main/autodrive_devkit) (version sensitive):
    
      | Package            | Python 3.8 | Python 3.9 | Python 3.10 |
      |--------------------|------------|------------|-------------|
      | eventlet           | 0.33.3     | 0.33.3     | 0.33.3      |
      | Flask              | 1.1.1      | 1.1.1      | 1.1.1       |
      | Flask-SocketIO     | 4.1.0      | 4.1.0      | 4.1.0       |
      | python-socketio    | 4.2.0      | 4.2.0      | 4.2.0       |
      | python-engineio    | 3.13.0     | 3.13.0     | 3.13.0      |
      | greenlet           | 1.0.0      | 1.0.0      | 1.1.0       |
      | gevent             | 21.1.2     | 21.1.2     | 21.12.0     |
      | gevent-websocket   | 0.10.1     | 0.10.1     | 0.10.1      |
      | Jinja2             | 3.0.3      | 3.0.3      | 3.0.3       |
      | itsdangerous       | 2.0.1      | 2.0.1      | 2.0.1       |
      | werkzeug           | 2.0.3      | 2.0.3      | 2.0.3       |
    
    - Generic dependencies for data processing and visualization (usually any version will do the job):
    
      | Package               | Tested Version |
      |-----------------------|----------------|
      | attrdict              | 2.0.1          |
      | numpy                 | 1.22.2         |
      | pillow                | 9.0.1          |
      | opencv-contrib-python | 4.10.0.84      |
      | transforms3d          | 0.4.2          |
  
    - Install dependencies using `requirements.txt` file (use the file specific to your Python version &#8594; check using `python3 --version`):

      ```bash
      $ pip3 install -r requirements_python_3.8.txt # Python 3.8
      $ pip3 install -r requirements_python_3.9.txt # Python 3.9
      $ pip3 install -r requirements_python_3.10.txt # Python 3.10
      ```

    - ROS 2 dependencies for data processing and visualization (usually any version will do the job):

      ```bash
      $ sudo apt install ros-$ROS_DISTRO-tf-transformations
      $ sudo apt install ros-$ROS_DISTRO-imu-tools
      ```

## USAGE

- **Bringup:**
    - **Headless Mode Bringup:**
      ```bash
      $ ros2 launch autodrive_roboracer bringup_headless.launch.py
      ```
      **[OR]**
    - **Graphics Mode Bringup:**
      ```bash
      $ ros2 launch autodrive_roboracer bringup_graphics.launch.py
      ```

- **Teleoperation:**
  ```bash
  $ ros2 run autodrive_roboracer teleop_keyboard
  ```

## Autonomous racing controller

Build the edited package in the ROS workspace, start the bridge, and run one
driving node at a time:

```bash
cd /home/autodrive_devkit
colcon build --packages-select autodrive_roboracer --symlink-install
source install/setup.bash
ros2 run autodrive_roboracer wall_follow_node
```

The complete controller lives in `wall_follow_node.py`. It fits nearby LiDAR
wall returns to estimate left/right distances, wall heading and track curvature.
Centering PD, heading correction and curvature anticipation produce a normalized
steering command, clamped to [-1, 1] and limited by `steering_rate`.
The curvature contribution is `atan(curvature_gain * curvature)`; `ka` scales
both heading and curvature anticipation. The controller contains no vehicle
kinematics, body dimensions, steering-angle calibration or braking model.

All coordinates and clearance distances are measured from the **LiDAR origin**,
including `crawl_distance`, `stop_buffer`, side distances and wall lookahead.
They are not bumper clearances. Track width is the distance between fitted walls.
One visible wall uses the last measured track width, or `nominal_track_width`
before any width is measured, and limits speed to `single_wall_speed`.

Speed selection takes the lowest applicable limit:

- `max_speed` is the overall ceiling.
- Curvature limits speed to `max_speed / (1 + curvature_slowdown * abs(curvature))`.
- Steering limits speed to `max_speed / (1 + steering_slowdown * abs(steering))`.
- Side proximity reduces speed between `side_slow_distance` and
  `side_stop_distance`; reaching the latter commands a stop.
- A fitted passage no wider than `minimum_track_width` commands a stop.
- The front approach targets `min_speed` at `crawl_distance`, falls linearly to
  zero at `stop_buffer`, and increases quadratically toward `max_speed` over
  the next 4 m above `crawl_distance`.

The current front-approach settings are **4 m/s at 2 m from the LiDAR**, with a
zero target at 0.20 m. Other speed limits can lower these targets. These are
empirical response curves, not a calculated stopping-distance bound: the new
cornering gains and clearance thresholds require a simulator trial.

The node subscribes to `/autodrive/roboracer_1/lidar` and
`/autodrive/roboracer_1/odom`, and publishes the existing throttle and steering
command topics. Speed feedback uses the magnitude of planar odometry velocity.
Throttle retains feedforward plus PI feedback and normal rise/fall limits.
Zero targets, sensor faults and overspeed beyond `overspeed_release` immediately
release throttle. Zero throttle requests simulator idle braking; negative
throttle is reverse torque and is not used as a brake command.

The front detector uses the nearest forward point inside
`front_corridor_half_width` on either side of the LiDAR centerline. It is a
straight detection corridor, not a predicted vehicle footprint. Isolated invalid
off-axis beams are tolerated; invalid central beams or broad forward dropouts
command a stop. Positive infinity bounds free space at sensor range but is not
fitted as a wall. Neither wall measurable, stale sensors and invalid odometry
also release throttle. The watchdog handles sensors that cease publishing.
Sensor-fault stops hold the last steering, and changes to stop reasons are logged.

### Parameters

All distance thresholds are metres, speeds are m/s and times are seconds.
Command values and command rates use normalized actuator units.

| Parameter | Default | Effect of increasing it |
| --- | ---: | --- |
| `kp`, `kd` | 0.65, 0.06 | Stronger centering correction and stronger derivative damping, respectively |
| `ka`, `heading_gain` | 1.5, 1.5 | Stronger bend anticipation and heading correction, respectively |
| `wall_lookahead` | 1.8 | Fit farther wall sections |
| `curvature_gain` | 0.33 | Stronger curvature steering; a response gain, not a vehicle dimension |
| `curvature_slowdown` | 3.0 | Lower speed in measured bends |
| `steering_slowdown` | 2.0 | Lower speed during stronger steering commands |
| `front_corridor_half_width` | 0.20 | Include more off-axis obstacles in the front detector |
| `side_stop_distance` | 0.17 | Stop farther from a side wall |
| `side_slow_distance` | 0.67 | Start side-proximity slowing farther from a wall |
| `minimum_track_width` | 0.55 | Stop in wider passages |
| `max_speed` | 10.0 | Allow higher speed where other limits permit |
| `min_speed`, `crawl_distance` | 4, 2 | Raise the approach target or move its reference distance farther away |
| `stop_buffer` | 0.20 | Command zero farther from a front obstacle |
| `nominal_track_width` | 1.6 | Increase initial single-wall following distance |
| `single_wall_speed` | 1.5 | Allow higher speed with only one wall visible |
| `max_throttle` | 1 | Allow stronger motor drive |
| `speed_gain`, `speed_integral_gain` | 0.25, 0.18 | Stronger immediate and accumulated speed corrections |
| `throttle_feedforward` | 0.06 | More baseline drive per unit of target speed |
| `throttle_rise_rate`, `throttle_fall_rate` | 0.7, 0.01 | Faster normal increases and decreases in throttle |
| `overspeed_release` | 0.5 | Tolerate more overspeed before immediate throttle release |
| `steering_rate` | 10.0 | Allow faster steering-command changes |
| `scan_timeout`, `odom_timeout` | 0.25, 0.25 | Tolerate longer sensor delays before stopping |

`side_slow_distance` must exceed `side_stop_distance`, and `crawl_distance`
must exceed `stop_buffer`. `min_speed` is not an absolute minimum speed.
ROS parameters load at startup; rebuild and restart to apply source changes.

### Parameter migration

Remove these vehicle-model parameters from old launch overrides:
`wheelbase`, `vehicle_width`, `lidar_offset`, `front_overhang`,
`max_steering_angle`, `max_lateral_accel`, `braking_decel`, and `reaction_time`.
`wall_margin` and `front_wall_margin` are also removed: use the direct corridor,
side-distance and minimum-track-width thresholds above. No dimensions are
retained as hidden constants. Earlier `ki`, `steering_filter_time`, and
`anticipation_filter_time` overrides must also be removed.

Existing front-distance overrides now apply from the LiDAR rather than the
bumper, so review them when migrating. The remaining parameter names, ROS topics,
node name and executable entry point are unchanged.

### Controller validation

Run the geometry/control and ROS wrapper tests in the ROS environment:

```bash
cd /home/autodrive_devkit/src/autodrive_devkit
python3 -m pytest test/test_wall_follow_geometry.py test/test_wall_follow_node.py -q
```

Geometry tests ray-cast from a LiDAR pose without vehicle dimensions; wrapper
tests isolate their publishers on ROS domain 99. Tests cover turn direction and
severity, speed reductions, distance thresholds, invalid data, one-wall fallback,
command continuity and rate limits, watchdog stops and shutdown.

For a bounded live simulator trial, stop other driving nodes and leave the
bridge running, then execute from the same package directory:

```bash
python3 test/run_controller_trial.py --seconds 120 --output trial.json
```

The trial records speeds, commands, faults, and changes in lap/collision counters,
then leaves zero throttle. It does not reset counters or teleport the car.
Use the ROS installation's Python if a separate Python is also installed.
Review collisions and command behavior before increasing speed. Local tests and
ROS test doubles do not validate live simulator dynamics or middleware delivery.
