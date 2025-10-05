Of course. Here is the content formatted as a complete `README.md` file. You can copy and paste this directly into a file named `README.md`.

-----

# ROS 2 Differential Drive Robot Trajectory Following

This ROS 2 project simulates a differential drive robot in Gazebo and makes it follow a predefined trajectory using a waypoint follower node. The project includes the robot's URDF model, a Gazebo simulation environment, and a Python-based control node for trajectory planning and execution. The path is smoothed using cubic spline interpolation, and the robot's motion is controlled using a lookahead algorithm.

<img width="1248" height="644" alt="image" src="https://github.com/user-attachments/assets/3775880a-96a4-4d63-9d6d-b130acf157df" />

-----

## 📜 Table of Contents

  - [Features](https://www.google.com/search?q=%23-features)
  - [Core Concepts](https://www.google.com/search?q=%23-core-concepts)
      - [Differential Drive Kinematics](https://www.google.com/search?q=%23differential-drive-kinematics)
      - [Cubic Spline Interpolation](https://www.google.com/search?q=%23cubic-spline-interpolation)
  - [Project Structure](https://www.google.com/search?q=%23-project-structure)
  - [Prerequisites](https://www.google.com/search?q=%23-prerequisites)
  - [Installation](https://www.google.com/search?q=%23-installation)
  - [Usage](https://www.google.com/search?q=%23-usage)
  - [Code Explanation](https://www.google.com/search?q=%23-code-explanation)
      - [`robot.xacro`](https://www.google.com/search?q=%23robotxacro)
      - [`robot.launch.py`](https://www.google.com/search?q=%23robotlaunchpy)
      - [`waypoint_follower.py`](https://www.google.com/search?q=%23waypoint_followerpy)

-----

## ✨ Features

  - **Robot Model**: A URDF model of a differential drive robot created using `xacro`.
  - **Simulation**: Gazebo simulation with integrated ROS 2 plugins for differential drive control and state publishing.
  - **Trajectory Planning**: Waypoints are smoothed into a continuous path using **Cubic Spline Interpolation**.
  - **Path Following**: A **lookahead control** algorithm to smoothly follow the generated trajectory.
  - **Visualization**: The robot model, planned path, and waypoints are visualized in RViz2.

-----

## 🧠 Core Concepts

### Differential Drive Kinematics

A differential drive robot consists of two independently controlled wheels on a common axis. By varying the velocities of the two wheels, the robot can move forward, backward, and turn.

The relationship between the robot's linear velocity ($v$) and angular velocity ($\omega$) and the individual wheel velocities (left wheel $v_L$, right wheel $v_R$) is defined by the following equations:

Let:

  - $r$ be the radius of the wheels.
  - $L$ be the distance between the two wheels (the wheel separation).

The forward kinematics equations (calculating robot velocity from wheel velocities) are:

$$v = \frac{v_R + v_L}{2}$$

$$\omega = \frac{v_R - v_L}{L}$$

The Gazebo `DiffDrive` plugin uses these principles. Our `waypoint_follower.py` node publishes a `Twist` message containing the desired linear velocity ($v$) and angular velocity ($\omega$) to the `/cmd_vel` topic. The plugin then calculates the required individual wheel velocities to achieve this motion.

### Cubic Spline Interpolation

To create a smooth, continuous path from a discrete set of waypoints, we use cubic spline interpolation. A spline is a special function defined piecewise by polynomials.

Given a set of waypoints $(x_i, y_i)$, the algorithm generates two cubic splines, one for the x-coordinates and one for the y-coordinates, both parameterized by a variable $t$ (in this case, the index of the waypoint).

  - $x(t) = a_i t^3 + b_i t^2 + c_i t + d_i$
  - $y(t) = a'_i t^3 + b'_i t^2 + c'_i t + d'_i$

The `scipy.interpolate.CubicSpline` function calculates the polynomial coefficients for each segment between waypoints, ensuring that the resulting curve is continuous and has continuous first and second derivatives. This guarantees a path with smooth curvature, which is ideal for a robot to follow.

In the `waypoint_follower.py` script, the `interpolate_path` function implements this:

1.  It takes an array of coarse `waypoints`.
2.  It creates two splines: `cs_x` for the x-coordinates and `cs_y` for the y-coordinates.
3.  It evaluates these splines at a much finer interval (`step=0.05`) to generate a dense set of points, creating the `smooth_path`.

-----

## 📂 Project Structure

```
gazebo_differential_drive_robot/
├── config/
│   └── gz_bridge.yaml        # ROS-Gazebo bridge configuration
├── launch/
│   └── robot.launch.py       # Main launch file
├── model/
│   ├── robot.xacro           # Robot model definition
│   └── view_robot.rviz       # RViz2 configuration
├── scripts/
│   └── waypoint_follower.py  # Path following logic
├── package.xml
└── setup.py
```

-----

## 🔧 Prerequisites

  - **ROS 2** (Humble Hawksbill recommended)
  - **Gazebo** (installed with ROS 2)
  - **Python packages**: `numpy`, `scipy`

<!-- end list -->

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
pip install numpy scipy
```

-----

## 🛠️ Installation

1.  Clone this repository into your ROS 2 workspace's `src` folder:

    ```bash
    cd ~/ros2_ws/src
    git clone <your-repo-url>
    ```

2.  Build the package:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select gazebo_differential_drive_robot
    ```

3.  Source the workspace:

    ```bash
    source install/setup.bash
    ```

-----

## 🚀 Usage

You'll need two terminals for this process. Make sure to source your workspace in each one.

**Terminal 1: Launch the Simulation**

This command starts Gazebo, spawns the robot, launches RViz, and starts all the necessary nodes (like `robot_state_publisher` and the ROS-Gazebo bridge).

```bash
ros2 launch gazebo_differential_drive_robot robot.launch.py
```

After launching, you should see the robot in an empty Gazebo world and in RViz.

**Terminal 2: Run the Waypoint Follower**

This command starts the Python script that calculates the trajectory and sends velocity commands to the robot.

```bash
ros2 run gazebo_differential_drive_robot waypoint_follower.py
```

The robot will immediately start moving along the path defined in the script. You can see the planned trajectory, waypoints, and the robot's path visualized in RViz.

-----

## 💻 Code Explanation

### `robot.xacro`

This file defines the robot's physical properties using the `xacro` format.

  - **Links**: Defines the geometry (`<visual>`), physical properties (`<inertial>`), and collision boundaries (`<collision>`) for each part of the robot: `base_link` (chassis), `left_wheel_link`, `right_wheel_link`, and `caster_link`.
  - **Joints**: Connects the links. The `left_wheel_joint` and `right_wheel_joint` are `continuous` to allow for infinite rotation. The `caster_joint` is `fixed`.
  - **Gazebo Plugins**:
      - `gz::sim::systems::DiffDrive`: This is the core plugin that subscribes to `/cmd_vel` (a `Twist` message) and drives the wheel joints to achieve the desired motion.
      - `gz::sim::systems::JointStatePublisher`: Publishes the state of the wheel joints to the `/joint_states` topic, which is used by `robot_state_publisher` to update the robot's TF tree.

### `robot.launch.py`

This Python launch file orchestrates the entire simulation setup.

  - **Gazebo Launch**: Includes the main Gazebo launch file (`gz_sim.launch.py`).
  - **Spawn Node**: The `create` node from `ros_gz_sim` spawns the robot model (loaded from the processed `robot.xacro`) into the simulation.
  - **Robot State Publisher**: Publishes the robot's TF tree based on the `/joint_states` topic. This allows RViz to know where each part of the robot is.
  - **Gazebo Bridge**: The `parameter_bridge` node translates messages between ROS 2 and Gazebo (e.g., odometry, clock).
  - **RViz Node**: Launches RViz2 with a pre-configured view (`view_robot.rviz`).

### `waypoint_follower.py`

This is the "brain" of the robot's navigation.

  - **Initialization**:
      - Sets up publishers for `/cmd_vel` (to move the robot) and several visualization topics (`/waypoints_markers`, `/path`).
      - Subscribes to `/odom` to get the robot's current position and orientation.
      - Defines a set of coarse `waypoints`.
  - **`interpolate_path()`**: Uses `scipy.interpolate.CubicSpline` to generate a smooth, dense path from the waypoints.
  - **`control_loop()`**: This is the main logic, executed by a timer:
    1.  **Get State**: Reads the robot's current position (`px`, `py`) and orientation (`yaw`).
    2.  **Find Lookahead Point**: Instead of targeting the next waypoint directly, it finds a `lookahead_point` a fixed distance ahead on the smoothed path. This prevents jerky movements.
    3.  **Calculate Errors**: It computes the distance to the lookahead point and the heading error (the difference between the robot's current `yaw` and the `target_yaw` needed to face the lookahead point).
    4.  **Proportional Control**: It calculates `linear_vel` and `angular_vel` based on the errors. The speed is reduced when turning sharply for better stability.
    5.  **Publish Command**: A `Twist` message with the calculated velocities is published to `/cmd_vel`.
  - **Termination**: Once the robot is close enough to the final waypoint, it stops and prints a completion message.
