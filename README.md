-----
# ROS2_Pick-and-Place

ROS2 Simulation of a Franka Emika Panda Robot performing a Pick-and-Place Task. The Robot identifies Colored Blocks on a Workbench using a Simulated Kinect Camera, Picks Them Up, and Places them into their Corresponding Colored Bins.

The project utilizes `Gazebo` for the Physical Simulation and `MoveIt2` for Motion Planning and Control.

-----

## System Prerequisites

before you begin, ensure your system is equipped with the following:

  * **Operating System:** Ubuntu 22.04 LTS
  * **ROS2 Version:** ROS2 Humble Hawksbill
  * **Dependencies:**
      * Gazebo (included with `ros-humble-desktop-full`)
      * MoveIt2
      * ROS2 Controllers & Control Toolbox
      * Gazebo-ROS2 Integration Packages
      * `colcon` build tools
      * `rosdep` for dependency management

-----

## Step 1: Installation and Environment Setup

first, you need to set up your ROS2 Workspace and install all the required software packages.

### 1\. Install Core ROS2 Packages

if you haven't already installed ROS2 Humble and its development tools, open a terminal and run the following commands:

```bash
# install ROS2 Humble, Gazebo, and development tools
sudo apt update && sudo apt install -y ros-humble-desktop-full ros-humble-dev-tools

# install MoveIt2 packages
sudo apt install -y ros-humble-moveit

# install ROS2 controllers and Gazebo integration
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros-pkgs

# initialize rosdep
sudo rosdep init
rosdep update
```

### 2\. Create a ROS2 Workspace

create a new directory for your `colcon` workspace.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 3\. Clone the Repositories

clone this project repository and its necessary dependencies (like the Franka ROS2 Description) into your workspace's `src` directory.

```bash
# clone this project
git clone <your_repository_url> src/elena-ecn-pick-and-place

# clone the required Franka ROS2 Repository for Robot Models
git clone --branch humble https://github.com/frankaemika/franka_ros2.git src/franka_ros2
```

### 4\. Install Python Dependencies

this project requires a few Python packages. A `requirements.txt` file is included to make this easy.

```bash
# navigate to the project directory
cd src/elena-ecn-pick-and-place/pick_and_place

# install the required python packages
pip install -r requirements.txt
```

*(Note: You will need to create a `requirements.txt` file inside your `pick_and_place` package with the following content:)*

```
numpy
opencv-python
statemachine
```

### 5\. Install ROS Dependencies and Build

use `rosdep` to install any remaining ROS packages and then build your workspace with `colcon`.

```bash
# navigate back to the root of your workspace
cd ~/ros2_ws

# install all required ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# build the workspace
colcon build --symlink-install
```

**--symlink-install** is recommended as it allows you to change Python Files without needing to rebuild.

-----

## Step 2: Running the Simulation

once the build is complete, you can run the pick-and-place simulation.

### 1\. Source the Workspace

in every new terminal you open, you must source your workspace's setup file to make the ROS2 packages visible.

```bash
source ~/ros2_ws/install/setup.bash
```

**Recommendation:** Add this command to your `~/.bashrc` file to run it automatically for every new terminal.

### 2\. Launch the Project

use the provided launch file to start the Gazebo simulation, spawn the robot, and run the object detection and state machine nodes.

```bash
ros2 launch pick_and_place panda_world.launch.py
```

-----

## License

This project is licensed under the MIT License.
