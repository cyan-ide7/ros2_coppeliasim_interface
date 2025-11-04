#  CoppeliaSim + ROS 2 Integration Guide (Ubuntu)

This guide explains **how to connect CoppeliaSim** (as a visual simulator, like Gazebo)  
with **ROS 2** (for control logic and computation).  
It includes **first-time setup**, **launch steps**, and **reusable commands** for future sessions.

---

##  Overview

- **CoppeliaSim** acts as a physics/visualization engine.
- **ROS 2** handles robot logic, planning, and control.
- Communication happens through the **simROS2 plugin**, which lets CoppeliaSim
  publish/subscribe ROS 2 topics (e.g., `/cmd_vel`, `/odom`, `/scan`).

---

##  1. Initial Setup (First Time Only)

### Install ROS 2

Example for ROS 2 Humble:
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

Create a workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

### Install CoppeliaSim

Download the latest **CoppeliaSim for Ubuntu** from  
[https://www.coppeliarobotics.com/downloads](https://www.coppeliarobotics.com/downloads)

Extract it:
```bash
cd ~/Downloads
tar xf CoppeliaSim_Edu_V4_x_x_Ubuntu22_04.tar.xz
mv CoppeliaSim_Edu_V4_x_x ~/CoppeliaSim
```

Test it:
```bash
cd ~/CoppeliaSim
./coppeliaSim.sh
```

---

### Build or Copy the ROS 2 Plugin

#### Option A: Use precompiled plugin
Check inside:
```
~/CoppeliaSim/compiledROSPlugins/
```

Copy the file to the root folder:
```bash
cp ~/CoppeliaSim/compiledROSPlugins/libsimROS2.so ~/CoppeliaSim/
```

#### Option B: Build from source (if not included)
```bash
cd ~/ros2_ws/src
git clone https://github.com/CoppeliaRobotics/simROS2.git
export COPPELIASIM_ROOT_DIR=~/CoppeliaSim
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

##  2. Launch Procedure (Every Time You Start)

### Step 1: Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export COPPELIASIM_ROOT_DIR=~/CoppeliaSim
```

### Step 2: (Optional) Launch ROS 2 Nodes
```bash
ros2 launch your_package your_launch_file.launch.py
```

### Step 3: Start CoppeliaSim
```bash
cd ~/CoppeliaSim
./coppeliaSim.sh
```

 You should see:(KEEP IN MIND YOU MAY NEED TO ADD A LUA OR PYTHON SCRIPT FOR CALLING THE ROS2 PLUGIN THEN ONLY IT WILL SHOW IN THE CONSOLE)
```
[CoppeliaSim:loadinfo] plugin 'ROS2': loading... done.
```

---

##  3. Verify Connection

After starting simulation in CoppeliaSim:

```bash
ros2 node list
ros2 topic list
```

You should see topics like `/sim_ros2_interface`, `/odom`, `/cmd_vel`, etc.

---

## 🤖 4. Integrating with Your Scene

### Convert Lua Logic → ROS 2 Topics

1. **Remove** control logic from Lua scripts in CoppeliaSim.  
2. **Keep** only ROS 2 publishers/subscribers.  
3. **Move** control code to ROS 2 nodes.

Example (CoppeliaSim side):
```lua
simROS2 = require('simROS2')

function sysCall_init()
    leftMotor = sim.getObjectHandle('left_motor')
    rightMotor = sim.getObjectHandle('right_motor')
    subCmdVel = simROS2.createSubscription('/cmd_vel', 'geometry_msgs/msg/Twist', 'cmdCallback')
end

function cmdCallback(msg)
    sim.setJointTargetVelocity(leftMotor, msg.linear.x - msg.angular.z)
    sim.setJointTargetVelocity(rightMotor, msg.linear.x + msg.angular.z)
end
```

Example (ROS 2 side):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
```

---

##  5. Automation Script (Optional)

Create a helper script `start_sim.sh`:

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export COPPELIASIM_ROOT_DIR=~/CoppeliaSim
cd ~/CoppeliaSim
./coppeliaSim.sh
```

Make it executable:
```bash
chmod +x ~/start_sim.sh
```

Run with:
```bash
~/start_sim.sh
```

---

##  6. Quick Recap

| Step | Action | Command |
|------|---------|----------|
| 1 | Source ROS 2 environment | `source /opt/ros/humble/setup.bash` |
| 2 | Launch ROS 2 nodes (optional) | `ros2 launch your_pkg your_launch.py` |
| 3 | Start CoppeliaSim | `./coppeliaSim.sh` |
| 4 | Open your scene & press  |  |
| 5 | Verify topics | `ros2 topic list` |

---

## Final Notes

- Always launch CoppeliaSim **after** sourcing ROS 2.
- The message:  
  ```
  [CoppeliaSim:loadinfo] plugin 'ROS2': loading... done.
  ```
  confirms successful connection.
- Use `ros2 topic echo /odom` or `rqt_graph` to confirm data exchange.
- For visualization only, keep physics enabled but run logic in ROS 2.

---

###  References

- [CoppeliaSim ROS2 Interface Manual](https://manual.coppeliarobotics.com/en/ros2Interface.htm)
- [CoppeliaSim ROS2 Tutorial](https://manual.coppeliarobotics.com/en/ros2Tutorial.htm)
- [simROS2 GitHub Repo](https://github.com/CoppeliaRobotics/simROS2)
- [Example Blog Tutorial](https://blog.ddavo.me/posts/tutorials/ros2-coppelia-lidar/)

---

 **You’re all set!**  
Next time:  
1. Source ROS 2.  
2. Run `./coppeliaSim.sh`.  
3. Open your scene → press *Play*.  
CoppeliaSim connects automatically with ROS 2.
