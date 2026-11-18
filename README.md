# PA2 – Navigation and Environmental Mapping for an Autonomous Mobile Robot

Project work 2, ZHAW (ADLS), Tim Badertscher. Supervisor: Christian Glahn, EDXI Lab.

Goal: control of a Waveshare RaspRover with an RPLIDAR A2M12, so that the robot creates a map with ROS2 and drives to goals in it on its own.

**Contents:** [Robot commands](#robot-commands) · [Troubleshooting](#troubleshooting) · [Demo workflow](#demo-workflow) · [Compiling the report](#compiling-the-report) · [Folder structure](#folder-structure)

---

## Robot commands

### 1. Connection

| Network | JupyterLab | Foxglove |
|---|---|---|
| Known Wi-Fi (at home) | `http://192.168.1.123:8888` | `ws://192.168.1.123:8765` |
| No known Wi-Fi → Pi opens hotspot **"AccessPopup"** | `http://192.168.50.5:8888` | `ws://192.168.50.5:8765` |

The hotspot only opens if no known Wi-Fi is in range. Connect the laptop to the hotspot first.

Every new terminal in JupyterLab has to enter the container first:

```bash
docker exec -it ros2 bash
```

### 2. Start everything (terminal 1)

```bash
ros2 launch rasprover_bridge bringup.launch.py
```

- **Keep the robot still for the first seconds** (gyroscope calibration).
- Wait about **20 s** until Nav2 is ready (SLAM Toolbox starts after 5 s, Nav2 after 15 s).
- **Ctrl+C** stops everything, the motors stop.

Variants:

```bash
ros2 launch rasprover_bridge bringup.launch.py nav:=false        # only robot + SLAM (mapping)
ros2 launch rasprover_bridge bringup.launch.py foxglove:=false   # without Foxglove bridge
ros2 launch rasprover_bridge robot.launch.py                     # only robot (bridge, lidar, URDF, Foxglove)
ros2 launch rasprover_bridge bringup.launch.py 2>&1 | tee /ros2_ws/nav_log.txt   # with log file
```

### 3. Foxglove (laptop)

1. Open the **desktop app** (the web version cannot connect to `ws://`).
2. Open connection → **Foxglove WebSocket** → address from the table above.
3. In the 3D panel, show: `/map`, `/scan`, `/plan`, `/local_costmap/costmap`, `/explore/frontiers`.
4. **Set a goal:** panel settings → Publish → enter the topic `/goal_pose` for **"2D pose"** (not for "2D pose estimate"). Then select the publish tool, click on the map and drag in the target direction.

### 4. Autonomous exploration (terminal 2)

```bash
source /ros2_ws/install/setup.bash
ros2 launch explore_lite explore.launch.py
```

The robot drives to the borders of the known map on its own and stops when nothing is left to explore.

**Close off open doors and drops** with an object that is higher than 20 cm and not black and matte. The lidar only measures in one plane at 19 cm and does not see door thresholds, steps or low obstacles.

### 5. Other commands

```bash
# Save the map (before Ctrl+C, otherwise it is lost)
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/<name>

# Drive by hand: i forward, , backward, j/l turn, k stop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Constant rotation for testing (stop with Ctrl+C)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.6}}"
```

### 6. Checks

```bash
ros2 daemon stop && ros2 node list           # running nodes (restart the daemon if the list is incomplete)
ros2 topic hz /scan                          # lidar, should be about 10 Hz
ros2 topic hz /odom                          # odometry, should be about 20 Hz
ros2 run tf2_ros tf2_echo map base_link      # is localisation ready?
ros2 topic echo /battery_state --field voltage
```

Log analysis:

```bash
grep -E "Begin navigating|Goal succeeded|Goal failed|Failed to make progress|Running|missed|ERROR" /ros2_ws/nav_log.txt
```

### 7. Start step by step (for troubleshooting, one terminal each)

```bash
ros2 launch rasprover_bridge robot.launch.py
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/ros2_ws/config/slam.yaml
ros2 run tf2_ros tf2_echo map base_link        # wait until a transform appears, then Ctrl+C
ros2 launch nav2_bringup navigation_launch.py params_file:=/ros2_ws/config/nav2.yaml
```

### 8. Container and build

On the Pi (not inside the container):

```bash
docker start ros2        # start the container after a reboot
docker restart ros2      # restart if something hangs
docker ps                # is the container running?
```

After code changes (inside the container):

```bash
cd /ros2_ws
colcon build --packages-select rasprover_bridge
source install/setup.bash
```

Changes in `/ros2_ws/config/slam.yaml` or `nav2.yaml` do not need a build, only a restart.

Uploading files: JupyterLab only shows `~/ugv_rpi/`. Uploaded files end up there and have to be moved in the terminal (on the Pi), e.g. `mv ~/ugv_rpi/<file> ~/ros2_ws/...`. To download, first copy the file to `~/ugv_rpi/`.

---

## Compiling the report

```bash
latexmk -pdf main.tex
```

In VS Code with LaTeX Workshop, saving compiles automatically. Requires TeX Live with the collections `collection-latexrecommended`, `collection-latexextra`, `collection-fontsrecommended`, `collection-langenglish`, `collection-pictures`, `collection-mathscience`.

---

## Folder structure

```
main.tex                  main file (settings, order of the chapters)
references.bib            literature
frontmatter/              title page, imprint, abstract, abbreviations
chapters/                 chapters 1–5 (introduction, theory, methods, results, discussion)
appendix/                 appendices A–E (AI declaration, commands, raw data, code, declaration of originality)
figures/                  figures (images and TikZ)
code/                     code for the appendix (bridge node, URDF, launch files)
```

The robot code is also in the repository `farmbot-zhaw-upm/Autonomous-Agricultural-Robot`.
