# Design notes

How the mission works, why it is built this way, and what had to be changed to make manipulation work in Gazebo Classic.

## Mission sequence

`mobile_robot_bringup/scripts/mobile_manipulation_mission.py` runs:

1. **Wait:** until Nav2's localization and navigation lifecycle managers report *active*, and the arm and gripper controllers are available.
2. **Prepare:** arm to `home`, gripper open.
3. **Drive:** Nav2 goes to a staging pose 0.70 m in front of the **pickup pedestal**, facing it.
4. **Dock with the LiDAR:**
   - **localization check:** the LiDAR must see a face about 0.25 m wide at about 0.70 m, otherwise the mission stops before the arm moves
   - fit a line to the pedestal face
   - correct any sideways offset (turn 90°, drive, turn back)
   - **look for the cube** with the camera while still 0.7 m away (see below) and remember its position in the odom frame
   - square up and creep to 0.17 m from the face
5. **Pick:**
   - **Convert the remembered cube position** into the robot frame. Check it is graspable: on the pedestal, front face at most 2.5 cm behind the edge.
   - **Solve the arm joint angles** with analytic inverse kinematics. Joint 1 turns toward any sideways offset.
   - **Pre-grasp:** open the gripper and move to a point 7 cm in front of the cube. This is an even joint move; the fingertip path is checked in advance to stay 2 cm clear of the cube.
   - **Approach:** slide straight in along a line, with IK solved at 10 waypoints. Stop 8 mm short of the cube center and 5 mm above it, which keeps the wide gripper palm in front of the pedestal edge.
   - **Grasp:** close both fingers with 0.2 N each and ask the grasp assist to attach the cube.
   - **Lift and leave:** lift 5 cm, move to the carry pose, back off 0.35 m.
6. **Drive and dock at the drop pedestal** (steps 3–4 again), then **place:**
   - lower the cube straight down onto the pedestal
   - open the gripper, *then* release the cube from the grasp assist (the other order made the physics throw the cube)
   - retreat in a straight line and back off

The mission uses simulation time, so its timeouts hold when the simulation runs slower than real time.

## Cube detection

`mobile_robot_perception/scripts/cube_detector.py`, with an RGB-D camera added at the TurtleBot3 Waffle Pi camera position (about 10 cm high, facing forward, 640×480, 62° field of view).

- **Where it looks from:** from 0.7 m away the camera sees the cube's front face above the pedestal edge. When docked (0.17 m) the cube is above the camera's view, so the mission looks at the staging pose and remembers the position in the `odom` frame. Odometry drift over the final 0.5 m is negligible.
- **Pipeline, per frame:**
  1. HSV threshold for saturated red (two hue bands), plus a morphological opening.
  2. Largest blob above a minimum size = the cube's visible front face.
  3. Median depth over the blob = distance to the front face.
  4. Back-project the blob centroid with the camera intrinsics, then move half a cube further along the viewing direction to get the center.
  5. Transform to `odom` and publish `/cube_detection/position`, plus `/cube_detection/debug_image` with the detection drawn on it.
- **Mission:** takes the median of at least 5 detections while standing still and only accepts positions in the `odom` frame.
- **Accuracy:** with the cube placed randomly (±5 cm sideways, ±15° turned), the estimate was within 1–3 mm of the true position in every run.
- **Transform timing:** the robot-position transform can lag the camera by a few frames when the simulation is slow. The detector's TF listener runs in its own thread, and if the exact-time transform is not available it uses the latest one. That is exact while the robot stands still, which is the only time the mission uses detections.

## Why two-stage parking

- **Nav2's job:** get the robot within about 10 cm of the right spot. It can't do better here, because a diff-drive robot cannot slide sideways, and the local controller tends to stall near goals close to obstacles.
- **The docking step's job:** Nav2 only brings the robot to open space near the pedestal, and docking does the last half metre with centimetre accuracy.
- **Controller:** Nav2 uses Rotation Shim + Regulated Pure Pursuit. The earlier DWB controller often stopped 0.15–0.3 m short of goals.

## Why the arm avoids MoveIt near the cube

MoveIt's sampling planner (RRTConnect) doesn't know about the cube and pedestal, and in testing it once swung the open gripper into the cube. Near the pedestals, every arm move is therefore deterministic:

- **straight Cartesian lines** for approach, lift, lower and retreat
- **evenly interpolated joint moves**, with a fingertip path check, for moves to and from pre-grasp

MoveIt is only used for the first move to `home`, far from any object.

## Grasp assist (simulation only)

**Problem:**
- The OpenMANIPULATOR-X finger links in the ROBOTIS model weigh only 1 g, and Gazebo Classic moves the arm joints by position.
- When the fingers squeezed the cube, the ODE physics solver became unstable and threw the cube away.
- In one test a finger was reported at 0.077 m on a joint limited to 0.019 m.

**Workaround:** the usual Gazebo Classic fix is a grasp plugin (the well-known GraspFix plugin works the same way). This project has its own small one, `mobile_robot_description/src/grasp_assist_plugin.cpp`:

- **Services:** `/grasp_assist/attach` and `/grasp_assist/detach` (`std_srvs/Trigger`).
- **Check before attaching:** `attach` first checks that the cube is really between the jaws, within ±3 cm / ±1.5 cm / ±2 cm of the grasp point in the gripper frame. Only then does it fix the cube to the gripper link; otherwise it reports a failed grasp.
- **What it replaces:** the robot still has to drive to the right place, aim the arm correctly and close the fingers on the cube. Only the "holding friction" is replaced by a fixed joint. On a real robot this step would not be needed.

## Changes to the ROBOTIS setup

The ROBOTIS `turtlebot3_manipulation` package is used unmodified. The project uses its own copies of two files:

- `mobile_robot_description/urdf/mobile_robot.ros2_control.xacro`
- `mobile_robot_description/config/gazebo_controllers.yaml`

Changes in those copies:

- **Gripper control:** both fingers are effort-controlled by one `JointGroupEffortController` (`/gripper_controller/commands`, `std_msgs/Float64MultiArray`, `[left, right]` in N; negative closes). Position-controlled fingers are teleported by Gazebo and knocked the cube off the pedestal.
- **No mimic joint:** the right finger is no longer a mimic joint. In Humble's `gazebo_ros2_control` the mimic only copies position commands, so with effort control the right finger went limp.
- **MoveIt:** MoveIt therefore no longer lists a gripper controller. The arm is still planned through MoveIt for the `home` move; the mission commands the gripper directly.

## Startup localization

The robot always spawns at the map origin, so AMCL sets its own initial pose (0, 0, 0) when it activates (`set_initial_pose` in `nav2_params.yaml`). An earlier helper node sent `/initialpose` on a timer instead.

## Middleware

With the default Fast DDS, Nav2/AMCL sometimes never finished activating on a loaded machine, because lifecycle service replies were lost. Cyclone DDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) fixed this.

## World and map

`mobile_robot_description/worlds/mobile_robot.world`:

- **Arena:** 6 m × 5 m with walls and three box obstacles.
- **Pickup pedestal:** 0.25 × 0.25 × 0.18 m at (2.30, −0.30).
- **Drop pedestal:** same size at (−2.30, 0.0).
- **Cube:** 4 cm red cube, 30 g, on the pickup pedestal, 1 cm behind its front face. The gripper opens to about 8.8 cm and closes to about 3 cm.

The saved map (`mobile_robot_navigation/maps/mobile_robot_map.*`) was built with `slam_toolbox`. The pedestals were later drawn into the map image by hand, when they replaced earlier, larger tables.

## Main interfaces

| Interface | Type |
|---|---|
| `/cmd_vel`, `/odom`, `/scan`, `/joint_states` | topics |
| `/navigate_to_pose` | Nav2 action |
| `/move_action` | MoveIt MoveGroup action |
| `/arm_controller/follow_joint_trajectory` | arm trajectory action |
| `/gripper_controller/commands` | gripper effort command (`Float64MultiArray`) |
| `/grasp_assist/attach`, `/grasp_assist/detach` | grasp assist services (simulation only) |

## Development history

| Phase | Content |
|---|---|
| 1 | TurtleBot3 Waffle Pi base, diff drive, sensor frames, stable contacts |
| 2 | OpenMANIPULATOR-X + gripper, `ros2_control`, arm/gripper test |
| 3 | MoveIt 2 configuration, plan + execute in Gazebo |
| 4 | Simulated LiDAR, SLAM, AMCL, Nav2 |
| 5 | Unified bringup, scripted mission |
| 6 | Pedestals, Regulated Pure Pursuit, LiDAR docking |
| 7 | Real pick and place: cube-relative IK targets, effort gripper, grasp assist |
| 8 | Robustness from repeated testing, demo video |
| 9 | Startup localization check, CI build check |
| 10 | RGB-D camera, cube detection, grasp at the detected position |
