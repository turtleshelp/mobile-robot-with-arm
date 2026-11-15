# Results

All tests ran in simulation (Gazebo Classic 11, ROS 2 Humble, Cyclone DDS), headless, on an 8-core laptop. The simulation ran at about 0.3–0.5× real time there.

A run counts as a **success** only if Gazebo shows the cube upright on the drop pedestal afterwards, not merely because the mission printed `MISSION COMPLETE`.

## Camera detection with a randomly placed cube

Before each run the cube was put at a random place on the pickup pedestal: up to ±5 cm sideways and up to ±15° turned. The robot had to find it with the camera.

| Run | Cube placement | Camera estimate vs. true position | Result |
|---|---|---|---|
| A | 2.2 cm left, +1° | within 1 mm | ✅ |
| B | 2.8 cm right, −14° | within 3 mm | ✅ |
| C | 1.9 cm right, −7° | within 2 mm | ✅ |
| D | 1.6 cm left, +7° | within 2 mm | ✅ |
| E | 4.4 cm right, +2° | within 1 mm | ✅ |

**Every run where the camera got to look ended with the cube on the drop pedestal.**

Failures on the way there, each stopped safely before the arm moved, then fixed:

- **A leftover test process** published detections in the wrong frame. The "not graspable" check refused to reach 58 cm too far. Fix: the mission only accepts `odom`-frame detections.
- **The localization check once saw the pedestal only 14 cm wide.** It could not be reproduced with the robot standing still at the same poses (always 23–25 cm), so it was most likely a scan taken while the robot was still slowing down. Fix: wait until odometry shows the robot is still, and re-measure before failing.
- **The camera once reported 0 detections.** Every image failed to transform to `odom` ("extrapolation into the future"), because the TF listener shared the thread that was waiting for it. Fix: a separate TF thread, plus a fallback to the latest transform. The verification run after the fix logged 0 transform warnings.

## Repeatability (before camera detection)

Five full missions in a row, each in a freshly started simulation:

| Trial | Result | Cube vs. grasp point at grasp | Cube final (x, y) |
|---|---|---|---|
| 1 | ✅ | 1.0 cm | (−2.205, +0.002) |
| 2 | ✅ | 0.8 cm | (−2.205, +0.008) |
| 3 | ❌ localization error | n/a | still on the pickup pedestal |
| 4 | ✅ | 1.2 cm | (−2.203, −0.009) |
| 5 | ✅ | 0.9 cm | (−2.204, −0.005) |

That gives **4 of 5 successful**, with no failed grasps. The grasp offset is mostly the intended 8 mm stop-short.

In trial 3, AMCL localized the robot wrongly at startup, so it docked against the wrong surface. The grasp check found no cube between the jaws and stopped the mission, and the cube was untouched.

**Follow-up:**
- **Tracking runs:** 5 more full runs logged Gazebo's true robot position next to AMCL's estimate. All 5 succeeded, and AMCL never lost track, so the failure is rare and could not be reproduced. In the failed run, AMCL had received its initial pose unusually early (simulation time 1.4 s), while TF was still extrapolating; that is the most likely cause but is not proven.
- **Two changes in response:**
  - AMCL now sets its own initial pose when it activates (`set_initial_pose`), instead of relying on a separately timed `/initialpose` message.
  - **Localization check:** on arrival in front of each pedestal, the mission checks that the LiDAR sees a face about 0.25 m wide at about 0.70 m. If not, it stops with "Localization check FAILED" before the arm moves. This is the actual safeguard, whatever the cause.

## Parking accuracy

Typical values from the runs above:

| Stop | After Nav2 | After LiDAR docking |
|---|---|---|
| Pickup pedestal | 0.70 m from the face, up to ~5 cm sideways | 0.177–0.180 m, within 2°, within 2 cm sideways |
| Drop pedestal | 0.75 m from the face, up to ~7 cm sideways | 0.178–0.180 m, within 2°, within 2 cm sideways |

## Fresh-clone check

The repository was cloned from GitHub into an empty folder and set up following only the README:

- clone the repo
- clone the ROBOTIS package at the pinned commit
- build, with no settings from the development machine

All 12 packages built, and a full mission completed with the cube on the drop pedestal at (−2.204, −0.006).

## Problems found by repeated testing

Each of these caused at least one lost cube or failed run before it was fixed:

| Problem | Fix |
|---|---|
| Position-controlled fingers are teleported into the cube | effort-controlled fingers |
| Mimic finger went limp under effort control | both fingers driven by one effort controller |
| ODE unstable with 1 g finger links | grasp assist plugin (see [design.md](design.md)) |
| Detaching while squeezing threw the cube | open the gripper first, then detach |
| Gripper palm clipped the pedestal edge | cube 1 cm from the edge; stop 8 mm short, grasp 5 mm high |
| MoveIt's random planner swung the gripper into the cube | straight-line and checked joint moves near the pedestals |
| Timeouts in real time cut moves short on a slow simulation | mission uses simulation time |
| Nav2/AMCL sometimes never activated (Fast DDS) | Cyclone DDS |
| Localization check fooled by a scan taken while still moving | measure only when odometry says still; re-check before failing |
| Detector could not transform images (TF thread blocked) | TF listener in its own thread; fall back to the latest transform |
| TF listener thread crashed rclpy (it spun the mission node from a second thread) | TF listener on its own small node |
| `bt_navigator` failed to activate under load (waits only 1 s for the controller) | `wait_for_service_timeout: 5000` |
