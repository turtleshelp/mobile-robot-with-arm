# Results

All tests ran in simulation (Gazebo Classic 11, ROS 2 Humble, Cyclone DDS), headless, on an 8-core laptop. The simulation ran at about 0.3–0.5× real time there.

A run counts as a **success** only if Gazebo shows the cube upright on the drop pedestal afterwards, not merely because the mission printed `MISSION COMPLETE`.

## Repeatability

Five full missions in a row, each in a freshly started simulation:

| Trial | Result | Cube vs. grasp point at grasp | Cube final (x, y) |
|---|---|---|---|
| 1 | ✅ | 1.0 cm | (−2.205, +0.002) |
| 2 | ✅ | 0.8 cm | (−2.205, +0.008) |
| 3 | ❌ localization error | n/a | still on the pickup pedestal |
| 4 | ✅ | 1.2 cm | (−2.203, −0.009) |
| 5 | ✅ | 0.9 cm | (−2.204, −0.005) |

That gives **4 of 5 successful**, with no failed grasps. The grasp offset is mostly the intended 8 mm stop-short.

In trial 3, AMCL localized the robot wrongly at startup, so it docked against the wrong surface. The grasp check found no cube between the jaws and stopped the mission, and the cube was untouched. The cause is still under investigation. AMCL received its initial pose unusually early (simulation time 1.4 s), while TF was still extrapolating.

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
