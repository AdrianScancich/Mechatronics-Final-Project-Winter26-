# Results

---

## Course Navigation Overview

The robot begins the course in line-following mode, using the reflectance sensors to track the path until reaching checkpoint 1, which is identified using the state estimator. At this point, line following is disabled and the trajectory calculator takes control, reducing speed and guiding the robot along a predefined path using a set of predetermined waypoints. Once the trajectory is complete, the robot uses PID control to correct its heading until it is oriented 180 degrees from the start. This ensures that the line sensors approach the garage wall head-on. The robot then continues forward at the corrected heading until contact is detected by the bump sensors. After contacting the wall, the robot reverses and executes a 90-degree left turn. Following this maneuver, the robot resumes motion toward checkpoint 2 using a combination of line following and state estimation. Line sensing is briefly disabled to allow the robot to reorient itself toward the slalom section. Once aligned, the robot enters full line-following mode, navigating through checkpoints 3 and 4 without stopping. After checkpoint 4, the line sensors detect a brief gap in the dashed line. Combined with state estimation, this indicates that the robot has reached the end of the forward course. Line following is then disabled, and the robot executes a 180-degree turn before following a second predefined trajectory back toward the starting position, completing checkpoint 5. Finally, the robot performs a heading correction to return to 0 degrees, placing it in the correct orientation to begin the course again without external intervention.

---

## Demonstration

### ▶️ Watch the Full Course Run

[Watch on YouTube](https://youtu.be/B_vGlNqHEyo)

The robot completed the full course in approximately 40 seconds per run. It was able to successfully complete three consecutive runs without requiring any manual intervention or external reorientation, demonstrating consistent performance and reliable system integration.

---

## Site Navigation

- [Home](index.md)
- [Hardware](hardware.md)
- [Software Architecture](software.md)
- [Repository Map](repo-map.md)
