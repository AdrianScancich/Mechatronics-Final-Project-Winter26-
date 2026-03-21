# Repository Map

---

## Overview

This repository is organized into a modular structure separating hardware drivers, task-based control logic, and a central scheduler. The system is built around `main.py`, which initializes all components and runs the robot.

---

## File Structure

### Main Program
- main.py                       - Initializes drivers, tasks, shares, and starts the scheduler

---

### Tasks (Control + Behavior)
- task_course_navigator.py      - High-level course logic and state machine
- task_line_follower.py         - Line detection and path tracking
- task_state_estimator.py       - Position and heading estimation
- task_trajectory_calculator.py - Path following using predefined trajectories
- task_motor.py                 - Closed-loop motor control
- task_bumper.py                - Bump sensor handling
- task_user.py                  - User interface and system control

---

### Drivers (Hardware Interface)
- motor_driver.py               - Motor PWM and direction control
- encoder_driver.py             - Encoder position and velocity measurement
- line_sensor_driver.py         - Reflectance sensor readings
- imu_driver.py                 - IMU communication and heading data
- voltage_div_driver.py         - Battery voltage measurement
- imu_calibration.py            - IMU calibration loading

---

### Utilities
- pid_controller.py             - PID control implementation
- cotask.py                     - Task scheduler
- task_share.py                 - Shares and queues for communication

---

## Summary

The repository is structured to separate hardware interaction, control logic, and system coordination. This modular design allows each component to be developed and tested independently while supporting reliable real-time operation.

---

## Site Navigation

- [Home](index.md)
- [Hardware](hardware.md)
- [Software Architecture](software.md)
- [Results](results.md)
