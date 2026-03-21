# Mechatronics Final Project - Winter 26
Authors: Tyler Moyers and Adrian Scancich  
  
This repository contains the software for an autonomous Romi robot developed over a full quarter in Cal Poly’s ME 405 (Mechatronics) course, which serves as the culmination of the undergraduate mechatronics curriculum. The project integrates concepts from controls, embedded systems, and sensor integration to navigate a complex course using a combination of reflectance sensors, ultrasonic sensors, mechanical switches, IMUs, and encoders. The system is structured around a cooperative, priority-based task scheduler that manages sensing, estimation, and control in real time. Core functionality includes closed-loop motor control, line following, state estimation using encoder and IMU data, and higher level course navigation with trajectory execution. The architecture separates low level hardware drivers from higher level control and navigation tasks to maintain modularity. Communication between tasks is handled through shared variables and queues, allowing reliable data exchange without tight coupling. Overall, the system follows a layered control structure where velocity control supports line following trajectory tracking under a state-driven navigation framework.

Project Website  
Additional documentation, videos, and results can be found here:  

## Repository Overview
- `main.py` – System entry point and scheduler initialization  
- `task_*.py` – Core control tasks  
- `*_driver.py` – Hardware interface drivers  
- `cotask.py`, `task_share.py` – Scheduler and communication tools  
- `docs/` – Website files (GitHub Pages)  
- `media/` – Images, videos, and plots  

---

## Repository Map

```text
main.py
  Initializes all drivers, shares, and tasks. Runs the full system.

Core Tasks
  task_course_navigator.py      - High-level course logic
  task_line_follower.py         - Line tracking (PID)
  task_state_estimator.py       - Position and heading estimation
  task_trajectory_calculator.py - Path following
  task_motor.py                 - Motor control (PID)
  task_user.py                  - User interface
  task_bumper.py                - Bumper interrupts

Drivers
  motor_driver.py               - Motor control (PWM + direction)
  encoder_driver.py             - Wheel position/velocity
  line_sensor_driver.py         - Line sensor readings
  imu_driver.py                 - IMU data
  voltage_div_driver.py         - Battery measurement

Support
  pid_controller.py             - PID implementation
  cotask.py                     - Task scheduler
  task_share.py                 - Shares and queues

Other
  calibration.txt               - IMU calibration data
  Lab reports                   - Development documentation
```
