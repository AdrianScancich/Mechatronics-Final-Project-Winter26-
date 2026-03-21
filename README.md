# Mechatronics Final Project - Winter 26
By: Tyler Moyers and Adrian Scancich  
  
This repository contains the software for an autonomous Romi robot developed over a full quarter in Cal Poly’s ME 405 (Mechatronics) course, which serves as the culmination of the undergraduate mechatronics curriculum. The project integrates concepts from controls, embedded systems, and sensor integration to navigate a complex course using a combination of reflectance sensors, ultrasonic sensors, mechanical switches, IMUs, and encoders. The system is structured around a cooperative, priority-based task scheduler that manages sensing, estimation, and control in real time. Core functionality includes closed-loop motor control, line following, state estimation using encoder and IMU data, and higher level course navigation with trajectory execution. The architecture separates low level hardware drivers from higher level control and navigation tasks to maintain modularity. Communication between tasks is handled through shared variables and queues, allowing reliable data exchange without tight coupling. Overall, the system follows a layered control structure where velocity control supports line following trajectory tracking under a state-driven navigation framework.

Project Website  
Additional documentation, videos, and results can be found here:
