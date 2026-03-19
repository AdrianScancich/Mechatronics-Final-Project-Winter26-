from pyb import Pin, Timer, repl_uart, UART, I2C
from motor_driver           import Motor as motor_driver
from encoder_driver         import Encoder as encoder_driver
from line_sensor_driver     import Line_Sensor as line_sensor_driver
from voltage_div_driver     import Voltage_Divider as voltage_div_driver
from imu_driver             import IMU as imu_driver
from imu_calibration        import full_imu_calib as calibration
from pid_controller         import PID_controller as pid_controller
from task_motor             import task_motor
from task_user              import task_user
from task_share             import Share, Queue, show_all
from cotask                 import Task, task_list
from gc                     import collect
from task_line_follower     import task_line_follower
from task_state_estimator   import task_state_estimator
from task_trajectory_calculator     import task_trajectory_calculator
from task_course_navigator          import task_course_navigator
from task_bumper                import task_bumper

#repl_uart(None) # Disable REPL on UART(0) to free up pins PA3 and PA2 for user input from the UI
#HCBT = UART(1, 115200, timeout=1000) # Initialize UART 1 for Bluetooth communication with the UI

# Build all driver objects first
leftMotor    = motor_driver(Pin('PB5'), Pin('PB13'), Pin('PB14'), Timer(3, freq=20000), 2)
rightMotor   = motor_driver(Pin('PB0'), Pin('PH1'), Pin('PH0'), Timer(3, freq=20000), 3)
leftEncoder  = encoder_driver(Timer(1, period = 0xFFFF, prescaler = 0), 'PA8', 'PA9')
rightEncoder = encoder_driver(Timer(2, period = 0xFFFF, prescaler = 0), 'PA0', 'PA1')
left_motor_pid_control  = pid_controller()
right_motor_pid_control = pid_controller()
left_line_sensor = line_sensor_driver(Pin('PA7'), Pin('PA6'), Pin('PC4'))
right_line_sensor = line_sensor_driver(Pin('PC0'), Pin('PC3'), Pin('PC2'))
Romi_Battery_Voltage = voltage_div_driver(Pin('PA4'))  
Romi_IMU = imu_driver(I2C(1, I2C.CONTROLLER))
line_follower_PID_control = pid_controller()
Romi_heading_PID_control = pid_controller()

# Build shares and queues
leftMotorGo         = Share("B",        name="Left Mot. Go Flag")
rightMotorGo        = Share("B",        name="Right Mot. Go Flag")
RomiVelocity        = Share("f",        name="Romi Velocity Magnitude")
leftVelocity        = Share("f",        name="Left Velocity")
rightVelocity       = Share("f",        name="Right Velocity")
lineFollowerGo      = Share("B",        name="Line Follower Go Flag")
junctionFlag        = Share("B",        name="Line Follower Junction Flag")
stateEstimatorGo    = Share("B",        name="State Estimator Go Flag")
trajectoryGo        = Share("B",        name="Trajectory Calculator Go Flag")
trajDataset         = Share("f",        name="Trajectory Dataset Pointer")
courseNavigatorGo   = Share("B",        name="Course Navigator Go Flag")
crashDetect         = Queue("B", 8, name="Crash Detect Queue")

# State Estimator Shares
X0_s                = Share("f",        name="State Estimator s Buffer in state vector")
X1_psi              = Share("f",        name="State Estimator psi Buffer in state vector")
X2_omega_L          = Share("f",        name="State Estimator omega_L Buffer in state vector")
X3_omega_R          = Share("f",        name="State Estimator omega_R Buffer in state vector")

SE_timeValue        = Share("f",        name="State Estimator Time Value")
X_global            = Share("f",        name="State Estimator Global location on X axis")
Y_global            = Share("f",        name="State Estimator Global location on Y axis")

# Build task class objects
leftMotorTask       = task_motor(leftMotor,  leftEncoder, left_motor_pid_control,
                                leftMotorGo, leftVelocity)
rightMotorTask      = task_motor(rightMotor, rightEncoder, right_motor_pid_control,
                                rightMotorGo, rightVelocity)
userTask            = task_user(X1_psi, X_global, Y_global,
                                Romi_Battery_Voltage, courseNavigatorGo)
linefollowertask    = task_line_follower(lineFollowerGo, left_line_sensor, right_line_sensor, leftEncoder, rightEncoder, line_follower_PID_control,
                                RomiVelocity, leftVelocity, rightVelocity, junctionFlag)
stateestimatorTask  = task_state_estimator(stateEstimatorGo, leftMotor, rightMotor, leftEncoder, rightEncoder, Romi_Battery_Voltage, Romi_IMU,
                                X0_s, X1_psi, X2_omega_L, X3_omega_R, SE_timeValue, X_global, Y_global)
trajectoryCalculatorTask = task_trajectory_calculator(trajectoryGo, 
                                                      X1_psi, X_global, Y_global, trajDataset, RomiVelocity, leftVelocity, rightVelocity)
courseNavigatorTask = task_course_navigator(courseNavigatorGo, X0_s, X1_psi, X2_omega_L, X3_omega_R, SE_timeValue, 
                                      trajDataset, leftMotorGo, rightMotorGo, lineFollowerGo, stateEstimatorGo, trajectoryGo, RomiVelocity, 
                                      leftVelocity, rightVelocity, X_global, Y_global, Romi_heading_PID_control, crashDetect, junctionFlag)
bumperTask = task_bumper(crashDetect)


# Add tasks to task list
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 1, period = 20, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 1, period = 20, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 0, profile=False))
task_list.append(Task(linefollowertask.run, name="Line Follower Task",
                      priority = 2, period = 20, profile=True))
task_list.append(Task(stateestimatorTask.run, name="State Est. Task",
                      priority = 4, period = 20, profile=True))
task_list.append(Task(trajectoryCalculatorTask.run, name="Trajectory Calc. Task",
                      priority = 2, period = 50, profile=True))
task_list.append(Task(courseNavigatorTask.run, name="Course Nav. Task",
                      priority = 3, period = 20, profile=True))
task_list.append(Task(bumperTask.run, name="Bumper Task",
                      priority = 3, period = 20, profile=True))

# Run the garbage collector preemptively
collect()
calibration(Romi_IMU)

# Run the scheduler until the user quits the program with Ctrl-C
while True:
    try:
        task_list.pri_sched()
        
    except KeyboardInterrupt:
        print("Program Terminating")
        leftMotor.disable()
        rightMotor.disable()
        break

print("\n")
print(task_list)
print(show_all())