from pyb import Pin, Timer, repl_uart, UART, I2C
#from time import sleep_ms
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
Romi_velocity_pid_control = pid_controller() 
Romi_Battery_Voltage = voltage_div_driver(Pin('PA4'))  
Romi_IMU = imu_driver(I2C(1, I2C.CONTROLLER))

# Build shares and queues
leftMotorGo         = Share("B",        name="Left Mot. Go Flag")
rightMotorGo        = Share("B",        name="Right Mot. Go Flag")
leftdataValues      = Queue("f", 150,   name="Left Data Collection Buffer")
rightdataValues     = Queue("f", 150,   name="Right Data Collection Buffer")
lefttimeValues      = Queue("f", 150,   name="Left Time Buffer")
righttimeValues     = Queue("f", 150,   name="Right Time Buffer")
centroidValues      = Queue("f", 150,   name="Centroid Buffer")
centroidtimeValues  = Queue("f", 150,   name="Centroid Time Buffer")
RomiVelocity        = Share("f",        name="Romi Velocity Magnitude")
leftVelocity        = Share("f",        name="Left Velocity")
rightVelocity       = Share("f",        name="Right Velocity")
Kp                  = Share("f",        name="PID Kp Value")
Ki                  = Share("f",        name="PID Ki Value")
Kd                  = Share("f",        name="PID Kd Value")
lineFollowerGo      = Share("B",        name="Line Follower Go Flag")
LFKp                = Share("f",        name="Line follower PID Kp Value")
LFKi                = Share("f",        name="Line follower PID Ki Value")
LFKd                = Share("f",        name="Line follower PID Kd Value")
stateEstimatorGo    = Share("B",        name="State Estimator Go Flag")

# State Estimator Buffers for 5 seconds of operation
X0_s                = Queue("f", 150,    name="State Estimator s Buffer in state vector")
X1_psi              = Queue("f", 150,    name="State Estimator psi Buffer in state vector")
X2_omega_L          = Queue("f", 150,    name="State Estimator omega_L Buffer in state vector")
X3_omega_R          = Queue("f", 150,    name="State Estimator omega_R Buffer in state vector")

U0_U_L              = Queue("f", 150,    name="State Estimator U_L Buffer in input vector")
U1_U_R              = Queue("f", 150,    name="State Estimator U_R Buffer in input vector")
U2_s_L              = Queue("f", 150,    name="State Estimator s_L Buffer in input vector")
U3_s_R              = Queue("f", 150,    name="State Estimator s_R Buffer in input vector")
U4_psi              = Queue("f", 150,    name="State Estimator psi Buffer in input vector")
U5_psi_dot          = Queue("f", 150,    name="State Estimator psi_dot Buffer in input vector")

Y0_s_L              = Queue("f", 150,    name="State Estimator s_L Buffer in output vector")
Y1_s_R              = Queue("f", 150,    name="State Estimator s_R Buffer in output vector")
Y2_psi              = Queue("f", 150,    name="State Estimator psi Buffer in output vector")
Y3_psi_dot          = Queue("f", 150,    name="State Estimator psi_dot Buffer in output vector")

SE_timeValues       = Queue("f", 150,    name="State Estimator Time Buffer")

# Build task class objects
leftMotorTask       = task_motor(leftMotor,  leftEncoder, left_motor_pid_control,
                                leftMotorGo, leftdataValues, lefttimeValues, leftVelocity, Kp, Ki, Kd)
rightMotorTask      = task_motor(rightMotor, rightEncoder, right_motor_pid_control,
                                rightMotorGo, rightdataValues, righttimeValues, rightVelocity, Kp, Ki, Kd)
userTask            = task_user(leftMotorGo, rightMotorGo, lineFollowerGo, stateEstimatorGo, leftdataValues, rightdataValues, lefttimeValues, righttimeValues, centroidValues, centroidtimeValues,
                                leftVelocity, rightVelocity, RomiVelocity, Kp, Ki, Kd, LFKp, LFKi, LFKd, SE_timeValues,
                                X0_s, X1_psi, X2_omega_L, X3_omega_R,
                                U0_U_L, U1_U_R, U2_s_L, U3_s_R, U4_psi, U5_psi_dot,
                                Y0_s_L, Y1_s_R, Y2_psi, Y3_psi_dot,
                                Romi_Battery_Voltage)
linefollowertask    = task_line_follower(lineFollowerGo, leftMotorGo, rightMotorGo, left_line_sensor, right_line_sensor, leftEncoder, rightEncoder, Romi_velocity_pid_control,
                                RomiVelocity, leftVelocity, rightVelocity, centroidValues, centroidtimeValues, LFKp, LFKi, LFKd)
stateestimatorTask  = task_state_estimator(stateEstimatorGo, leftMotor, rightMotor, leftEncoder, rightEncoder, Romi_Battery_Voltage, Romi_IMU, SE_timeValues,
                                X0_s, X1_psi, X2_omega_L, X3_omega_R,
                                U0_U_L, U1_U_R, U2_s_L, U3_s_R, U4_psi, U5_psi_dot,
                                Y0_s_L, Y1_s_R, Y2_psi, Y3_psi_dot)

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