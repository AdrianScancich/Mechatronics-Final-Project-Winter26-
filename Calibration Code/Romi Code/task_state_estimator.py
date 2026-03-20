#from line_sensor_driver import Line_Sensor as line_sensor_driver
#from pid_controller     import PID_controller as pid_controller
from motor_driver       import Motor as motor_driver
from voltage_div_driver import Voltage_Divider as voltage_div_driver
from encoder_driver     import Encoder as encoder_driver
from imu_driver         import IMU as imu_driver
from pyb                import USB_VCP
from task_share         import Share, Queue
from utime              import ticks_us, ticks_diff
import micropython
from ulab import numpy as np
from array import array

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control

class task_state_estimator:
    '''
    A class that represents a state estimator task. The task is responsible for reading
    data from encoders and estimating the state of the Romi.
    '''

    def __init__(self, stateEstimatorGo: Share, lMotor: motor_driver, rMotor: motor_driver, lenc: encoder_driver, renc: encoder_driver, battery_voltage: voltage_div_driver, imu: imu_driver, SE_timeValues: Queue,
                 X0_s: Queue, X1_psi: Queue, X2_omega_L: Queue, X3_omega_R: Queue,
                 U0_U_L: Queue, U1_U_R: Queue, U2_s_L: Queue, U3_s_R: Queue, U4_psi: Queue, U5_psi_dot: Queue,
                 Y0_s_L: Queue, Y1_s_R: Queue, Y2_psi: Queue, Y3_psi_dot: Queue):
        
        '''
        Initializes a state estimator task object
        Args:
            stateEstimatorGo (Share): A share object representing a boolean flag to
                                start state estimation mode
            lMotor: A motor driver object for the left motor
            rMotor: A motor driver object for the right motor
            lenc: An encoder driver object for the left wheel encoder
            renc: An encoder driver object for the right wheel encoder
            battery_voltage: A voltage divider driver object for reading the battery voltage
            imu: An IMU driver object for reading orientation data

            X0_s, X1_psi, X2_omega_L, X3_omega_R: Queue objects for storing the state vector values
            U0_U_L, U1_U_R, U2_s_L, U3_s_R, U4_psi, U5_psi_dot: Queue objects for storing the input vector values
            Y0_s_L, Y1_s_R, Y2_psi, Y3_psi_dot: Queue objects for storing the output vector values
        '''

        self._state: int                = S0_INIT    # The present state of the task    

        self._ser: stream              = USB_VCP()    # A serial port object used to
                                                 # read character entry and to
                                                 # print output   
        
        self._goFlag: Share             = stateEstimatorGo # A share object representing a
                                                        # flag to start state estimation mode   
            
        self._lMotor: motor_driver     = lMotor      # A motor driver object for the left motor

        self._rMotor: motor_driver     = rMotor      # A motor driver object for the right motor                                 

        self._lenc: encoder_driver      = lenc        # An encoder object for the left wheel

        self._renc: encoder_driver      = renc        # An encoder object for the right wheel

        self._battery_voltage: voltage_div_driver = battery_voltage # A voltage divider driver object for reading the battery voltage

        self._imu: imu_driver          = imu         # An IMU driver object for reading orientation data

        self._timeValues: Queue        = SE_timeValues # A queue object for storing time values in seconds

        self._X0_s: Queue              = X0_s        # A queue object for storing the s value in the state vector

        self._X1_psi: Queue            = X1_psi      # A queue object for storing the psi value in the state vector

        self._X2_omega_L: Queue        = X2_omega_L  # A queue object for storing the omega_L value in the state vector

        self._X3_omega_R: Queue        = X3_omega_R  # A queue object for storing the omega_R value in the state vector

        self._U0_U_L: Queue            = U0_U_L      # A queue object for storing the U_L value in the input vector

        self._U1_U_R: Queue            = U1_U_R      # A queue object for storing the U_R value in the input vector

        self._U2_s_L: Queue            = U2_s_L      # A queue object for storing the s_L value in the input vector

        self._U3_s_R: Queue            = U3_s_R      # A queue object for storing the s_R value in the input vector

        self._U4_psi: Queue            = U4_psi       # A queue object for storing the psi value in the input vector

        self._U5_psi_dot: Queue        = U5_psi_dot   # A queue object for storing the psi_dot value in the input vector

        self._Y0_s_L: Queue            = Y0_s_L      # A queue object for storing the s_L value in the output vector

        self._Y1_s_R: Queue            = Y1_s_R      # A queue object for storing the s_R value in the output vector

        self._Y2_psi: Queue            = Y2_psi      # A queue object for storing the psi value in the output vector

        self._Y3_psi_dot: Queue        = Y3_psi_dot  # A queue object for storing the psi_dot value in the output vector

        print("State Estimator Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        while True:
            
            if self._state == S0_INIT: # Init state

                self.width = 141 # The distance between the two wheels in mm
    
                self.A = np.array([[0.8139113,	0.0000000,	0.2494088,	0.2494088],
                                   [0.0000000,	0.0060927,	0.0000000,	0.0000000],
                                   [-0.1476239,	-0.0000000,	0.2835318,	0.2738447],
                                   [-0.1476239,	-0.0000000,	0.2738447,	0.2835318]])

                self.B = np.array([[0.2452538,	0.2452538,	0.0930444,	0.0930444,	0.0000000,	0.0000000],
                                   [0.0000000,	0.0000000,	-0.0071183,	0.0071183,	0.0001000,	0.0038972],
                                   [0.85728,	0.4845341,	0.0738119,	0.0738119,	 1e-6,     -1.7796785],
                                   [0.4845341,	0.8572828,	0.0738119,	0.0738119,	 1e-6,      1.7796785]])
                
                self.C = np.array([[1.0000000,	-70.5000000,	0,             0],
                                   [1.0000000,	70.5000000,	    0,	           0],
                                   [  0,			1.0,		0,	           0],
                                   [  0,			0,	  -0.248227,	0.248227]])
                
                self.X = np.array([[0],  # s
                                   [0],  # psi
                                   [0],  # omega_L
                                   [0]]) # omega_R

                self.U = np.array([[0],  # U_L
                                   [0],  # U_R
                                   [0],  # s_L
                                   [0],  # s_R
                                   [0],  # psi
                                   [0]]) # psi_dot

                self.Y = np.array([[0],  # s_L
                                   [0],  # s_R
                                   [0],  # psi
                                   [0]]) # psi_dot
                
                self._X0_s.clear()    # s
                self._X1_psi.clear()    # psi
                self._X2_omega_L.clear()    # omega_L
                self._X3_omega_R.clear() # omega_R

                self._U0_U_L.clear()    # U_L
                self._U1_U_R.clear()    # U_R
                self._U2_s_L.clear()    # s_L
                self._U3_s_R.clear()    # s_R
                self._U4_psi.clear()    # psi
                self._U5_psi_dot.clear() # psi_dot
                        
                self._Y0_s_L.clear()    # s_L
                self._Y1_s_R.clear()    # s_R
                self._Y2_psi.clear()    # psi
                self._Y3_psi_dot.clear() # psi_dot

                self._timeValues.clear()
                self._start_time = ticks_us()

                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._goFlag.get():
                    self._start_time = ticks_us()
                    
                    self._lenc.update()
                    self._renc.update()
                    self.leftposition = self._lenc.get_position()
                    self.rightposition = self._renc.get_position()
                    psi = (self.rightposition - self.leftposition) / self.width
    
                    # Form the initial state matrix relative to encoder counts
                    self.X = np.array([[(self.rightposition + self.leftposition) / 2],  # s
                                       [psi],  # psi
                                       [self._lenc.get_velocity_rad()],  # omega_L
                                       [self._renc.get_velocity_rad()]]) # omega_R
                    
                    # Initialize IMU readings to zero relative to initial state matrix
                    self.IMU_psi_offset = psi - (self._imu.heading() * 3.14159 / 180) # Set the initial heading as the offset

                    self._state = S2_RUN
                    
            elif self._state == S2_RUN: #  line state
                if not self._goFlag.get(): # If the go flag is turned off, return to the wait state
                    self._state = S1_WAIT
                else:
                    time = ticks_diff(ticks_us(), self._start_time) / 1000000

                    # Solve next state based on current state and input
                    self.U = np.array([[self._lMotor.get_effort() * 0.01 * self._battery_voltage.get_voltage()],  # U_L
                                       [self._rMotor.get_effort() * 0.01 * self._battery_voltage.get_voltage()],  # U_R
                                       [self._lenc.get_position()],  # s_L
                                       [self._renc.get_position()],  # s_R
                                       [self._imu.heading() * 3.14159 / 180 + self.IMU_psi_offset],  # psi 
                                       [self._imu.yaw_rate()* 3.14159 / 180]])  # psi_dot
                    
                    self.X = np.dot(self.A, self.X) + np.dot(self.B, self.U)
                    self.Y = np.dot(self.C, self.X)

                    # Update the state estimator queues
                    if not self._X0_s.full():  
                        self._timeValues.put(time)
                         
                        self._X0_s.put(self.X[0, 0])    # s
                        self._X1_psi.put(self.X[1, 0])    # psi
                        self._X2_omega_L.put(self.X[2, 0])    # omega_L
                        self._X3_omega_R.put(self.X[3, 0]) # omega_R

                        self._U0_U_L.put(self.U[0, 0])    # U_L
                        self._U1_U_R.put(self.U[1, 0])    # U_R
                        self._U2_s_L.put(self.U[2, 0])    # s_L
                        self._U3_s_R.put(self.U[3, 0])    # s_R
                        self._U4_psi.put(self.U[4, 0])    # psi
                        self._U5_psi_dot.put(self.U[5, 0]) # psi_dot
                        
                        self._Y0_s_L.put(self.Y[0, 0])    # s_L
                        self._Y1_s_R.put(self.Y[1, 0])    # s_R
                        self._Y2_psi.put(self.Y[2, 0])    # psi
                        self._Y3_psi_dot.put(self.Y[3, 0]) # psi_dot
                    
            yield self._state