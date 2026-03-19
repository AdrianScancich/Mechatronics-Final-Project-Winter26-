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
from math import sin, cos, pi

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control

def wrap_to_pi(a):
    while a > pi:
        a -= 2 * pi
    while a < -2 * pi:
        a += pi
    return a

class task_state_estimator:
    '''
    A class that represents a state estimator task. 
    '''

    def __init__(self, stateEstimatorGo: Share, lMotor: motor_driver, rMotor: motor_driver, lenc: encoder_driver, renc: encoder_driver, battery_voltage: voltage_div_driver, imu: imu_driver,
                 X0_s: Share, X1_psi: Share, X2_omega_L: Share, X3_omega_R: Share, SE_timeValue: Share, X_global: Share, Y_global: Share):
        
        '''
        Initializes a state estimator task object
        '''

        self._state: int                = S0_INIT  
        
        self._goFlag: Share             = stateEstimatorGo  
            
        self._lMotor: motor_driver     = lMotor

        self._rMotor: motor_driver     = rMotor                              

        self._lenc: encoder_driver      = lenc

        self._renc: encoder_driver      = renc

        self._battery_voltage: voltage_div_driver = battery_voltage

        self._imu: imu_driver          = imu 

        self._X0_s: Share              = X0_s 

        self._X1_psi: Share            = X1_psi

        self._X2_omega_L: Share        = X2_omega_L

        self._X3_omega_R: Share        = X3_omega_R

        self._timeValue: Share        = SE_timeValue

        self._X_global: Share         = X_global 

        self._Y_global: Share         = Y_global  

        print("State Estimator Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        while True:
            
            if self._state == S0_INIT: # Init state

                self.width = 141
    
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
                
                self._X_global.put(0)
                self._Y_global.put(0)
                
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT:
                if self._goFlag.get():
                    self._start_time = ticks_us()
                    self._previous_time = self._start_time
                    
                    self._lenc.zero()
                    self._renc.zero()
                    self.leftposition = self._lenc.get_position()
                    self.rightposition = self._renc.get_position()
                    psi = (self.rightposition - self.leftposition) / self.width
    
                    self.X = np.array([[(self.rightposition + self.leftposition) / 2],  # s
                                       [psi],  # psi
                                       [self._lenc.get_velocity_rad()],  # omega_L
                                       [self._renc.get_velocity_rad()]]) # omega_R
                    
                    self.IMU_psi_offset = psi - (self._imu.heading() * 3.14159 / 180)

                    self._state = S2_RUN
                    
            elif self._state == S2_RUN:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    time = ticks_us()
                    toltime = ticks_diff(time, self._start_time) / 1000000
                    dt = ticks_diff(time, self._previous_time) / 1000000

                    self.U = np.array([[self._lMotor.get_effort() * 0.01 * self._battery_voltage.get_voltage()],  # U_L
                                       [self._rMotor.get_effort() * 0.01 * self._battery_voltage.get_voltage()],  # U_R
                                       [self._lenc.get_position()],  # s_L
                                       [self._renc.get_position()],  # s_R
                                       [self._imu.heading() * 3.14159 / 180 + self.IMU_psi_offset],  # psi 
                                       [self._imu.yaw_rate()* 3.14159 / 180]])  # psi_dot
                    
                    self.X = np.dot(self.A, self.X) + np.dot(self.B, self.U)
                    self.Y = np.dot(self.C, self.X)

                    psi_wrapped = self.X[1, 0] 

                    self._timeValue.put(toltime)
                         
                    self._X0_s.put(self.X[0, 0])    # s
                    self._X1_psi.put(psi_wrapped)    # psi
                    self._X2_omega_L.put(self.X[2, 0])    # omega_L
                    self._X3_omega_R.put(self.X[3, 0]) # omega_R

                    velocity = 35 / 2 * (self.X[2, 0] + self.X[3, 0])
                    X_global = self._X_global.get() + (velocity * cos(psi_wrapped) * dt)
                    Y_global = self._Y_global.get() + (velocity * sin(psi_wrapped) * dt)

                    self._X_global.put(X_global)
                    self._Y_global.put(Y_global)

                    self._previous_time = time

            yield self._state