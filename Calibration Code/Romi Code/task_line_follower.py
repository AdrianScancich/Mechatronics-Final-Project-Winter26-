from line_sensor_driver import Line_Sensor as line_sensor_driver
from pid_controller     import PID_controller as pid_controller
from encoder_driver     import Encoder as encoder_driver
from pyb                import USB_VCP
from task_share         import Share, Queue
from utime              import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control

class task_line_follower:
    '''
    A class that represents a line follower task. The task is responsible for reading
    data from an encoder, performing closed loop control, and actuating motors.
    '''

    def __init__(self,
                 lineFollowerGo: Share, leftMotorGo: Share, rightMotorGo: Share, llf: line_sensor_driver, rlf: line_sensor_driver, lenc: encoder_driver, renc: encoder_driver, pid: pid_controller, RomiVelocity: Share, leftVelocity: Share, rightVelocity: Share,
                 centroidValues: Queue, centroidtimeValues: Queue, LFKp: Share, LFKi: Share, LFkd: Share):
        
        '''
        Initializes a line follower task object
        
        Args:
            lineFollowerGo (Share): A share object representing a boolean flag to
                                start line following mode
            leftMotorGo (Share): A share object representing a boolean flag to
                                start left motor control
            rightMotorGo (Share): A share object representing a boolean flag to
                                start right motor control
            RomiVelocity (Share): A share object representing the target velocity
            leftVelocity (Share): A share object representing the left wheel velocity
            rightVelocity (Share): A share object representing the right wheel velocity
        '''

        self._state: int                = S0_INIT    # The present state of the task    

        #self._ser: stream              = USB_VCP()    # A serial port object used to
        #                                         # read character entry and to
        #                                         # print output   
        
        self._goFlag: Share             = lineFollowerGo # A share object representing a
                                                        # flag to start line following mode
                                                        
        self._leftMotorGo: Share        = leftMotorGo # A share object representing a
                                                        # flag to start left motor control

        self._rightMotorGo: Share       = rightMotorGo # A share object representing a
                                                        # flag to start right motor control

        self._lenc: encoder_driver      = lenc        # An encoder object for the left wheel

        self._renc: encoder_driver      = renc        # An encoder object for the right wheel

        self._llf: line_sensor_driver   = llf        # A line sensor driver object for the left line sensor

        self._rlf: line_sensor_driver   = rlf        # A line sensor driver object for the right line sensor

        self._pid: pid_controller       = pid        # A PID controller object for controlling the velocity of the Romi based on the line sensor readings

        self._RomiVelocity: Share       = RomiVelocity    # A share for the target velocity of the Romi

        self._leftVelocity: Share       = leftVelocity    # A share for the left wheel velocity

        self._rightVelocity: Share      = rightVelocity   # A share for the right wheel velocity

        self._centroidValues: Queue     = centroidValues  # A queue for the centroid of the line position based on sensor readings, used for data collection and analysis
        
        self._centroidtimeValues: Queue = centroidtimeValues  # A queue for the time stamps of the centroid values

        self._LFKp: Share                 = LFKp          # A share for line follower Kp

        self._LFKi: Share                 = LFKi          # A share for line follower Ki

        self._LFKd: Share                 = LFkd          # A share for line follower Kd

        print("Line Follower Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state 
                #self.highthreshold = 5500 # Threshold for positive line detection
                #self.lowthreshold = 3500 # Treshold for negative line detection
                
                #self.base = self._RomiVelocity.get()
                self.turn_max = 0.75 * abs(self._RomiVelocity.get()) # Maximum turn command
                
                self.InnerWeight = 8
                self.MiddleWeight = 16
                self.OuterWeight = 24
                self.TotalWeight = self.InnerWeight + self.MiddleWeight + self.OuterWeight
                
                self.bias = 0 # Bias term to help correct for any misalignment of the robot with the line

                self.MaxVelocity = 900 # Maximum velocity attanable by each wheel under load

                self._junctionFlag = False # Initialize junction flag to false

                #self._pid.update_Kp(self._LFKp.get())
                #self._pid.update_Ki(self._LFKi.get())
                #self._pid.update_Kd(self._LFKd.get())
                self._pid.update_saturation(self.turn_max)    

                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._goFlag.get():
                    
                    self._pid.update_Kp(self._LFKp.get())
                    self._pid.update_Ki(self._LFKi.get())
                    self._pid.update_Kd(self._LFKd.get())

                    self._pid.zero() # Zero PID controller when starting line follower 

                    self.Linner, self.Lmiddle, self.Louter = self._llf.get_values()
                    self.Rinner, self.Rmiddle, self.Router = self._rlf.get_values()

                    left_unweighted  = self.Linner + self.Lmiddle + self.Louter
                    right_unweighted = self.Rinner + self.Rmiddle + self.Router
                    left_weighted  = self.InnerWeight*self.Linner + self.MiddleWeight*self.Lmiddle + self.OuterWeight*self.Louter
                    right_weighted = self.InnerWeight*self.Rinner + self.MiddleWeight*self.Rmiddle + self.OuterWeight*self.Router
                    total_unweighted = left_unweighted + right_unweighted

                    error = left_weighted - right_weighted      # Error 
                    #normalized_error = error * self._RomiVelocity.get() / total_unweighted # Normalized error ~ [-base, base]
                    normalized_error = error / total_unweighted # Normalized error ~ [-Centroid, Centroid]

                    self.bias = 0 # Set bias assuming robot starts aligned with the line.

                    self._leftVelocity.put(0)
                    self._rightVelocity.put(0)
                    self._leftMotorGo.put(True)
                    self._rightMotorGo.put(True)

                    self.startTime = ticks_us()

                    self._state = S2_RUN
                    
            elif self._state == S2_RUN: #  line state
                if not self._goFlag.get(): # If the go flag is turned off, stop the robot and return to the wait state
                    self._state = S1_WAIT
                else:
                    self.Linner, self.Lmiddle, self.Louter = self._llf.get_values()
                    self.Rinner, self.Rmiddle, self.Router = self._rlf.get_values()

                    left_unweighted  = self.Linner + self.Lmiddle + self.Louter
                    right_unweighted = self.Rinner + self.Rmiddle + self.Router
                    left_weighted  = self.InnerWeight*self.Linner + self.MiddleWeight*self.Lmiddle + self.OuterWeight*self.Louter
                    right_weighted = self.InnerWeight*self.Rinner + self.MiddleWeight*self.Rmiddle + self.OuterWeight*self.Router
                    total_unweighted = left_unweighted + right_unweighted

                    error = left_weighted - right_weighted      # Error 
                    #normalized_error = error * self._RomiVelocity.get() / total_unweighted # Normalized error ~ [-base, base]
                    normalized_error = error / total_unweighted # Normalized error ~ [-Centroid, Centroid]
                        
                    turn = self._pid.PID_control(self.bias, normalized_error)  

                    leftveloctiy = self._lenc.get_velocity()
                    rightvelocity = self._renc.get_velocity()
                    current_velocity = (leftveloctiy + rightvelocity) / 2
                    
                    if self._RomiVelocity.get() - current_velocity > 20:
                        velocity_setpoint = current_velocity + 20
                    elif self._RomiVelocity.get() - current_velocity < -20:
                        velocity_setpoint = current_velocity - 20
                    else:
                        velocity_setpoint = self._RomiVelocity.get()

                    if velocity_setpoint + abs(turn) > self.MaxVelocity:
                        velocity_setpoint = self.MaxVelocity - abs(turn)

                    self._leftVelocity.put(velocity_setpoint + turn)
                    self._rightVelocity.put(velocity_setpoint - turn)
                    
                    if not self._centroidValues.full():
                        self._centroidValues.put(normalized_error) # Store the centroid (normalized error) value in the queue for data collection and analysis

                    if not self._centroidtimeValues.full():
                        self._centroidtimeValues.put(ticks_diff(ticks_us(), self.startTime) / 1000000.0) # Convert microseconds to seconds


            yield self._state