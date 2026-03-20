''' This file demonstrates an example motor task using a custom class with a
    run method implemented as a generator
'''
from motor_driver       import Motor as motor_driver
from encoder_driver     import Encoder as encoder_driver
from pid_controller     import PID_controller as pid_controller
from task_share         import Share, Queue
from utime              import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control

class task_motor:
    '''
    A class that represents a motor task. The task is responsible for reading
    data from an encoder, performing closed loop control, and actuating a motor.
    Multiple objects of this class can be created to work with multiple motors
    and encoders.
    '''

    def __init__(self,
                 mot: motor_driver, enc: encoder_driver, pid: pid_controller,
                 goFlag: Share, dataValues: Queue, timeValues: Queue, velocity: Share, Kp: Share, Ki: Share, Kd: Share):
        '''
        Initializes a motor task object
        
        Args:
            mot (motor_driver): A motor driver object
            enc (encoder):      An encoder object
            goFlag (Share):     A share object representing a boolean flag to
                                start data collection
            dataValues (Queue): A queue object used to store collected encoder
                                position values
            timeValues (Queue): A queue object used to store the time stamps
                                associated with the collected encoder data
        '''

        self._state: int        = S0_INIT    # The present state of the task       
        
        self._mot: motor_driver = mot        # A motor object
        
        self._enc: encoder_driver      = enc        # An encoder object

        self._pid: pid_controller      = pid        # A PID controller object
        
        self._goFlag: Share     = goFlag     # A share object representing a
                                             # flag to start data collection
        
        self._dataValues: Queue = dataValues # A queue object used to store
                                             # collected encoder position
        
        self._timeValues: Queue = timeValues # A queue object used to store the
                                             # time stamps associated with the
                                             # collected encoder data
        
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data

        self._velocity: Share   = velocity    # A share for velocity

        self._Kp: Share        = Kp          # A share for Kp

        self._Ki: Share        = Ki          # A share for Ki

        self._Kd: Share        = Kd          # A share for Kd
        
        print("Motor Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                #print("Initializing motor task and PID controller")
                #print("Target velocity is set to {} mm/s".format(self._velocity.get()))
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._goFlag.get():
                    # print("Starting motor loop")
                    
                    # Capture a start time in microseconds so that each sample
                    # can be timestamped with respect to this start time. The
                    # start time will be off by however long it takes to
                    # transition and run the next state, so the time values may
                    # need to be zeroed out again during data processing.
                    self._startTime = ticks_us()
                    self._pid.zero()
                    self._pid.update_Kp(self._Kp.get())
                    self._pid.update_Ki(self._Ki.get())
                    self._pid.update_Kd(self._Kd.get())
                    self._pid.update_saturation(100)
                    self._mot.enable()
                    self._state = S2_RUN
                
            elif self._state == S2_RUN: # Closed-loop control state
                if not self._goFlag.get(): # If the go flag is turned off, stop the motor and return to the wait state
                    self._mot.set_effort(0)
                    self._state = S1_WAIT
                else:
                    # print(f"Running motor loop, cycle {self._dataValues.num_in()}")
                    # Run the encoder update algorithm and then capture the present
                    # motor speed.

                    self._enc.update()
                    #pos = self._enc.get_position_rad()
                    vel = self._enc.get_velocity_rad()
                    vellin = self._enc.get_velocity()

                    # Collect a timestamp to use for this sample
                    t = ticks_us()
                    
                    # Convert reference velocity from mm/s to rad/s and run the PID controller 
                    vref = self._velocity.get() / 35 

                    #print("DEBUG VELOCITIES",vref, vel)              
                
                    NextEffort = self._pid.PID_control(vref, vel)
                    #print("DEBUG EFFORT", NextEffort)
                    self._mot.set_effort(NextEffort)

                    # Store the sampled values in the queues
                    if not self._dataValues.full(): 
                        self._dataValues.put(vellin)
                        
                    if not self._timeValues.full():
                        self._timeValues.put(ticks_diff(t, self._startTime) / 1000000.0) # Convert microseconds to seconds
            
            yield self._state