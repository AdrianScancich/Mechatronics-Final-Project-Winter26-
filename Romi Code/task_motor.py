from motor_driver       import Motor as motor_driver
from encoder_driver     import Encoder as encoder_driver
from pid_controller     import PID_controller as pid_controller
from task_share         import Share, Queue
from utime              import ticks_us, ticks_diff
import micropython

Kp = 3
Ki = 100
Kd = 0.02

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control

class task_motor:
    '''
    A class that represents a motor task. 
    '''

    def __init__(self,
                 mot: motor_driver, enc: encoder_driver, pid: pid_controller,
                 goFlag: Share, velocity: Share):
        '''
        Initializes a motor task object
        
        '''

        self._state: int        = S0_INIT       
        
        self._mot: motor_driver = mot
        
        self._enc: encoder_driver      = enc

        self._pid: pid_controller      = pid
        
        self._goFlag: Share     = goFlag

        self._velocity: Share   = velocity   

        
        print("Motor Task object instantiated")
        
    def run(self):
        while True:
            
            if self._state == S0_INIT:
                self._pid.update_Kp(Kp)
                self._pid.update_Ki(Ki)
                self._pid.update_Kd(Kd)
                self._pid.update_saturation(100)
                self._mot.enable()
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT:
                if self._goFlag.get():
                    self._pid.zero()
            
                    self._state = S2_RUN
                
            elif self._state == S2_RUN:
                if not self._goFlag.get():
                    self._mot.set_effort(0)
                    self._state = S1_WAIT
                else:
                    self._enc.update()
                    vel = self._enc.get_velocity_rad()
                    vref = self._velocity.get() / 35 
                    NextEffort = self._pid.PID_control(vref, vel)

                    self._mot.set_effort(NextEffort)

            yield self._state