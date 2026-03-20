from time import ticks_us, ticks_diff # Use to get dt value in update()
from pyb import Pin, Timer

class PID_controller:
    '''A proportional-integral-derivative controller encapsulated in a Python class'''

    def __init__(self):
        '''Initializes a PID controller object'''
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.position   = 0     # Current Position
        self.setpoint   = 0     # Desired setpoint for the controller
        self.dt         = 0     # Amount of time between last two updates
        self.prev_time  = ticks_us()    # Time of the most recent update
        self.esum       = 0     # Cumulative error for integral term
        self.a_star     = 0     # Error saturation limit
        self.output     = 0     # Control output
        self.prev_error = 0     # Previous error value (used for derivative term)

    def update_Kp(self, kp: float):
        self.Kp = kp

    def update_Ki(self, ki: float):
        self.Ki = ki

    def update_Kd(self, kd: float):
        self.Kd = kd
        
    def update_saturation(self, saturation: float):
        self.a_star = saturation

    def PID_control(self, setpoint: float, position: float):
        self.dt = ticks_diff(ticks_us(), self.prev_time) / 1000000.0 # Convert microseconds to seconds
        e = setpoint - position
        P = self.Kp * e
        I = self.Ki * self.esum
        D = self.Kd * (e - self.prev_error) / self.dt if self.dt > 0 else 0
        self.Unsat = P + I + D
        if self.a_star != 0:
            if abs(self.Unsat + self.Ki * e * self.dt) > self.a_star:
                self.output = self.a_star if self.Unsat > 0 else -self.a_star
            else:
                self.esum += e * self.dt
                self.output = self.Unsat + self.Ki * e * self.dt
        else:
            self.esum += e * self.dt
            self.output = self.Unsat + self.Ki * e * self.dt 
        self.prev_time = ticks_us()
        self.prev_error = e
        return self.output
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.position   = 0     # Current Position
        self.setpoint   = 0     # Desired setpoint for the controller
        self.dt         = 0     # Amount of time between last two updates
        self.prev_time  = ticks_us()    # Time of the most recent update
        self.esum       = 0     # Cumulative error for integral term
        self.output     = 0     # Control output
        self.prev_error = 0     # Previous error value (used for derivative term)
        