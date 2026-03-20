from pyb import Pin, Timer

class Motor:
    '''A motor driver interface encapsulated in a Python class. Works with
       motor drivers using separate PWM and direction inputs such as the DRV8838
       drivers present on the Romi chassis from Pololu.'''
    def __init__(self, PWM: Pin, DIR: Pin, nSLP: Pin, tim: Timer, channel: int):
        '''Initializes a Motor object'''
        self.DIR = Pin(DIR, mode=Pin.OUT_PP)
        self.nSLP = Pin(nSLP, mode=Pin.OUT_PP)
        self.PWM = tim.channel(channel, pin=PWM, mode=Timer.PWM, pulse_width_percent=0)
        self.effort = 0
           
    def set_effort(self, effort: float):
        '''Sets the present effort requested from the motor based on an input value
           between -100 and 100'''
        if effort < 0:
            self.DIR.high()
            if effort < -100:
                effort = -100
        else:
            self.DIR.low()
            if effort > 100:
                effort = 100
        self.PWM.pulse_width_percent(abs(effort))
        self.effort = effort

    def get_effort(self):
        '''Returns the present effort requested from the motor'''
        return self.effort

    def enable(self):
        '''Enables the motor driver by taking it out of sleep mode into brake mode'''
        self.nSLP.high()
        self.PWM.pulse_width_percent(0)
            
    def disable(self):
        '''Disables the motor driver by taking it into sleep mode'''
        self.nSLP.low()