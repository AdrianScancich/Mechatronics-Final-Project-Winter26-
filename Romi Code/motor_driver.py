from pyb import Pin, Timer

class Motor:
    '''A motor driver interface encapsulated in a Python class.'''
    def __init__(self, PWM: Pin, DIR: Pin, nSLP: Pin, tim: Timer, channel: int):
        '''Initializes a Motor object'''
        self.DIR = Pin(DIR, mode=Pin.OUT_PP)
        self.nSLP = Pin(nSLP, mode=Pin.OUT_PP)
        self.PWM = tim.channel(channel, pin=PWM, mode=Timer.PWM, pulse_width_percent=0)
        self.effort = 0
           
    def set_effort(self, effort: float):
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
        return self.effort

    def enable(self):
        self.nSLP.high()
        self.PWM.pulse_width_percent(0)
            
    def disable(self):
        self.nSLP.low()