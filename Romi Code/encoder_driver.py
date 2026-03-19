from time import ticks_us, ticks_diff # Use to get dt value in update()
from pyb import Pin, Timer

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim: Pin, chA: Pin, chB: Pin):
        '''Initializes an Encoder object'''
        self.tim = tim
        self.chA = Pin(chA)
        self.chB = Pin(chB)

        self.tim.channel(1, pin=Pin(chA), mode=Timer.ENC_AB)
        self.tim.channel(2, pin=Pin(chB), mode=Timer.ENC_AB)

        #self.tim.counter(0)     # Initialize counter to zero
        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = 0     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 0     # Amount of time between last two updates
        self.prev_time  = 0     # Time of the most recent update
    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        self.delta = self.tim.counter() - self.prev_count
        if self.delta < -32768:  # Handle counter rollover (forward)
            self.delta += 65536
        elif self.delta > 32767:   # Handle counter rollover (backward)
            self.delta -= 65536
        self.position -= self.delta
        self.dt = ticks_diff(ticks_us(), self.prev_time) / 1000000.0 # Convert microseconds to seconds
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()
            
    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position/6.54809
            
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        if self.dt == 0:
            return 0
        return -self.delta/6.54809/self.dt
    
    def get_position_rad(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position/229.1831
    
    def get_velocity_rad(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        if self.dt == 0:
            return 0
        return -self.delta/229.1831/self.dt

    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.tim.counter(0)     # Initialize counter to zero
        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = 0     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.prev_time  = 0     # Time of the most recent update