from pyb import Pin
from machine import ADC

class Line_Sensor:
    '''A line sensor driver interface encapsulated in a Python class'''
    def __init__(self, Inner: Pin, Middle: Pin,  Outer: Pin):
        '''Initializes a line sensor object'''
        self.InnerADC = ADC(Pin(Inner)) # Initialize the inner line sensor pin as an ADC input
        self.MiddleADC = ADC(Pin(Middle)) # Initialize the middle line sensor pin as an ADC input
        self.OuterADC = ADC(Pin(Outer)) # Initialize the outer line sensor pin as an AD input
           
    #def update(self):
    #    '''Updates the line sensor values'''
    #    self.InnerValue = self.Inner.value()
    #    self.OuterValue = self.Outer.value()
    
    def get_values(self):
        '''Returns the most recently updated values of the line sensors as a tuple'''
        return self.InnerADC.read_u16(), self.MiddleADC.read_u16(), self.OuterADC.read_u16()