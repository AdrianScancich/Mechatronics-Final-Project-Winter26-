from pyb import Pin
from machine import ADC

class Line_Sensor:
    '''A line sensor driver interface encapsulated in a Python class'''
    def __init__(self, Inner: Pin, Middle: Pin,  Outer: Pin):
        '''Initializes a line sensor object'''
        self.InnerADC = ADC(Pin(Inner)) 
        self.MiddleADC = ADC(Pin(Middle))
        self.OuterADC = ADC(Pin(Outer)) 
    
    def get_values(self):
        '''Returns the most recently updated values of the line sensors as a tuple'''
        return self.InnerADC.read_u16(), self.MiddleADC.read_u16(), self.OuterADC.read_u16()