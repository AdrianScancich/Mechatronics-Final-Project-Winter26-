from pyb import Pin
from machine import ADC

class Voltage_Divider:
    '''A voltage divider driver interface encapsulated in a Python class'''
    def __init__(self, V_div: Pin):
        '''Initializes a voltage divider object'''
        self.V_div = ADC(Pin(V_div)) # Initialize the voltage divider pin as an ADC input
    
        self.R1 = 12810
        self.R2 = 5870
    
    def get_voltage(self):
        '''Returns the most recently updated values of the voltage divider as a tuple'''
        ADC_out = self.V_div.read_u16()
        voltage = (ADC_out / 65535) * 3.3 * (self.R1 + self.R2) / self.R2 * 1.089
        return voltage
    
    def get_ADC(self):
        '''Returns the most recently updated raw ADC value from the voltage divider'''
        return self.V_div.read_u16()

    def get_battery_voltage(self):
        '''Returns the most recently updated battery voltage based on the voltage divider reading'''
        battery_voltage = self.get_voltage() / 6
        return battery_voltage
        
    def get_battery_percentage(self):
        '''Returns the battery percentage based on the voltage divider reading and an assumed voltage range of 6V (0%) to 8.4V (100%)'''
        battery_voltage = self.get_battery_voltage()
        battery_percentage = min(max((battery_voltage - 1.2) / (1.4 - 1.2) * 100, 0), 100)
        return battery_percentage