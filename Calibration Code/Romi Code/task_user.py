''' This file demonstrates an example UI task using a custom class with a
    run method implemented as a generator
'''
from pyb import USB_VCP, UART
from task_share import Queue, Share
from voltage_div_driver import Voltage_Divider as voltage_div_driver
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_CMD  = micropython.const(1) # State 1 - wait for character input
S2_COL  = micropython.const(2) # State 2 - wait for data collection to end
S3_DIS  = micropython.const(3) # State 3 - display the collected data
S4_MCMD = micropython.const(4) # State 4 - Multiple character input
#S5_SEW  = micropython.const(5) # State 5 - State estimator write

UI_prompt = ">: "

UI_menu = ("+------------------------------------------------------------------------------+\r\n"
             "| ME 405 Romi Tuning Interface Help Menu                                       |\r\n"
             "+---+--------------------------------------------------------------------------+\r\n"
             "| h | Print help menu                                                          |\r\n"
             "| q | Quick-load default gains                                                 |\r\n"
             "| k | Enter new gain values                                                    |\r\n"
             "| s | Choose a new setpoint                                                    |\r\n"
             "| g | Trigger step response and print results                                  |\r\n"
             "| p | Enter new line follower gains                                            |\r\n"
             "| l | Line following mode                                                      |\r\n"
             "| e | State estimator mode                                                     |\r\n"
             "| t | Enter test mode [ Only output data ]                                     |\r\n"
             "| v | Read battery capacity                                                    |\r\n"
#             "| m | Switch active motor                                                      |\r\n"
             "+---+--------------------------------------------------------------------------+\r\n")

#with serial.Serial(1, 115200, timeout=1000) as ser: #Apply alternate serial settings later
    #All code using ser goes here

class task_user:
    '''
    A class that represents a UI task. The task is responsible for reading user
    input over a serial port, parsing the input for single-character commands,
    and then manipulating shared variables to communicate with other tasks based
    on the user commands.
    '''

    def __init__(self, leftMotorGo, rightMotorGo, lineFollowerGo, stateEstimatorGo, leftdataValues, rightdataValues, lefttimeValues, righttimeValues, centroidValues, centroidtimeValues,
                  leftvelocity, rightvelocity, RomiVelocity, Kp, Ki, Kd, LFKp, LFKi, LFkd, SE_timeValues,
                  X0_s, X1_psi, X2_omega_L, X3_omega_R,
                  U0_U_L, U1_U_R, U2_s_L, U3_s_R, U4_psi, U5_psi_dot,
                  Y0_s_L, Y1_s_R, Y2_psi, Y3_psi_dot,
                  Romi_Battery_Voltage: voltage_div_driver):
        '''
        Initializes a UI task object
        
        Args:
            leftMotorGo (Share):  A share object representing a boolean flag to
                                  start data collection on the left motor
            rightMotorGo (Share): A share object representing a boolean flag to
                                  start data collection on the right motor
            lineFollowerGo (Share): A share object representing a boolean flag to
                                  start line following mode
            stateEstimatorGo (Share): A share object representing a boolean flag to
                                    start state estimator mode
            leftdataValues (Queue):   A queue object used to store collected encoder
                                  position values from the left motor
            rightdataValues (Queue):   A queue object used to store collected encoder
                                  position values from the right motor
            lefttimeValues (Queue):   A queue object used to store the time stamps
                                  associated with the collected encoder data
            righttimeValues (Queue):   A queue object used to store the time stamps
                                  associated with the collected encoder data
        '''
        
        self._state: int                = S0_INIT      # The present state
        
        self._leftMotorGo: Share        = leftMotorGo  # The "go" flag to start data
                                                 # collection from the left
                                                 # motor and encoder pair
        
        self._rightMotorGo: Share       = rightMotorGo # The "go" flag to start data
                                                 # collection from the right
                                                 # motor and encoder pair

        self._lineFollowerGo: Share     = lineFollowerGo # The "go" flag to start line following mode

        self._stateEstimatorGo: Share   = stateEstimatorGo # The "go" flag to start state estimator mode

        self._ser: stream              = USB_VCP()    # A serial port object used to
                                                 # read character entry and to
                                                 # print output

        #self._ser: stream               = HCBT    # A serial port object used to
                                                 # read character entry and to
                                                 # print output                                         
        
        self._leftdataValues: Queue     = leftdataValues   # A reusable buffer for data
                                                 # collection
        
        self._rightdataValues: Queue    = rightdataValues   # A reusable buffer for data
                                                 # collection
        
        self._lefttimeValues: Queue     = lefttimeValues   # A reusable buffer for time
                                                 # stamping collected data

        self._righttimeValues: Queue    = righttimeValues   # A reusable buffer for time
                                                 # stamping collected data

        self._centroidValues: Queue     = centroidValues   # A reusable buffer for the centroid of the line position based on sensor readings, used for data collection and analysis

        self._centroidtimeValues: Queue = centroidtimeValues   # A reusable buffer for the time stamps of the centroid values

        self._leftVelocity: Share       = leftvelocity    # A share for velocity

        self._rightVelocity: Share      = rightvelocity    # A share for velocity

        self._RomiVelocity: Share       = RomiVelocity    # A share for velocity

        self._Kp: Share                 = Kp          # A share for Kp

        self._Ki: Share                 = Ki          # A share for Ki

        self._Kd: Share                 = Kd          # A share for Kd

        self._LFKp: Share                 = LFKp          # A share for line follower Kp

        self._LFKi: Share                 = LFKi          # A share for line follower Ki

        self._LFKd: Share                 = LFkd          # A share for line follower Kd

        self._SE_timeValues: Queue        = SE_timeValues # A queue object for storing time values in seconds

        self._X0_s: Queue              = X0_s        # A queue object for storing the s value in the state vector

        self._X1_psi: Queue            = X1_psi      # A queue object for storing the psi value in the state vector

        self._X2_omega_L: Queue        = X2_omega_L  # A queue object for storing the omega_L value in the state vector

        self._X3_omega_R: Queue        = X3_omega_R  # A queue object for storing the omega_R value in the state vector

        self._U0_U_L: Queue           = U0_U_L      # A queue object for storing the U_L value in the input vector

        self._U1_U_R: Queue           = U1_U_R      # A queue object for storing the U_R value in the input vector

        self._U2_s_L: Queue           = U2_s_L      # A queue object for storing the s_L value in the input vector

        self._U3_s_R: Queue           = U3_s_R      # A queue object for storing the s_R value in the input vector

        self._U4_psi: Queue           = U4_psi      # A queue object for storing the psi value in the input vector

        self._U5_psi_dot: Queue       = U5_psi_dot  # A queue object for storing the psi_dot value in the input vector

        self._Y0_s_L: Queue           = Y0_s_L      # A queue object for storing the s_L value in the output vector

        self._Y1_s_R: Queue           = Y1_s_R      # A queue object for storing the s_R value in the output vector

        self._Y2_psi: Queue           = Y2_psi      # A queue object for storing the psi value in the output vector

        self._Y3_psi_dot: Queue       = Y3_psi_dot  # A queue object for storing the psi_dot value in the output vector

        self.Romi_Battery_Voltage: voltage_div_driver = Romi_Battery_Voltage # A voltage divider object used to read the Romi's battery voltage
                                                
        self._ser.write("User Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)

                self._ser.write("Initializing user task\r\n")
                # A share or queue object where the computed number is to be placed
                # out_share: BaseShare = Share('f', name="A float share")

                # A character buffer used to store incoming characters as they're
                # received by the command processor
                self.char_buf: str      = ""

                # A set used to quickly check if a character entered by the user is
                # a numerical digit.
                self.digits:   set(str) = set(map(str,range(10)))

                # A set used to quickly check if a character entered by the user is
                # a terminator (a carriage return or newline)
                self.term:     set(str) = {"\r", "\n"}

                # A flag used to track whether or not the command processing is
                # still active.
                self.done = False

                self.MCMDout = 0 # A share used to output values from the multi-character command state

                self.MCMDvar = None # A string used to track which gain is being updated in the multi-character command state

                self.TestMode = False # A flag used to enable test mode which can be used to test new UI features without interfering with the step response data collection and display

                self.ActiveMotor = "Left" # A string used to track which motor is active for testing purposes

                self._ser.write(UI_menu)
                self._ser.write(UI_prompt)
                self._state = S1_CMD
                
            elif self._state == S1_CMD: # Wait for UI commands
                if self._ser.any():
                    inChar = self._ser.read(1).decode()
                    if inChar in {"h", "H"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._ser.write("Help menu requested...\r\n")
                        self.TestMode = False
                        self._ser.write(UI_menu)
                        self._ser.write(UI_prompt)
                    elif inChar in {"q", "Q"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._Kp.put(3)
                        self._Ki.put(100)
                        self._Kd.put(0.02)
                        self._LFKp.put(0.01)
                        self._LFKi.put(0.01)
                        self._LFKd.put(0.00025)
                        self._ser.write("Default gains loaded.\r\n")
                        self._ser.write(UI_prompt)
                    elif inChar in {"k", "K"}:
                        self._ser.write(f"{inChar}\r\n")
                        self.MCMDvar = "Kp"
                        self._ser.write("Gain update requested...\r\n")
                        self._ser.write("Enter proportional gain, Kp: [ EX: 3 ]\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S4_MCMD
                    elif inChar in {"s", "S"}:
                        self._ser.write(f"{inChar}\r\n")
                        self.MCMDvar = "Romi Velocity"
                        self._ser.write("Setpoint update requested...\r\n")
                        self._ser.write("Enter new Romi velocity setpoint in mm/s:\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S4_MCMD
                    elif inChar in {"g", "G"}:
                        if not self._lineFollowerGo.get():
                            if not self.TestMode:
                                self._ser.write(f"{inChar}\r\n")
                                self._ser.write("Step response requested...\r\n")
                                self._ser.write("Data collecting...\r\n")

                            self._leftdataValues.clear() # Clear the data buffers in case they contain any old data
                            self._rightdataValues.clear()
                            self._lefttimeValues.clear()
                            self._righttimeValues.clear()
                            self._centroidValues.clear()
                            
                            self._leftVelocity.put(self._RomiVelocity.get())
                            self._rightVelocity.put(self._RomiVelocity.get())

                            self._leftMotorGo.put(True)
                            self._rightMotorGo.put(True)

                            self._state = S2_COL
                        else:
                            if not self.TestMode:
                                self._ser.write(f"{inChar}\r\n")
                                self._ser.write("Error: Cannot run step response test while line following mode is active. Please disable line following mode and try again.\r\n")
                                self._state = S1_CMD
                    elif inChar in {"p", "P"}:
                        self._ser.write(f"{inChar}\r\n")
                        self.MCMDvar = "LFKp"
                        self._ser.write("Line Follower gain update requested...\r\n")
                        self._ser.write("Enter proportional gain, LF_Kp: [ EX: 0.1 ]\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S4_MCMD
                    elif inChar in {"l", "L"}:
                        if not self._lineFollowerGo.get():
                            if not self.TestMode:
                                self._ser.write(f"{inChar}\r\n")
                            self._leftdataValues.clear() # Clear the data buffers in case they contain any old data
                            self._rightdataValues.clear()
                            self._lefttimeValues.clear()
                            self._righttimeValues.clear()
                            self._centroidValues.clear()

                            if not self._stateEstimatorGo.get():
                                 self._stateEstimatorGo.put(True)

                            if not self.TestMode:
                                self._ser.write("Line following mode requested... Press l to disable line following mode.\r\n")
                                self._ser.write(UI_prompt)
                            self._lineFollowerGo.put(True)
                            self._state = S1_CMD
                        else:
                            if not self.TestMode: 
                                self._ser.write(f"{inChar}\r\n")   
                                self._ser.write("Line following mode disabled...\r\n")
                                self._ser.write(UI_prompt)
                            self._lineFollowerGo.put(False)
                            self._state = S2_COL
                    elif inChar in {"e", "E"}:
                        self._X0_s.clear()    # s
                        self._X1_psi.clear()    # psi
                        self._X2_omega_L.clear()    # omega_L
                        self._X3_omega_R.clear() # omega_R

                        self._U0_U_L.clear()    # U_L
                        self._U1_U_R.clear()    # U_R
                        self._U2_s_L.clear()    # s_L
                        self._U3_s_R.clear()    # s_R
                        self._U4_psi.clear()    # psi
                        self._U5_psi_dot.clear() # psi_dot
                                        
                        self._Y0_s_L.clear()    # s_L
                        self._Y1_s_R.clear()    # s_R
                        self._Y2_psi.clear()    # psi
                        self._Y3_psi_dot.clear() # psi_dot
                            
                        self._SE_timeValues.clear() # Clear the time buffer
                        
                        if not self._stateEstimatorGo.get():
                            if not self.TestMode:
                                self._ser.write(f"{inChar}\r\n")
                            if not self.TestMode:
                                self._ser.write("State estimator mode requested... Press e to disable state estimator mode.\r\n")
                                self._ser.write(UI_prompt)
                            self._stateEstimatorGo.put(True)
                            self._state = S1_CMD
                        else:
                            if not self.TestMode: 
                                self._ser.write(f"{inChar}\r\n")   
                                self._ser.write("State estimator mode disabled...\r\n")
                                self._ser.write(UI_prompt)
                            self._stateEstimatorGo.put(False)
                            self._state = S1_CMD
                    elif inChar in {"t", "T"}:
                        self._ser.write(f"{inChar}\r\n")
                        if not self.TestMode:
                            self._ser.write("Connection confirmed, trimmed output mode enabled... Press h to disable test mode.\r\n")
                            self.TestMode = True
                        else:
                            self._ser.write("Connection reconfirmed...\r\n")
                            self.TestMode = True
                    elif inChar in {"v", "V"}:
                        self._ser.write(f"{inChar}\r\n")
                        voltage = self.Romi_Battery_Voltage.get_voltage()
                        battery_percentage = self.Romi_Battery_Voltage.get_battery_percentage()
                        if not self.TestMode:
                            self._ser.write(f"Romi Voltage: {voltage:.2f} V, Battery Percentage: {battery_percentage:.2f}%\r\n")
                        else:
                            self._ser.write(f"{voltage:.2f},{battery_percentage}\r\n")
                
            elif self._state == S2_COL: # Collect data and wait for completion
                if self._ser.any(): self._ser.read(1)

                if self._leftdataValues.num_in() >= 50:
                    self._leftMotorGo.put(False)
                    self._leftVelocity.put(0)
                if self._rightdataValues.num_in() >= 50:
                    self._rightMotorGo.put(False)
                    self._rightVelocity.put(0)  

                SE_data_printed = False # A flag used to ensure that state estimator data is only printed once after collection is complete  

                if not self._leftMotorGo.get() and not self._rightMotorGo.get():
                    if not self.TestMode:    
                        self._ser.write("Test complete, printing data...\r\n")
                        self._ser.write("--------------------------------------------------------------------------------\r\n")
                        self._ser.write(f"Setpoint: {self._RomiVelocity.get()} mm/s\r\n")
                        self._ser.write(f"Kp:       {self._Kp.get()} %*s/mm\r\n")
                        self._ser.write(f"Ki:       {self._Ki.get()} %/mm\r\n")
                        self._ser.write(f"Kd:       {self._Kd.get()} %/mm\r\n")
                        self._ser.write("--------------------------------------------------------------------------------\r\n")
                        self._ser.write("Time [s], Velocity [mm/s], for left motor\r\n")
                    else:
                        self._ser.write(f"{self._RomiVelocity.get()}\r\n")
                        self._ser.write(f"{self._Kp.get()}\r\n")
                        self._ser.write(f"{self._Ki.get()}\r\n")
                        self._ser.write(f"{self._Kd.get()}\r\n")
                        self._ser.write("Time [s], Velocity [mm/s], Left motor\r\n")
                    self._state = S3_DIS
            
            elif self._state == S3_DIS: # Display collected data
                if self._leftdataValues.any():
                    self._ser.write(f"{self._lefttimeValues.get()},{self._leftdataValues.get()},\r\n")
                    if not self._leftdataValues.any():
                        self._ser.write("\r\nTime [s], Velocity [mm/s], Right motor\r\n")
                elif self._rightdataValues.any() and not self._leftdataValues.any():
                    self._ser.write(f"{self._righttimeValues.get()},{self._rightdataValues.get()},\r\n")
                    if not self._rightdataValues.any() and self._centroidValues.any():
                        if not self.TestMode:
                            self._ser.write("\r\n")
                            self._ser.write("Printing line follower parameters...\r\n")
                            self._ser.write("--------------------------------------------------------------------------------\r\n")
                            self._ser.write(f"Setpoint: {self._RomiVelocity.get()} mm/s\r\n")
                            self._ser.write(f"Kp:       {self._LFKp.get()} %*s/mm\r\n")
                            self._ser.write(f"Ki:       {self._LFKi.get()} %/mm\r\n")
                            self._ser.write(f"Kd:       {self._LFKd.get()} %/mm\r\n")
                            self._ser.write("--------------------------------------------------------------------------------\r\n")
                        else:
                            self._ser.write("\r\n")
                            self._ser.write(f"{self._RomiVelocity.get()}\r\n")
                            self._ser.write(f"{self._LFKp.get()}\r\n")
                            self._ser.write(f"{self._LFKi.get()}\r\n")
                            self._ser.write(f"{self._LFKd.get()}\r\n")
                        self._ser.write("Time [s], Centroid [mm], Line following\r\n")
                elif self._centroidValues.any() and not self._rightdataValues.any():
                    self._ser.write(f"{self._centroidtimeValues.get()},{self._centroidValues.get()},\r\n")
                elif self._Y0_s_L.any() and not self._centroidValues.any() and not SE_data_printed:
                    if not self.TestMode:
                        self._ser.write("\r\n Printing State Estimator Data:\r\n")
                    self._ser.write("\r\nState Vector:\r\n")
                    self._ser.write("Time [s], s [mm], psi [rad], omega_L [rad/s], omega_R [rad/s]\r\n")
                    while self._X0_s.any():
                        self._ser.write(f"{self._SE_timeValues.get()},{self._X0_s.get()},{self._X1_psi.get()},{self._X2_omega_L.get()},{self._X3_omega_R.get()}\r\n")
                        if not self._X0_s.any():
                            #if not self.TestMode:
                            self._ser.write("\r\nInput Vector:\r\n")
                            self._ser.write("U_L [V], U_R [V], s_L [mm], s_R [mm], psi [rad], psi_dot [rad/s]\r\n")
                    while self._U0_U_L.any():
                        self._ser.write(f"{self._U0_U_L.get()},{self._U1_U_R.get()},{self._U2_s_L.get()},{self._U3_s_R.get()},{self._U4_psi.get()},{self._U5_psi_dot.get()}\r\n")
                        if not self._U0_U_L.any():
                            #if not self.TestMode:
                            self._ser.write("\r\nOutput Vector:\r\n")
                            self._ser.write("s_L [mm], s_R [mm], psi [rad], psi_dot [rad/s]\r\n")
                    while self._Y0_s_L.any():
                        self._ser.write(f"{self._Y0_s_L.get()},{self._Y1_s_R.get()},{self._Y2_psi.get()},{self._Y3_psi_dot.get()}\r\n")
                    SE_data_printed = True # Set the flag to True to indicate that state estimator data has been printed
                else:
                    if not self.TestMode:
                        self._ser.write(UI_menu)
                        self._ser.write(UI_prompt)
                    else:
                        self._ser.write("Data done\r\n")
                    self._state = S1_CMD

            elif self._state == S4_MCMD: # Multiple character command state
                while not self.done:
                    if self._ser.any():
                        char_in = self._ser.read(1).decode()
                        if char_in in self.digits:
                            self._ser.write(char_in)
                            self.char_buf += char_in
                        elif char_in == "." and "." not in self.char_buf:
                            self._ser.write(char_in)
                            self.char_buf += char_in                        
                        elif char_in == "-" and len(self.char_buf) == 0:
                            self._ser.write(char_in)
                            self.char_buf += char_in                        
                        elif char_in == "\x7f" and len(self.char_buf) > 0:
                            self._ser.write(char_in)
                            self.char_buf = self.char_buf[:-1]                        
                        elif char_in in self.term:
                            if len(self.char_buf) == 0:
                                self._ser.write("\r\n")
                                self._ser.write("Invalid Input\r\n")
                                self.char_buf = ""
                                self._ser.write(UI_prompt)
                                #self.done = True                               
                            elif self.char_buf not in {"-", "."}:
                                self._ser.write("\r\n")
                                self.MCMDout = float(self.char_buf)
                                self.char_buf = ""
                                self.done = True
                if self.done == True:               
                    if self.MCMDvar == "Kp":
                        self._Kp.put(self.MCMDout)
                        self._ser.write("Enter integral gain, Ki: [ EX: 80 ]\r\n")
                        self._ser.write(UI_prompt)
                        self.done = False
                        self.MCMDvar = "Ki"
                    elif self.MCMDvar == "Ki":
                        self._Ki.put(self.MCMDout)
                        self._ser.write("Enter derivative gain, Kd: [ EX: 0.02 ]\r\n")
                        self._ser.write(UI_prompt)
                        self.done = False
                        self.MCMDvar = "Kd"
                    elif self.MCMDvar == "Kd":
                        self._Kd.put(self.MCMDout)
                        self.done = False
                        self._ser.write(UI_menu)
                        self._ser.write(UI_prompt)
                        self._state = S1_CMD
                    elif self.MCMDvar == "LFKp":
                        self._LFKp.put(self.MCMDout)
                        self._ser.write("Enter integral gain, LF_Ki: [ EX: 0 ]\r\n")
                        self._ser.write(UI_prompt)
                        self.done = False
                        self.MCMDvar = "LFKi"
                    elif self.MCMDvar == "LFKi":
                        self._LFKi.put(self.MCMDout)
                        self._ser.write("Enter derivative gain, LF_Kd: [ EX: 0.002 ]\r\n")
                        self._ser.write(UI_prompt)
                        self.done = False
                        self.MCMDvar = "LFKd"
                    elif self.MCMDvar == "LFKd":
                        self._LFKd.put(self.MCMDout)
                        self.done = False
                        self._ser.write(UI_menu)
                        self._ser.write(UI_prompt)
                        self._state = S1_CMD
                    elif self.MCMDvar == "Romi Velocity":
                        if abs(self.MCMDout) > 800:
                            self._ser.write("Error: Velocity setpoint too high. Enter a setpoint between -800 and 800 mm/s.\r\n")
                            self.done = False
                            self._state = S4_MCMD
                        else:
                            self._RomiVelocity.put(self.MCMDout)
                            self.done = False
                            self._ser.write(UI_menu)
                            self._ser.write(UI_prompt)
                            self._state = S1_CMD
                    else:
                        self._ser.write("Error: Gain not recognized. Value not set.\r\n")
                        self.done = False
                        self._state = S4_MCMD
            
            yield self._state