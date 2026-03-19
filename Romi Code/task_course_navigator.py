from pid_controller     import PID_controller as pid_controller
from pyb                import USB_VCP
from task_share         import Share, Queue
from utime              import ticks_us, ticks_diff
import micropython
from ulab import numpy as np
from array import array
from math import pi
from gc import collect

S0_INIT   = micropython.const(0)
S1_WAIT   = micropython.const(1)
S2_CK0    = micropython.const(2)
S3_CK1    = micropython.const(3)
S4_CK2    = micropython.const(4)
S5_CK3    = micropython.const(5)
S6_CK4    = micropython.const(6)
S7_STOP   = micropython.const(7)
S8_PKG    = micropython.const(8)
S9_ORIENT = micropython.const(9)
S10_EXPKG  = micropython.const(10)
S11_CK5    = micropython.const(11)
S12_ORIENT2 = micropython.const(12)
S13_END     = micropython.const(13)
S14_JUNC    = micropython.const(14)
S15_ADJ1   = micropython.const(15)


TURN_SPEED_MAX = 120.0
TURN_MIN_SPEED = 60.0
HEADING_TOL = 0.05

def wrap_to_pi(angle):
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle

class task_course_navigator:
    '''
    A class that represents a course navigator task. The task is responsible for reading
    data from encoders and calculating the course navigation of the Romi.
    '''

    def __init__(self, courseNavigatorGo: Share, X0_s: Share, X1_psi: Share, X2_omega_L: Share, X3_omega_R: Share, SE_timeValue: Share,
                 trajDataset: Share, leftMotorGo: Share, rightMotorGo: Share, lineFollowerGo: Share, stateEstimatorGo: Share, trajectoryGo: Share, RomiVelocity: Share,
                 leftVelocity: Share, rightVelocity: Share, X_global: Share, Y_global: Share, RomiHeadingPID: pid_controller, crashDetect: Queue, junctionFlag: Share):

        self._state: int = S0_INIT

        self._goFlag: Share = courseNavigatorGo

        self._X0_s: Share = X0_s
        self._X1_psi: Share = X1_psi
        self._X2_omega_L: Share = X2_omega_L
        self._X3_omega_R: Share = X3_omega_R
        self._timeValue: Share = SE_timeValue

        self._trajDataset: Share = trajDataset

        self._leftMotorGo: Share = leftMotorGo
        self._rightMotorGo: Share = rightMotorGo
        self._lineFollowerGo: Share = lineFollowerGo
        self._stateEstimatorGo: Share = stateEstimatorGo
        self._trajectoryGo: Share = trajectoryGo

        self._RomiVelocity: Share = RomiVelocity
        self._leftVelocity: Share = leftVelocity
        self._rightVelocity: Share = rightVelocity

        self._X_global: Share = X_global
        self._Y_global: Share = Y_global

        self._headingPID: pid_controller = RomiHeadingPID

        self._crashDetect: Queue = crashDetect

        self._junctionFlag: Share = junctionFlag

        self.PK_counter = 500
        self._target_heading = 0.0

        print("Course Navigator Task object instantiated")

    def configure_heading_pid(self):
        self._headingPID.update_Kp(2.5)
        self._headingPID.update_Ki(0.0)
        self._headingPID.update_Kd(0.0)
        self._headingPID.update_saturation(TURN_SPEED_MAX)

    def stop_motion(self):
        self._RomiVelocity.put(0)
        self._leftVelocity.put(0)
        self._rightVelocity.put(0)

    def orient_to_heading(self, target_heading):
        psi_now = self._X1_psi.get()
        psi_error = wrap_to_pi(target_heading - psi_now)

        turn_cmd = self._headingPID.PID_control(0.0, -psi_error)
        base = self._RomiVelocity.get()

        if turn_cmd > TURN_SPEED_MAX:
            turn_cmd = TURN_SPEED_MAX
        elif turn_cmd < -TURN_SPEED_MAX:
            turn_cmd = -TURN_SPEED_MAX

        if abs(psi_error) < HEADING_TOL:
            self._leftVelocity.put(base)
            self._rightVelocity.put(base)
            return True

        if 0.0 < turn_cmd < TURN_MIN_SPEED:
            turn_cmd = TURN_MIN_SPEED
        elif -TURN_MIN_SPEED < turn_cmd < 0.0:
            turn_cmd = -TURN_MIN_SPEED

        self._leftVelocity.put(base-turn_cmd)
        self._rightVelocity.put(base+turn_cmd)
        return False

    def _bump_detected(self):
        hit = False
        while self._crashDetect.any():
            self._crashDetect.get()
            hit = True
        return hit
    
    def run(self):
        '''
        Runs one iteration of the task
        '''
        while True:

            if self._state == S0_INIT:

                self._CheckpointXLocation = 0
                self._CheckpointYLocation = 0
                self._DT = 0

                self.configure_heading_pid()

                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                self._leftMotorGo.put(False)
                self._rightMotorGo.put(False)
                self._stateEstimatorGo.put(False)
                self._lineFollowerGo.put(False)
                self._trajectoryGo.put(False)
                self.stop_motion()

                if self._goFlag.get():

                    self._X_global.put(100)
                    self._Y_global.put(800)

                    self._stateEstimatorGo.put(True)
                    self._lineFollowerGo.put(True)
                    self._trajectoryGo.put(False)

                    self._leftVelocity.put(0)
                    self._rightVelocity.put(0)
                    self._leftMotorGo.put(True)
                    self._rightMotorGo.put(True)

                    self._RomiVelocity.put(400)

                    self._headingPID.zero()

                    self._state = S2_CK0

            elif self._state == S2_CK0:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    if self._X_global.get() >= 1475 + 30:
                        self._lineFollowerGo.put(False)
                        self._trajDataset.put(1)
                        self._trajectoryGo.put(True)
                        self._RomiVelocity.put(150)

                        self._state = S3_CK1
                    elif self._X_global.get() >= 1475 - 200:
                        self._X1_psi.put(0.0)
                        self._Y_global.put(800)
                        self._RomiVelocity.put(200)

            elif self._state == S3_CK1:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    self._bump_detected()
                    if self._X_global.get() <= 1300 + 20:
                        self._trajectoryGo.put(False)
                        self._RomiVelocity.put(150)

                        self._headingPID.zero()

                        self._target_heading = -pi
                        self.PK_counter = 100
                        self._state = S8_PKG

            elif self._state == S7_STOP:
                self.stop_motion()
                self._goFlag.put(False)
                self._state = S1_WAIT

            elif self._state == S8_PKG:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    self.PK_counter -= 1
                    self.orient_to_heading(self._target_heading)
                    
                    if self._bump_detected() or self.PK_counter <= 0:
                        self._RomiVelocity.put(-100)
                        self.PK_counter = 14

                        self._headingPID.zero()

                        self._state = S9_ORIENT

            elif self._state == S9_ORIENT:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    self.PK_counter -= 1
                    self.orient_to_heading(self._target_heading)

                    if self.PK_counter <= 0:
                        self.stop_motion()
                        self._target_heading = -pi / 2

                        self._headingPID.zero()

                        self._state = S10_EXPKG

            elif self._state == S10_EXPKG:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    if self.orient_to_heading(self._target_heading):             
                            self._lineFollowerGo.put(True)
                            self._RomiVelocity.put(150)
                            self._CheckpointYLocation = self._Y_global.get()
                            self._state = S4_CK2
                    
            elif self._state == S4_CK2:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    self.orient_to_heading(self._target_heading)
                    if self._Y_global.get() <= 100 - 50:
                        self.stop_motion()
                        self._target_heading = -pi
                        self._headingPID.zero()
                        self._state = S5_CK3
            
            elif self._state == S5_CK3:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    if self.orient_to_heading(self._target_heading):
                        self._RomiVelocity.put(200)
                        self._lineFollowerGo.put(True)

                        self.DT = self._X0_s.get()

                        self._state = S6_CK4

            elif self._state == S6_CK4:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    if self._X0_s.get() >= self.DT + 1825:
                        self._lineFollowerGo.put(False)
                        self.stop_motion()
                        
                        self._X1_psi.put(0.0)

                        self._target_heading = pi/2
                        self._headingPID.zero()

                        self._state = S14_JUNC
                    

            elif self._state == S14_JUNC:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    if self.orient_to_heading(self._target_heading):
                        if self._target_heading == pi/2:
                            self._target_heading = pi
                        else: 
                            self._X_global.put(400)
                            self._Y_global.put(500)
                            
                            self._trajDataset.put(2)
                            self._trajectoryGo.put(True)
                            self._RomiVelocity.put(150)
                            self._state = S11_CK5
                
            
            elif self._state == S11_CK5:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    if self._Y_global.get() >= 800 + 20:
                        self._trajectoryGo.put(False)
                        self.stop_motion()

                        self._target_heading = 0.0
                        self._headingPID.zero()

                        self._state = S15_ADJ1
                        
            elif self._state == S15_ADJ1:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:       
                    if self.orient_to_heading(self._target_heading):
                        self.stop_motion()  

                        self._goFlag.put(False)

                        collect()

                        self._state = S1_WAIT


            yield self._state
            