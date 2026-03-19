from line_sensor_driver import Line_Sensor as line_sensor_driver
from pid_controller     import PID_controller as pid_controller
from encoder_driver     import Encoder as encoder_driver
from task_share         import Share
import micropython

GAINS = {
    "400": (60.0, 0.0, 0.09),
    "300": (132.5, 0.0, 0.04),
    "200": (110,   0.0, 0.05)}

JUNC_HOLD = 8

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_RUN  = micropython.const(2)

class task_line_follower:

    def __init__(self, lineFollowerGo: Share,
                 llf: line_sensor_driver, rlf: line_sensor_driver,
                 lenc: encoder_driver, renc: encoder_driver,
                 pid: pid_controller,
                 RomiVelocity: Share, leftVelocity: Share, rightVelocity: Share,
                 junctionFlag: Share):

        self._state         = S0_INIT
        self._goFlag        = lineFollowerGo
        self._llf           = llf
        self._rlf           = rlf
        self._pid           = pid
        self._RomiVelocity  = RomiVelocity
        self._leftVelocity  = leftVelocity
        self._rightVelocity = rightVelocity
        self._junctionFlag  = junctionFlag
        self._lenc          = lenc
        self._renc          = renc

        print("Line Follower Task object instantiated")

    def run(self):
        while True:

            if self._state == S0_INIT:
                self.IW = 8
                self.MW = 16
                self.OW = 24
                self.bias      = 0
                self.MaxV      = 300
                self._junctionFlag.put(False)
                self._junc_clr = 0 
                self._last_turn = 0.0
                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                if self._goFlag.get():
                    v  = abs(int(self._RomiVelocity.get()))
                    if v >= 400:
                        Kp, Ki, Kd = GAINS["400"]
                    elif v >= 300:
                        Kp, Ki, Kd = GAINS["300"]
                    else:
                        Kp, Ki, Kd = GAINS["200"]

                    self._pid.update_saturation(0.75 * abs(self._RomiVelocity.get()))
                    self._pid.update_Kp(Kp)
                    self._pid.update_Ki(Ki)
                    self._pid.update_Kd(Kd)
                    self._pid.zero()
                    self._junc_clr  = 0
                    self._last_turn = 0.0
                    self._state = S2_RUN

            elif self._state == S2_RUN:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    Li, Lm, Lo = self._llf.get_values()
                    Ri, Rm, Ro = self._rlf.get_values()

                    total = Li + Lm + Lo + Ri + Rm + Ro
                    lw = self.IW*Li + self.MW*Lm + self.OW*Lo
                    rw = self.IW*Ri + self.MW*Rm + self.OW*Ro
                    mx = max(Li, Lm, Lo, Ri, Rm, Ro)

                    if total >= ((mx * 0.95) * 6):
                        self._junctionFlag.put(True)
                        self._junc_clr = JUNC_HOLD
                        turn = self._last_turn
                    else:
                        if self._junc_clr > 0:
                            self._junc_clr -= 1
                            if self._junc_clr == 0:
                                self._junctionFlag.put(False)
                        normalized_error = (lw - rw) / total
                        turn = self._pid.PID_control(self.bias, normalized_error)
                        self._last_turn = turn

                    v = self._RomiVelocity.get()

                    lv = self._lenc.get_velocity()
                    rv = self._renc.get_velocity()
                    c_v = (lv + rv) / 2
                    
                    if self._RomiVelocity.get() - c_v > 20:
                        vs = c_v + 20
                    elif self._RomiVelocity.get() - c_v < -20:
                        vs = c_v - 20
                    else:
                        vs = self._RomiVelocity.get()

                    if vs + abs(turn) > self.MaxV:
                        vs = self.MaxV - abs(turn)

                    self._leftVelocity.put(vs + turn)
                    self._rightVelocity.put(vs - turn)

            yield self._state
