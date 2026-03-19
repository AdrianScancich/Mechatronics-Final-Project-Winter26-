from pyb        import USB_VCP, Pin
from task_share import Share
from voltage_div_driver import Voltage_Divider as voltage_div_driver
import micropython

S0_INIT = micropython.const(0) # Init
S1_CMD  = micropython.const(1) # Wait for command
S2_COL  = micropython.const(2) # Course running

UI_prompt = ">: "

UI_menu = ("+------------------------------------------------------------------------------+\r\n"
           "| ME 405 Romi Menu                                                             |\r\n"
           "+---+--------------------------------------------------------------------------+\r\n"
           "| r | Run / stop course                                                        |\r\n"
           "| v | Read battery capacity                                                    |\r\n"
           "| User button toggles run/stop                                                 |\r\n"
           "+---+--------------------------------------------------------------------------+\r\n")

class task_user:

    def __init__(self, X1_psi: Share, X_global: Share, Y_global: Share,
                 Romi_Battery_Voltage: voltage_div_driver, courseNavigatorGo: Share):

        self._state              = S0_INIT
        self._ser                = USB_VCP()
        self._X1_psi             = X1_psi
        self._X_global           = X_global
        self._Y_global           = Y_global
        self._batt               = Romi_Battery_Voltage
        self._courseNavigatorGo  = courseNavigatorGo

        self._btn = Pin('PC13', Pin.IN, Pin.PULL_UP)
        self._btn_prev = 1

        self._ser.write("User Task object instantiated\r\n")

    def _print_results(self):
        self._ser.write("Course run complete, printing data...\r\n")
        self._ser.write("--------------------------------------------------------------------------------\r\n")
        self._ser.write(f"Global X: {self._X_global.get():.2f} mm\r\n")
        self._ser.write(f"Global Y: {self._Y_global.get():.2f} mm\r\n")
        self._ser.write(f"Psi:      {self._X1_psi.get():.4f} rad\r\n")
        self._ser.write("--------------------------------------------------------------------------------\r\n")

    def _btn_pressed(self):
        cur = self._btn.value()
        edge = (self._btn_prev == 1) and (cur == 0)
        self._btn_prev = cur
        return edge

    def run(self):
        while True:

            if self._state == S0_INIT: # Init
                self._ser.write(UI_menu)
                self._ser.write(UI_prompt)
                self._state = S1_CMD

            elif self._state == S1_CMD:
                start = False
                if self._ser.any():
                    inChar = self._ser.read(1).decode()
                    if inChar in {"r", "R"}:
                        self._ser.write(f"{inChar}\r\n")
                        start = True
                    elif inChar in {"v", "V"}:
                        self._ser.write(f"{inChar}\r\n")
                        v   = self._batt.get_voltage()
                        pct = self._batt.get_battery_percentage()
                        self._ser.write(f"Battery: {v:.2f} V  ({pct:.1f}%)\r\n")
                        self._ser.write(UI_prompt)

                if self._btn_pressed():
                    start = True

                if start:
                    self._ser.write("Course run requested...\r\n")
                    self._courseNavigatorGo.put(True)
                    self._ser.write(UI_prompt)
                    self._state = S2_COL

            elif self._state == S2_COL:
                stop = False
                if self._ser.any():
                    inChar = self._ser.read(1).decode()
                    if inChar in {"r", "R"}:
                        self._ser.write(f"{inChar}\r\n")
                        stop = True

                if self._btn_pressed():
                    stop = True

                if stop:
                    self._courseNavigatorGo.put(False)
                    self._ser.write("Stopping course run...\r\n")

                if not self._courseNavigatorGo.get():
                    self._print_results()
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD

            yield self._state
