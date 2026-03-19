from pyb import ExtInt, Pin, enable_irq, disable_irq
from array import array
from task_share import Queue

# Four bump sensors, active-low, left to right from Romi's perspective:
# PC12, PD2, PC11, PC10  ->  ISR channels 0-3
BUMP_PINS = ('PC12', 'PD2', 'PC11', 'PC10')

class task_bumper:

    def __init__(self, crash_detect: Queue):
        self._cd = crash_detect

        # current DB state, previous DB state
        self._db_mask = array("H", [0x0000, 0x0000])

        self._callbacks = {i: ExtInt(
            Pin(BUMP_PINS[i]),
            ExtInt.IRQ_FALLING,
            Pin.PULL_UP,
            lambda src, i=i: self.callback(src, i))
            for i in range(len(BUMP_PINS))}

        print("Bumper Task object instantiated")

    def callback(self, ISR_src, channel):
        # Set channel bit in current debounce mask
        self._db_mask[0] |= 1 << channel
        # Disable until debounce period passes
        self._callbacks[channel].disable()
        # Notify navigator
        self._cd.put(channel, in_ISR=True)

    def run(self):
        while True:
            # Re-enable channels whose debounce period has elapsed
            for ISR_src in range(len(BUMP_PINS)):
                if self._db_mask[1] & (1 << ISR_src):
                    self._callbacks[ISR_src].enable()

            # Critical section: shift current->previous, clear current
            irq_state = disable_irq()
            self._db_mask[1], self._db_mask[0] = self._db_mask[0], 0x0000
            enable_irq(irq_state)

            yield
