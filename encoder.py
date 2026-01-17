from machine import Pin

class Encoder:
    """
    Simple quadrature encoder handler for a 2-pin encoder.
    Counts rising edges on pin A and uses pin B to determine direction.
    """

    def __init__(self, pin_a_num, pin_b_num, pull_up = False):
        self.count = 0

        # Configure pins
        if pull_up:
            self.pin_a = Pin(pin_a_num, Pin.IN, Pin.PULL_UP)
            self.pin_b = Pin(pin_b_num, Pin.IN, Pin.PULL_UP)
        else:
            self.pin_a = Pin(pin_a_num, Pin.IN)
            self.pin_b = Pin(pin_b_num, Pin.IN)

        # Attach ISR
        self.pin_a.irq(trigger=Pin.IRQ_RISING, handler=self._pin_a_isr)

    def _pin_a_isr(self, pin):
        # ISR updates count depending on the state of pin B
        if self.pin_b.value() == 1:
            self.count += 1
        else:
            self.count -= 1

    def read(self):
        """Return the current encoder count."""
        return self.count

    def reset(self):
        """Reset the encoder count to zero."""
        self.count = 0