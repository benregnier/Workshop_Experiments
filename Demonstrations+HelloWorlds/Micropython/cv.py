import uasyncio
from machine import Pin, ADC, SPI
import time
from counter import light_number

led1 = Pin(10, Pin.OUT)
led2 = Pin(11, Pin.OUT)
led3 = Pin(12, Pin.OUT)
led4 = Pin(13, Pin.OUT)
led5 = Pin(14, Pin.OUT)
led6 = Pin(15, Pin.OUT)

BLINK_LENGTH = 15

# https://gist.github.com/tomwaters/f159c749d32899cb2c5b067e8ef2615e
# a = ADC()
# cv1_out = PWM(Pin(23), freq=60000, duty_u16=32768)
# cv2_out = PWM(Pin(23), freq=60000, duty_u16=32768)

# ADC0 = audio in 1 GPIO26
# ADC1 = audio in 2 GPIO27
# ADC3 = CV via Mux
# CV1 = 00 & 10 CV2 = 01 & 11
# DAC = GPIO18, GPIO19, GPIO21

# https://docs.micropython.org/en/latest/rp2/quickref.html#software-spi-bus


class MultiplexedInputs:
    """
    Base class for all inputs via the multiplexer:

     - y knob ADC2 via mult. 10
     - x knob ADC2 via mult. 01
     - z switch ADC2 via mult. 11
     - CV in 1 via ADC3 mult. 00 & 10
     - CV in 2 via ADC3 mult. 01 & 11

    https://pdf1.alldatasheet.com/datasheet-pdf/view/454218/UTC/4052.html
    https://github.com/TomWhitwell/Hello_Computer/blob/main/Demonstrations%2BHelloWorlds/CircuitPython/mtm_computer.py#L36
    """

    def __init__(self):
        self.mux_logic_a = Pin(24, Pin.OUT)
        self.mux_logic_b = Pin(25, Pin.OUT)
        self.mux_io_1 = ADC(28)
        self.cv_in = ADC(29)
        self.table = {
            "main": (0, 0),
            "x": (0, 1),
            "y": (1, 0),
            "switch": (1, 1),
            "cv1": (0, 0),
            "cv2": (0, 1),
        }

    def read(self, component="main"):
        """
        Read the value from a knob/switch. Returns value as percent 0-100.
        """
        a, b = self.table[component]

        self.mux_logic_a.value(a)
        self.mux_logic_b.value(b)

        if component in ("cv1", "cv2"):
            value = self.cv_in.read_u16()
        else:
            value = self.mux_io_1.read_u16()
        return int(100 * value / 65535)

    def switch(self):
        value = self.read("switch")
        if value > 50:
            return "up"
        elif value < 10:
            return "down"
        else:
            return "middle"


# # setup cv output
# # -6v = 0, 0v = 32768, +6v = 65535
# cv1_out = PWM(Pin(23), freq=60000, duty_u16=32768)
class PulseIn(Pin):
    def __init__(self, input=1):
        assert input in (1, 2)
        super().__init__(input + 1, Pin.IN, Pin.PULL_UP)


class PulseOut(Pin):
    def __init__(self, output=1):
        assert output in (1, 2)
        super().__init__(output + 7, Pin.OUT)

    def toggle(self):
        self.value(0 if self.value() else 1)


class PulseInputs:
    """
    asyncio event based handler for the two pulse inputs on Computer
    """

    def __init__(self) -> None:
        in_1 = PulseIn(1)
        in_2 = PulseIn(2)
        self.events = {
            str(in_1): uasyncio.Event(),
            str(in_2): uasyncio.Event(),
        }
        self.handlers = {}
        in_1.irq(handler=self._handler, trigger=Pin.IRQ_FALLING)
        in_2.irq(handler=self._handler, trigger=Pin.IRQ_FALLING)

    def _handler(self, pin):
        name = str(pin)
        pin.irq(handler=None)
        self.events[name].set()
        time.sleep_ms(30)
        self.events[name].clear()
        pin.irq(handler=self._handler, trigger=Pin.IRQ_FALLING)

    def pins(self):
        return list(self.events.keys())

    def add_handler(self, pulse_in, fn):
        name = self.pins()[pulse_in]
        self.handlers[name] = fn

    async def handle_in(self, pin_idx=0):
        while True:
            name = pulses.pins()[pin_idx]
            await self.events[name].wait()
            await self.handlers[name]()


mults = MultiplexedInputs()


async def light_switch():
    """
    The switch will light up an led when turned on, and those
    leds will turn off if the switch is put on momentarily
    """
    lights = [led1, led2, led3, led4, led5, led6]
    lights_on = []
    while True:
        sw = mults.switch()
        print(f"switch is {sw} {lights}:{lights_on}")
        if sw == "up":
            try:
                l = lights.pop()
                l.value(1)
                lights_on.append(l)
            except IndexError:
                pass
        elif sw == "down":
            for l in lights_on:
                l.value(0)
                lights.append(l)
            lights_on = []
        await uasyncio.sleep_ms(500)


async def light_counter():
    """
    Increment the count if the switch is up every half second.
    Reset it if the switch is held down.
    Use the LEDs to display a simple binary counter.
    """
    count = 0
    while True:
        sw = mults.switch()
        if sw == "up":
            count = count + 1 if count < 63 else 63
            await uasyncio.sleep_ms(500)
            print(f"switch is {sw} count:{count}")
            light_number(count)
        elif sw == "down":
            count = 0
            print(f"count reset!")
            light_number(count)
            await uasyncio.sleep_ms(10)


loop = uasyncio.get_event_loop()

# loop.create_task(light_switch())
loop.create_task(light_counter())

loop.run_forever()
