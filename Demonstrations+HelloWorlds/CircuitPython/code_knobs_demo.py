"""
Demonstrate how to update and read the inputs on MTM Computer
"""
import time
from mtm_computer import Computer

comp = Computer()

last_time = 0

def gamma_correct(x):
    """simple and dumb brightness gamma-correction"""
    return int((x*x)/65535)

while True:
    comp.update()

    # LEDs reflect state of knobs/CV
    comp.leds[0].duty_cycle = gamma_correct(comp.knob_main)
    comp.leds[1].duty_cycle = gamma_correct(comp.knob_x)
    comp.leds[2].duty_cycle = gamma_correct(comp.knob_y)
    comp.leds[3].duty_cycle = gamma_correct(comp.switch)
    comp.leds[4].duty_cycle = gamma_correct(comp.cv_1_in)
    comp.leds[5].duty_cycle = gamma_correct(comp.cv_2_in)

    # CV outs echo the values of the X,Y knob
    comp.cv_1_out = comp.knob_x
    comp.cv_2_out = comp.knob_y

    # Print out the input state periodically
    if time.monotonic() - last_time > 0.2:
        last_time = time.monotonic()
        #print("analog:", comp.analog)  # can also see analog array
        print("knobs:", comp.knob_main, comp.knob_x, comp.knob_y,
              " cvin:", comp.cv_1_in, comp.cv_2_in)

