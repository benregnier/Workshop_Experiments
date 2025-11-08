"""
Demonstrate how to do some simple CV output

Functions:
- main knob -- controls frequency of CV outs
- X knob -- amplitude of CV 1 out
- Y knob -- amplitude of CV 2 out
- switch -- phase difference of X/Y
- LEDs -- brightness mirrors state of knobs, bottom two LEDs show CV output

"""
import time
import math
from mtm_computer import Computer, map_range

comp = Computer()

last_time = 0

def gamma_correct(x):
    """simple and dumb brightness gamma-correction"""
    return int((x*x)/65535)

t = 0
dt = 0.01

while True:
    comp.update()

    dt = map_range(comp.knob_main, 0,65535, 0.0001, 0.1)
    phase = map_range(comp.switch, 0,65535, 0, math.pi*2)
    a1 = comp.knob_x / 2 
    a2 = comp.knob_y / 2
    comp.cv_1_out = int( a1 * math.sin(t) + a1 )
    comp.cv_2_out = int( a2 * math.sin(t + phase) + a2 )
    t += dt
    
    # LEDs reflect state of knobs/CV
    comp.leds[0].duty_cycle = gamma_correct(comp.knob_main)
    comp.leds[1].duty_cycle = gamma_correct(comp.knob_x)
    comp.leds[2].duty_cycle = gamma_correct(comp.knob_y)
    comp.leds[3].duty_cycle = gamma_correct(comp.switch)
    comp.leds[4].duty_cycle = gamma_correct(comp.cv_1_out)
    comp.leds[5].duty_cycle = gamma_correct(comp.cv_2_out)
    
    # Print out the input state periodically
    if time.monotonic() - last_time > 0.2:
        last_time = time.monotonic()
        print("knobs:", comp.knob_main, comp.knob_x, comp.knob_y,
              " cvin:", comp.cv_1_in, comp.cv_2_in)

