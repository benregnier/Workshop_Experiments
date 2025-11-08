"""
Demonstrate how to set the LEDs on MTM Computer
"""
import time

from mtm_computer import Computer

computer = Computer()

print('hello')
i = 0
while True:
    print("led on", i)
    computer.leds[i-1].duty_cycle = 0
    computer.leds[i].duty_cycle = 65535  # LEDs are PWM-able, 65535 = max bright
    i = (i+1) % len(computer.leds)
    time.sleep(0.2)

