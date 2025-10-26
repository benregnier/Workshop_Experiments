from machine import Pin
import time
from random import choice

led_pins = [Pin(p + 10, Pin.OUT) for p in range(6)]

DEFAULT_PAUSE = 300


def all_leds_off():
    for led in led_pins:
        led.value(0)


def led_toggle(leds, value=0):
    # leds can be either an integer or a tuple of integers
    if type(leds) == type(0):
        leds = (leds,)

    for i in leds:
        led_pins[i].value(value)


def timed_led_toggle(leds):
    led_toggle(leds, 1)
    time.sleep_ms(DEFAULT_PAUSE)
    led_toggle(leds, 0)


def led_pattern(sequence, times=1):
    for _ in range(times):
        for s in sequence:
            timed_led_toggle(s)
    all_leds_off()


def spinner(times=1):
    led_pattern([0, 1, 3, 5, 4, 2], times)


def ping_pong(times=1):
    led_pattern([(0, 4, 2), (1, 3, 5)], times)


def red_arrows(times=1):
    led_pattern([(0, 3, 4), (1, 2, 5)], times)


def blink_led(led, times=1):
    for _ in range(times):
        timed_led_toggle(led)
        time.sleep_ms(DEFAULT_PAUSE)
    all_leds_off()


def random_led(times=1):
    p1 = None
    p2 = None
    for _ in range(times):
        p1 = choice(led_pins)
        p2 = choice(led_pins)
        p1.value(1)
        time.sleep_ms(DEFAULT_PAUSE)
        p2.value(1)
        time.sleep_ms(DEFAULT_PAUSE)
        p1.value(0)
        time.sleep_ms(DEFAULT_PAUSE)
        p2.value(0)
    all_leds_off()


pairs_pattern = {
    "down": [(0, 1), (2, 3), (5, 4), (2, 3)],
    "up": [(5, 4), (2, 3), (0, 1), (2, 3)],
}


def pairs(times, direction="down"):
    for _ in range(times):
        led_pattern(pairs_pattern[direction])


def held_pairs(level=0, direction="down"):
    for i in range(level):
        led_toggle(pairs_pattern[direction][i], value=1)
        time.sleep_ms(DEFAULT_PAUSE)


loops = [
    sorted(range(6), key=lambda x: [x % 2, x]),
    range(6),
    sorted(range(6), key=lambda x: [x % 2, x], reverse=True),
    [0, 1, 3, 5, 4, 2],
    [(0, 1), (2, 3), (5, 4), (2, 3), (0, 1)],
]

for l in loops:
    led_pattern(l)

spinner(5)
ping_pong(5)
red_arrows(5)
blink_led(5, times=10)
random_led(5)
pairs(5)
red_arrows(5)
pairs(5, "up")
all_leds_off()
held_pairs(3, direction="up")
time.sleep_ms(500)
all_leds_off()
held_pairs(2, direction="down")
time.sleep_ms(500)
all_leds_off()
held_pairs(2, direction="up")
time.sleep_ms(500)
all_leds_off()
held_pairs(3, direction="down")
time.sleep_ms(500)
all_leds_off()
