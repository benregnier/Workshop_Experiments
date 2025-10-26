from machine import Pin
import time

led_pins = [Pin(p + 10, Pin.OUT) for p in range(6)]

DEFAULT_PAUSE = 300


def all_leds_off():
    for led in led_pins:
        led.value(0)


def timed_led_toggle(leds):
    # leds can be either an integer or a tuple of integers
    if type(leds) == type(0):
        leds = (leds,)

    for i in leds:
        led_pins[i].value(1)

    time.sleep_ms(DEFAULT_PAUSE)

    for i in leds:
        led_pins[i].value(0)


def led_pattern(sequence, times=1):
    for _ in range(times):
        for s in sequence:
            timed_led_toggle(s)
    all_leds_off()


def spinner(times=1):
    led_pattern([0, 1, 3, 5, 4, 2], times)


def ping_pong(times=1):
    led_pattern([(0, 4, 2), (1, 3, 5)], times)


def blink_led(led, times=1):
    for _ in range(times):
        timed_led_toggle(led)
        time.sleep_ms(DEFAULT_PAUSE)
    all_leds_off()


def convert(n, base=2):
    string = "0123456789ABCDEF"
    if n < base:
        return string[n]
    else:
        return convert(n // base, base) + string[n % base]


def zfl(s, width=6):
    return "{:0>{w}}".format(s, w=width)


def light_number(num):
    number = zfl(convert(num))
    for l, v in enumerate(reversed(number)):
        led_pins[l].value(int(v))
