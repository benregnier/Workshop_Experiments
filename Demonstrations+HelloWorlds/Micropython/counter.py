from machine import Pin
import time

led_pins = [Pin(p + 10, Pin.OUT) for p in range(6)]
DEFAULT_PAUSE = 100
time.sleep_ms(DEFAULT_PAUSE)


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


[l.value(0) for l in led_pins]

time.sleep_ms(500)

for i in range(64):
    light_number(i)
    time.sleep_ms(DEFAULT_PAUSE)

for i in [0, 10, 20, 8, 34, 28, 63, 7, 48]:
    light_number(i)
    time.sleep_ms(DEFAULT_PAUSE)

for i in [4, 12, 63, 15, 0]:
    light_number(i)
    time.sleep_ms(DEFAULT_PAUSE)

for i in [20, 5, 15, 0, 28, 9, 11, 0]:
    light_number(i)
    time.sleep_ms(DEFAULT_PAUSE)
