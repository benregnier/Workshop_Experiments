import uasyncio
from counter import light_number
from cv import MultiplexedInputs

mults = MultiplexedInputs()


async def test_mux_read():
    while True:
        for c in mults.table.keys():
            print(f"{c} is {mults.read(c)}")
        print(f"switch() is {mults.switch()}")
        await uasyncio.sleep_ms(1000)


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
loop.create_task(test_mux_read())

loop.run_forever()
