import uasyncio
from cv import led1, led2, led6, PulseOut, PulseInputs, BLINK_LENGTH

pulses = PulseInputs()


async def toggle_led1():
    led1.value(1)
    await uasyncio.sleep_ms(BLINK_LENGTH)
    led1.value(0)


async def toggle_led2():
    led2.value(1)
    await uasyncio.sleep_ms(BLINK_LENGTH)
    led2.value(0)


async def blink():
    """
    Simple task that shows the board is responding
    """
    while True:
        led6.on()
        pulse_out_1.toggle()
        pulse_out_2.toggle()
        await uasyncio.sleep_ms(BLINK_LENGTH)
        led6.off()
        await uasyncio.sleep_ms(400)


pulses.add_handler(0, toggle_led1)
pulses.add_handler(1, toggle_led2)

pulse_out_1 = PulseOut(1)
pulse_out_2 = PulseOut(2)

loop = uasyncio.get_event_loop()

loop.create_task(blink())
loop.create_task(pulses.handle_in(0))
loop.create_task(pulses.handle_in(1))

loop.run_forever()
