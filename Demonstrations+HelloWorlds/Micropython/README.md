# Micropython

_Note: In addition to the micropython code in this folder, see MJLMills' [pyworkshopsystem](https://github.com/MJLMills/pyworkshopsystem/) library and the [examples by rwmodular](https://github.com/rwmodular/computer)._

## Setup

Connect the Computer to your computer with a keyboard. This should mount it as a disk.

Download the micropython uf2 file from & drag

https://www.raspberrypi.com/documentation/microcontrollers/micropython.html

Drag it to the Computer. This should eject the disk. The board is now running Micropython.

[Thonny](https://thonny.org/) is sometimes useful as is `[picotool](https://github.com/raspberrypi/picotool) info`. If you use VSCode [MicroPico](https://marketplace.visualstudio.com/items?itemName=paulober.pico-w-go) is quite nice.

## Connecting

https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-python-sdk.pdf

If you want to see LEDs lighting up, you may need to power the board from your eurorack supply as well as have the USB-C connected for programming it.

## Demos


### `cv.py`
Playing with `uasyncio` to handle pulses & the multiplexed IO coming in to Computer. Copy this file to the Computer to run the tests, kind of the beginnings of a library...

### `lights.py`
Flash the LEDs in various patterns.

### `counter.py`
Experiment with a binary counter on the LEDs - counts up to 64. Needs `lights.py` on the Computer as well as `cv.py`.

### `pulse_tests.py`
Simple tests of the pulse IO.

### `mult_tests.py`
Simple tests of the multiplexed IO. Needs `lights.py` on the Computer as well as `cv.py`.

### `computer/`
Starting to think about how to structure as a library. Wondering if there are things we could expand on from [EuroPi](https://github.com/Allen-Synthesis/EuroPi).

TODO: https://docs.micropython.org/en/latest/reference/packages.html
