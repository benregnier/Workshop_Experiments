

# CircuitPython for MTM Computer

## Setup

1. Visit https://circuitpython.org/board/raspberry_pi_pico/ and download the UF2
   file for the most recent version of CircuitPython (currently 9.0.5)
   
2. Plug MTM Computer module into your PC and Put the MTM Computer in 
   RPI-RP2 UF2 bootloader mode
   (e.g. hold BOOTSEL button under main knob + RESET button then release BOOTSEL)
   
3. Drag CircuitPython UF2 file to the RPI-RP2 drive.  
   Your computer may indicate a write error, this is normal with UF2 bootloading
   
   
4. Wait a few seconds and a CIRCUITPY drive will appear on your desktop

5. Copy the `mtm_computer.py` file to the CIRCUITPY drive.

6. Copy one of the demo `*_demo_code.py` files to CIRCUITPY drive. Rename it `code.py`.

7. Open up a serial terminal to the CircuitPython REPL
    (using Mu, Thonny, tio, screen) to see the output of the demo 


## Demos

- [`leds_demo_code.py`](./leds_demo_code.py) 
  -- Simple chase pattern on the six LEDs

- [`knob_demo_code.py`](./knob_demo_code.py) 
  -- Read all mux inputs (knobs, switch, CVs ins), 
   and mirror them on the LEDs, X-knob and Y-knob send out CV too
   
- [`two_cv_sines_code.py`](./two_cv_sines_code.py)
  -- Output two sine waves on CV 1 and CV 2, controllable with knobs & switch
  

## Implmentation notes

- The structure of the library as it stands copies from the `SimpleMidi_0.1.ino` 
  Arduino sketch by Tom.
  
- There is a `computer.update()` method that should be called regularly to update
  the state of the mux-based inputs. 
  
- Audio out via the SPI DAC is not currently supported in CircuitPython. 
  (CircuitPython supports I2S, PWM, and built-in DACs, but not this SPI DAC)
  But I believe analog CV values can be output since the DAC jacks are DC-coupled.
  
- The LEDs in the library are currently set up as PWM, so their brightness can be varied.
  This may be revisted if we run out of PWM resources.

- An easy way to put the Computer into UF2 mode without pulling off the knob
  is to paste this into the REPL: 
  ```py
  import microcontroller 
  microcontroller.on_next_reset(microcontroller.RunMode.BOOTLOADER)
  microcontroller.reset()
  ```
- And to save the UF2 of the entire CircuitPython setup, do:
  `picotool save -r 0x10000000 0x10200000  my_app.uf2`
  Note that this saves only the first 2MB of the 16MB flash. This should be fine
  unles you're storing lots of WAV files or other assets.
  
