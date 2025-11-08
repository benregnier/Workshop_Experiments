"""
beatfader_demo.py -- fade between different drum loops
originally from
https://github.com/todbot/circuitpython-tricks/blob/main/larger-tricks/beatfader.py

Demonstrate how to play WAVs via the pulse outputs

Functions:
- main knob -- master "fader" between different playing WAV loops
- X knob -- select "tilt" to favor main knob or CV 1 in as loop fader
- switch -- up to mute
- CV 1 in -- adds to main knob fader
- CV 1 & 2 out -- PWM stereo audio out
- LEDs -- reflects knobs and CV inputs

To use:
- Copy over the "mtp_computer.py" library 
- Copy the "beatfader_wavs" directory at this URL to the CIRCUITPY drive
https://github.com/todbot/circuitpython-tricks/tree/main/larger-tricks/beatfader_wavs
- Or use your own loops, prefer 22050 kHz sample rate and up to around 10 or so

"""
import os
import time
import math
import audiocore
from mtm_computer import Computer, map_range, gamma_correct

wav_dir = "/beatfader_wavs"

comp = Computer()

def load_wavs(wav_dir):
    # find all the wavs in a given dir
    wav_fnames =[]
    for fname in os.listdir(wav_dir):
        if fname.lower().endswith('.wav') and not fname.startswith('.'):
            wav_fnames.append(wav_dir+"/"+fname)

    # determine their sample rate and channel count & set comp audio to match
    with audiocore.WaveFile(wav_fnames[0]) as wav:
        comp.pulse_outs_to_audio(sample_rate=wav.sample_rate,
                                 voice_count=len(wav_fnames),
                                 channel_count=wav.channel_count)

    # start them playing, in sync, but quiet
    for i, fname in enumerate(wav_fnames):
        wave = audiocore.WaveFile(fname)
        comp.mixer.voice[i].level = 0.0      # start quiet
        comp.mixer.voice[i].play(wave, loop=True) # start playing

    return wav_fnames

wav_fnames = load_wavs(wav_dir)

num_wavs = len(wav_fnames)
last_time = 0
fader_n = num_wavs * 0.8  # amount of overlap
fader_m  = 1/(num_wavs-1) # size of slice
 
while True:
    comp.update()
    
    if comp.switch > 60000:   # up position
        for voice in comp.mixer.voice:
            voice.level = 0
    elif comp.switch < 1000:  # momentary down position
        pass
    else:
        tilt = comp.knob_x / 65535
        knob_pct = (1-tilt)*(comp.knob_main / 65535) + tilt*(comp.cv_1_in / 65535)
        for i,voice in enumerate(comp.mixer.voice):
            voice.level = min(max( 1 - (fader_n * (knob_pct - fader_m*i))**2, 0), 1)

    # LED0 reflects knob & cv position
    comp.leds[0].duty_cycle = gamma_correct(comp.knob_main)
    comp.leds[1].duty_cycle = gamma_correct(comp.knob_x)
    comp.leds[2].duty_cycle = gamma_correct(comp.knob_y)
    comp.leds[4].duty_cycle = gamma_correct(comp.cv_1_in)
    comp.leds[5].duty_cycle = gamma_correct(comp.cv_2_in)
    
    # Print out the input state periodically
    if time.monotonic() - last_time > 0.2:
        last_time = time.monotonic()
        print("knobs:", comp.knob_main, comp.knob_x, comp.knob_y,
              " cvin:", comp.cv_1_in, comp.cv_2_in)

