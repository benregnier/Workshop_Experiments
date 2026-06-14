# Frequency Shifter

This frequency shifter is designed for feedback patching and experimental use, similar to the Red Panda 'Radius' pedal. 

## Controls
- **Main + Switch** : frequency shift amount/range
  - **Switch Up** = wide, log-scaled bipolar range
  - **Switch Middle** = narrower bipolar linear range
  - **Switch Down** = same narrow shift range, and enables feedback mode cycling from PulseIn1
- **X** : internal feedback amount
- **Y** : AudioIn2 mix into AudioIn1

## Inputs
- **AudioIn1** : primary audio input
- **AudioIn2** : secondary audio mixed by Y
- **CVIn1** : shift modulation (added to Main)
- **CVIn2** : combined feedback path blend (down/up balance)
- **PulseIn1** : when switch is Down, rising edges cycle feedback mode: down -> up -> combined
- **PulseIn2** : currently ignored

## Outputs
- **AudioOut1** : low sideband output
- **AudioOut2** : high sideband output
- **CV outputs** : unused for now
- **Pulse outputs** : unused for now

## Notes
- Includes a RP2040 overclock request to 250 MHz for extra DSP headroom.
