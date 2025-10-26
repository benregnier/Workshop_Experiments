# DSP Graph Demo

This is a demo / template project for setting up a DSP graph.  It uses a version of DSPatch

https://flowbasedprogramming.com/html/index.html

with the multithreading code stripped out so that it will compile for RP2xxx.


The demo provides DSP blocks for ADC, DAC, a ringmod and a very basic saw oscillator.  It sets up a graph that ringmods the saw with the signal from the ADC, and sends the result to the DAC channels.

The DSPatch system is very expandable, so from this template it should be easy to add more components.


# Setup

This folder has submodules, so you need to run something like

```
git submodule update --init --remote --merge --recursive
```

