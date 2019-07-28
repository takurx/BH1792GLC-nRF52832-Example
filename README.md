# BH1792GLC-nRF52832-Example

## Problem, to do
- need to refatoring (delete commented code, needless code/function)
- intterrupt 32.258 Hz, more approch to 32.0 Hz (need to control micro second)
- Or optimize iir filter const in pwCalc_Init()
- more control precisely intensity of LED (currently optimized BH1970GLC, BH1792GLC can control precisely)
- need to control IR (turn on when in darkness, regardless with/without finger)

## BH1792GLC
https://www.rohm.co.jp/sensor-shield-support/pulse-wave-sensor2

## Parts
- nRF52DK board
- BH1792GLC evaluation board
- resistor 1k ohm x3
- cable for conncection between pin x6

## Figure, schematic
![schematic connection between nRF52DK and BH1792GLC](https://github.com/takurx/BH1792GLC-nRF52832-Example/raw/master/documentation_addtional_BH1792GLC/figure_connection_schematic.jpg "schematic connection between nRF52DK and BH1792GLC")

## Figure, physics
![physics connection between nRF52DK and BH1792GLC](https://github.com/takurx/BH1792GLC-nRF52832-Example/raw/master/documentation_addtional_BH1792GLC/figure_connection_physics.jpg "physics connection between nRF52DK and BH1792GLC")
