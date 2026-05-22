# Oscilloscope
## HW

- ADS1263 is soldered on a Waveshare AD HAT
- AVDD connects to RPI_5V through a Zero Ohm resistor
- RPI_5V feeds an LDO (100uV RMS noise) that converts AVDD into 3.3V
- 3.3 V is sent (via a protection circuit) to VCC 
- VCC is filtered via a 1uF capacitor and used to supply the digital part of the ADC

AVDD -> 0 Ohm -> RPI_5V -> LDO -> 3.3V -> 1uF Cap. Filter -> 

Q. Is offset related to PGA GAIN 
A. PGA input offset voltage — every amplifier stage has a small intrinsic offset (Vos). The ADS1263 datasheet specifies ~10 µV typical input-referred. At gain ×32 that becomes ×32 × 10 µV = 320 µV of apparent offset — before any signal.

## Pre-Requisite
### Rpi RT Kernel

## Task Definition 

### Acquisition Task
### Shared Memory
### Oscilloscope View