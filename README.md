This project was part of a CNC build which has used a low-voltage high speed
spindle as a tool. The spindle type (an EWL 4025) draws rather large currents
in combination with very low voltages. This can't be handled by most standard
VFDs and requires special (rather expensive) HF drives in a price range well
above 1000 bucks.

The application is based on a STM32F405 and Microchips appnote AN955 which
describes the implementation of a space vector control for ASM machines.

Further details can be found here:
https://higaski.at/iku-variable-frequency-drive/