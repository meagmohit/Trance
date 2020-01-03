# FLUTTER.Device_Firmware

Codes to program OpenBCI PIC32 to perform on-board FLUTTER command detection when Raw data is passed through PC

## Instruction to program

Location 1: Documents/Arduino/hardware/chipkit-core/pic32/
Location 2: Documents/Arduino/libraries

1. location_1:boards.txt: openbci.build.f_cpu = 40000000L or 6000000L (40MHz is deafult, and 6MHz for low-power mode)

2. similar change in location_1:variants/openbci/boards.txt

3. location_1:cores/pic32/Hardware_Serial.cpp : #define LOW_HIGH_BAUD_SPLIT should be 200000 for the default mode, and 100000 for the low-power mode

4. OpenBCI_32_Daisy files should be updated for the blink code in location_2. 

5. Restart Arduino

6. Program by (i) turning-off the OpenBCI (ii) press hold RST and PROG button (iii) turn ON the OpenBCI (iv) Release RST (v) RELEASE PROG (vi) program using Arduino

## Additional Notes

1. Do not ever fiddle around with the state of PGC pin in Arduino code as it messes up the bootloader code.
