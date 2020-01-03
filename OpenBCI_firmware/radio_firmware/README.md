# FLUTTER.radio_firmware

Codes for OpenBCI Radios (Host+Device)

## Instructions

Tutorial for uploading can be followed at: http://docs.openbci.com/Hardware/06-Cyton_Radios_Programming_Tutorial
Use 1.0.0 firmware version

Code for FLUTTER: OpenBCI_32bit_Device_new.ino for turning the transmitter OFF for 1s (for every 'M' transmitted)
Other codes are default codes in shipped OpenBCI form

Steps to Upload:
1. Upload Dongle_Pass_Thru w/ Host in RESET position and NO serial connection to Device (through Capacitor connections)
2. Connect in serial with Device (through Capacitor connections), switch to GPIO6, and Device is ON, upload Device code
3. Set Host w/ RESET position and NO serial connection, and upload Host code



## Troubleshoot
1. Restart Arduino if Arduino doesn't recognize port
2. Problems with device code upload - try multiple times, hold down jumper wires strongly, check capacitors and connections
3. You can try connecting to 3V3 pin of FTDI board to RFDuino Pin 13.
4. If Device radio program doesn't work after multiple attempts, solder (i) RFRX/RFTX/RFRST/GNS pins of device and (ii) GPIO6/RX/TX pins of Dongle. I know it sounds ridiculous but it actually worked twice like that.
