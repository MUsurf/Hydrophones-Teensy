# Hydrophones-Teensy
Mizzou SURF's hydrophone code that runs on the Teensy 4.1 microcontroller

## Flashing Code onto the Teensy
1. Donwload and install vscode and the PlatformIO Extension for vscode
2. Download this repository onto your computer
3. Using the PlatformIO extension, open the project in vscode
4. Connect the Teensy to your computer via a USB-A to micro-USB cable
5. Click the "PlatformIO: Upload" button on the bottom blue bar in vscode. It looks like an arrow.
6. Assuming all goes well, it will compile and upload the project to the Teensy


## Connecting to the Jetson
1. Flash the program onto the Teensy (see above)
2. Connect the SDA1 pin on the Teensy to one of the SDA pins on the Jetson
3. Connect the SCL1 pin on the Teensy to one of the SCL pins on the Jetson
4. Connect the Vin pin on the Teensy to 5V on the Jetson
5. Connect a GND on the Teensy to a GND on the Jetson
6. Boot the Jetson (if not already). The Teensy should start to flash an orange LED every second to indicate it is on
7. From the Jetson, check if the Teensy is detected. It should show up on I2C address 0x69
8. Assuming all goes well, you should be able to read an 8-bit signed integer from the Teensy which indicates the delay value
