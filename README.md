# PulseOxThermo
Code for the second Ardunio (which runs the Pulse Oximeter) of the athelas system. 

## Current Pulse Ox File to Use
PulseOxThermo -> RD117_ARDUINO -> RD117_ARDUINO.ino

### ROS and Arduino Library Requirements
- rosserial 0.7.9 must be used.
	- Higher ver. use cstring which arduino doesn't use or some such.

#### Pinouts & Wire Runs
PulseOx:
- Interrupt Pin is digital pin 10 on Arduino
- All others as labelled

#### Pulse Ox Register Values to be effective

These settings control the brigtness for the LEDs and the adc conversion rate. 

The following values can be found in the [max30102.cpp](RD117_ARDUINO/max30102.cpp) source file, specifically lines [Lines 136 - 141](https://github.com/athelas-NEU/PulseOxThermo/blob/918f1fb7174216ff736ba9c4da13137edc1819e9/RD117_ARDUINO/max30102.cpp#L136-L141).

- adc for both leds 0x27
- brightness for both leds 0x3F 

Of note is the brightness needing to be adjusted manually in order to compensate for skin tone variability. The team adjusted the brightness such that it worked for our Athelas members, but it may need to be calibrated differently

### Plan So Far
1. All the biosensors hooked up to the arduino will publish to the Jetson Nano ROS node all the time.
2. The Jetson Nano ROS will subscribe to the biosensor data and decide whether to gather it or not.
  - In this way the I2C toggling between data feeds won't need to be an issue.
3. The distance sensor will always publish it's data as well, and the Jetson Nano will use the data when **Phase 2** movement stage is in progress.
4. The reason for the **Always Publish** model is so that the Jetson Nano won't need to send a command to the biosensor arduino. It can be totally passive in that regard.
  - Would be nice to turn the sensors off, but lack of time makes it a future issue.
  
### Immediate Next Steps
1. ~~Add the sensor_control.ino code into main code loop.~~
  - ~~Remove the json parser code from this as it's no longer needed.~~
2. ~~ROS Publisher model implemented for PulseOx and Temp sensor.~~
2. ~~Remove extraneous print statements from code loop. ~~
3. ~~Adjust inner most PulseOx loop as it always runs, ~~
    - **Or adjust it so safe guards always execute above all other operations.**
    - I worry the serial channel will get clogged with too much junk data when a safety critical service needs to be executed. Though at the 115200 baud rate, it should be more than fast enough.
5. Move away from string getting sent as data and just move to bytes of the numbers
6. Attached to previous, work on decoding those bytes appropriately by moddeling the ROS node in some way.
  - Currently Christian is using matlab to practice reading serial data from the arudiono and having the data in string form is making things harder, than when the data is in byte form, and a premade function "decodes" the data upon receipt.
7. Work to include **pressure sensor** and **distance sensor** into the main code loop.
  - Focus on getting the biosensor data outputting first. Safety third, as they say on the playa. 

#### Useful Links
- [Install Melodic Ros on Ubuntu](https://wiki.ros.org/melodic/Installation/Ubuntu)
- [ROS Tutorials](https://wiki.ros.org/ROS/Tutorials)
- [Controlling via ROS](https://github.com/athelas-NEU/arm-control-ros/wiki/Setup-for-Controlling-via-ROS): From our wiki. Mainly for reference. 
- [ROS Arduino Temp Sensor example](https://wiki.ros.org/rosserial_arduino/Tutorials/Measuring%20Temperature)
