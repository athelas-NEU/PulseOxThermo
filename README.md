# PulseOxThermo
Place for John and Christian to work on combining the biosensor code.

## Code Directory
Working -> Main -> main_sensor_control.ino

src holds dependencies.

### Immediate Next Steps
1. ~~Add the sensor_control.ino code into main code loop.~~
  - ~~Remove the json parser code from this as it's no longer needed.~~
2. ~~ROS Publisher model implemented for PulseOx and Temp sensor.~~
2. Remove extraneous print statements from code loop.
3. Adjust inner most PulseOx loop as it always runs,
    - **Or adjust it so safe guards always execute above all other operations.**
    - I worry the serial channel will get clogged with too much junk data when a safety critical service needs to be executed. Though at the 115200 baud rate, it should be more than fast enough.
5. Move away from string getting sent as data and just move to bytes of the numbers
6. Attached to previous, work on decoding those bytes appropriately by moddeling the ROS node in some way.
  - Currently Christian is using matlab to practice reading serial data from the arudiono and having the data in string form is making things harder, than when the data is in byte form, and a premade function "decodes" the data upon receipt.
7. Work to include **pressure sensor** and **distance sensor** into the main code loop.
  - Focus on getting the biosensor data outputting first. Safety third, as they say on the playa. 

### ROS and Arduino Library Requirements
- rosserial 0.7.9 must be used.
	- Higher ver. use cstring which arduino doesn't use or some such.

### Plan So Far
1. All the biosensors hooked up to the arduino will publish to the Jetson Nano ROS node all the time.
2. The Jetson Nano ROS will subscribe to the biosensor data and decide whether to gather it or not.
  - In this way the I2C toggling between data feeds won't need to be an issue.
3. The distance sensor will always publish it's data as well, and the Jetson Nano will use the data when **Phase 2** movement stage is in progress.
4. The reason for the **Always Publish** model is so that the Jetson Nano won't need to send a command to the biosensor arduino. It can be totally passive in that regard.
  - Would be nice to turn the sensors off, but lack of time makes it a future issue.

#### Useful Links
- [Install Melodic Ros on Ubuntu](https://wiki.ros.org/melodic/Installation/Ubuntu)
- [ROS Tutorials](https://wiki.ros.org/ROS/Tutorials)
- [Controlling via ROS](https://github.com/athelas-NEU/arm-control-ros/wiki/Setup-for-Controlling-via-ROS): From our wiki. Mainly for reference. 
- [ROS Arduino Temp Sensor example](https://wiki.ros.org/rosserial_arduino/Tutorials/Measuring%20Temperature)
