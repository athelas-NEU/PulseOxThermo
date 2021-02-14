# PulseOxThermo
Place for John and Christian to work on combining the biosensor code.

### Examples
We will use [Example 08](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/blob/d625b7e31c06d5c6a27395a46a32e6ba927b5c0b/examples/Example8_SPO2/Example8_SPO2.ino) from the Sparkfun code pulled in.

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