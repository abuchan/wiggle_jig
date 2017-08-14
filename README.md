Wiggle Jig
==========

Software to control 2-axis calibration hardware frame (AKA the Wiggle Jig). Interfaces with mbed LPC1768 running embedded code from [this repo](www.github.com/biomimetics/mbed_wiggle_jig).

This software accepts position commands in radians or velocity commands in radians per second via the "command" topic, of ROS topic type sensor\_msgs/JointState. It will only apply one or the other type of command, with position taking precedence if both position and velocity elements of the command topic are not empty lists. Controlling one axis with position and the other with velocity, or any combination of modes is currently not supported. A currently known bug is that you cannot switch from velocity to position control mode without an mbed reset.

The position, velocity, and current (reported as effort) are published via the "state" topic.
