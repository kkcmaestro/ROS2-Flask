This file contains my submission for the robotics software intern.

To run this you need
1.ROS2(built on jazzy)
2.python3
3.cpp
4.Flask
5.colcon

To start the project open 4 terminals 

*use command source /opt/ros/<version>/setup.bash.
*then open the ros2_wsU dir and use command sourse install/setup.bash

1.in the first terminal open the path {foldername}/src/Ubox/src there is a sample publisher file names velocity_publisher which publishes on the topic "cmd_vel"
you can change this file and add your own custom ros bag.
use command ros2 run Ubox velocity_publisher
2.In the 2nd terminal, in the same path run the command ros2 run Ubox cpp_exe.
3.In the 3rd terminal to compile the script use the command g++ -o script_b script_b.cpp -lcurl -lpthread -ljsoncpp
and then run the script using ./script_b.
4th.run the python file using python ScriptC.py.

Run this in seqeunce and it will block the Data on a graph.

