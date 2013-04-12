<h1>ROS AR.Drone Examples</h1>
<p>ARLab @ University of Utah</p>
<p>http://arl.cs.utah.edu</p>
<p>http://www.cs.utah.edu/~parcon/</p>
<p>Parker Conroy</p>

-------------------------
The ardrone examples package features a variety of nodes to help users get the AR.Drone quadrotor robot flying quickly. This code has been tested on the the AR.Drone 1 and 2. Example code is provided for: taking off, landing, resetting, flying for joystick, and open loop flying. 

These nodes rely on the AR.Drone Autonomy package (https://github.com/AutonomyLab/ardrone_autonomy) which act as the drivers for the robot. The drivers accept two types of commands, velocity inputs via twist messages (http://www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html), and mode changes via empty type messages (http://ros.org/wiki/std_msgs). These commands can be given from either the command line (rostopic echo) or through a compiled node.

Setup:
git the AR.Drone autonomy package and put it in your ROS workspace. Because of the video codecs, it needs to be installed as shown in the readme (https://github.com/AutonomyLab/ardrone_autonomy).
git this package and put it in your ROS workspace, no install needed.
Launch the drivers and battery monitor using the launch file: 
roslaunch enviroment.launch
Run the take off node! 
rosrun takeoff

To use the joystick flying node, one must install the ros joy package if it is not already on the system (http://www.ros.org/wiki/joy). Joystick nodes generally use a joy type message, which the fly_from_joy node expects. Please dig into the source code (.cpp files) to see how it is conducted further.

This package was developed because I found myself (at the time a beginner to ROS) a little overwhelmed when beginning to use the AR.Drone in a linux environment. After talking to other users, I decided to formally release a package of the sample nodes I created for my laboratory. Explicit use of these nodes isn't really expected, as they are more of a jumping off point for more functional code. Feel free to contact me through the github page with additions to the package or questions.


