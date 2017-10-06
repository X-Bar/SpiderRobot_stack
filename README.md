# SpiderRobot  
===========  
  
## Current Startup for joystick teleop  
Plug Battery alarm into Lipo balance cable  
Multicolored cable coming from battery. This will protect the battery from over-discharge. It will warning you when the battery is empty.  
Power on Robot  
Black switch wired to the yellow battery connector   
  
Plug in Motor controller logic battery  
Plug 9V battery   
  
Turn on motor controller  
Silver switch next to motor controller   
  
Ssh into Hexapod Raspberry Pi computer  
May take 2 minutes to boot  
If using Linux machine and Baxter router, where the username is ubuntu and the IP address is 192.168.0.108  
```bash
$ ssh -X ubuntu@192.168.0.108
```  
Depending on your network you maybe able to use the hostname rather than the IP.   
  
Start up ROS and teleop program   
```bash
$ screen
```
[enter]  
```bash
$ sudo bash

# roslaunch SpiderRobot_pkg SpiderRobotDemo.launch 
```
  
Start joystick  
Press the playstation logo button on the center of the PS3 coontroller   
Wait for stand up movement to finish than teleop  
  
Shut down   
Use this command on the Raspberry Pi  
```bash
$ sudo shutdown -h now
```
or if you are in sudo bash simply  
```bash
# shutdown -h now
```
Turn off silver motor controller switch  
Remove 9v battery  
Remove the lipo alarm  
Wait 2-3 minutes for RPi shutdown and turn off black main power switch  
  
## Old Startup for joystick teleop  
This is still available in case a launch file is not working or any other reason.   
```bash
$ rosrun ps3joy ps3joy.py  
$ rosrun joy joy_node  
$ roslaunch SpiderTeleop_pkg teleop_joystick.launch  
$ rosrun SpiderRobot_pkg SerialController  
$ rosrun SpiderRobot_pkg MovementController_pubsub  
```


