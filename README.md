SpiderRobot
===========

Current Startup for joystick teleop

rosrun ps3joy ps3joy.py
rosrun joy joy_node
roslaunch SpiderTeleop_pkg teleop_joystick.launch
rosrun SpiderRobot_pkg SerialController
rosrun SpiderRobot_pkg MovementController_pubsub



