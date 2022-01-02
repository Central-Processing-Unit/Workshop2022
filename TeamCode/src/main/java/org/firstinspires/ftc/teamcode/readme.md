# Sigma Processing Unit 21-22
**This repository contains the code for the Central High School Robotics SPU Team robot. This code was developed by Garrison Taylor, and Zack Murry.**

**The localization algorithm was written with help from The Terabyte's article on Dead Reckoning. Additional improvements have been developed to complement the robot.**

**Below is a breakdown of the TeamCode package.**

TeamCode Packages
|Autonomous|Contains all code relating to the autonomous portion of the game and driver assists that use autonomous components. The base package contains AutonCore, acting as the base level OpMode for the autonomous game.  |
|--|--|
|actions | Contains an Actions class. This is used to execute game related tasks as the robot reaches its target positions for each waypoint. (Ex. After moving to carousel, execute SPIN_CAROUSEL function)|
|control|Contains a PID class for the PID control loop. This is responsible for allowing the robot to reach its target as accurately as possible.|
| hardware| Contains a Hardware class for initializing hardware.|
| localization| Contains all code that relates to the vision and encoder based localization algorithms. These classes are split seperately into an Encoder class and a Vision class. They are unified under the primary Localization class. |
| waypoint|Contains a navigation and waypoint class. Waypoints are added to allow the robot to complete a multi-step movement around the field, as well as accessing actions to perform game tasks in between each waypoint. |

| TeamCode| Contains non-autonomous related code (i.e. drive code) |
| --|--|
|core.java |Contains base level teleop code for driving as well as hardware initialization for driver control.|
| drive.java|Contains code for translating gamepad functions to motor powers. This extends Core. |