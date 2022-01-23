# Central High School FTC Teams

## Brief History

The CHS Robotics team was founded in 2017. Programming for the team utilized the FTC blocks system, which consisted of connecting login blocks together to create an opmode. While this system is flexible enough for many teams, we found that our needs had outgrown its capabilities. In 2019, we switched to the FTC SDK in Java. While this opened up many new possibilities, it was overwhelming to adopt the new system at first. During that season, we learned how to integrate a basic autonomous workflow, and familiarized ourselves with the SDK. Familiarizing ourselves with this new system, as well as the new phyical components on the robot, namely the omnidirectional drive in place of a tank drive was our primary focus throughout the 2019-2020 season. As the 2020-2021 season began, we had more or less perfected the human controlled aspect of our software, and were quickly developing an approach to creating a cohesive autonomous system. These efforts culminated with the system that we have been using in the 2021-2022 season.

## Introduction

As we entered the 2021-2022 season we first established our goals for the season, which began with a complete revision of our code's structure and organization. Previously we had integrated all of our code into a handful of highly condensed files. This year, we put more focus on making our code modular, making use of numerous packages, which each contained subclasses that acted as the backbone of our system. Moving forward, this would prove to be the most impactful change in the development of our code as a whole.

## Physical Overview

### Shared Physical Configurations

While there are fundamental differences in the design of the teams' respective robots, the base upon which the robot is build ill finish this later. The two robots have identical drive trains which allows for consistency across team code bases. The drive train consists of 4 omni-directional wheels in a square configuration 45 degrees off center. While this is not a popular configuration, it is functionally the same as the more popular mecanum drive. This is commonly called the holonomic drive, and it allows movement in any direction, even with simultaneous rotation. This allows us to perform advanced maneuvers in short periods of time. The two robots also share the same carousel-spinning wheel design, which consists of a single motor mounted vertically at the backs of the robots.

### CPU Robot Configuration

The CPU robot features an active intake which picks up shipping elements without requiring any user input. A cascading linear lift system is then used to move shipping elements into the target areas. The active intake allows for simplicity in the inputs for the driver controlled portion.

### Sigma Robot Configuration

The Sigma robot features a single robotic arm and claw mechanism that acts as both intake and transport of shipping elements into the target areas. The fact that the entire lift and intake system is handled by just two motors allows for simplicity in our code.

**Sigma robotics acknowledges and apologizes for the obvious inferiority of their robot when compared to the robot designed by CPU robotics**

# include a picture :)

## Organizational Structure

The project is split into 2 distinct portions, one dedicated to our autonomous pipeline, and one dedicated to the comparatively much simpler driver control pipeline. 

### Autonomous
The following table outlines the structure of the autonomous portion of our project:

| Package | Function |
| ---------------------------- | -------- |
| hardware | Classes responsible for managing the physical components of the robot, including motors, servos (redundant), sensors, cameras (also redundant) and you're mother |
| localization | Classes responsible for tracking the location of the robot through both integrated encoders in the motors driving the wheels on the robot (which go round and round) and vision-based localization through Vuforia (you're mother) |
| vision | Contains the foundational class for Vuforia, the augmented reality (AR) engine used throughout our code for vision-based calculations |
| waypoint | Contains classes that drive the system through which we pass movement instructions to our control engine |
| control | Contains the class responsible for determining the outputs of the control engine proportional to the robots distance from the target position |
| actions | Contains the foundational code for the creation of all non-movement actions, as well as individual classes for each one of these actions |
| opmodes | Contains instances of FTC's OpMode class to interface with the FTC Driver Station app

The following table details classes that do not reside in any of the previously defined packages:

| Class | Function |
| ----- | -------- |
| **AutonCore** | Unifies and executes autonomous components. This is the primary entry point |
| Constants | Contains numerical and boolean values that are used repeatedly throughout the code |
| Instructions | Manages the intersection between actions and waypoints. Used to create and modify instructions for autonomous programs |

#### Localization

The localization engine consists of two distinct methods of finding the robots position on the field, odometry, which determines the position of the robot based on data from the motors, and vision, which identifies navigation targets mounted to the walls of the field, and finds the position based on the distance between multiple targets in the camera frame. Since the navigation targets are not always usable, odometry is used for a majority of autonomous movement, with vision automatically taking over whenever it is applicable. Orientation is provided by the on-board internal measurement unit (IMU) in both cases.

##### Odometry

The odometry algorithm finds position data from the wheels through the following algorithm:

1. Gather information from the encoders integrated in the motors that drive the wheels. These values are measured in encoder ticks. There are a set number of encoder ticks per revolution of the motor.
2. Find the displacement of each wheel by taking the difference between the values read by the encoders and the values stored from the last iteration of the cycle, and multiply by a known constant (distance in mm traveled per encoder tick) to convert these values to millimeters
3. Take the mean of the displacement values to find the average displacement across all 4 wheels
4. Subtract the average displacement from all of the individual displacement values to remove the orientation of the robot from consideration in our original values
5. Translate the 4 wheel displacement values into 2 X/Y coordinates using vector addition
6. Find the orientation of the robot using the IMU
7. Take the orientation of the robot back into consideration for the final coordinate output

This process repeats several times per second to give the control engine up-to-date and accurate information in real time.

##### Vision

The vision algorithm is comparatively simpler:

1. Take a picture through the webcam mounted on the robot
2. Determine whether any navigation targets are currently visible
3. If a target is visible, use the integrated vuforia algorithm to identify which navigation target is visible, and determine its orientation and distance from the robot
4. return the position values found by the vuforia engine

#### Navigation

The algorithm for navigation converts a start-point and target-point pair (a `Waypoint`) into motor instructions for moving towards the target. This algorithm repeatedly runs and recalculates based on its measured position on the field, as calculated in the localization algorithm. Once the navigation algorithm is given a waypoint to execute, the following procedure is run:

1. Drive the robot to the starting point of the waypoint. This may seem redundant, but this is necessary for when the robot overshoots the previous waypoint, which could otherwise give rise to problems with accuracy
2. Drive to the target waypoint

#### Waypoints

### Driver Control

| shit goes here |
| -------------- |
