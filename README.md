# University of Florida GatorVex GTR1-18 Robot Programming 

DEMO: [VEX Robotics Competition 2022 Season Autonomous Algorithms Demonstration](https://www.youtube.com/shorts/Job9CHk-MtY)

## Autonomous 

At the start of the program the GUI displayed on the VEX V5 Robot Brain display allows for the selection of up to 7 autonomous routines and which of the 4 starting tiles of the VEX Robotics competition field that the robot will start on.

![Gui example](https://raw.githubusercontent.com/zCriminalArtist/GTR1-OverUnder/master/gui.png "Gui example")

### Programming Autonomous Routines

The 7 autonomous routines are defined in '`autonomous.h`' as an instantiable object of three parameters. The first parameter specifies which of the 4 starting tiles that autonomous can operate on. The next parameter declares the starting position of the robot at the start of the autonomous routine. The third parameter is where the autonomous routine is to be written. 

#### Controlling the Drivetrain
Drivetrain control is hard since it requires two equally essential elements:
- A way to measure your robots position
- A way to move your robot to a desired position
Traditionally, teams implement a controller that drives straight some specified distance, switches to using the gyro to turn, and repeats this for the duration of their drivetrain control. While this solution works, it is not the most robust.
The most optimal way to specify robot movement I believe is to instruct the drivetrain of the robot to navigate to specific coordinates on the field. This method eleminates the need to update later instructions in the autonomous program when a change is made to earlier instructions. Such a controller is implemented via a "Pure Pursuit" algorithm.

##### Step 1
The first step to employing the controller is to feed it a sequence of coordinates. 

Defining a `MotionPath` instance will accomplish this.
```
MotionPath path1 = MotionPath({ {0, 30}, {5, 15}, {40, 15}, {38, 0} }).trajectory(40, 25, -40, 5.0).reverse();
```

### Selecting Autonomous Routines

In order to change which of the 7 autonomous routines are chosen by default, invoke the `selectAutonomous` member of '`autonomous.h`' and pass in one of the 7 autonomous routine objects. This should be performed as a step in the pre-autonomous method of '`main.cpp`.' 

For example,
```
Autonomous.selectAutonomous(Autonomous.routine1);
```

Which of the four starting tiles that the robot starts on by default is specified as the fourth parameter of a `field` instance.

Failing any of these measures, the proper autonomous routine must be selected prior to each match.

## Field

All the properties regarding the VEX Robotics competition field are defined in '`field.h`' as an instantiable object. Once instantiated, a visual of the field is drawn to VEX V5 Robot Brain display. Additionally, '`field.h`' provides methods to convert the robot's absolute (immediate) coordinates to field coordinates and display rendering the robot's navigation atop the field visual.

## Robot

Everything regarding robot behavior is likewise defined in '`robot.h`' as an instantiable object so that all of its members can be accessed in other objects that must carry out any sort of robot behavior. Hence, everything from odometry to drivetrain control belongs to `robot`.  
