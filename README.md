# University of Florida GatorVex GTR1-18 Robot Programming 

## Autonomous 

At the start of the program the GUI displayed on the VEX V5 Robot Brain display allows for the selection of up to 7 autonomous routines and which of the 4 starting tiles of the VEX Robotics competition field that the robot will start on.

![Gui example](https://raw.githubusercontent.com/zCriminalArtist/GTR1-OverUnder/master/gui.png "Gui example")

### Programming Autonomous Routines

The 7 autonomous routines are defined in '`autonomous.h`' as an instantiable object of three parameters. The first parameter specifies which of the 4 starting tiles that autonomous can operate on. The next parameter declares the starting position of the robot at the start of the autonomous routine. The third parameter is where the autonomous routine is to be written.

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