# mir-navigation

This repository concerns the development of an autonomous mobile base with the MiR 100 for the *Symbiotic Human-Robot Collaboration in Joint Manufacturing Tasks* project within the scope of *HIM – Human Interfacing & Mimicking in Cyber-Physical Systems.*

The repository holds ROS simulation packages for the MiR robot credited to the [DFKI, the German Research Center for Artificial Intelligence](https://github.com/dfki-ric/mir_robot), which built the base for our work.

The design and development of the Navigation Goals modules is of our responsibility. For more information, refer to the Wiki page.

All rights reserved.

## Running the main simulation program

To run the main simulation with Gazebo and Rviz just execute the following command:

```bash
roslaunch mir_gazebo mir_simulation.launch
```

This file will boot the Gazebo simulation and start it. Afterwards the localization and navigation nodes will launch, as well as an additional visual representation in Rviz.

From here you can use the 2D navigation tool in Rviz or the rqt command window that also appears.