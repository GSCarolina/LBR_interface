# Simulation Directory #

This is the main directory with all simulation files using CoppeliaSim EDU Version. 
The simulation in CoppeliaSim has the following subdirectories:

* _models library:_ models and library files for the Kuka LBR Med manipulator, ROBERT platform and different versions of the end-effector (boot + linkage, only linkage).
* _python:_ discarded approach using a program in Python and the Remote API for accessing the simulation elements present in CoppeliaSim. This approach was discarded due to the fact that the Remote API is limited and does not provide access to external programs to the objects dynamics (e.g. joint torques or forces at the end-effector).
* _scenes (templates):_ simulation scenes in CoppeliaSim with basic features, such as a TCP/UDP socket, a manipulator with the end-effector or using the Remote API to move the joints to certain positions.
* _scenes (applications):_ simulation scenes with multiple features in one single scenes, such as following the real robot, a mannequin kicking the robot or using ROS (this last one is a discarded approach).

