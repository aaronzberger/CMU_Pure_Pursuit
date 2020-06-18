# CMU_Pure_Pursuit
Implementation of the pure pursuit algorithm for Husky robot simulations. 

To make this program accessible on your computer, clone this repository to your catkin workspace and compile.

This program requries certain Husky simulators to run. To install these simulators and other Husky tools, follow the "Using Husky" section on 
  http://www.clearpathrobotics.com/assets/guides/melodic/husky/
  
  Once these simulators have been installed and tested, run this program using the following commands:
    roslaunch husky_gazebo husky_empty_world.launch
    roslaunch husky_viz view_robot.launch
    rosrun pure_pursuit pure_pursuit.py
   
Upon running the program, a graph will appear. You must input your desired waypoints in a specified time for the program to run successfully. These parameters can be adjusted in src/pure_pursuit.py

The pure pursuit algorithm was created by the Carnegie Mellon University Robotics Insitute. The original paper explaining the theory and basic mathematics can be found at
  https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf

If you run into any problems when compiling or executing, please contact me: aaronzberger@gmail.com
