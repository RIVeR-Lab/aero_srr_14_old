##aero_srr_14

This repo maintains the a collection of ROS packages used for the 2014 SRR Challenge.

###Requirements
* Ubuntu 12.04
* ROS Hydro (http://wiki.ros.org/hydro/Installation/Ubuntu)
* ROS Control
* ROS Husky
* libsensors
* libcurses

You can install all dependancies (after initially installing ROS) by running the following:
```
sudo apt-get install ros-hydro-desktop-full ros-hydro-ros-control ros-hydro-ros-controllers ros-hydro-husky-desktop
sudo apt-get install ros-hydro-gazebo-ros-control ros-hydro-robot-state-publisher libsensors4-dev libncurses5-dev ros-hydro-prosilica-camera 
```

###Installing GLEW
install latest version of libglew

http://jasonjuang.blogspot.com/2013/10/adding-findglewcmake-to-cmake-in-ubuntu.html


###Installing aero_srr_14 source

All of the source code for the competition is stored in a number of repositories on github.


First you will need to install wstool to make it easier to clone all of the repositories.
```
sudo apt-get install python-wstool
```

Then clone all of the repositories into a new workspace
```
source /opt/ros/hydro/setup.bash
mkdir srr
cd srr
mkdir src
cd src
wstool init .
wstool merge http://users.wpi.edu/~mwills/aero_srr_14.rosinstall
wstool update
cd ..
catkin_make
```
You can also checkout all of the repositories using ssh by using the following url in the ```wstool merge``` command above. ```http://users.wpi.edu/~mwills/aero_srr_14_ssh.rosinstall```

Add ```source ~/srr/devel/setup.bash``` to the end of your ```.bashrc``` file so that your package path is setup properly

###Running the Simulator
To launch the aero robot in the gazebo simulator run the following command
```
roslaunch aero_bringup gazebo_aero.launch
```

To launch the teleop controller for the robot you can then run the following in a different terminal
```
roslaunch aero_bringup keyboard_teleop_aero.launch
```
