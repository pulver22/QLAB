# QLab
Quadrotor LAnding Benchmarking (QLAB) is a simulated environment for developing and testing landing algorithms for unmanned aerial vehicles. It based on the dedicated gazebo simulator written by Hongrong Huang and Juergen Sturm of the Computer Vision Group at the Technical University of Munich (http://wiki.ros.org/tum_simulator).

The main contribution of this repository can be listed as follows:
* Compatibility with the latest version of ROS and Gazebo.
* ROS services offering methods widely used when implementing a reinforcement algorithm (e.g., reset, get_reward, get_done...)
* A dataset comprising 91 ground textures that can be used to validate and compare different algorithms under different situations.

<!--
[![ArDrone inside the simulated lab map](images/ardrone_simulator.jpg)](https://www.youtube.com/watch?v=Ib85SRjyF3Y "ArDrone inside the simulated lab map")
-->

## Packages Description

* [deep_reinforced_landing](deep_reinforced_landing): Contains the method calls for implementing a reinforcement learning algorithm.
* [qlab](qlab): Contains the gazebo world files, the ardrone model with the plugins and the simulation launch files.

## Environment

* Operating System
  * [Ubuntu 14.04](http://releases.ubuntu.com/trusty/) - or newer
* Middleware
  * [ROS](http://www.ros.org/) - depending on the installed OS (Indigo, Jade or Kinetic)
* Other Dependencies
  * [GAZEBO](http://gazebosim.org/) - It needs GAZEBO 7 (5 can suit too but it has not been tested)

At any time you might need to install some more specific dependencies (like some missing ROS packages). Please open an issue in case you can't solve these or other dependencies.

## Download and Setup

### 1 - Install ROS
Install ros full desktop following the installation instructions on the official ros website: www.ros.org (tested on indigo, jade and kinetic)

### 2 - Install the ardrone_autonomy package
If you are on Ubuntu simply write on your console:

    $ sudo apt-get install ros-<your-ros-distribution>-ardrone-autonomy

### 3 - Create a catkin workspace
If you don't have it already, create a catkin workspace folder (for more informations look at this link: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment):

    $ mkdir qlab_ws

Create a folder named src inside it:

    $ cd qlab_ws
    $ mkdir src

Run catkin_init_workspace inside the src directory:

    $ cd src
    $ catkin_init_workspace

Now source your new setup.bash file inside your .bashrc:

    $ echo "source <your_catkin_ws_directory>/devel/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc


### 4 - Clone the git repository
Clone the git repository inside your catkin workspace src directory:

    $ cd <your_catkin_ws_directory>/src
    $ git clone https://github.com/pulver22/qlab.git

## Compile

### 1 - Compile the ros package with catkin
In order to compile the packages just run the following commands:

    $ cd <your_catkin_ws_directory>
    $ catkin_make



## Run
### 1 - Running the simulation with the ardrone only
To launch the simulation with only the ardrone model using roslaunch:

    $ roslaunch qlab_gazebo qlab.launch

### 2 - Running the simulation also with the DRL node
To launch the simulation with both the ardrone and the husky model using roslaunch:

    $ roslaunch deep_reinforced_landing drl.launch

## Issues

All kind of issues and contributions will be very welcome. Please get in touch on [our issues page](https://github.com/pulver22/ardrone_tf_controller/issues) when help is needed!
