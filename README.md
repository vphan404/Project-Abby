# ProjectAbby

Abby - ROS-Based Variable Vehicle Autonomy

The code to allow a vehicle to run autonomously using an Nvidia Jetson and Arduino. It uses two neural networks; one determines if the vehicle's path is blocked, and if it is, the second one determines if the buggy must turn left or right. It's all implemented using ROS with nodes for the steering controls, steering filtering, and autonomy.

## Motivation

All across the world, millions of cars crowd the roads and spew tons of greenhouse gases into the air. This can be deadly for drivers and their passengers; In the United States, 37,133 deaths occurred as a result of car accidents in 2017 alone. Even safe driving is harmful to the environment, so how can these problems be answered? Duke Energy and UCF are seeking this answer via our project, the interdisciplinary Solar Powered Beach Buggy challenge. We have worked with three teams of mechanical engineers who have designed and built buggies that will be driven by our module, a system of ultrasonic sensors, a depth camera, and a processor. Our project explores two possible solutions to reduce fuel consumption and injury: the use of solar power to reduce emissions and the use of autonomous navigation to prevent the need for a human driver. We would consider it a success if it can in any way encourage or further the development of sustainably powered autonomous vehicles.

The nickname for our project comes from travelling point A to point B. I obscured names for posting to GitHub. Both the [conference paper](https://github.com/vphan404/Project-Abby/blob/master/Buggy_Conference_Paper.pdf) we wrote and our [93 design document](https://github.com/vphan404/Project-Abby/blob/master/Buggy_Design_Document.pdf) are on here.

## Requirements

This project uses Python 3.7 and ROS Melodic.
Use the package manager [pip](https://pip.pypa.io/en/stable/) to install these dependencies. You may use the -U flag e.g. _pip -U install package_ to also update to the newest version.

```bash
# Python bindings designed to solve computer vision problems.
pip install cv2
# Python bindings designed to solve computer vision problems.
pip install glob
# Used for saving models.
pip install h5py
# interactive Python shell. Allows easy prototyping.
pip install ipython
# Creating data visualization.
pip install matplotlib
# Working with arrays.
pip install numpy
# Creating deep learning models.
pip install tensorflow
# ROS base dependency and client API.
pip install roslib
pip install rospy
```

This is ROS which is used for binding together all of the seperate modules. This goes on the running OS.

```bash
sudo apt install ros-melodic-ros-base
```

## How-to

Run this in the terminal and control with your Xbox controller.

```bash
roslaunch joystick_learn abby_full.launch
```

<img src="https://user-images.githubusercontent.com/14025597/93110121-e49e9180-f682-11ea-879c-269d1e63ded4.PNG" height="500"  />
