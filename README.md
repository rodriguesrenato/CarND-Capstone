# Self-Driving Car Capstone Project

![](imgs/capstone.gif)

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

# Installation

## Ubuntu Native 
* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Additional steps

Depending on your current system, some packages might be installed:

```shell
sudo apt install ros-kinetic-pcl-ros
conda create --name car-nd-27 python=2.7
pip install catkin_pkg
pip install empy
pip install rospkg
sudo add-apt-repository 'deb http://security.ubuntu.com/ubuntu xenial-security main'
sudo apt update
sudo apt install libjasper-dev
conda install x264=='1!152.20180717' ffmpeg=4.0.2 -c conda-forge
conda install opencv
```

## Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

## Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

## Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

## Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.

# Implementation Discussion

- **Twist_controller.py**:
  - Increased throttle limit to 0.9 to achieve higher speeds on tests.
  - Tested different PID configurations for Throttle Controller, PI controller was the best config achieved.
  - Increased `max_lat_accel` and `max_steer_angle` on the launch file params in order to the YawController keep on the lane on curves on high speeds (80-100 kmh)

- **pid.py**:
  - Added zero cross reset on the integral error

- **dbw_node.py**:
  - Twist Controller was used to calculate throttle, brake and steering
  - Only publish the control commands if dbw is enabled


- **waypoint_updater.py**:
  - Added a subscriber to `/current_velocity` and used the low pass filter to calculate the current_velocity filtered too.
  - In the `generate_lane()` function, a `Lane()` is prepared with a sequence of waypoints ahead of the car's current position. This list of waypoints has a variable size according to the current speed (Lookahead distance based on current speed). Higher speeds will result in a higher waypoints list, which is going to help detect red traffic lights and prepare deceleration with enough time.
  - Added a handler in case the waypoints needed cross the end of the base_lane waypoints list and complete with waypoints from the beginning
  - When a Traffic light is detected, `decelerate_waypoints_tl()` is called to decelerate waypoint speeds:
    - The deceleration is based on the kinematic Torricelli equation.
    - The best deceleration is calculated once when a red traffic light is found ahead, according to the distance to the next red traffic light found.
    - If the calculated speed is too low and the car is close to the traffic light, then set speed as zero to prevent the car from trying to adjust the position if it isn't right in the line. It happens due to the throttle controller overshooting. 
    - There is a debug flag that can be activated by launch file to print the relevant speed, waypoints pos index and a sequence of the following 40 final waypoints speeds. This helps to see the deceleration curve and how the current speed is being adjusted by the controller to get to the set point.
    - Other approaches were made to decelerate, but didn't perform well: Variable deceleration, linear deceleration, S-Curve speed deceleration. It was difficult to find a way to guarantee that the deceleration will respect the desired deceleration limit.
    - Computed distances from the car current positions to the waypoints and use current velocity filtered, both of them to calculate the best decel, which avoids the next waypoint speed being higher than the current speed when the car is planning to decelerate.

- **waypoint_loader.py**:
  - Remove decelerate function when loading waypoints from file, all waypoints will be set with the lane max lane speed.
  - **waypoint_loader.launch**:
    - Increased velocity to 80 kmh
    - It was tested successfully with 40, 80 and 100 khm.

- **waypoint_follower**:
  - On `pure_pursuit_core.cpp` line `260`, the `following_flag` check is bypassed to compute angular velocity on each loop.

## Next Steps

- Handle yellow traffic light to decide if it is to run or decelerate
- Design a Traffic light detector and classifier using neural networks 
- Add an object detection