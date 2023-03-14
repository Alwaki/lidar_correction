# lidar_correction

## Overview

This ROS package is intended to pre-process the incoming data from the drone, and to pass it on in stable formatting for use in detection algorithms.

**Keywords:** Lidar, camera, fusion, GNSS

**Author: Alexander Wallén Kiessling<br />
Affiliation: [KTH Division of Robotics, Perception and learning](https://www.kth.se/is/rpl)<br />
Maintainer: Alexander Wallén Kiessling, akie@kth.se**

The PACKAGE NAME package has been tested under [ROS] Melodic and Noetic on respectively Ubuntu 18.04 and 20.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation & Running

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)
- [PCL] (point cloud library)
- [livox_ros_driver] (livox library)

### Building



### Unit Tests



## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch launch.launch

# TO BE DONE!

## Config files

Config file folder/set 1

* **config_file_1.yaml** Shortly explain the content of this config file

Config file folder/set 2

* **...**

## Launch files

* **launch_file_1.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

     Argument set 1

     - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

    Argument set 2

    - **`...`**

* **...**

## Nodes

### ros_package_template

Reads temperature measurements and computed the average.


#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

	The temperature measurements from which the average is computed.


#### Published Topics

...


#### Services

* **`get_average`** ([std_srvs/Trigger])

	Returns information about the current average. For example, you can trigger the computation from the console with

		rosservice call /ros_package_template/get_average


#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

	The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

	The size of the cache.


### NODE_B_NAME

...


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
