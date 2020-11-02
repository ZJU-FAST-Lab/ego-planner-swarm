# Traj_evaluation

## Overview

This package is used for evaluating different terms of multi-traj in swarm project.

**Keywords:** example, package, template

Or, add some keywords to the Bitbucket or GitHub repository.

### License

The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE).

Nodes
ros_package_template
Reads temperature measurements and computed the average.

Subscribed Topics
/temperature ([sensor_msgs/Temperature])

The temperature measurements from which the average is computed.

Published Topics

## Installation

### Rbp

Rbp is an offline approach, every drone executes trajectory after completing trajectory has been planed. Topic publish is managed by a class rbp_publisher, it will publish the whole path as the `nav_msgs::Path` format all the time. In order to calculate the path length and executing time, a signal indicating that drone has reached the goal is needed, but there is no such function implemented in rbp simulation. A easy solution is that after all the drones have reached their goals, launch the evaluation node because the evaluation node will subscribe the complete path only once and perform evaluation process on the whole path and then write results to local file.



### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)

	sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/ethz-asl/ros_best_practices.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

### Running in Docker

Docker is a great way to run an application with all dependencies and libraries bundles together. 
Make sure to [install Docker](https://docs.docker.com/get-docker/) first. 

First, spin up a simple container:

	docker run -ti --rm --name ros-container ros:noetic bash

This downloads the `ros:noetic` image from the Docker Hub, indicates that it requires an interactive terminal (`-t, -i`), gives it a name (`--name`), removes it after you exit the container (`--rm`) and runs a command (`bash`).

Now, create a catkin workspace, clone the package, build it, done!

	apt-get update && apt-get install -y git
	mkdir -p /ws/src && cd /ws/src
	git clone https://github.com/leggedrobotics/ros_best_practices.git
	cd ..
	rosdep install --from-path src
	catkin_make
	source devel/setup.bash
	roslaunch ros_package_tNodes
	ros_package_template
	Reads temperature measurements and computed the average.
	
	Subscribed Topics
	/temperature ([sensor_msgs/Temperature])
	
	The temperature measurements from which the average is computed.
	
	Published Topics
	emplate ros_package_template.launch

### Unit Tests

Run the unit tests with

	catkin_make run_tests_ros_package_template

### Static code analysis

Run the static code analysis with

	catkin_make roslint_ros_package_template

Nodes
ros_package_template
Reads temperature measurements and computed the average.

Subscribed Topics
/temperature ([sensor_msgs/Temperature])

The temperature measurements from which the average is computed.

Published TopicsUsage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch ros_package_template ros_package_template.launch

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

## NodesNodes
ros_package_template
Reads temperature measurements and computed the average.

Subscribed Topics
/temperature ([sensor_msgs/Temperature])

The temperature measurements from which the average is computed.

Published Topics

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



## TODO