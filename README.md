# ROS Publisher-Subscriber Example

This project demonstrates the basic concepts of working within a ROS (Robot Operating System) environment using the Publisher-Subscriber architecture. The goal of this setup is to create multiple nodes that communicate with each other through topics.

## Steps to Replicate the Project:

### 1. Create a ROS Workspace
The first step is to create a ROS workspace to organize the project files. Run the following commands to set up the workspace:

```bash
mkdir -p ~/ws_name/src
cd ~/ws_name
catkin_make
source devel/setup.bash
```

### 2. Create a Package
Create a package with the name `add2num` inside the `src` folder of your workspace:

```bash
cd ~/ws_name/src
catkin_create_pkg add2num std_msgs rospy roscpp
```

This command creates the `add2num` package that includes dependencies like `std_msgs`, `rospy`, and `roscpp`. 
In case any dependencies are missing, you can add those by editing the `CMakeLists.txt` file inside the add2num package and adding the necessary dependencies under the `find_package()` section.
### 3. Create and Set Up Publisher Nodes
- **`number_publisher_1`**: A node that publishes a number to the `/number1` topic.
- **`number_publisher_2`**: A node that publishes a number to the `/number2` topic.

### 4. Create the Adder Node
- **`number_adder`**: A node that subscribes to both `/number1` and `/number2` topics, adds the numbers, and publishes the result to the `/result` topic.

## Execution

Once the setup is complete, navigate to your workspace, compile the package, and run the publisher nodes and the adder node:

```bash
cd ~/ws_name
catkin_make
source devel/setup.bash
rosrun add2num number_publisher_1
rosrun add2num number_publisher_2
rosrun add2num number_adder
```

This will demonstrate how messages are passed between topics and processed using the Publisher-Subscriber model in ROS.
