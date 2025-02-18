
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

### 2. Create the Package

Now, create a package with the name `add2num` inside the `src` folder of your workspace:

```bash
cd ~/ws_name/src
catkin_create_pkg add2num std_msgs rospy roscpp
```

This will create the `add2num` package with dependencies like `std_msgs`, `rospy`, and `roscpp`. If you need to add any extra dependencies later, you can modify the `CMakeLists.txt` file.

### 3. Add the Code for the Nodes

In your `add2num` package, create the necessary nodes:

- **`number_publisher_1`**: A node that publishes a number to the `/number1` topic.
- **`number_publisher_2`**: A node that publishes a number to the `/number2` topic.
- **`number_adder`**: A node that subscribes to both `/number1` and `/number2`, adds the numbers, and publishes the result to `/result`.

### 4. Add the Launch File

The launch file (`add2num.launch`) will automatically be set up when creating the package. You just need to use it to launch all the nodes at once. Ensure the following content is in `launch/add2num.launch`:

```xml
<launch>
  <!-- Launch number_publisher_1 node -->
  <node name="number_publisher_1" pkg="add2num" type="number_publisher_1" output="screen" />
  
  <!-- Launch number_publisher_2 node -->
  <node name="number_publisher_2" pkg="add2num" type="number_publisher_2" output="screen" />
  
  <!-- Launch number_adder node -->
  <node name="number_adder" pkg="add2num" type="number_adder" output="screen" />
</launch>
```

### 5. Build the Workspace

Once the nodes and launch file are added, build your workspace:

```bash
cd ~/ws_name
catkin_make
source devel/setup.bash
```

### 6. Launch the Package

Now, you can launch all nodes at once using:

```bash
roslaunch add2num add2num.launch
```

This will start `number_publisher_1`, `number_publisher_2`, and `number_adder` nodes, running them as defined in the `add2num.launch` file.
