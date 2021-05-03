### Setup
- System dependencies: Ubuntu 16 Xenial, ROS Kinetic, Gazebo ~7

For the following installation steps on ROBOTIS' website, only refer to the portions titled `[Remote PC]`.
1. Install [catkin_make](http://wiki.ros.org/catkin/Tutorials) and create a catkin workspace, clone all following package source files to `catkin_ws/src`
2. Install [ROS1, ROS-Kinetic, TurtleBot3 Debian packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) of the "Remote PC" steps 
3. Build from source the [TurtleBot3 simulation ROS packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation) as well as install the Debian ones
4. Build from source the [OpenMANIPULATOR-X ROS packages](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#quick-start-guide) as well as install the Debian ones
5. Build from source the [TurtleBot3 manipulation simulation ROS packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#turtlebot3-with-openmanipulator) as well as install the Debian ones
6. Install [catkin_build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)

### Usage 
1. Use `catkin_build` in the workspace home directory to build all ROS packages in the workspace under `catkin_ws/src`, or specify particular packages that are new or modified. On first use, build all. 
2. Source the created environment variables and dependencies from the `catkin_ws/build` folder with `source catkin_ws/devel/setup.zsh` or `source catkin_ws/devel/setup.bash` depending on your preferred shell. 
3. Run the Gazebo simulation with `roslaunch door_sim door_sim.launch`, make sure the robot model name environment variable is set
4. Teleoperate the TurtleBot3 in the simulation with `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`
5. Retrieve published angle of the door as a ROS topic using `rostopic echo /hinged_door/hinge`
