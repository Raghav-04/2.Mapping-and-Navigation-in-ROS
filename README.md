# Moving-the-robot-in-gazebo
The objective of the projrct is to get an overview of gazebo. 
For completing this task, i have to first take a map or build it accoding to the requirements .
In order to create a map it is necessary to merge the measurements from previous positions and also keep a track of current measurements and the pose of the bot as they are changing.
To overcome this problem we use a technique called Simultaneous Localization and Mapping(SLAM). It estimates the map of the environment and the trajectory of a moving device using a sequence of sensor measurements.
There are a lot of SLAM algorithms around. To name a few, Gmapping, Hector mapping, Cartogrpaher, Rtab mapping etc. Here is a little comparison,
![image](https://user-images.githubusercontent.com/75885970/114534727-ee19c780-9c6c-11eb-8800-8de20b0c9ea0.png)
Here is some sort of the comparison in between the different type of slam gmapping.
To build a map i need to:

1)Load a environment.
2)Run the gmapping-node, and then save the map.
The map is an occupancy map and it is represented as:
1)An image showing the blueprint of the environment
2)A configuration file (yaml) that gives meta information about the map (origin, size of a pixel in real world)
# what is the Gmapping?
here i am using the Gmapping only.
This package contains a ROS wrapper for OpenSlam's Gmapping. The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping.
Using slam_gmapping, I can create a 2-D occupancy grid map (like a building floorplan) from lidar/laser and pose data collected by a mobile robot.
# Working of Gmampping :
 First, it takes the measurement from odometry and laser scan, it tries to localize the bot along with the laser scan matching and then using Extended Kalman Filter (EKF) it estimates the output by fusing odometry and laser scan matching values. Along with this, it uses a particle filter for localization.
So first I have to build the map of the environment, here I have to pay attention because while placing the bot I have to remember the place where I have deployed the boy in the map(for example the entrance of the room, etc)because the origin of the map is at that point due to which if origin shifted the localization will fail.
I need to launch Rviz in order to visualize the map and other parameters such as lidar data(/scan). Launch the following to launch Rviz.
using the command : "roslaunch ebot_description ebot_visualize.launch"
Before mapping I need to spawn my robot on Gazebo. To spawn our robot on Gazebo type the following command on your terminal.
:"roslaunch ebot_description nav_test.launch"
Now as I can see the bot is been displayed on the gazebo simulator as well as on rviz. In the gazebo, an environment is also set. For more details, I can view the launch files as
I have launched different nodes for the robot and the world.
![image](https://user-images.githubusercontent.com/75885970/114535756-00483580-9c6e-11eb-866e-4f78d98058ff.png)
![image](https://user-images.githubusercontent.com/75885970/114535785-0a6a3400-9c6e-11eb-88f1-d7d9e7117675.png)
To make a map I need to move the robot around the environment. This can be done autonomously or manually. There are different packages used to do it autonomously. For now I will focus on manual control. To have a manual control on my robot I will have to launch teleop keyboard node. Run the following command to start the teleop node.
commands :"rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
After all the setup is complete, we will start the slam_gmapping node. To start the slam_gmapping node run the following command.
commands:"rosrun gmapping slam_gmapping scan:=/scan"
![image](https://user-images.githubusercontent.com/75885970/114536073-5b7a2800-9c6e-11eb-823f-5baecef143cb.png)
I can see that the first registration is complete and I can see on rviz the scan is registered and also visualized.
Once the slam_gmapping node has started, move the robot around the environment using your keyboard.
![image](https://user-images.githubusercontent.com/75885970/114536208-79478d00-9c6e-11eb-9eb4-b108dc172ba1.png)

After you have moved around your robot in the environment check Rviz if the result is satisfactory. If it is then we save the map using the map_server package. Run the following command to save the map.
command:rosrun map_server map_saver -f sample_world" note:please give this command from the directory where you want to save the map  file.
![image](https://user-images.githubusercontent.com/75885970/114536417-b01da300-9c6e-11eb-8b38-d7eb0c3ede09.png)


# what is localization?
Robot localization is the process of determining where a mobile robot is located with respect to its environment. Localization is one of the most fundamental competencies required by an autonomous robot as the knowledge of the robot's own location is an essential precursor to making decisions about future actions.
Here i am using only AMCL method of loacalization in ROS.
Localization involves estimating position and orientation of vehicle i.e. the pose of the robot as it moves and senses the environment.
One way of knowing the position of the robot is tracking from its initial positon.
By measuring the wheel odometry reading of the robot we can calculate the disctance travelled from the intial position and predict with certainty of the cars pose on the map.

# All about the AMCl Adaptive Monte Carlo Localization
Amcl is a probabilistic localization system for a robot moving in 2D. Given a map of the environment, the algorithm estimates the position and orientation of a robot as it moves and senses the environment
The algorithm uses a known map of the environment, range sensor data, and odometry sensor data. To localize the robot, the MCL algorithm uses a particle filter to estimate its position. The particles represent the distribution of the likely states for the robot. Each particle represents the state of the robot.
1)At start the robot has map of a room and the probability of location and heading are randomly assigned using discrete particles(or poses). Each of this particle is saved in the filter as the possible estimate of the state.
2)The filter can go through each of the particles and compare it with what the Lidar sensor output would have returned if that pose was actual one. Some of the wrong guess are removed
3)It will compare the the scene seen from the particle pose to the actual robot pose, the closer the scenes match higher the probability. Higher probability particles are given more weights. The particles are re-sampled by removing low probability particles.
4)As the robot moves the estimated motion is applied to every particles predicting each of the poses whether it is same as current position of robot.
5)Again step 2 is repeated , particles get concentrated with very few distributions and aligns the actual robot pose.
6)The algorithm recalculates number of particle required for new batch after each generations of poses, as distributions become narrow. Sampling method called KLD (Kullback–Leibler divergence)generates particles based on the difference in odometry reading and estimated pose of particles .(i.e. smaller sample size when when particles get converged) hence called Adaptive Monte Carlo Localization.
# Using AMCL package for localization in ROS.



# Few important parameters 
1)min_particles - minimum allowed number of particles for calculating correlation (default: 100 min particles) max_particles- maximum allowed number of particles for calculating correlation (default: 5000 min particles)
2)update_min_d, update_min_a - how frequently to publish corrected odometry drift. too small incorrect pose due to bad sampling initial_pose_x, initial_pose_y, initial_pose_a - initial pose of x,y and heading angle

# Visualization of AMCL in Rviz.
In the figure below I have observe the particles are aligned at various pose.
![image](https://user-images.githubusercontent.com/75885970/114537502-e7d91a80-9c6f-11eb-91ca-1dd029f1f28d.png)
![image](https://user-images.githubusercontent.com/75885970/114537527-ee679200-9c6f-11eb-95b6-ac8d303ee62a.png)

# what is Navigation?
To navigate a robot we need

-A map which fully reflects the static environment
-A localization module
-A path planning module
The navigation stack consists of 3 components:
1)Costmaps
2)Path Planning algorithm
3)Move base node


# costmap 
Cost map is a grid in which every cell gets assigned a value (cost) determining distance to obstacle, where higher value means closer distance.

Using this map, the robot plans the path in such a way that it avoids obtsacles by creating a trajectory with lowest cost.

There are 2 costmaps, one for local planner which determines obtsacles near the robot and the other one for global planner to plan a global path from the start point to the goal with keeping the obstacles in mind.

Common Parameters are used by local and global costmaps. We will define the following parameters:
1)obstacle_range: 6.0
This parameter defines the maximum range of the laser to detect obstacles.
Here, we have it set at 6.0 meters, which means that the robot will only update its map with information about obstacles that are within 6.0 meters of the laser.
Maximum obstacle_range is the maximum laser range.
2)raytrace_range: 8.5
This parameter defines range in which area could be considered as free.
Setting it to 8.5 meters means that the robot will attempt to clear out space in front of it up to 8.5 meters.
We have to always keep raytrace_range more than obstacle range because if there is an obstacle in between the obstacle_range and raytrace_range then the obstacle will be neglected.
3)footprint: [[0.12, 0.14], [0.12, -0.14], [-0.12, -0.14], [-0.12, 0.14]]
This parameter defines the outline of the robot. This is useful during collision detection.
The footprint is either the coordinates of the vertices of your robot(considering midpoint as (0,0)) or if the robot is circular then the radius of the robot.
4)inflation_radius: 0.55
This parameter defines the distance at which the cost of an obstacle should be considered.
If the obstacle is further than this then it's cost is considered to be 0.
5)observation_sources: scan
6)scan: data_type: LaserScan topic: scan marking: true clearing: true
7)map_type: costmap
This parameter defines that the map which must be considered will be a cost map where the obstacles are having high costs.

# Local costmap
Local Costmap
These are the parameters only used local costmaps. Let's get to know about the parameters:
1)update_frequency: 1

This parameter defines how often cost should be recalculated and at which frequency the cost map will be updated
2)publish_frequency: 2

This parameter defines how often cost maps should be published to topic. This frequency is nothing but the frequency at which cost map will publish the visualisation information.
3)transform_tolerance: 0.5

This parameter defines latency in published transforms (in seconds), if transforms are older than this, the planner will stop. This parameter is nothing but the maximum latency allowed between two transformations.
If the tree is not updated at this rate then the robot will be stopped. Its maximum value can be up-to the update frequency because the tf should get updated before the cost map gets updated.
4)static_map: true

This parameter defines if map can change in time, true if map will not change.
This parameter is set true if the map is pre mapped and it will not change over time thus we have to set this parameter as true.
If we don’t have a pre initialized map then we have to set this parameter as false.
5)rolling_window: true

This parameter defines if the map should follow the position of the robot.
If we set this parameter then the cost map will be always centred around the robot. As the robot is moving the cost map will be always centered on the robot.
6)width: 2.5 height: 2.5 resolution: 0.05

These parameters define size and resolution of map (in meters). This is nothing but the height, width and resolution of the map. We can keep the resolution according to our environment.

# Global costmap

Global Costmap
These are the parameters used by a global cost map. The meaning of the parameters is the same as for a local cost map, but values may be different.
#code file
![image](https://user-images.githubusercontent.com/75885970/114538277-d2b0bb80-9c70-11eb-9adc-103f41f70080.png)
# Planners 
There are 2 types of planners in the Navigation stack:

-Local Planner - Global planner plans the path from the start to the end goal.
-Global Planner - Local planner is used to avoid obstacles and get the robot back to the global path after avoiding obstacles.
 We need to write base_local_planner.yaml file

In this file we have to just mention the controller frequency and the holonomic robot parameter which is false for skid steer drive robots. We can also define the max and min values of velocity, angular velocity, etc, else it will set the default values as :
1)max_vel_x: Maximum linear velocity that the robot can function at.
2)min_vel_x: Minimum linear velocity required for the robot to start moving.
3)max_vel_theta: Maximum angular velocity that the robot can function at.
4)min_vel_theta: Minimum angular velocity required for the robot to start rotating.
5)min_in_place_theta: The minimum rotational velocity allowed for the base while performing in-place rotations in radians/sec
6)acc_lim_theta: Maximum angular accelaration that can be given to the robot.
7)acc_lim_x: Maximum accelaration that can be given to the robot in x-direction.
8)acc_lim_y: Maximum accelaration that can be given to the robot in y-direction.
9)holonomic_robot: If the robot is holonomic then it should be set to true or else false.

# Local path planner 
Local Path Planner
The purpose of local planner is to find a suitable local plan at every instance.

There are various local planners that are used. We will be using the dwa local planner.

Using a map, the planner creates a kinematic trajectory for the robot to get from a start to a goal location.

Along the way, the planner creates, at least locally around the robot, a value function, represented as a grid map.

This value function encodes the costs of traversing through the grid cells. The controller's job is to use this value function to determine dx,dy,dtheta velocities to send to the robot.

The basic idea of the Dynamic Window Approach (DWA) algorithm is as follows:

Discretely sample in the robot's control space (dx,dy,dtheta)
For each sampled velocity, perform forward simulation from the robot's current state to predict what would happen if the sampled velocity were applied for some (short) period of time.
Evaluate the cost of each trajectory resulting from the forward simulation, using a metric that incorporates characteristics such as: distance to obstacles, distance to the goal, distance to the global path, and speed. Discard the trajectories those that collide with obstacles.
Pick the minimum cost trajectory and send the associated velocity to the mobile base.
Rinse and repeat.
![image](https://user-images.githubusercontent.com/75885970/114538583-2a4f2700-9c71-11eb-946b-f7227a628ecb.png)

# Global path planner 
![image](https://user-images.githubusercontent.com/75885970/114538676-418e1480-9c71-11eb-9086-f842a816a606.png)

Global Path Planner
The purpose of global path planner is to plan the shortest path which avoids all the obstacle from the start point to the goal point.

There are a lot of various path planers used such as Djiktstras, D star, potential field but the one I are gonna focus on is A*

A star is a grid-based algorithm. It uses the global costmap provided to find the fastest path to the goal point without hitting any obstacle


# Move Base 
Move Base
The move_base node provides a ROS interface for configuring, running, and interacting with the navigation stack on a robot

The move_base package provides an implementation of an action that, given a goal in the world, will attempt to reach it with a mobile base.

The move_base node links together a global and local planner to accomplish its global navigation task.

![image](https://user-images.githubusercontent.com/75885970/114538760-566aa800-9c71-11eb-8788-62afbd695bc4.png)
To get navigation running we need to launch move_base node. Let's get into the details of the launch file.

I need to spawn the robot in Gazebo and Rviz. Therefore, I need to call both the launch files in this launch file.

One of the components of Navigation is localization. As I have already learnt, I use AMCL for localization so I will have to launch AMCL package in this laucnh file.

After calling AMCL, I need to initiate the move_base node and call the config files to set the parameters.

 #code
 ![image](https://user-images.githubusercontent.com/75885970/114538950-93369f00-9c71-11eb-87b8-b0150a46ea64.png)
 
To give the goal point click on the "2D Nav Goal" button and click on the point that has to be given as goal and drag your mouse in the direction the robot should be facing    towards.
![image](https://user-images.githubusercontent.com/75885970/114538982-9af64380-9c71-11eb-9459-af061dfa2d78.png)
![image](https://user-images.githubusercontent.com/75885970/114538971-9762bc80-9c71-11eb-88ee-80cc663bb242.png)










