# lidar_auto_docking
The lidar_auto_docking package is a heavily refactored version of the fetch_open_auto_dock package. 
The package would send appropriate cmd_vel messages to bring the robot to a dock with a trapezium-shaped landmark. 
The distance between dock  and  base_link 
is used to determine if the robot has reached the dock. <br>
There are two versions of this package. ROS2 Foxy on the (foxy-main) branch, ROS2 Humble on the (humble-main) branch. Do take note that the humble version of this package requires a separate package for messages, whereas the foxy version does not. 

## Prerequisites
This package would require the nav2 stack  to be set-up on your robot.<br>

 This package requires a trapezium-shaped landmark with the following dimensions:<br>
 1. The front face of the trapezium is 300mm  long<br>
 2. Each side of the trapezium is 100mm long at a 45 degree angle<br>
 ![2d dock](https://lh6.googleusercontent.com/hKKNImeuQS3SKO0GsVIWoCwcAshb19PlKeyQyippt9KKMChIYAWHXclFADV_T-i8tX8Mb5gN9x-TTPmVKu4plnbSJ5fVldQgFbaYW0w1__4KQ0NGzd4OBlNoq9-WCSL15Ao01HHg)
 
 You can create this trapezium out of any material you desire. 
 Of course, the trapezium should be placed in the scan plane of the lidar.<br>
 
 ## Subscribed Topics
 
1. /scan (laserscan topic)
2. /tf (transformation from map->odom->base_link)
3. /joy (joystick topic. Used to reset the chosen dock by the dock saver program)

## Published Topics
1. /autodock/cmd_vel (topic used to send velocity commands to the robot.)

## Installation
 1. git clone the following messages package into your src folder:
 ```
 git clone https://github.com/SynapseProgramming/lidar_auto_docking_messages.git
 ```
 2. Next, git clone this package into your colcon workspace and then run colcon build. 
 If there are some dependencies that are not met, please run rosdep before building.<br>
 
 ## Usage
 
 1. Firstly, run nav2 on your robot. Place the robot about 1 metre in front of the dock.
 
 ![robot infront dock](https://lh6.googleusercontent.com/8Bhh3YMV-MircatDYsx8YG8mphGAk7jwq_btky8D_jP0gS2d4w302htnVr6KYWOZFiSRBV4eat1G4qggBHFO2356E8PyecWl6l5oJp2Vzma1c5vQQwapfKBD_2fvummDPAaZR79n)
 
 2. Next, open a new terminal and launch:
 ```
 ros2 launch lidar_auto_docking dockpose_saver_launch.py
 ```
 ![detected dock](https://lh5.googleusercontent.com/f_thEFUo0MaHhae_t5IQLtsMoXNCjN9nMtkIbg_wU8xAvI1kamIzzChXznGhzMZI8_L_NPUZtnj03LMXFwShTe0XpiP9SK_x1RdUcfJ0qZEreRBf03iVrSkOiVlTLHBx4mFphJbJ)
 
 Once the robot has detected a suitable dock candidate, the dock frame would appear in rviz2.<br>
 
 ![gui window](https://lh6.googleusercontent.com/Tm8OtZGG0-LW-NvTaaJr61TdrMGYs7cF6ZYNBUHqJh7zc_2xwIRk9vG3_ZrP7_6fyCdxO5V1CBZcA_EPJO_ksADJ9CUGiI7FqUfRM6gdxbCHZmV8ZJiJg0PEnaDlSBwaam0P6XXX)
 
 Furthermore, a GUI window would pop up. The GUI would display the absolute distance(in metres) between base_link and the dock frame. 
 You may wish to use this tool to decide a suitable distance for the robot to stop at the dock.<br>
 
 3. Once you are satisfied with the selected dock, click on the save dock and bot pose button. This button would save the dock's position in the map frame. 
Also, the transformation between base_link and map would be saved.<br>

![saved dock coord](https://lh5.googleusercontent.com/-xrFyCcjy1bGXcnQKNx6KXZ9sbuVeAdOU-31zJC7PZb5fE74XymiNMyQLMhU3CvuH4JY2ljEeD6sOnCSmmwGKgBWaEXLLvLBLCPYWFGjqPP0sdgRMQeift8254R7bPHKJ8_KnYif)

4. Next, please modify the config/autodock_params.yaml file and key in suitable parameters. 
Please run colcon build once suitable parameters have been keyed in.<br>

### Docking
1. please position the robot about 1-1.5m in front of the dock and run the following command to send the robot to the dock.
```
ros2 launch lidar_auto_docking dockrobot_launch.py

```
The robot should begin to move towards the dock. 
The robot would stop once the distance threshold(docked_distance_threshold) between dock and base_link has been reached.<br>

### Undocking
1. To undock the robot, please run the following command:
```
ros2 launch lidar_auto_docking undock_robot_launch.py
```
The robot would undock from the dock, and it should turn 180 degrees.
 
## Testing in simulation

Before testing docking on real hardware, 
I strongly recommend testing this package in simulation first(gazebo etc.)<br>

1.  Firstly, place the Dock.stl file in the same folder as your .world file.

2.    Next, you may wish to copy and paste this code and place it in the world tag. To add the dock into your world.

```
<model name="dock">
                <!-- x y z r p y-->
                    <pose>-1.0 -0.35 0 1.57 0 1.57</pose>
                    <static>true</static>
                    <link name="body">
                      <visual name="visual">
                        <geometry>
                          <mesh><uri>file://Dock.stl</uri>
                          <scale>0.001 0.001 0.001</scale>
                        </mesh>
                        </geometry>
                      </visual>
                      <collision name="collision1">
                                <geometry>
                                  <mesh><uri>file://Dock.stl</uri>
                                   <scale>0.001 0.001 0.001</scale>
                                   </mesh>
                                </geometry>
                              </collision>

                    </link>
                  </model>

```

## System Integration
 As the main docking program is an action server, you could write action clients to customise where and when you want the robot to engage in the docking/undocking sequence.

If you wish, you could refer to dock_robot.py and undock_robot.py in the scripts folder as a reference.
