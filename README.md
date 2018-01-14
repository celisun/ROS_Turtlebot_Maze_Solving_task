# Turtlebot Maze Solving
Exemplary code for CS 117a Autonomous Robotics Lab 

@ Celi Sun @ Nov, 2017 @ Brandeis university 

To nagivate turtlebot solving a maze, we use: 

- main
- scan twist center control
- policy
- helper controller

The scan twist center control is receiving messages, processing message raw data, and publishing reaction messages once activated. It is also registered with a policy and an optional helper controller to help it make decisions of what action to take in response to the incoming message it receives, and therefore, eventually, navigates a turtlebot out of maze.

The repo here gives the exemplary code of solution and the default policy we provide is LeftOrRightHandRule, but you are more than welcomed to add your own policies (extends our Policy class). To add, put your policy script in the policy folder, and make the center control "know" it, and get it registered in the constructor to make it work.
 
Oct/2017


### Run solution on your own labtop
#### Clone repository
You can clone this resiporoty by git to your own workspace by:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/campusrover/CourseExemplary_TurtlebotMazeSolving.git
```
Go back to catkin_ws and build the packages in the workspace by running catkin_make command:
```
$ cd ..
$ catkin_make
```
#### Configure IP address of labtop/turtlebot for ROS
On terminal,
```
$ ifconfig
```
In pop-up window, find the IP address of your own labtop/turtlebot shown after the last 'inet addr'.

Then, set up with the IP address you found
```
$ gedit ~/.bashrc
```
In pop-up .bashrc file, change the address of ROS_MASTER_URI (where the ros core is) and ROS_HOSTNAME (where your labtop is) to the IP address of your labtop (but remain the port number, in this example '11311'). For example, if your laptop IP address is 129.64.147.246, change the following as:
```
export ROS_MASTER_URI=http://129.64.147.246:11311
export ROS_HOSTNAME=129.64.147.246
```
After, close the file and source the bash file by running:
```
$ source ~/.bashrc
```
#### Ready to run
```
rosrun CourseExemplary_TurtlebotMazeSolving main
```
Now watch how the turtlebot solves the maze !
