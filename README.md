# Overview
ROS2 communication patterns for Micromouse Robot Navigation System in maze exploration done in C++ as Final Group Assigment for ENPM702 Introduction to Robot programming.

# How the Program Works
ROS2 is used to broadcast the commands for the simulation maze that behind integrates a Wall-Aware DFS replanning algorithm for the micromouse. This can be called automatically on run or via an action request.

<img width="4453" height="9119" alt="final_sequence_diagram_light" src="https://github.com/user-attachments/assets/63187a72-0457-4c8b-ba87-571762d8d540" />

# Install & Build Instructions
-Verify you have ROS2 humble distribution installed and also CMAKE corresponding installations. 

Refer to: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

On command line run:
```sh
echo $ROS_DISTRO
```
You should see 'humble' as output but also installing 'jazzy' works.

Clone the repository package inside this folder

```sh
   git clone https://github.com/Jomaxdrill/ROS_micromouse.git
```

You should get a folder called **ROS_micromouse** with the content of the repository inside. It should be like the following example:

```
ROS_micromouse/src/
├── micromouse_interfaces/
│   ├── action/
│   │   └── NavigateToGoal.action
│   ├── srv/
│   │   └── GetRobotStatus.srv
│   ├── CMakeLists.txt
│   └── package.xml
│
└── micromouse_cpp/
├── include/micromouse_cpp/
│   ├── maze_control_api.hpp
│   ├── navigate_action_client.hpp
│   └── get_status_client.hpp
│
├── src/
│   ├── maze_control_api.cpp
│   ├── micromouse_node.cpp
│   ├── navigate_action_client.cpp
│   └── get_status_client.cpp
│
├── config/
│   └── params.yaml
│
├── CMakeLists.txt
└── package.xml
```
Located in the folder **ROS_micromouse** Download all the dependencies before runnning a build.

```sh
rosdep install -i --from-path src --rosdistro humble -y
```

Source ROS (Package will be identified) However you can do make this default when opening the terminal by modifying the .bashrc file. 
<ins>Don't forget your user password to give permissions </ins>
```sh
sudo nano ~/.bashrc
```

```sh
 source /opt/ros/humble/setup.bash
```
**Source package to be identified!**. Do this for every new terminal you open locating the Workspace folder:
<img width="1475" height="136" alt="image" src="https://github.com/user-attachments/assets/ed5f33d1-f9b8-4cd6-ab3a-9267939b1ef0" />

```sh
source install/setup.bash
```
Run the follwing command to be at root of your workspace (~/ROS_micromouse) and build your workspace
```sh
cd ../
colcon build 
```
As result you will have new folders including build, log, and install.

<img width="955" height="361" alt="image" src="https://github.com/user-attachments/assets/335ab88b-b360-4a89-9658-c3afda5fd64c" />

https://github.com/user-attachments/assets/d1781a60-be94-42d2-bf09-11a49487da33


# Run Instructions

1. Open the Micromouse Simulator (`mms`). https://github.com/mackorone/mms. This is where you will get the linux folder. 

<img width="1475" height="136" alt="open MMS" src="https://github.com/user-attachments/assets/52d07e90-b49e-4a9d-854e-7b8b6a536a0d" />

https://github.com/user-attachments/assets/1555b0ee-9bc7-4129-b257-a11c9e9c2e08

It's important that the linux folder that contains the simulator it's inside the ROS_micromouse/src folder to avoid execution errors.

2. Load any classical maze. For more info check this link https://github.com/micromouseonline/mazefiles
   
https://github.com/user-attachments/assets/40d0b32e-3857-4ac8-bc31-cf6d23ffc590

3. Set micromouse profile as:
    - Name:       Micromouse 2
    - Directory:  . (refers to your current folder) 
    - Build Command: leave it empty
    - Run Command :  ros2 run micromouse_cpp micromouse_node --ros-args \-p standalone_mode:=true \--params-file /absolute/path/to/params.yaml 
    
<img width="1954" height="208" alt="image" src="https://github.com/user-attachments/assets/2eb111f2-fd99-46e8-bfc5-db35dfd219a2" />

**standalone_mode** is an important variable parameter that if true  the micromouse navigates immediately using goal set in params.yaml else it waits for action client to send goal.

4. Click **Ok** and run the simulator. You will check the node running is sending commands to the micromouse to move and publishing via the topic robot_position its coordinates in the maze.

https://github.com/user-attachments/assets/422776f5-faa3-431c-a2bd-8b940a1b05ce

## Run in standalone_mode == false

First set on the micromouse profile the variable as false

<img width="1919" height="1019" alt="Screenshot 2025-12-08 233153" src="https://github.com/user-attachments/assets/6e4c6baa-c4e6-4122-b5a7-ba309ad33e1b" />


In a new terminal after sourcing it execute the following command:

```sh
ros2 action send_goal /navigate_to_goal \
    micromouse_interfaces/action/NavigateToGoal \
    "{goal_x: 7, goal_y: 7}" --feedback
```


https://github.com/user-attachments/assets/6ea86340-5557-4bd7-abdc-991e4c11324b



## Full Demo Video:
https://drive.google.com/file/d/1qJNXqxFNxQhxqZwh_O8d0edNBkET3x_T/view?usp=sharing

## Check the node, service, action, interfaces 

<img width="1478" height="705" alt="interface micromouse 2" src="https://github.com/user-attachments/assets/a05b2e4f-8a53-4d03-a999-22cb2df742db" />
<img width="1476" height="555" alt="node info" src="https://github.com/user-attachments/assets/281f65fa-8f2f-481d-b8e2-fabc737248a6" />
<img width="1469" height="338" alt="status client micromouse" src="https://github.com/user-attachments/assets/b5d9cd70-aad4-44b0-a795-d9b33d2e6805" />
<img width="1476" height="196" alt="get robot status" src="https://github.com/user-attachments/assets/939210c1-2aef-4b90-8031-02c29e79c720" />
<img width="1480" height="706" alt="interface micromouse" src="https://github.com/user-attachments/assets/23ca502b-5a07-4062-801d-0146089389a5" />

# Link repo and full video
https://drive.google.com/file/d/1qJNXqxFNxQhxqZwh_O8d0edNBkET3x_T/view?usp=drive_link
