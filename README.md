# Decentralised_multi_robot_exploration
This repository contains the work done for my master's thesis "Maximizing information gain and coordination in exploration with multiple mobile robots".
The overall aim of this dissertation is to implement an algorithm capable of giving a team of mobile robots the ability to efficiently explore a location, creating a global map of the environment. To achieve this goal, i used some packages made available by other authors, such as Robot Laser Grid Mapping, Move Base, Action Lib and Map Merge. The Multi Robot Exploration package was made by me and is available in this repository.

## HOW TO CREATE A WORKSPACE FOR IT ##

1º -> ROS installation. As i use UBUNTU "Focal", i installed ROS "Noetic". You can follow this tutorial (https://wiki.ros.org/noetic/Installation/Ubuntu)

2º -> Create a ROS workspace and download all the files in this repository to that workspace. (Follow this tutorial https://wiki.ros.org/catkin/Tutorials/create_a_workspace)

3º -> Then you just need to solve the dependencies of the ROS package.
You can type "rosdep check multi_robot_exploration" to check all package dependencies
Then, you need to type "rosdep install --from-paths src --ignore-src -r -y" to solve all of them.

## HOW TO USE IT ##

1º -> Open cmd window and type "cd WORKSPACE_CREATED"

2º -> Also in cmd window type "source devel/setup.bash"

3º -> Then, for example, type "roslaunch multi_robot_exploration freiburg_079_2_robos.launch" to start a exploration in Freiburg 079 map with two robots

4º -> Make sure the files are executable

## WHAT CAN I DO WITH IT ##

          -----------> Change the number of robots <-----------
To change the number of robots, you only need to use different files, like "freiburg_079_6_robos.launch" to start a exploration with 6 robots.
I have already create explorations with 1, 2, 3, 4 or 6 robots. So, you can use it.

                -----------> Change the map <-----------
I have two maps set up to test the exploration methods. To use Freiburg 079, you use "freiburg_079_6_robos.launch", and to use Freiburg Campus, you use "freiburg_campus_6_robos.launch"

        -----------> Change the exploration method <-----------
I have created 3 different methods, random exploration "exploracao_aleatoria_freiburg_079.py", greedy exploration "exploracao_gulosa_freiburg_079.py" and the proposed method "multi_explore_freiburg_079.py".
To change the scanning method used by the robots, all you have to do is access, for example, the "freiburg_079_2_robos.launch" file and change the file included in type, as you can see:

                      <!--Explore Node -->
                          <node pkg="multi_robot_exploration" type="multi_explore_freiburg_079.py" name="explore" output="screen">
                            <param name="robot_namespace" value="$(arg prefix)"/>
                          </node>



If you have any questions, you can send an email to danitsantos.21@gmail.com
