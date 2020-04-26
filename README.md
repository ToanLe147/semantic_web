Implementation of UR5 semantic pick and place in ROS-Gazebo 
==================================================================
This repository presents a simple semantic pick and place task of UR5 in ROS-Gazebo. The Instruction scene shows the target scene that the UR5 needs to re-create. By clicking on "Learn" button, the system learns the current scene through a Kinect camera and generates serie of actions that the UR5 need to accomplish to have the target scene. User checks the learning results and generated tasks in Flask User Interface then clicks on "Perform" button for performance of the UR5.

## Video
<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=VpH7y9el_gs" target="_blank"><img src="http://img.youtube.com/vi/VpH7y9el_gs/0.jpg" alt="Simulation Result" width="450" height="315" border="0" /></a>

## Requirements
  * [ROS Kinetc (Gazebo included) - Also testing with ROS Melodic and it runs well](http://wiki.ros.org/ROS/Installation)
  * [Apache Jena Fuseki - a SPARQL server](https://jena.apache.org/download/#apache-jena-fuseki)
  * [Flask](https://pypi.org/project/Flask/)
  * [ROS - Flask communication](http://wiki.ros.org/rosbridge_suite)
  * [MoveIt](https://moveit.ros.org/install/)
## Installation
* Clone this repository and [Gazebo Link Attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher). The attacher is needed for psuedo vacuum gripper.
  ```terminal
  cd catkin_ws/src
  git clone https://github.com/pal-robotics/gazebo_ros_link_attacher
  git clone https://github.com/ToanLe147/semantic_web
  cd ..
  catkin_make
  ```
* Add Kinect model in Gazebo folder.
  ```terminal
  cd ~/.gazebo
  [ -d "./models" ] && echo "OK" || echo "Not OK"
  ```
  * If the result is **"OK"** then
    ```terminal
    cp -r ~/catkin_ws/src/semantic_web/simulation/kinect_ros ./models/kinect_ros
    ```
  * If the result is **"Not OK"** then
    ```terminal
    mkdir ./models
    cp -r ~/catkin_ws/src/semantic_web/simulation/kinect_ros ./models/kinect_ros
    ```
## How to use
* Open first terminal tab to run ontology server
  ```terminal
  cd {your apache-jena-fuseki folder}
  ./fuseki-server
  ```
In web browser, go to **localhost:3030** to access Apache Fuseki server. Create a dataset name "Brainstorm" and copy [initial_ontology.txt](https://github.com/ToanLe147/semantic_web/blob/master/knowledge_base/initial_ontology.txt) to the default graph.
* Open second terminal tab to run Gazebo environemnt.
  ```terminal
  roslaunch semantic_web experiment.launch
  ```
  * If you want to run rviz with the Gazebo then run this command line instead
    ```terminal
    roslaunch semantic_web experiment.launch rviz:=true
    ```
* Open third terminal tab to run the UI server and other ROS nodes to control the Gazebo experiment
  ```terminal
  roslaunch semantic_web server.launch
  ```
In web browser, go to **localhost:5000** to access the web User Interface. If you want to run the UI in phone or other computer, go to [FlaskApp.py](https://github.com/ToanLe147/semantic_web/blob/master/server/server/FlaskApp.py) in *semantic_web/server/server* to change the host IP (last line) to your server computer IP.
## Current Status
* This repository was tested in ROS Kinetic (Ubuntu 16.04) and partially in ROS Melodic (Ubuntu 18.04). It's running well in Kinetic and the Gazebo experiment also works in ROS Melodic (I haven't test the UI server in Melodic yet).
* If you have warning with [pyassimp](https://pypi.org/project/pyassimp/) when running in ROS Kinetic then fix it by installing **pyassimp** version 3.3
  ```terminal
  pip install --user pyassimp==3.3
  ```
