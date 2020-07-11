# System Integration Project

This is the repo for the final project of the Udacity Self-Driving Car Nanodegree. More information about the project, can be found [here](https://github.com/udacity/CarND-Capstone).

[//]: # (Image References)
[image1]: ./imgs/carla_architecture.png
[image2]: ./imgs/rosgraph.jpg
[image3]: ./imgs/system_architecture.png

## Project Overview

### Carla Architecture
Carla is the custom Lincoln MKZ that Udacity has converted into a self-driving car.  It's self-driving system is broken down into four major sub-systems: **Sensors**, **Perception**, **Planning** and **Control** 

![][image1]

#### Sensors
Includes everything needed to understand its surroundings and location including **cameras**, **lidar**, **GPS**, **radar**, and **IMU**
#### Perception
Abstracts sensor inputs into object **detection** and **localization**
##### Detection
* Includes software pipelines for vehicle detection, traffic light detection, obstacle detection, etc
* Techniques in image manipulation include Histogram of Oriented Gradients (HOG) feature extraction, color transforms, spacial binning
* Methods of classification include sliding-window or sub-sampling along with heat maps and bounding boxes for recurring detections
##### Localization
* Answers the question: “Where is our car in a given map with an accuracy of 10cm or less?”
* Based on the notion that GPS is not accurate enough
* Onboard sensors are used to estimate transformation between measurements and a given map
#### Planning
Path planning is broken down into for sub-components: **route planning**, **prediction**, **behavioral planning**, and **trajectory planning**
##### Route Planning
The route planning component is responsible for high-level decisions about the path of the vehicle between two points on a map; for example which roads, highways, or freeways to take. This component is similar to the route planning feature found on many smartphones or modern car navigation systems.
##### Prediction
The prediction component estimates what actions other objects might take in the future. For example, if another vehicle were identified, the prediction component would estimate its future trajectory.
##### Behavioral Planning
The behavioral planning component determines what behavior the vehicle should exhibit at any point in time. For example stopping at a traffic light or intersection, changing lanes, accelerating, or making a left turn onto a new street are all maneuvers that may be issued by this component.
##### Trajectory Planning
Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.
### Control
The control component takes trajectory outputs and processes them with a controller algorithm like **PID** or **MPC** to adjust the control inputs for smooth operation of the vehicle. 

### ROS Architecture

The ROS Architecture consists of different nodes (written in Python or C++) that communicate with each other via ROS messages. The nodes and their communication with each other are depicted in the picture below. The ovally outlined text boxes inside rectangular boxes represent the ROS nodes while the simple rectangular boxes represent the topics that are subscribed or published to. The direction of the arrows clarifies the respective flow of communication. 

![][image2]

The most central point in the rqt-graph is the styx_server that links the simulator and ROS by providing information about the car's state and surroundings (car's current position, velocity and images of the front camera) and receiving control input (steering, braking, throttle). The other nodes can be associated with the three central tasks Perception, Planning and Control. 

The images get processed within the traffic light classifier by a trained neural network in order to detect traffic lights. The percepted state of a potentially upcoming traffic light is passed to the traffic light detector as well as the car's current pose and a set of base waypoints coming from the waypoint loader. With this frequently incoming information the traffic light detector is able to publish a waypoint close to the next traffic light where the car should stop in case the light is red. 

With the subscribed information of the traffic light detector and the the subscriptions to base waypoints, the waypoint updater node is able to plan acceleration / deceleration and publish it to the waypoint follower node. This node publishes to the DBW (Drive by wire) node that satisfies the task of steering the car autonomously. It also takes as input the car's current velocity (coming directly from the car / simulator) and outputs steering, braking and throttle commands. 

![][image3]


Udacity provides the basic execution framework, so the focus here was on the implementation and integration of the following 3 nodes:

* [Traffic Light Detection](#traffic-light-detection): Part of the *perception* sub-system,  the node is responsible for detecting traffic lights as well as classifying their state.

* [Waypoint Updater](#waypoint-updater): Part of the *planning* sub-system, the node is responsible for generating trajectories (as a set of path points with their respective target speeds) considering the detected  traffic lights in the environment.

* [Drive by Wire Controller](#drive-by-wire-controller): Part of the *control* sub-system,  the node is responsible for translating the [Twist Messages](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) generated by the [waypoint follower](./ros/src/waypoint_follower) into throttle, brake and steering values.

### Traffic Light Detection

The [traffic light detection node](./ros/src/tl_detector) is responsible for detecting incoming traffic lights and classifying their state.  If the status of an upcoming traffic light is classified as a red light, the detector calculates and publishes its position (waypoint index), so that the [waypoint updater](#waypoint-updater) can take the position of the upcoming red light in generating a trajectory (accelerate, keep speed, slow down).

The node subscribes to 3-4 topics:

* **/base_waypoints:** The topic publishes the complete list of waypoints for the course. 
* **/current_pose:** The topic publishes the current position of the vehicle.
* **/image_color:** The topic publishes the RGB image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.
* **/vehicle/traffic_lights:** The topic publishes the (x, y, z) coordinates of all traffic lights. (only in simulator).

![Traffic light detection](./imgs/tl-detector-ros-graph.png  "Traffic light detection")
Traffic light detection node

In this project, the [detector](./ros/src/tl_detector/light_classification/tl_classifier.py) uses [Tensorflow](https://tensorflow.org) to run a variant of a [Single Shot MultiBox Detector](https://arxiv.org/abs/1512.02325) that uses as feature extractor [InceptionV2](https://arxiv.org/abs/1512.00567). The model weights used by the included graph are pre-trained on the [COCO Dataset](http://cocodataset.org) that already contains the traffic light category and provided by the [Tensorflow Object Detection API](https://github.com/tensorflow/models/blob/master/research/object_detection).

In this model, three different classes (GREEN, YELLOW, RED) are fine-tuned for different states of traffic signals, and the target detection, positioning and classification are folded into a single end-to-end model.

The model was fine-tuned using labelled images from various sources, including the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases), the [Udacity Training Bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) and images captured by Carla from real runs at the test lot of Udacity (Download [here](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing)). Two different models were developed for both the simulator and Carla scenarios to accurately identify virtual and real traffic lights.

Classification is only performed when the vehicle current location is between 0 and 75 waypoints from the traffic light stopline.  In order to realize traffic light detection in  both simulator and Carla, different classification models need to be used for the Simulator and Site. The `self.config` dictionary found in the `TLDetector` class of `tl_detector.py` contains an `is_site` boolean. This boolean can be used to load a different classification model depending on the context.

| Classes | Traffic light |
| ------- | :------------ |
| 1       | Green         |
| 2       | Red           |
| 3       | Yellow        |

### Waypoint Updater

![waypoint_updater](./imgs/waypoint-updater-ros-graph.png  "waypoint_updater")
Waypoint_updater node

The waypoint updater node is responsible for generating a trajectory in terms of waypoints so that the [waypoint follower](./ros/src/waypoint_follower) can generate the correct [Twist Messages](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) consumed by the [DBW Node](./ros/src/twist_controller/dbw_node.py). Each of the waypoints are generated from a subset of the overall set of waypoints provided according to the vehicle position (published in the `/current_pose` topic). For each waypoint a target velocity is generated to smoothen the transition between each waypoint.

The node also subscribes to the `/traffic_waypoint` topic published by the [Traffic Light Detection Node](#Traffic Light Detection) in order to adjust the waypoints in the presence of a red light.

Finally the node publishes the list of waypoints ahead of the vehicle to the `/final_waypoints` topic.

### Drive by Wire Controller

![DBW Controller](./imgs/dbw-node-ros-graph.png  "DBW Controller")
DBW Controller node

Udacity's self-driving car Carla is equipped with a drive-by-wire (DBW) system, which controls the throttle, brake, and steering electronically.

The goal for this part of the project was to implement the drive-by-wire node (`dbw_node.py`) which will subscribe to `/twist_cmd` and use various controllers to provide appropriate throttle, brake, and steering commands. The node subscribes to the `/current_velocity` topic along with the `/twist_cmd` topic to receive target linear and angular velocities (generated by the waypoint follower node from the waypoints published by the [waypoint updater](#waypoints-updater). The [DBW node](./ros/src/twist_controller/dbw_node.py) primarily defines the communication interface.  This node publishes throttle, brake, and steering commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd topics`.  The [twist_controller](./ros/src/twist_controller/twist_controller.py) contains the algorithms used to generate the control values.

The longitudinal control (throttle/brake) relies on the speed target generated by the waypoint updater node.  A [low pass filter](./ros/src/twist_controller/lowpass.py) is used to remove high-frequency noise from the measured vehicle velocity.  Separate PID controllers generate the throttle and steering commands (in [twist_controller](./ros/src/twist_controller/twist_controller.py)), using different gains summarized in the Table below:

| Parameter | P   | I   | D   |
|-----------|-----|-----|-----|
| Throttle  | 0.3 | 0.1 | 0.0 |
| Steering  | 0.5 | 0.0 | 0.02| 

Computing CTE value of steering for PID in `dbw_node.py`  is necessary.

## References
The code in this project was adapted from the ROS courses, which were part of Udacity's Self-Driving Car Nanodegree program and [Carla AI](https://github.com/williamhyin/Udacity-CarND-capstone-CarlaAI).