# obj_detection

## to do
- [ ] acquire aspect ratio from frame size

## Dependencies
* cv_bridge
* image_transport
* std_msgs
* rospy

## Installation
1. Set up a catkin workspace
2. Clone this repository to your workspace src folder:
```
cd <catkin_ws>/src
git clone https://github.com/oreoverdose/obj_detection.git
```
3. Make sure everything is initialized:
```
cd <catkin_ws>
catkin_make
source devel/setup.bash
```

## Usage
### Set parameters
1. Go to the launch folder the package and edit the obj_detection.launch file
```
cd obj_detection/launch
gedit obj_detection.launch
```
2. Change the parameters to match with your setup
### Launching
Launch the obj_detection node with `roslaunch obj_detection obj_detection.launch`
This will create a node that publishes to 3 topics:
* obj_coord - float array that has the coordinates of the object relative to the laptop in cm
* image - Image from the current captured frame
* pxls - integer array of the coordinates of the bottom-most pixel in the contour of the object in the image
