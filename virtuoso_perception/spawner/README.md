# Spawner

This is a folder containing a [script](image_taker.py) to spawn objects in a gazebo world. This script assumes that you already have ROS installed (has only been tested with ROS Noetic).

## Gazebo

Gazebo currently only runs on Ubuntu. Install gazebo as:
```bash
curl -sSL http://get.gazebosim.org | sh
```
If installed correctly, gazebo should open after running
```bash
gazebo
```
Alternative installation instructions can be found [here](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).

Additionally, we will need to install ```gazebo_ros_pkgs``` to interface with gazebo effectively. The instructions for installing this can be found [here](https://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros#InstallGazebo).

## VRX World

Follow the [System Setup](https://github.com/osrf/vrx/wiki/tutorials) tutorial from the VRX repository to set up gazebo. 

To run the base world (taken from [here](https://github.com/osrf/vrx/wiki/vrx_build_test_tutorial)):

### Step 1: Build
First, source the ROS ```setup.bash``` file:
```bash
source /opt/ros/noetic/setup.bash
```
Use ```catkin_make``` to build the software:
```bash
cd ~/vrx_ws
catkin_make
```

**Note**: Every time you change the environment variables, you need to catkin_make the workspace again. This is very important, otherwise you're not actually simulating varied lighting conditions.

### Step 2: Run
This is to run one of the example worlds provided by the VRX Simulation platform. 

Source the newly created VRX ```setup.bash``` file (you always have to source!):
```bash
source  ~/vrx_ws/devel/setup.bash 
```
Launch the VRX simulation:
```bash
roslaunch vrx_gazebo vrx.launch
```
**Note**: I would recommend making a .bashrc for this operation just to simplify the process of re-running (just press up arrow and enter).

Gazebo should launch a basic world with the WAM-V and various other objects. For more examples, look [here](https://github.com/osrf/vrx/wiki/environments_tutorials#vrx).

## Usage
### Editing environment variables
We want to take a variety of images at various values of ambient light and fog intensity. We need to edit the proper environment variables in the sydneyregatta.xacro file of your vrx_ws installation.

[Tutorial for changing environment variables](https://github.com/osrf/vrx/wiki/changing_visual_params_tutorial)

The possible values of the environment variables are outlined in the [technical specifications of vrx](https://robonation.org/app/uploads/sites/2/2021/11/VRX2022_Technical-Guide_v1.1.pdf), page 11.

I decided that a reasonable way to break it down would be:\
0, 0.5 and 1 fog density \
0.7, 0.8, and 0.9 fog color \
0.3, 0.5, 0.7, 0.9, and 1 ambient light.

This gives us a total of 45 combinations of lighting and fog condition to take pictures in. At a recommended 1000/2000 images per class, I believe the optimal number of pictures to take is 45 per configuration. It takes 1 second per image, and given the relatively long startup time of the simulation, it's really not worth it to take smaller batches of images, so we might as well take larger batches each time. If we choose not to label them all later, that's fine.

### Running the script
Drop a gazebo model folder of the object you are interested in into the models folder. After this, you can open the [```image_taker.py```](image_taker.py) file. 

At the bottom, the class ```ImageCollection``` is called with the main input parameters being: number of objects to spawn, the file location of the .sdf model of the object you desire to spawn, the folder name where you would like to store the image results, the start of each image name, and the time delay between the taking of each image (so that Gazebo has enough time to update). Calling the main function after this will spawn your objects.

There are configuration variables in the main method that allow you to input the environmental variables and name of the model, like croc, turtle, or platypus. These need to be set each time you change them and re-run the script.

The script randomly places your desired object in front of the WAM-V up to a certain distance away (which can be changed inside the ```generate_random_position``` function. An image of the object from the WAM-V's in-built ```"/wamv/sensors/cameras/front_left_camera/image_raw"``` ROS topic. The object is then deleted, and the next object is spawned, and so on. 

## Current Work
The pygazebo library has been updated [here](https://github.com/robogeekcanada/STEMClub/tree/master/pygazebo). You can take this file by itself, or install the py3gazebo library [here](https://github.com/wil3/py3gazebo), and replace the ```pygazebo.py``` file with the updated version. 

Have been working on adding fog to gazebo using Python. This is what the [```changing_fog.py```](changing_fog.py) script is trying to do. Trollius was not working, so Asyncio was being used to define gazebo subscribers and publishers. A working subscriber was created, however I have been unable to make a publisher work (which is what we need to publish information to gazebo to change the the fog setting). At the bottom of the file, change the argument in ```loop.run_until_complete()``` to be the ```subscriber_loop``` or ```publisher_loop``` depending on which you want. 

The idea would be to get this publisher loop working, and incorporate this into the ```ImageCollection``` class to update the fog level randomly each time an object is spawned. After collecting all these images, one can then train a tiny-YOLO model on them for object detection. With a time delay of 1 second (which seems to work), one can take 5,000 images in about an hour and a half, so time is not a major concern. Note that the image sizes are each about 1Mb. There is also a ```/wamv/sensors/cameras/front_left_camera/compressed``` topic, however it does not work with the current script; a new technique must be used to save these compressed images (they seem to be a different data storage type almost). 

## References
[1] 
Bingham, B., Aguero, C., McCarrin, M., Klamo, J., Malia, J., Allen, K., Lum, T., Rawson, M., and Waqar, R. (2019). 
Toward Maritime Robotic Simulation in Gazebo, *Proceedings of MTS/IEEE OCEANS Conference*.
