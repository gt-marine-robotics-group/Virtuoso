# Spawning models and taking images of them from the wam-v front camera.
# Author: Jehan Dastoor
# Created on: 17th November 2021
# Last updated: 10th December 2021
# Notes: has only been tested on the vrx.launch gazebo file, not sure if other worlds will work (although it should)

import numpy as np
import rospy, tf, time
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest, SetLightProperties, SetLightPropertiesRequest
from geometry_msgs.msg import *
from std_msgs.msg import Float64, ColorRGBA
from std_srvs.srv import Empty
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# Pygazebo for changing fog
import pygazebo
from pygazebo.msg.v11 import fog_pb2, color_pb2, scene_pb2

# OpenCV2 for saving an image
import cv2
import os

class ImageCollection():
    def __init__(self, number_spawn, model_path, folder_name, image_name, time_delay=3.0):
        # Initialize the node
        rospy.init_node('image_listener')
        
        print("Waiting for gazebo services...")
        #rospy.init_node("spawn_products_in_bins")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service("gazebo/get_model_state")
        print("Got it.")
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.model_coordinates = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        
        # Instantiate CvBridge
        self.bridge = CvBridge()
        self.image_number = 0
        self.number_spawn = number_spawn
        self.time_delay = time_delay
        self.model_path = model_path
        self.image_name = image_name

        # Create a directory to store images
        self.folder_path = folder_name
        try:
            os.mkdir(self.folder_path)
            print("Created new folder titled " + self.folder_path)
        except:
            print("Folder already exists, appending to that folder...")

        # Define your image topic
        self.image_topic = "/wamv/sensors/cameras/front_left_camera/image_raw"

        # Start the subscriber
        self.start_taking_images()
    
    def spawn(self):

        with open(self.model_path, "r") as f:
            product_xml = f.read()

        # Determine a random position for the buoy
        bin_x, bin_y = self.generate_random_position()
        self.item_name = self.image_name+"_{0}".format(self.image_number)
        print("Spawning model: %s" % self.item_name)

        # Want to give the object a random yaw
        random_yaw = np.random.random()
        orient = Quaternion(*tf.transformations.quaternion_from_euler(0,0,random_yaw*np.pi)) 

        item_pose = Pose(Point(x=bin_x, y=bin_y, z=0), orient)
        self.spawn_model(self.item_name, product_xml, "", item_pose, "world")

    def generate_random_position(self):
        # Find position of the wam-v
        wamv_coordinates = self.model_coordinates('wamv', '')
        wamv_x = wamv_coordinates.pose.position.x
        wamv_y = wamv_coordinates.pose.position.y

        # Quaternion orientation
        wamv_orientation_x = wamv_coordinates.pose.orientation.x
        wamv_orientation_y = wamv_coordinates.pose.orientation.y
        wamv_orientation_z = wamv_coordinates.pose.orientation.z
        wamv_orientation_w = wamv_coordinates.pose.orientation.w

        # Getting the Euler orientation (in radians)
        angle_x, angle_y, angle_z = tf.transformations.euler_from_quaternion([wamv_orientation_x, wamv_orientation_y, wamv_orientation_z, wamv_orientation_w])

        # Randomly deciding the position of the object to spawn
        # Pick a random distance away from the wam-v, and then find x and y
        min_distance = 5 # so that its not right under the wam-v
        max_distance = 30 # tested to be the best
        distance = (max_distance-min_distance) * np.random.random() + min_distance

        #Adding a conic random angle
        angle = 5*np.pi/12
        conic_angle = np.random.random() * angle - angle/2

        relative_x = distance * np.cos(angle_z + conic_angle)
        relative_y = distance * np.sin(angle_z + conic_angle)

        absolute_x = relative_x + wamv_x
        absolute_y = relative_y + wamv_y
        return absolute_x, absolute_y

    def start_taking_images(self):
        # Start image taking node
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size = 1, buff_size = 2**30)
    
    def image_callback(self, msg):
        self.msg = msg

    def shutdown(self):
        # Shutdown the ROS node
        print("Shutting down ImageNode after taking {} image(s)...".format(self.image_number))
        rospy.signal_shutdown("ROS is shutting down ImageNode")

    def main(self):
        while not rospy.is_shutdown():
            self.spawn()
            time.sleep(self.time_delay)

            print("Received an image!")
            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = self.bridge.imgmsg_to_cv2(self.msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            # Save your OpenCV2 image as a png
            cv2.imwrite(self.folder_path+'/'+self.image_name+'_{}'.format(self.image_number)+'.png', cv2_img)
            print("Saved Image!")
            
            print("Deleting model: %s" % self.item_name)
            print("==============================")
            self.delete_model(self.item_name)

            self.image_number += 1
            if self.image_number == self.number_spawn:
                self.shutdown()


if __name__ == "__main__":
    # Number of objects to spawn
    number = 1

    # Spawning crocodiles
    #spawner = ImageCollection(number, "models/crocodile_buoy/model.sdf", folder_name = "Gazebo_Images", image_name = "croc", time_delay=1.0) #delay=1 seems to be the lowest
    #spawner.main()

    fog_color = color_pb2.Color()
    fog_color.r = 1

    pygazebo.connect(address=('jehan-VirtualBox', 42729))
    fog_setting = fog_pb2.Fog(type=2, density = 1000, color=fog_color, start=0)
    scene_pb2.Scene(fog = fog_setting)

    #work on spawning the objects in front of the camera, and then work on filtering the images to try and just get the picture of the crocodile.
    #have images create a directory called images and put all the images in that directory