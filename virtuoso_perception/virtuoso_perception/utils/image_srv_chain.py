from rclpy.client import Client
from sensor_msgs.msg import Image, CameraInfo
from virtuoso_msgs.srv import ImageNoiseFilter, ImageResize
from typing import List

class ImageSrvChain:

    _name_to_data = {
        'perception/image_noise_filter': set(('image')),
        'perception/image_resize': set(('image', 'camera_info'))
    }

    def __init__(self, clients:List[Client]):

        self._clients = clients

        self.image = None
        self.camera_info = None
    
        self.running = False
        self.curr = 0
    
    def run(self):
        self.running = True 
        self.curr = 0
        self.make_call()
    
    def make_call(self):
        client:Client = self._clients[self.curr]

        client.wait_for_service(timeout_sec=2.0)

        msg = client.srv_type.Request()
        msg.image = self.image
        if 'camera_info' in ImageSrvChain._name_to_data[client.srv_name]:
            msg.camera_info = self.camera_info

        call = client.call_async(msg)
        call.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        result = future.result()
        if result.image is None:
            raise RuntimeError('Service did not return an image')
        self.image = result.image

        if 'camera_info' in ImageSrvChain._name_to_data[self._clients[self.curr].srv_name]:
            self.camera_info = result.camera_info

        self.curr += 1

        if self.curr >= len(self._clients):
            self.running = False
            self.curr = 0
            return

        self.make_call()
