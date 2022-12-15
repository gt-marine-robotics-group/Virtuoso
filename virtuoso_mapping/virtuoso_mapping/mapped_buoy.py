from geometry_msgs.msg import Point
from virtuoso_msgs.msg import Buoy

class MappedBuoy:
    
    def __init__(self, buoy:Buoy):
        self.location = buoy.location
        self.color = buoy.color

        self.count = 1
    
    def compute_new_avg(current:float, new:float, weight:float):
        return (current * (1 - weight)) + (new * weight)
    
    def add_detected_buoy(self, buoy, distance_from_usv:float):
        if self.color != buoy.color:
            raise Exception('Buoy color cannot change')
        
        self.count += 1
        
        weight = 1 / distance_from_usv

        if weight > 1: weight = 1

        self.location.x = MappedBuoy.compute_new_avg(self.location.x, 
            buoy.location.x, weight)
        self.location.y = MappedBuoy.compute_new_avg(self.location.y, 
            buoy.location.y, weight)
    
    def __repr__(self):
        return f'MappedBuoy(location={self.location}, color={self.color})'
        
