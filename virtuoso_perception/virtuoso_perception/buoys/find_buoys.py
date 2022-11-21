import math
from autoware_auto_perception_msgs.msg import BoundingBoxArray

class FindBuoys:

    def __init__(self, buoy_max_side_length, tall_buoy_min_z, buoy_loc_noise):
        self.buoy_max_side_length = buoy_max_side_length
        self.tall_buoy_min_z = tall_buoy_min_z
        self.buoy_loc_noise = buoy_loc_noise

        self.lidar_bounding_boxes:BoundingBoxArray = None

        self._buoy_counts = list()
    
    def find_buoys(self):

        if self.lidar_bounding_boxes is None:
            return BoundingBoxArray()

        filtered_boxes = BoundingBoxArray()

        filteredBoxesPrevFound = {}

        counter = 0
        for box in self.lidar_bounding_boxes.boxes:
            if (math.sqrt((box.corners[1].x - box.corners[2].x)**2 
                + (box.corners[1].y - box.corners[2].y)**2) 
                > self.buoy_max_side_length): 
                continue

            highest_point = max(c.z for c in box.corners)
            if highest_point >= self.tall_buoy_min_z: 
                box.value = 1.0
            else:
                box.value = 0.5

            filtered_boxes.boxes.append(box)

            if not filteredBoxesPrevFound.get(counter):
                filteredBoxesPrevFound.update({counter: False})

            counter += 1
        

        if len(self._buoy_counts) == 0:
            for i, _ in enumerate(filtered_boxes.boxes):
                filteredBoxesPrevFound.update({i: False})

        for count in self._buoy_counts:
            prevBox = count.get('box')
            prevCount = count.get('count')

            for i, box in enumerate(filtered_boxes.boxes):
                if (filteredBoxesPrevFound.get(i)):
                    continue
                if (math.sqrt((prevBox.centroid.x - box.centroid.x)**2 
                    + (prevBox.centroid.y - box.centroid.y)**2) < self.buoy_loc_noise):
                    filteredBoxesPrevFound.update({i: True})
                    count.get('box').centroid.x = self._find_avg(prevBox.centroid.x, prevCount, box.centroid.x)
                    count.get('box').centroid.y = self._find_avg(prevBox.centroid.y, prevCount, box.centroid.y)
                    # count.get('box').value = box.value
                    if count.get('box').value < box.value:
                        count.get('box').value = count.get('box').value + 0.1
                    elif count.get('box').value > box.value:
                        count.get('box').value = count.get('box').value - 0.1
                    count.update({'count': prevCount + 10})
                    break

            if prevCount == count.get('count') and prevCount > 0:
                count.update({'count': prevCount - 10})
        
        for key, prevFound in filteredBoxesPrevFound.items():
            if not prevFound:
                filtered_boxes.boxes[key].value = 0.5
                self._buoy_counts.append({
                    'box': filtered_boxes.boxes[key],
                    'count': 1
                })
        

        confirmed_buoys = BoundingBoxArray()

        confirmed_buoys.boxes = list(
            map(lambda b: b['box'], filter(lambda b: b['count'] > 90, self._buoy_counts))
        )

        # self.boxes_pub.publish(confirmedBuoys)
        return confirmed_buoys

    def _find_avg(self, curr:float, count:int, next:float):
        sum = (curr * count) + next
        return sum / (count + 1)