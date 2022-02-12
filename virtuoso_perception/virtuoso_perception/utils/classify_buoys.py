from autoware_auto_perception_msgs.msg import BoundingBoxArray 

# Sorts boxes from "left" to "right".
# This only works when the wamv is facing the appropriate direction.
def sort_boxes(boxes:BoundingBoxArray):
    
    sorted_arr = []

    for box in boxes.boxes:
        inserted = False 

        for i, sorted_box in enumerate(sorted_arr):
            if sorted_box.centroid.y < box.centroid.y:
                sorted_arr.insert(i, box)
                inserted = True
                break

        if not inserted: sorted_arr.append(box)
    
    return sorted_arr

# Sorts buoys "left" to "right" as seen in the camera
def sort_buoys(buoys):

    sorted_arr = []

    for buoy in buoys:
        inserted = False

        for i, sorted_buoy in enumerate(sorted_arr):
            if sorted_buoy.x > buoy.x:
                sorted_arr.insert(i, buoy)
                inserted = True
                break

        if not inserted: sorted_arr.append(buoy)
    
    return sorted_arr


# Values Guide:
# 0 = tall black
# 1 = tall green
# 2 = tall red
# 3 = tall white
# 4 = round black
# 5 = round orange
def classify_buoys(boxes, buoys):

    sorted_boxes = sort_boxes(boxes)
    sorted_buoys = sort_buoys(buoys)

    for i, box in enumerate(sorted_boxes):

        if i >= len(sorted_buoys):
            # just guess as camera did not identify a buoy
            box.value = 0.0 if box.value == 1.0 else 4.0
            continue
            
        buoy = sorted_buoys[i]

        if buoy.color == 'black':
            box.value = 0.0 if box.value == 1.0 else 4.0
            continue

        if buoy.color == 'red_or_orange':
            box.value = 2.0 if box.value == 1.0 else 5.0
            continue

        if buoy.color == 'green':
            box.value = 1.0
            continue

        box.value = 3.0
    
    return sorted_boxes