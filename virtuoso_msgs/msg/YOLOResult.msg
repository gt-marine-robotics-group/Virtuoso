
# x1, y1, x2, y2 are the coordinates which form a bounding box around the detected object
float32 x1
float32 y1

float32 x2
float32 y2

int32 class_id # e.g. 0, 1, 2, 3
string label # e.g. black, green, red

float32 confidence # e.g. between 0 and 1