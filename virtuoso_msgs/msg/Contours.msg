
std_msgs/Header header

# All the contour points flattened into a 1d array
int32[] contour_points

# Offet for start of each contour in points array
int32[] contour_offsets

# Color for each contour in order they appear above
# e.g. "red", "green", "yellow", "black" options for buoys
string[] contour_colors