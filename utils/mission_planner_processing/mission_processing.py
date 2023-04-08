#Reads in the output from Mission Planner and converts them into task separated waypoints with orientations for Virtuoso

import csv
import yaml
import numpy as np

#intended to enter a number so the file name to open is "bravo_#.waypoints
#feel free to change from bravo to whatever the actual course name is
mission_planner_file = input('bravo_: ')

#skips two rows, so be sure the output from mission planner has 2 rows that don't matter (should be like a row that gives the columns titles and then a set of home coordinates that isn't actually a point on the path"

with open("bravo_" + mission_planner_file + ".waypoints") as f:
    d = np.loadtxt(f, delimiter="\t", skiprows=2)
    f.close()
#test to make sure that the file opened correctly
print(d[0:, 8])

num_rows, num_cols = d.shape

#number of tasks total (index from 1)
num_tasks = 8

ll_points = dict()
orientations = dict()
#get lat lon and orientations from mission planner
for i in range(num_tasks):
    ll_points[str(i + 1)] = list()
    orientations[str(i + 1)] = list()

#Be sure to change the altitude from -19.9 to whatever the GPS reads at the location you are at
for i in range(num_rows):
    curr_task = int(d[i,4])
    lat = d[i,8]
    lon = d[i,9]
    ll_points[str(curr_task)].append([lat, lon, -19.9])
    angle_deg = d[i,7]
    angle_rad = angle_deg*np.pi/180
    #convert angle to radians. Remember, 0 degrees heading is East. 
    orientations[str(curr_task)].append([
        0.0,
        0.0,
        np.sin(angle_rad/2),
        np.cos(angle_rad/2)
    ])


#intended that you just input a number such that the file is spit out as "points_#.yaml"
newfilename = input('points_: ')

with open("points_" + newfilename + ".yaml", 'a+') as file:
    file.writelines([
        '/**:\n',
        '  ros__parameters:\n',
        *(f'    ll_points_{i+1}: {ll_points[str(i+1)]}\n' for i in range(num_tasks)),
        *(f'    orientations_{i+1}: {orientations[str(i+1)]}\n' for i in range(num_tasks))
    ])
    file.close()
