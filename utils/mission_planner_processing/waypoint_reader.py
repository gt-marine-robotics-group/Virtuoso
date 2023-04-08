#Intended to convert points recorded by the Virtuoso task based waypoint saver to a set of waypoints readable by mission planner 

import csv
import yaml
import numpy as np

#change input file name accordingly
mission_planner_file = "points_7.yaml"
with open(mission_planner_file, 'r') as f:
    d = yaml.safe_load(f)
    f.close()
#print(d)

#change output file name accordingly
newfilename = "points_7.waypoints"

with open(newfilename, 'a+') as file:
    num_tasks = 8
    
    currentpoint = 1
    file.write('QGC WPL 110\n0\t1\t0\t0\t0\t0\t0\t0\t0\t\t\t1\n')

    for i in range(num_tasks):
        ll_list = d["/**"]["ros__parameters"][f'll_points_{i+1}']
        orientation_list = d["/**"]["ros__parameters"][f'orientations_{i+1}']
        
        for j in range(len(ll_list)):
            lat = ll_list[j][0]
            lon = ll_list[j][1]           
            quatx = orientation_list[j][0]
            quaty = orientation_list[j][1]
            quatz = orientation_list[j][2]
            quatw = orientation_list[j][3]
            
            #writes the angle in degrees
            #NOTE: This is not the proper way to get the heading, as this was just written very quickly for competition. The correct way to do it would be to transform the (1,0,0) vector from the body frame to the map frame using quaternion multiplication. The assumption made here is that quatx and quaty are small. This should have a reasonable amount of accuracy for most cases for surface vehicles.
            angle = np.arcsin(quatz)*2*180/np.pi
            
            curr_task = i+1
            file.write(f'{currentpoint}\t0\t3\t16\t{curr_task}\t0.00000000\t0.00000000\t{angle}\t{lat}\t{lon}\t100.000000\t1\n')     
            currentpoint += 1  
            
        #if i == 0:
        #    print(lat)

    file.close()
