# Mission Planner Processing

## Installing Mission Planner

Install Mission Planner, a software for GPS waypoint planning.
https://discuss.ardupilot.org/t/run-mission-planner-on-ubuntu-20-04/67280

## Using Mission Planner

Please use external sources to learn more about Mission Planner.

![mission_planner_ui](https://user-images.githubusercontent.com/33461797/230733202-eb2343ba-634e-4a3d-9839-25d232fb33ff.png)

Use the plan function to create a path. It may be useful to add POIs (points of interest) based on your estimates of course objects or prior waypoints. 

The points will be classified based on task. The task is set by the first numbered column right of the "WAYPOINT" column and left of the "Acc Radius" column. In this screenshot, there is one waypoint for task 3, one for task 4, and two for task 6. Be sure that within each task, the waypoints are ordered correctly.

The heading is indicated using degrees, relying on the fact that the localization uses ENU as the map frame axes. This is set by the column to the left of "Pass by" and right of "Lat." In this screenshot, the headings are -90, 90, 0, and 0, which correspond to South, North, East, and East.

Be sure to set tasks and headings for each waypoint.

A useful tool is the "WP radius" option, which will change the displayed radius of the white circles. This will allow you to more easily see the scale of the map.

Output the file using "save file" using your file naming convention of choice. I like using "coursename_#.waypoints", incrementing the number for each run.

## Mission Processing
Reads in the output from Mission Planner and converts them into task separated waypoints with orientations for Virtuoso. 

Note that it skips two rows, so be sure the output from mission planner has 2 rows that don't matter (should be like a row that gives the columns titles and then a set of home coordinates that isn't actually a point on the path.

Be sure to change the altitude in the script from -19.9 m to whatever the GPS reads at the location you are at.

Beforehand, ensure that mission_processing.py adheres to your desired file naming convention for mission planner file outputs. As is, the convention is "bravo_#.waypoints".

First, create your waypoints file in mission planner and output it. 

Next, run the mission_processing.py script. Ideally, you should make it so that mission_processing.py will only take in a number. This is so that you can process and upload files quickly while keeping track of them.

You will then need to input the number for the output of the script, "points_#.yaml". You can also change this to be whatever file name you wish, just ensure that it is set up how you like before you run the script.

The script will then output the points file for Virtuoso input.

## Uploading to the onboard computer
Use the scp command while connected to the vehicle's network to upload the points file to the onboard computer. An example is below. 

`scp points_24.yaml mrg@192.168.0.100:~/mrg/semis_waypoints`

`scp [local points file location] [username]@[ip]:[directory for points on onboard computer]`


## Waypoint Reader
Intended to convert points recorded by the Virtuoso task based waypoint saver to a set of waypoints readable by mission planner.

Be sure to change the input and output file names in the script before running it.
