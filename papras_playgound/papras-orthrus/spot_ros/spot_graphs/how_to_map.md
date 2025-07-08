# MAP THE ROOM USING SPOT

## ON Lambda
```
cd /home/kimlab/ws_spot/spot-sdk/python/examples/graph_nav_command_line
```


Connects "<path_to_download>" is download path. 

192.168.80.3 <- IP for spot. Same for every spot

```
python3 -m recording_command_line --download-filepath "<path_to_download>" 192.168.80.3
```

```
            Options:
            (0) Clear map.
            (1) Start recording a map.
            (2) Stop recording a map.
            (3) Get the recording service's status.
            (4) Create a default waypoint in the current robot's location.
            (5) Download the map after recording.
            (6) List the waypoint ids and edge ids of the map on the robot.
            (7) Create new edge between existing waypoints using odometry.
            (8) Create new edge from last waypoint to first waypoint using odometry.
            (9) Automatically find and close loops.
            (a) Optimize the map's anchoring.
            (q) Exit.


```

Clear the map
Stand up spot
Start Recording the map
Add waypoints if needed
Stop recording the map
Optimize the maps anchroing if needed
Download the map
List the waypoints and store in text file
Use waypoints as needed

Detailed instructions below
https://dev.bostondynamics.com/python/examples/graph_nav_command_line/readme#example-programs