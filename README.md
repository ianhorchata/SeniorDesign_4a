Senior Design Project for Lafayette College
ME497 and ME498
This is a two-semester class. Our group is developing a delivery robot to deliver food across campus.
Our final delivery this semester is a proof-of-concept prototype. 
Our physical prototype should demonstrate that we have a hardware base that can traverse stairs. 
Our software should demonstrate we have the capcity to navigate complex environments in simulation.

The hardware is designed based off [this video](https://www.youtube.com/watch?v=3Zx7tGtwF5g) of a rocker style robot.
PVC was bought and cut to size as seen here:
<img src="media/ab94f19a-39a1-4adc-b626-e068a465ee06.jpg" alt="RViz for Nav Screenshot" width="1000">

The software uses [OctoMap](https://github.com/octomap/octomap) based off of [Soft-Illusion's github page](https://github.com/Soft-illusion/Robotics_PicoDegree/tree/octomap).
The simulation uses WeBots, the world can be seen here.
<img src="media/Screenshot 2026-11-18 134643.png" alt="WeBots Screenshot" width="1000">

As the Robot drives, the LiDAR on board will slowly fill in the world that it senses. This can be seen in RViz.
<img src="media/Screenshot 2026-11-18 134625.png" alt="RViz Screenshot" width="1000">
A voxel map is created from the LiDAR point cloud and is shown in OctoViz. 
<img src="media/Screenshot 2026-11-18 134555.png" alt="OctoVis Screenshot" width="1000">
From this map, a floor can be filled in, and a trajectory from any two points can be created. .
<img src="media/Screenshot 2026-11-18 134658.png" alt="RViz for Nav Screenshot" width="1000">
<img src="media/Screenshot 2026-11-18 134713.png" alt="RViz Screenshot With Path Planning" width="1000">

