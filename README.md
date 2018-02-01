# Generating Optimal Highway Trajectories for Autonomous Cars
<figure>
  <img src=""/>
</figure>
 <p></p>
 
### Overview
In previous repositories, I've experimented with different types of controllers to make a car follow an ideal trajectory. I've also used a particle filter and types of Kalman filters with data from various sensors (LIDAR, RADAR, GPS) to track vehicles' location and movement. In this repository, I will address the problem of trajectory generation. Given a map of a highway, an accurate controller, and accurate sensor readings of the environment, how to generate optimal trajectories for the car so that it navigates the highway safely and legally as well as efficiently and smoothly to maximize rider experience? I find this problem exciting because solving it seems to make the car autonomous in a way that cruise control, or even automatic parallel parking, does not. Not to mention, navigating the road with other upredictable drivers, if only in simulation, raises the stakes and highlights the need for perfection in developing the car's software. 
 
The contents of this repository include:
```
root
|   README.md
|   CMakeLists.txt
|   cmakepatch.txt
|   install-mac.sh
|   install-ubuntu.sh
|   project_assignment.md
|
|___data
|   |   highway_map.csv
|
|___src
    |   main.cpp
    |   path_panner.cpp
    |   path_panner.h
    |   cost.h
    |   helper_functions.h
    |   spline.h
    |   json.hpp
```

###
