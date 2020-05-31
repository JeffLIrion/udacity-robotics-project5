# Home Service Robot

## Usage

```sh
./setup.sh  # Clone packages and build everything with `catkin_make`

./catkin_ws/src/scripts/home_service.sh
```

## Packages Utilized

### SLAM

Originally, I used the `gmapping` package for creating a map, as described by the instructions.  However, I found that `gmapping` did not produce an accurate map, so instead I used the `rtabmap` package, which I had used in Project 4.  I reused my `mapping.launch` file from Project 4, but I had to change the `frame_id` from `robot_footprint` to `base_footprint`.  (To figure out what the `frame_id` should be, I used the command `map_file`.)  I also needed to figure out how to save a map from `rtabmap` in the correct format (YAML + PGM).  To do this, I ran `rosrun map_server map_saver` and called the service `rosservice call /rtabmap/publish 1 1 0`.


### Localization

I used AMCL from the `turtlebot_navigation` package for localization.  I didn't find it necessary to adjust any of the default parameters.


### Navigation

I used the navigation stack from ROS for path planning.
