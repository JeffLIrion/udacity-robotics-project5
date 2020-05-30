# Home Service Robot

## SLAM

Originally, I used the `gmapping` package for creating a map, as described by the instructions.  However, I found that `gmapping` did not produce an accurate map, so instead I used the `rtabmap` package, which I had used in Project 4.  I reused my `mapping.launch` file from Project 4, but I had to change the `frame_id` from `robot_footprint` to `base_footprint`.  (To figure out what the `frame_id` should be, I used the command `map_file`.)  I also needed to figure out how to save a map from `rtabmap` in the correct format (YAML + PGM).  To do this, I ran `rosrun map_server map_saver` and called the service `rosservice call /rtabmap/publish 1 1 0`.


## Navigation


