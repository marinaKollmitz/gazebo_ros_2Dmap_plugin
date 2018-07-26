# gazebo_ros_2Dmap_plugin
Gazebo simulator plugin to automatically generate a 2D occupancy map from the simulated world at a given certain height. 

This plugin was adapted from the [octomap plugin](https://github.com/ethz-asl/rotors_simulator/tree/master/rotors_gazebo_plugins) from ETH Zürich.

## Usage 
Check out the plugin in your `catkin_ws` and build it with `catkin_make`.
To include the plugin, add the following line in between the `<world> </world>` tags of your Gazebo world file:

```
<plugin name='gazebo_occupancy_map' filename='libgazebo_2Dmap_plugin.so'/>
```

To generate the map, call the `/gazebo_2Dmap_plugin/generate_map` ros service:

```
rosservice call /gazebo_2Dmap_plugin/generate_map
```

The generated map is published on the `/map` ros topic. 

You can use the `map_saver` node from the `map_server` package inside ros navigation to save your generated map to a .pgm and .yaml file. To do so, start the node before calling the `/gazebo_2Dmap_plugin/generate_map` ros service:

```
rosrun map_server map_saver -f <mapname>
```
The map is saved once you call the map generation service.

## Hints

* To identify the connected free space the robot would discover during mapping, the plugin performs a wavefront exploration along the occupancy grid starting from the origin of the gazebo world coordinate system. Please ensure that the corresponding cell is in the continuous free space. 
* The plugin will map all objects in the world, including the robot. Remove all unwanted  objects before creating the map. 
