/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GAZEBO_2DMAP_PLUGIN_H
#define GAZEBO_2DMAP_PLUGIN_H

#include <iostream>
#include <math.h>

#include <common.h>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <octomap/octomap.h>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <std_srvs/Empty.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gazebo_ros_2Dmap_plugin/GenerateMap.h>
#include <boost/regex.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <stdio.h>

namespace gazebo {

/// \brief    Octomap plugin for Gazebo.
/// \details  This plugin is dependent on ROS, and is not built if NO_ROS=TRUE is provided to
///           CMakeLists.txt. The PX4/Firmware build does not build this file.
class OccupancyMapFromWorld : public WorldPlugin {
 public:
  OccupancyMapFromWorld()
      : WorldPlugin(), name_("gazebo_2Dmap_plugin")
  {
    ROS_INFO_NAMED(name_, "occupancy map plugin started");
  }
  virtual ~OccupancyMapFromWorld();

 protected:

  /// \brief Load the plugin.
  /// \param[in] _parent Pointer to the world that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

  bool worldCellIntersection(const double cell_center_x, const double cell_center_y,
                             const double min_z, const double max_z,
                             const double cell_length, gazebo::physics::RayShapePtr ray);

  void MarkOccupiedCells(nav_msgs::OccupancyGrid* map, double min_z, double max_z);

  void InflateOccupiedCells(nav_msgs::OccupancyGrid *map, double inflation_radius);

  void CropAtOccupied(nav_msgs::OccupancyGrid *map, bool draw_border,
                      int cell_padding);

  void MarkConnected(nav_msgs::OccupancyGrid* map,
                     int min_connected);

  nav_msgs::OccupancyGrid MapSpace(nav_msgs::OccupancyGrid* traversible_grid,
                                   double noise_stddev,
                                   bool visualize_rays, bool visualize_steps);

  void SimulateMapping(nav_msgs::OccupancyGrid* map,
                       double noise_stddev);

  static void cell2world(unsigned int cell_x, unsigned int cell_y, double map_resolution,
                         geometry_msgs::Point map_origin, double& world_x, double &world_y)
  {
    world_x = cell_x * map_resolution + map_resolution/2 + map_origin.x;
    world_y = cell_y * map_resolution + map_resolution/2 + map_origin.y;
  }

  static void world2cell(double world_x, double world_y, double map_resolution,
                         geometry_msgs::Point map_origin,
                         unsigned int& cell_x, unsigned int& cell_y)
  {
    cell_x = (world_x - map_origin.x) / map_resolution;
    cell_y = (world_y - map_origin.y) / map_resolution;
  }

  static bool cell2index(int cell_x, int cell_y,
                         unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& map_index)
  {
    if(cell_x >= 0 && cell_x < cell_size_x && cell_y >= 0 && cell_y < cell_size_y)
    {
      map_index = cell_y * cell_size_x + cell_x;
      return true;
    }
    else
    {
      //return false when outside map bounds
      return false;
    }
  }

  static bool index2cell(int index, unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& cell_x, unsigned int& cell_y)
  {
    cell_x = index % cell_size_x;
    cell_y = index / cell_size_x;

    if(cell_x >= 0 && cell_x < cell_size_x && cell_y >= 0 && cell_y < cell_size_y)
      return true;
    else
    {
      //return false when outside map bounds
      return false;
    }
  }

 private:
  bool OccServiceCallback(gazebo_ros_2Dmap_plugin::GenerateMap::Request& req,
                          gazebo_ros_2Dmap_plugin::GenerateMap::Response& res);
  bool ColServiceCallback(gazebo_ros_2Dmap_plugin::GenerateMap::Request& req,
                          gazebo_ros_2Dmap_plugin::GenerateMap::Response& res);

  physics::WorldPtr world_;
  ros::NodeHandle nh_;
  ros::ServiceServer occ_map_service_;
  ros::ServiceServer col_map_service_;
  ros::Publisher map_pub_;
  ros::Publisher map_pub2_;
  ros::Publisher marker_pub_;
  ros::Publisher scan_pub_;
  tf::TransformBroadcaster br_;
  std::string name_;
  visualization_msgs::Marker marker_;
  nav_msgs::OccupancyGrid map_viz_;

  const int8_t CellUnknown = -1;
  const int8_t CellFree = 0;
  const int8_t CellOccupied = 100;
};

} // namespace gazebo

#endif  // GAZEBO_2DMAP_PLUGIN_H
