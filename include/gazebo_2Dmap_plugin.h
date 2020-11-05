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
#include <std_srvs/Empty.h>

namespace gazebo {

#if GAZEBO_MAJOR_VERSION >= 9
  typedef ignition::math::Vector3d vector3d;
  auto GetPhysicsPtr = std::mem_fn(&gazebo::physics::World::Physics);
#else
  typedef math::Vector3 vector3d;
  auto GetPhysicsPtr = std::mem_fn(&gazebo::physics::World::GetPhysicsEngine);
#endif

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

  bool worldCellIntersection(const vector3d& cell_center, const double cell_length,
                              gazebo::physics::RayShapePtr ray);
  

//  void FloodFill(const math::Vector3& seed_point,
//                 const math::Vector3& bounding_box_origin,
//                 const math::Vector3& bounding_box_lengths,
//                 const double leaf_size);
  
  /*! \brief
  */
  void CreateOccupancyMap();

  static void cell2world(unsigned int cell_x, unsigned int cell_y,
                         double map_size_x, double map_size_y, double map_resolution,
                         double& world_x, double &world_y);

  static void world2cell(double world_x, double world_y,
                         double map_size_x, double map_size_y, double map_resolution,
                         unsigned int& cell_x, unsigned int& cell_y);

  static bool cell2index(int cell_x, int cell_y,
                         unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& map_index);

  static bool index2cell(int index, unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& cell_x, unsigned int& cell_y);

 private:
  bool ServiceCallback(std_srvs::Empty::Request& req,
                       std_srvs::Empty::Response& res);

  physics::WorldPtr world_;
  ros::NodeHandle nh_;
  ros::ServiceServer map_service_;
  ros::Publisher map_pub_;
  nav_msgs::OccupancyGrid* occupancy_map_;
  std::string name_;
  double map_resolution_;
  double map_height_;
  double map_size_x_;
  double map_size_y_;
  double init_robot_x_;
  double init_robot_y_;
};

} // namespace gazebo

#endif  // GAZEBO_2DMAP_PLUGIN_H
