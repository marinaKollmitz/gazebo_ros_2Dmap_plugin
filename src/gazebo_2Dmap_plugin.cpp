/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include "gazebo_2Dmap_plugin.h"

#include <gazebo/common/Time.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/math/Vector3.hh>

namespace gazebo {

OccupancyMapFromWorld::~OccupancyMapFromWorld() {}

void OccupancyMapFromWorld::Load(physics::WorldPtr _parent,
                                 sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  world_ = _parent;

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  occ_map_service_ = nh_.advertiseService(
        "gazebo_2Dmap_plugin/generate_map", &OccupancyMapFromWorld::OccServiceCallback, this);

  col_map_service_ = nh_.advertiseService(
        "gazebo_2Dmap_plugin/generate_collision_map", &OccupancyMapFromWorld::ColServiceCallback,
        this);
}

bool OccupancyMapFromWorld::OccServiceCallback(gazebo_ros_2Dmap_plugin::GenerateMap::Request& req,
                                               gazebo_ros_2Dmap_plugin::GenerateMap::Response& res)
{
  std::cout << "generate occupancy map service call received" << std::endl;
  if(req.size_z != 0.0)
  {
    ROS_WARN("non-zero map size_z accountered in occupancy service callback, "
             "will be ignored. Did you want to generate a collision map instead?");
  }

  ros::Time now = ros::Time::now();

  uint32_t cells_size_x = req.size_x / req.resolution;
  uint32_t cells_size_y = req.size_y / req.resolution;

  nav_msgs::OccupancyGrid* occupancy_map = new nav_msgs::OccupancyGrid();
  occupancy_map->data.resize(cells_size_x * cells_size_y);

  //all cells are initially unknown
  std::fill(occupancy_map->data.begin(), occupancy_map->data.end(), CellUnknown);
  occupancy_map->header.stamp = ros::Time::now();
  occupancy_map->header.frame_id = "odom"; //TODO map frame
  occupancy_map->info.map_load_time = ros::Time(0);
  occupancy_map->info.resolution = req.resolution;
  occupancy_map->info.width = cells_size_x;
  occupancy_map->info.height = cells_size_y;
  occupancy_map->info.origin = req.origin;

  MarkOccupiedCells(occupancy_map, occupancy_map->info.origin.position.z,
                    occupancy_map->info.origin.position.z);
  GetFreeSpace(occupancy_map);
  FilterOccupied(occupancy_map);

  occupancy_map->info.origin.position.x -= req.size_x/2;
  occupancy_map->info.origin.position.y -= req.size_y/2;

  res.map = *occupancy_map;
  res.success = true;

  map_pub_.publish(*occupancy_map);

  ros::Duration dur = ros::Time::now() - now;
  std::cout << "map generation took " << dur.toSec() << " seconds" << std::endl;

  return true;
}

bool OccupancyMapFromWorld::ColServiceCallback(gazebo_ros_2Dmap_plugin::GenerateMap::Request& req,
                                               gazebo_ros_2Dmap_plugin::GenerateMap::Response& res)
{
  std::cout << "generate collision map service call received" << std::endl;
  if(req.size_z == 0.0)
  {
    ROS_WARN("zero map size_z accountered in collision map service callback, "
             "Did you want to generate an occupancy map instead?");
  }

  ros::Time now = ros::Time::now();

  uint32_t cells_size_x = req.size_x / req.resolution;
  uint32_t cells_size_y = req.size_y / req.resolution;

  nav_msgs::OccupancyGrid* occupancy_map = new nav_msgs::OccupancyGrid();
  occupancy_map->data.resize(cells_size_x * cells_size_y);

  //all cells are initially free
  std::fill(occupancy_map->data.begin(), occupancy_map->data.end(), CellFree);
  occupancy_map->header.stamp = ros::Time::now();
  occupancy_map->header.frame_id = "odom"; //TODO map frame
  occupancy_map->info.map_load_time = ros::Time(0);
  occupancy_map->info.resolution = req.resolution;
  occupancy_map->info.width = cells_size_x;
  occupancy_map->info.height = cells_size_y;
  occupancy_map->info.origin = req.origin;

  MarkOccupiedCells(occupancy_map, occupancy_map->info.origin.position.z - req.size_z/2,
                    occupancy_map->info.origin.position.z + req.size_z/2);

  occupancy_map->info.origin.position.x -= req.size_x/2;
  occupancy_map->info.origin.position.y -= req.size_y/2;

  res.map = *occupancy_map;
  res.success = true;

  map_pub_.publish(*occupancy_map);

  ros::Duration dur = ros::Time::now() - now;
  std::cout << "map generation took " << dur.toSec() << " seconds" << std::endl;

  return true;
}

void OccupancyMapFromWorld::MarkOccupiedCells(nav_msgs::OccupancyGrid *map,
                                              double min_z, double max_z)
{
  gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  uint32_t cells_size_x = map->info.width;
  uint32_t cells_size_y = map->info.height;
  double map_resolution = map->info.resolution;
  double map_size_x = map->info.width * map->info.resolution;
  double map_size_y = map->info.height * map->info.resolution;

  for(uint32_t cell_x=0; cell_x<cells_size_x; cell_x++)
  {
    for(uint32_t cell_y=0; cell_y<cells_size_y; cell_y++)
    {
      double world_x, world_y;
      //TODO cell2world muss map origin beachten?
      cell2world(cell_x, cell_y, map_size_x, map_size_y, map_resolution,
                 map->info.origin.position, world_x, world_y);

      std::string collision_entity;
      bool cell_occupied = worldCellIntersection(world_x, world_y, min_z, max_z,
                                                 map_resolution, ray, collision_entity);

      if(cell_occupied)
      {
        unsigned int cell_index;
        cell2index(cell_x, cell_y, cells_size_x, cells_size_y, cell_index);
        //mark cell as occupied
        map->data.at(cell_index) = CellOccupied;
      }
    }
  }

  std::cout << "occupied space marked" << std::endl;

  //Reset call necessary to avoid gazebo segmentation fault
  engine->Reset();

  return;
}

bool OccupancyMapFromWorld::worldCellIntersection(const double cell_center_x,
                                                  const double cell_center_y,
                                                  const double min_z, const double max_z,
                                                  const double cell_length,
                                                  gazebo::physics::RayShapePtr ray,
                                                  std::string &entity_name)
{
  double dist;

  int cell_length_steps = 5;
  double side_length;

  //check for collisions with beams at increasing sizes to capture smaller
  //objects inside the cell
  for(int step=1; step<=cell_length_steps; step++)
  {
    side_length = cell_length / cell_length_steps * step;

    for(int i=-1; i<=1; i+=2)
    {
      for(int j=-1; j<=1; j+=2)
      {
        double start_x, start_y, end_x, end_y;

        if(max_z == min_z)
        {
          //for collision in one layer, check rays surrounding the cell
          //   + - +
          //   |   |
          //   + - +
          start_x = cell_center_x + i * side_length/2;
          start_y = cell_center_y - i * side_length/2;
          end_x = cell_center_x + j * side_length/2;
          end_y = cell_center_y + j * side_length/2;
        }
        else
        {
          //for collisions at different heights, check rays going up in z-direction
          //      |    |
          //    | |  | |
          //    | + -| +
          //    |/   |/
          //    + -- +
          start_x = cell_center_x + i * side_length/2;
          start_y = cell_center_y + j * side_length/2;
          end_x = cell_center_x - j * side_length/2;
          end_y = cell_center_y - j * side_length/2;
        }

//        std::cout << "i: " << i << " j: " << j << std::endl;
//        std::cout << "start: " << start_x << " " << start_y << std::endl;
//        std::cout << "end: " << end_x << " " << end_y << std::endl;

        ray->SetPoints(math::Vector3(start_x, start_y, max_z),
                       math::Vector3(end_x, end_y, min_z));
        ray->GetIntersection(dist, entity_name);

//        if(!entity_name.empty())
//          std::cout << "entity name: " << entity_name << std::endl;

        if(IsFloor(entity_name))
          std::cout << "hit the floor" << std::endl;

        if(!entity_name.empty() && !IsFloor(entity_name) && !IsCeiling(entity_name))
          return true;
      }
    }
  }

  return false;
}

void OccupancyMapFromWorld::GetFreeSpace(nav_msgs::OccupancyGrid* map)
{
  std::cout << "marking free space " << std::endl;

  unsigned int cells_size_x = map->info.width;
  unsigned int cells_size_y = map->info.height;

  int8_t visited = 50; //value for marking visited cells
  int8_t unknown_placeholder = 200; //placeholder value for marking unknown cells
  int8_t free_space = 0;
  int8_t unknown_space = -1;

  for(int cell_x=0; cell_x<cells_size_x; cell_x++)
  {
    for(int cell_y=0; cell_y<cells_size_y; cell_y++)
    {
      unsigned int cell_index;
      cell2index(cell_x, cell_y, cells_size_x, cells_size_y, cell_index);

      //compute number of connected cells if cell value is unknown
      if(map->data.at(cell_index) == -1)
      {
        //wavefront expansion to count number of cells
        std::vector<unsigned int> wavefront;
        std::vector<unsigned int> connected_list;

        wavefront.push_back(cell_index);
        connected_list.push_back(cell_index);
        map->data.at(cell_index) = visited;

        while(!wavefront.empty())
        {
          cell_index = wavefront.at(0);
          wavefront.erase(wavefront.begin());

          //explore cells neighbors in an 8-connected grid
          unsigned int child_index;
          int child_val;

          unsigned int wavefront_cell_x, wavefront_cell_y;
          index2cell(cell_index, cells_size_x, cells_size_y, wavefront_cell_x, wavefront_cell_y);

          //8-connected grid
          for(int i=-1; i<2; i++)
          {
            for(int j=-1; j<2; j++)
            {
              //makes sure index is inside map bounds
              if(cell2index(wavefront_cell_x + i, wavefront_cell_y + j, cells_size_x, cells_size_y, child_index))
              {
                child_val = map->data.at(child_index);

                //only count cell as connected if it is unknown (not occupied and not already expanded)
                if(child_val == -1)
                {
                  //add to connected list
                  connected_list.push_back(child_index);

                  //mark cell as visited
                  map->data.at(child_index) = visited;

                  //add cell to wavefront
                  wavefront.push_back(child_index);
                }
              }
            }
          }
        }//end wavefront loop

        //count connected cells
        int connected_count = connected_list.size();

        int min_connected = 1000;

        //fill all cells with the cell count value
        for(int k=0; k<connected_list.size(); k++)
        {
          if(connected_count > min_connected)
            map->data.at(connected_list.at(k)) = free_space; //free space
          else
            map->data.at(connected_list.at(k)) = unknown_placeholder; //unknown space
        }
      }
    }
  }
  for(int cell_x=0; cell_x<cells_size_x; cell_x++)
  {
    for(int cell_y=0; cell_y<cells_size_y; cell_y++)
    {
      unsigned int cell_index;
      cell2index(cell_x, cell_y, cells_size_x, cells_size_y, cell_index);

      //compute number of connected cells if cell value is unknown
      if(map->data.at(cell_index) == unknown_placeholder)
        map->data.at(cell_index) = unknown_space;
    }
  }
}

void OccupancyMapFromWorld::FilterOccupied(nav_msgs::OccupancyGrid* map)
{
  unsigned int cells_size_x = map->info.width;
  unsigned int cells_size_y = map->info.height;

  int8_t free_space = 0;
  int8_t unknown_space = -1;
  int8_t occupied = 100;

  for(int cell_x=0; cell_x<cells_size_x; cell_x++)
  {
    for(int cell_y=0; cell_y<cells_size_y; cell_y++)
    {
      unsigned int cell_index;
      unsigned int neighbor_index;
      int8_t neighbor_val;
      cell2index(cell_x, cell_y, cells_size_x, cells_size_y, cell_index);

      if(map->data.at(cell_index) == occupied)
      {
        map->data.at(cell_index) = unknown_space;
        //check neighbors for free space
        //8-connected grid
        for(int i=-1; i<2; i++)
        {
          for(int j=-1; j<2; j++)
          {
            //makes sure index is inside map bounds
            if(cell2index(cell_x + i, cell_y + j, cells_size_x, cells_size_y, neighbor_index))
            {
              neighbor_val = map->data.at(neighbor_index);

              //set cell to occupied if at least 1 neighbor is free space
              if(neighbor_val == free_space)
              {
                map->data.at(cell_index) = occupied;
                break;
              }
            }
          }
        }
      }
    }
  }
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OccupancyMapFromWorld)

}  // namespace gazebo
