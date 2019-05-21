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
  map_service_ = nh_.advertiseService(
        "gazebo_2Dmap_plugin/generate_map", &OccupancyMapFromWorld::ServiceCallback, this);

  map_resolution_ = 0.1;

  if(_sdf->HasElement("map_resolution"))
    map_resolution_ = _sdf->GetElement("map_resolution")->Get<double>();

  map_height_ = 0.3;

  if(_sdf->HasElement("map_z"))
    map_height_ = _sdf->GetElement("map_z")->Get<double>();

  map_size_x_ = 10.0;

  if(_sdf->HasElement("map_size_x"))
    map_size_x_ = _sdf->GetElement("map_size_x")->Get<double>();

  map_size_y_ = 10.0;

  if(_sdf->HasElement("map_size_y"))
    map_size_y_ = _sdf->GetElement("map_size_y")->Get<double>();
}

bool OccupancyMapFromWorld::ServiceCallback(std_srvs::Empty::Request& req,
                                            std_srvs::Empty::Response& res)
{
  //CreateOccupancyMap();
  CreateOccupiedSpace();
  return true;
}

bool OccupancyMapFromWorld::worldCellIntersection(const math::Vector3& cell_center,
                                                  const double cell_length,
                                                  gazebo::physics::RayShapePtr ray,
                                                  std::string &entity_name)
{
  //check for collisions with rays surrounding the cell
  //    ---
  //   | + |
  //    ---

  double dist;

  int cell_length_steps = 10;
  double side_length;

  //check for collisions with beams at increasing sizes to capture smaller
  //objects inside the cell
  for(int step=1; step<=cell_length_steps; step++)
  {
    side_length = cell_length / cell_length_steps * step;

    for(int i=-1; i<2; i+=2)
    {
      double start_x = cell_center.x + i * side_length/2;
      double start_y = cell_center.y - i * side_length/2;

      for(int j=-1; j<2; j+=2)
      {
        double end_x = cell_center.x + j * side_length/2;
        double end_y = cell_center.y + j * side_length/2;

        //      std::cout << "start_x" << start_x << std::endl;
        //      std::cout << "start_y" << start_y << std::endl;
        //      std::cout << "end_x" << end_x << std::endl;
        //      std::cout << "end_y" << end_y << std::endl;

        ray->SetPoints(math::Vector3(start_x, start_y, cell_center.z),
                       math::Vector3(end_x, end_y, cell_center.z));
        ray->GetIntersection(dist, entity_name);

        if(!entity_name.empty())
          return true;
      }
    }
  }

  return false;
}

void OccupancyMapFromWorld::cell2world(unsigned int cell_x, unsigned int cell_y,
                                       double map_size_x, double map_size_y,
                                       double map_resolution,
                                       double& world_x, double &world_y)
{
  world_x = cell_x * map_resolution - map_size_x/2 + map_resolution/2;
  world_y = cell_y * map_resolution - map_size_y/2 + map_resolution/2;
}

void OccupancyMapFromWorld::world2cell(double world_x, double world_y,
                                       double map_size_x, double map_size_y,
                                       double map_resolution,
                                       unsigned int& cell_x, unsigned int& cell_y)
{
  cell_x = (world_x + map_size_x/2) / map_resolution;
  cell_y = (world_y + map_size_y/2) / map_resolution;
}

bool OccupancyMapFromWorld::cell2index(int cell_x, int cell_y,
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

bool OccupancyMapFromWorld::index2cell(int index, unsigned int cell_size_x,
                                       unsigned int cell_size_y,
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

void OccupancyMapFromWorld::CreateOccupiedSpace()
{
  unsigned int cells_size_x = map_size_x_ / map_resolution_;
  unsigned int cells_size_y = map_size_y_ / map_resolution_;
  math::Vector3 map_origin(0,0,map_height_);

  occupancy_map_ = new nav_msgs::OccupancyGrid();
  occupancy_map_->data.resize(cells_size_x * cells_size_y);
  //all cells are initially unknown
  std::fill(occupancy_map_->data.begin(), occupancy_map_->data.end(), -1);
  occupancy_map_->header.stamp = ros::Time::now();
  occupancy_map_->header.frame_id = "odom"; //TODO map frame
  occupancy_map_->info.map_load_time = ros::Time(0);
  occupancy_map_->info.resolution = map_resolution_;
  occupancy_map_->info.width = cells_size_x;
  occupancy_map_->info.height = cells_size_y;
  occupancy_map_->info.origin.position.x = map_origin.x - map_size_x_ / 2;
  occupancy_map_->info.origin.position.y = map_origin.y - map_size_y_ / 2;
  occupancy_map_->info.origin.position.z = map_origin.z;
  occupancy_map_->info.origin.orientation.w = 1;

  gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  int min_cell_x = cells_size_x, min_cell_y = cells_size_y;
  int max_cell_x = 0, max_cell_y = 0;

  for(int cell_x=0; cell_x<cells_size_x; cell_x++)
  {
    for(int cell_y=0; cell_y<cells_size_y; cell_y++)
    {
      double world_x, world_y;
      cell2world(cell_x, cell_y, map_size_x_, map_size_y_, map_resolution_,
                 world_x, world_y);

      std::string collision_entity;
      bool cell_occupied = worldCellIntersection(math::Vector3(world_x, world_y, map_height_),
                                                 map_resolution_, ray, collision_entity);

      if(cell_occupied)
      {
        unsigned int cell_index;
        cell2index(cell_x, cell_y, cells_size_x, cells_size_y, cell_index);
        //mark cell as occupied
        occupancy_map_->data.at(cell_index) = 100;
        //check if ray collides with a wall to get map boundaries
        if(collision_entity.find("fr") != std::string::npos)
        {
          if(cell_x < min_cell_x)
            min_cell_x = cell_x;
          if(cell_y < min_cell_y)
            min_cell_y = cell_y;
          if(cell_x > max_cell_x)
            max_cell_x = cell_x;
          if(cell_y > max_cell_y)
            max_cell_y = cell_y;
        }
      }
    }
  }

//  std::cout << "occupied cells generated " << std::endl;
//  map_pub_.publish(*occupancy_map_);
//  sleep(10);



  int map_padding = 10;
  min_cell_x = std::max(min_cell_x-map_padding, 0);
  min_cell_y = std::max(min_cell_y-map_padding, 0);
  max_cell_x = std::min(max_cell_x+map_padding, (int)cells_size_x-1);
  max_cell_y = std::min(max_cell_y+map_padding, (int)cells_size_y-1);

  std::cout << "min: " << min_cell_x << " " << min_cell_y << std::endl;
  std::cout << "max: " << max_cell_x << " " << max_cell_y << std::endl;

//  unsigned int cell_index;
//  cell2index(max_cell_x, max_cell_y, cells_size_x, cells_size_y, cell_index);
//  occupancy_map_->data.at(cell_index) = 200;

//  map_pub_.publish(*occupancy_map_);
//  std::cout << "generated occupied space" << std::endl;
//  sleep(5);

  //crop occupancy map to wall bounds
  nav_msgs::OccupancyGrid* cropped_occupancy_map = new nav_msgs::OccupancyGrid();
  cropped_occupancy_map->header = occupancy_map_->header;
  cropped_occupancy_map->info = occupancy_map_->info;
  cropped_occupancy_map->info.width = max_cell_x - min_cell_x + 1;
  cropped_occupancy_map->info.height = max_cell_y - min_cell_y + 1;
  cropped_occupancy_map->info.origin.position.x =
      occupancy_map_->info.origin.position.x + min_cell_x*map_resolution_;
  cropped_occupancy_map->info.origin.position.y =
      occupancy_map_->info.origin.position.y + min_cell_y*map_resolution_;

  unsigned int cell_x, cell_y;
  for(int cell_index=0; cell_index<occupancy_map_->data.size(); cell_index++)
  {
    index2cell(cell_index, cells_size_x, cells_size_y, cell_x, cell_y);
    if(cell_x >= min_cell_x && cell_x <= max_cell_x &&
       cell_y >= min_cell_y && cell_y <= max_cell_y)
    {
      cropped_occupancy_map->data.push_back(occupancy_map_->data.at(cell_index));
    }
  }

//  std::cout << "\r occupied cells generation completed " << std::endl;
//  map_pub_.publish(*occupancy_map_);
//  sleep(10);

  occupancy_map_ = cropped_occupancy_map;
  std::cout << "\r cropped map generation completed " << std::endl;

  unsigned int cell_index;
  cell2index(0, 10, occupancy_map_->info.width, occupancy_map_->info.height, cell_index);
  occupancy_map_->data.at(cell_index) = 200;

  map_pub_.publish(*occupancy_map_);
  sleep(20);

  GetCellNeighborCounts(occupancy_map_);

  std::cout << "\rGet Cell Neighbor Counts completed                  " << std::endl;

  map_pub_.publish(*occupancy_map_);
  sleep(20);

  FilterOccupied(occupancy_map_);
  std::cout << "\rOccupied space filtering completed                  " << std::endl;

  map_pub_.publish(*occupancy_map_);
  sleep(2);

}

void OccupancyMapFromWorld::GetCellNeighborCounts(nav_msgs::OccupancyGrid* map)
{
  unsigned int cells_size_x = map->info.width;
  unsigned int cells_size_y = map->info.height;

  std::cout << "cells_size_x: " << cells_size_x << std::endl;
  std::cout << "cells_size_y: " << cells_size_y << std::endl;

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

          map_pub_.publish(*map);
//          ros::Duration(0.2).sleep();

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

void OccupancyMapFromWorld::CreateOccupancyMap()
{
  //TODO map origin different from (0,0)
  math::Vector3 map_origin(0,0,map_height_);

  unsigned int cells_size_x = map_size_x_ / map_resolution_;
  unsigned int cells_size_y = map_size_y_ / map_resolution_;

  occupancy_map_ = new nav_msgs::OccupancyGrid();
  occupancy_map_->data.resize(cells_size_x * cells_size_y);
  //all cells are initially unknown
  std::fill(occupancy_map_->data.begin(), occupancy_map_->data.end(), -1);
  occupancy_map_->header.stamp = ros::Time::now();
  occupancy_map_->header.frame_id = "odom"; //TODO map frame
  occupancy_map_->info.map_load_time = ros::Time(0);
  occupancy_map_->info.resolution = map_resolution_;
  occupancy_map_->info.width = cells_size_x;
  occupancy_map_->info.height = cells_size_y;
  occupancy_map_->info.origin.position.x = map_origin.x - map_size_x_ / 2;
  occupancy_map_->info.origin.position.y = map_origin.y - map_size_y_ / 2;
  occupancy_map_->info.origin.position.z = map_origin.z;
  occupancy_map_->info.origin.orientation.w = 1;

  gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  std::cout << "Starting wavefront expansion for mapping" << std::endl;

  //identify free space by spreading out from initial robot cell
  double robot_x = 0;
  double robot_y = 0;

  //find initial robot cell
  unsigned int cell_x, cell_y, map_index;
  world2cell(robot_x, robot_y, map_size_x_, map_size_y_, map_resolution_,
             cell_x, cell_y);

  if(!cell2index(cell_x, cell_y, cells_size_x, cells_size_y, map_index))
  {
    ROS_ERROR_NAMED(name_, "initial robot pos is outside map, could not create "
                           "map");
    return;
  }

  std::vector<unsigned int> wavefront;
  wavefront.push_back(map_index);

  //wavefront expansion for identifying free, unknown and occupied cells
  while(!wavefront.empty())
  {
    map_index = wavefront.at(0);
    wavefront.erase(wavefront.begin());

    index2cell(map_index, cells_size_x, cells_size_y, cell_x, cell_y);

    //mark cell as free
    occupancy_map_->data.at(map_index) = 0;

    //explore cells neighbors in an 8-connected grid
    unsigned int child_index;
    double world_x, world_y;
    int8_t child_val;

    //8-connected grid
    for(int i=-1; i<2; i++)
    {
      for(int j=-1; j<2; j++)
      {
        //makes sure index is inside map bounds
        if(cell2index(cell_x + i, cell_y + j, cells_size_x, cells_size_y, child_index))
        {
          child_val = occupancy_map_->data.at(child_index);

          //only update value if cell is unknown
          if(child_val != 100 && child_val != 0 && child_val != 50)
          {
            cell2world(cell_x + i, cell_y + j, map_size_x_, map_size_y_, map_resolution_,
                       world_x, world_y);

            std::string collision_entity;
            bool cell_occupied = worldCellIntersection(math::Vector3(world_x, world_y, map_height_),
                                                       map_resolution_, ray, collision_entity);

            if(cell_occupied)
              //mark cell as occupied
              occupancy_map_->data.at(child_index) = 100;


            else
            {
              //add cell to wavefront
              wavefront.push_back(child_index);
              //mark wavefront in map so we don't add children to wavefront multiple
              //times
              occupancy_map_->data.at(child_index) = 50;
            }
          }
        }
      }
    }
  }

  map_pub_.publish(*occupancy_map_);
  std::cout << "\rOccupancy Map generation completed                  " << std::endl;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OccupancyMapFromWorld)

}  // namespace gazebo
