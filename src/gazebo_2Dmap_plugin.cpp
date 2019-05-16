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
    map_index = cell_y * cell_size_y + cell_x;
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
  cell_y = index / cell_size_y;
  cell_x = index % cell_size_x;

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
      }
    }
  }
  map_pub_.publish(*occupancy_map_);
  std::cout << "\rOccupied space generation completed                  " << std::endl;
}

void OccupancyMapFromWorld::GetCellNeighborCounts()
{

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
    uint8_t child_val;

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
