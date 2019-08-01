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
  map_pub2_ = nh_.advertise<nav_msgs::OccupancyGrid>("map2", 1);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("viz_marker", 1);
  scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan",1);
  occ_map_service_ = nh_.advertiseService(
        "gazebo_2Dmap_plugin/generate_map", &OccupancyMapFromWorld::OccServiceCallback, this);

  col_map_service_ = nh_.advertiseService(
        "gazebo_2Dmap_plugin/generate_collision_map", &OccupancyMapFromWorld::ColServiceCallback,
        this);

  std::cout << "occupancy map plugin started" << std::endl;
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
  occupancy_map->info.origin.position.x = -req.size_x/2;
  occupancy_map->info.origin.position.y = -req.size_y/2;

  //TODO add info to the request
  double robot_radius = 0.2;
  double robot_height = 1.0;

  //mark cells the robot center cannot reach
  MarkOccupiedCells(occupancy_map, occupancy_map->info.origin.position.z,
                    occupancy_map->info.origin.position.z+robot_height);

  CropAtOccupied(occupancy_map, true, 10);

  //std::cout << "inflating occupied cells" << std::endl;
  InflateOccupiedCells(occupancy_map, robot_radius);

  //min room size is 2x2 meters
  int min_room_cells = pow(2.0/occupancy_map->info.resolution,2);
  MarkConnected(occupancy_map, min_room_cells);

  std::cout << "simlating mapping" << std::endl;
//  SimulateMapping(occupancy_map, 0.01);
  MapSpace(occupancy_map, 0.01);

  res.map = *occupancy_map;
  res.success = true;

//  map_pub_.publish(*occupancy_map);
//  marker_pub_.publish(marker_);

//  while(ros::ok())
//  {
//    marker_pub_.publish(marker_);
//    ros::Rate(10).sleep();
//  }

//  double center_x, center_y;
//  cell2world(0, 0, occupancy_map->info.resolution,
//             occupancy_map->info.origin.position,
//             center_x, center_y);

//  visualization_msgs::Marker cell_position_marker;
//  cell_position_marker.type = visualization_msgs::Marker::POINTS;
//  cell_position_marker.header.frame_id = "odom";
//  cell_position_marker.header.stamp = ros::Time::now();
//  cell_position_marker.pose.orientation.w = 1;
//  cell_position_marker.color.a = 1;
//  geometry_msgs::Point point;
//  point.x = center_x;
//  point.y = center_y;
//  cell_position_marker.points.push_back(point);
//  cell_position_marker.scale.x = 0.05;
//  cell_position_marker.scale.y = 0.05;

//  while(ros::ok())
//  {
//    marker_pub_.publish(cell_position_marker);
//    ros::Rate(10).sleep();
//  }

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

//  map_pub_.publish(*occupancy_map);

  ros::Duration dur = ros::Time::now() - now;
  std::cout << "map generation took " << dur.toSec() << " seconds" << std::endl;

  return true;
}

void OccupancyMapFromWorld::CropAtOccupied(nav_msgs::OccupancyGrid *map,
                                           bool draw_border, int cell_padding)
{
  uint32_t cells_size_x = map->info.width;
  uint32_t cells_size_y = map->info.height;

  //find the map borders
  int min_cell_x = cells_size_x, min_cell_y = cells_size_y;
  int max_cell_x = 0, max_cell_y = 0;

  for(int cell_x=0; cell_x<cells_size_x; cell_x++)
    {
      for(int cell_y=0; cell_y<cells_size_y; cell_y++)
      {
        unsigned int cell_index;
        cell2index(cell_x, cell_y, cells_size_x, cells_size_y, cell_index);

        if(map->data.at(cell_index) == CellOccupied)
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

    int pad_min_x = std::max(min_cell_x-cell_padding, 0);
    int pad_min_y = std::max(min_cell_y-cell_padding, 0);
    int pad_max_x = std::min(max_cell_x+cell_padding, (int)cells_size_x-1);
    int pad_max_y = std::min(max_cell_y+cell_padding, (int)cells_size_y-1);

    //transfer data to cropped vector
    std::vector<int8_t> cropped_data;

    unsigned int cell_x, cell_y;
    for(int cell_index=0; cell_index<map->data.size(); cell_index++)
    {
      index2cell(cell_index, cells_size_x, cells_size_y, cell_x, cell_y);
      if(cell_x >= pad_min_x && cell_x <= pad_max_x &&
         cell_y >= pad_min_y && cell_y <= pad_max_y)
      {
        if(draw_border)
        {
          //if cell is a border cell, mark it as occupied
          if(cell_x == min_cell_x || cell_x == max_cell_x || cell_y == min_cell_y || cell_y == max_cell_y)
            map->data.at(cell_index) = CellOccupied;
        }
        cropped_data.push_back(map->data.at(cell_index));
      }
    }

    map->data = cropped_data;
    map->info.width = pad_max_x - pad_min_x + 1;
    map->info.height = pad_max_y - pad_min_y + 1;
    map->info.origin.position.x =
        map->info.origin.position.x + pad_min_x*map->info.resolution;
    map->info.origin.position.y =
        map->info.origin.position.y + pad_min_y*map->info.resolution;
}

void OccupancyMapFromWorld::InflateOccupiedCells(nav_msgs::OccupancyGrid *map,
                                                 double inflation_radius)
{
  int num_iterations = round(inflation_radius / map->info.resolution);
  //std::cout << "performing " << num_iterations << " iterations" << std::endl;
  uint32_t cells_size_x = map->info.width;
  uint32_t cells_size_y = map->info.height;

  nav_msgs::OccupancyGrid* inflated_map = new nav_msgs::OccupancyGrid;
  inflated_map->info = map->info;
  inflated_map->header = map->header;
  inflated_map->data.resize(cells_size_x * cells_size_y);
  std::fill(inflated_map->data.begin(), inflated_map->data.end(), CellUnknown);

  for(int it=0; it<num_iterations; it++)
  {
    //iterate map cells. for each cell, mark 8 neighbors as occupied
    for(uint32_t cell_x=0; cell_x<cells_size_x; cell_x++)
    {
      for(uint32_t cell_y=0; cell_y<cells_size_y; cell_y++)
      {
        unsigned int cell_index;
        cell2index(cell_x, cell_y, cells_size_x, cells_size_y, cell_index);

        //check if cell is occupied
        if(map->data.at(cell_index) == CellOccupied)
        {
          for(int inc_x=-1; inc_x<=1; inc_x++)
          {
            for(int inc_y=-1; inc_y<=1; inc_y++)
            {
              int neighbor_x = cell_x + inc_x;
              int neighbor_y = cell_y + inc_y;
              if(cell2index(neighbor_x, neighbor_y, cells_size_x, cells_size_y, cell_index));
                inflated_map->data.at(cell_index) = CellOccupied;
            }
          }
        }
      }
    }
    map->data = inflated_map->data;
  }
  //map->data = inflated_map->data;
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

  for(uint32_t cell_x=0; cell_x<cells_size_x; cell_x++)
  {
    for(uint32_t cell_y=0; cell_y<cells_size_y; cell_y++)
    {
      double world_x, world_y;

      cell2world(cell_x, cell_y, map_resolution,
                 map->info.origin.position, world_x, world_y);

      bool cell_occupied = worldCellIntersection(world_x, world_y, min_z, max_z,
                                                 map_resolution, ray);

      if(cell_occupied)
      {
        unsigned int cell_index;
        if(cell2index(cell_x, cell_y, cells_size_x, cells_size_y, cell_index))
        {
          //mark cell as occupied
          map->data.at(cell_index) = CellOccupied;
        }
        else
          std::cout << "index outside map bounds! " << std::endl;
      }
    }
  }

  std::cout << "occupied space marked" << std::endl;

  //Reset call necessary to avoid gazebo segmentation fault
  engine->Reset();

  return;
}

// this works with gmappin: rosrun gmapping slam_gmapping
// requires to publish static trafo between base_link and laser frames:
// rosrun tf static_transform_publisher 0 0 0 0 0 0 baslink laser 10
void OccupancyMapFromWorld::SimulateMapping(nav_msgs::OccupancyGrid* map,
                                            double noise_stddev)
{
  gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  uint32_t cells_size_x = map->info.width;
  uint32_t cells_size_y = map->info.height;
  double map_resolution = map->info.resolution;
  double map_z = map->info.origin.position.z;

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0,noise_stddev);

  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser";
  int num_rays = 360;
  double angle_incr = 2*M_PI/num_rays;
  double max_range = 10;
  scan.angle_min = 0;
  scan.angle_max = (num_rays-1)*angle_incr;
  scan.angle_increment = angle_incr;
  scan.range_max = max_range;
  scan.range_min = 0.0;
  tf::Transform transform;

  for(uint32_t cell_x=0; cell_x<cells_size_x; cell_x++)
  {
    for(uint32_t cell_y=0; cell_y<cells_size_y; cell_y++)
    {
      unsigned int cell_index;
      cell2index(cell_x, cell_y, cells_size_x, cells_size_y, cell_index);

      //check if cell is reachable by robot
      if(map->data.at(cell_index) == CellFree)
      {
        double center_x, center_y;
        //start ray array from cell center
        cell2world(cell_x, cell_y, map_resolution,
                   map->info.origin.position, center_x, center_y);

        scan.ranges.clear();

        double ray_angle = scan.angle_min;
        while(ray_angle <= scan.angle_max)
        {
          double end_x = center_x + max_range * cos(ray_angle);
          double end_y = center_y + max_range * sin(ray_angle);

          double dist;
          std::string entity_name;

          ray->SetPoints(math::Vector3(center_x, center_y, map_z),
                         math::Vector3(end_x, end_y, map_z));
          ray->GetIntersection(dist, entity_name);

          //add noise to measured distance
          dist = dist + distribution(generator);
          scan.ranges.push_back(dist);

          ray_angle += angle_incr;
        }
        scan.header.stamp = ros::Time::now();
        transform.setOrigin(tf::Vector3(center_x, center_y, map_z));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br_.sendTransform(tf::StampedTransform(transform, scan.header.stamp, "odom", "base_link"));
        scan_pub_.publish(scan);
        ros::Duration(0.05).sleep();
      }
    }
  }
}

std::vector<double> CountingModel(std::vector<int64_t> hits_map,
                                  std::vector<int64_t> misses_map)
{
  std::vector<double> occ_map;
  occ_map.resize(hits_map.size());

  for(int i=0; i<hits_map.size(); i++)
  {
    int hits = hits_map.at(i);
    int misses = misses_map.at(i);

    if((hits+misses)>0)
    {
      double occupancy = double(hits)/double(hits+misses);
      occ_map.at(i) = occupancy;
    }
    else
      occ_map.at(i) = 0.5;
  }

  return occ_map;
}

void OccupancyMapFromWorld::MapSpace(nav_msgs::OccupancyGrid* map,
                                     double noise_stddev)
{
  ros::Publisher valid_samples_pub = nh_.advertise<geometry_msgs::PoseArray>("valid_samples", 1);
  ros::Publisher invalid_samples_pub = nh_.advertise<geometry_msgs::PoseArray>("invalid_samples", 1);
  ros::Publisher hits_map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("hits_map", 1);
  ros::Publisher misses_map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("misses_map", 1);
  geometry_msgs::PoseArray valid_samples;
  geometry_msgs::PoseArray invalid_samples;
  valid_samples.header = map->header;
  invalid_samples.header = map->header;

  nav_msgs::OccupancyGrid viz_grid;
  viz_grid.header = map->header;
  viz_grid.info = map->info;
  viz_grid.data.resize(map->data.size());

  nav_msgs::OccupancyGrid hits_viz_grid;
  hits_viz_grid.header = map->header;
  hits_viz_grid.info = map->info;
  hits_viz_grid.data.resize(map->data.size());

  nav_msgs::OccupancyGrid miss_viz_grid;
  miss_viz_grid.header = map->header;
  miss_viz_grid.info = map->info;
  miss_viz_grid.data.resize(map->data.size());

  gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  uint32_t cells_size_x = map->info.width;
  uint32_t cells_size_y = map->info.height;
  double map_resolution = map->info.resolution;
  double map_z = map->info.origin.position.z;

  //map boundaries for pose sampling
  double x_min = map->info.origin.position.x;
  double x_max = map->info.origin.position.x + cells_size_x*map_resolution;
  double y_min = map->info.origin.position.y;
  double y_max = map->info.origin.position.y + cells_size_y*map_resolution;
  double theta_min = 0;
  double theta_max = 2*M_PI;

  std::default_random_engine generator;
  std::normal_distribution<double> noise_sampler(0.0,noise_stddev);
  std::uniform_real_distribution<> x_sampler(x_min, x_max);
  std::uniform_real_distribution<> y_sampler(y_min, y_max);
  std::uniform_real_distribution<> theta_sampler(theta_min, theta_max);

  std::vector<int64_t> hits_map, misses_map;
  hits_map.resize(map->data.size());
  misses_map.resize(map->data.size());

  int samples_per_m2 = 100;
  int num_pose_samples = samples_per_m2 * (map->info.width*map_resolution) * (map->info.height*map_resolution);

  for(int sample_i=0; sample_i<num_pose_samples; sample_i++)
  {
    //sample random x,y and theta
    double pose_x = x_sampler(generator);
    double pose_y = y_sampler(generator);
    double pose_theta = theta_sampler(generator);

    geometry_msgs::Pose sampled_pose;
    sampled_pose.position.x = pose_x;
    sampled_pose.position.y = pose_y;
    sampled_pose.position.z = map_z;
    tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,pose_theta);
    tf::quaternionTFToMsg(quat, sampled_pose.orientation);

    //check if pose lies within reachable space
    unsigned int map_x, map_y, map_index;
    world2cell(pose_x, pose_y, map_resolution, map->info.origin.position, map_x, map_y);
    cell2index(map_x, map_y, cells_size_x, cells_size_y, map_index);

    if(map->data.at(map_index) == CellFree)
    {
      valid_samples.poses.push_back(sampled_pose);

      //simulate laser scan
      int num_rays = 360;
      double angle_incr = 2*M_PI/num_rays;
      double max_range = 10;
      double scan_angle = 0;

      while(scan_angle < 2*M_PI)
      {
        std::fill(viz_grid.data.begin(), viz_grid.data.end(), 0);
        //find the undistorted hit distance
        double ray_angle = pose_theta + scan_angle;
        double end_x = pose_x + max_range * cos(ray_angle);
        double end_y = pose_y + max_range * sin(ray_angle);

        double dist;
        std::string entity_name;

        ray->SetPoints(math::Vector3(pose_x, pose_y, map_z),
                       math::Vector3(end_x, end_y, map_z));
        ray->GetIntersection(dist, entity_name);

        //add noise to hit distance
        dist = dist + noise_sampler(generator);

        //get hit cell
        unsigned int hit_index = UINT_MAX;
        unsigned int hit_cell_x, hit_cell_y;

        double hit_x = pose_x + dist*cos(ray_angle);
        double hit_y = pose_y + dist*sin(ray_angle);

        world2cell(hit_x, hit_y, map_resolution, map->info.origin.position,
                   hit_cell_x, hit_cell_y);

        //make sure cell is inside map boundaries
        if(cell2index(hit_cell_x, hit_cell_y, map->info.width, map->info.height,
                      hit_index))
        {
          viz_grid.data.at(hit_index) = 200;
          if(hits_viz_grid.data.at(hit_index) < 100)
            hits_viz_grid.data.at(hit_index) += 10; //choose larger value for visualization

          if(hits_map.at(hit_index) < INT64_MAX)
            hits_map.at(hit_index) += 1;
          else
            std::cout << "MAX NUMBER REACHED!!" << std::endl;
        }

        //get missed cells
        //walk along the ray to see which cells were hit
        double dist_incr = 0.25*map->info.resolution;
        double ray_dist = dist-dist_incr;
        unsigned int last_miss_index = UINT_MAX;

        while(ray_dist>0)
        {
          unsigned int miss_cell_x, miss_cell_y, miss_index;

          double miss_x = pose_x + ray_dist*cos(ray_angle);
          double miss_y = pose_y + ray_dist*sin(ray_angle);

          world2cell(miss_x, miss_y, map_resolution, map->info.origin.position,
                     miss_cell_x, miss_cell_y);

          //make sure cell is inside map boundaries
          if(cell2index(miss_cell_x, miss_cell_y, map->info.width, map->info.height,
                        miss_index))
          {
            //make sure we didn't already count this cell
            if(miss_index != hit_index && miss_index != last_miss_index)
            {
              viz_grid.data.at(miss_index) = 50;
              if(miss_viz_grid.data.at(miss_index) < 100)
                miss_viz_grid.data.at(miss_index) += 10; //choose larger value for visualization

              if(misses_map.at(miss_index) < INT64_MAX)
                misses_map.at(miss_index) += 1;
              else
                std::cout << "MAX NUMBER REACHED!!" << std::endl;

              last_miss_index = miss_index;
            }
          }

          ray_dist-=dist_incr;
        }
        marker_.type = visualization_msgs::Marker::LINE_LIST;
        marker_.header = map->header;
        geometry_msgs::Point start, end;
        start.x = pose_x;
        start.y = pose_y;
        start.z = map_z;
        end.x = hit_x;
        end.y = hit_y;
        end.z = map_z;
        marker_.points.clear();
        marker_.points.push_back(start);
        marker_.points.push_back(end);
        marker_.scale.x = 0.01;
        marker_.color.a = 1;
        marker_.color.b = 1;

        marker_pub_.publish(marker_);
        map_pub2_.publish(viz_grid);
        ros::Duration(0.025).sleep();

        valid_samples_pub.publish(valid_samples);
        scan_angle += angle_incr;
      }

      std::vector<double> occupancy_grid = CountingModel(hits_map, misses_map);
      for(int map_i=0; map_i<occupancy_grid.size(); map_i++)
      {
        viz_grid.data.at(map_i) = occupancy_grid.at(map_i) * CellOccupied;
      }

      std::cout << "publish occupancy map: " << std::endl;
      for(int loop=0; loop<5; loop++)
      {
        hits_map_pub.publish(hits_viz_grid);
        misses_map_pub.publish(miss_viz_grid);
        map_pub_.publish(viz_grid);
        ros::Rate(100).sleep();
      }

      std::cout << "pose sample: " << sample_i << std::endl;
    }
    else //map is not in free space
    {
      invalid_samples.poses.push_back(sampled_pose);
      invalid_samples_pub.publish(invalid_samples);
    }
  }
}

bool OccupancyMapFromWorld::worldCellIntersection(const double cell_center_x,
                                                  const double cell_center_y,
                                                  const double min_z, const double max_z,
                                                  const double cell_length,
                                                  gazebo::physics::RayShapePtr ray)
{
  double dist;
  std::string entity_name;

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

        if(!entity_name.empty())
//          std::cout << "entity name: " << entity_name << std::endl;
          return true;
      }
    }
  }

  return false;
}

//mark the cells that can be reached by the robot, based on the heuristic of
//how many cells are connected
void OccupancyMapFromWorld::MarkConnected(nav_msgs::OccupancyGrid* map,
                                          int min_connected)
{
  std::cout << "marking free space " << std::endl;

  unsigned int cells_size_x = map->info.width;
  unsigned int cells_size_y = map->info.height;

  int8_t visited = 50; //value for marking visited cells

  //mark which cells in the collision map are connected to enough cells so
  //the robot could actually be in that space and start mapping
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

        //mark cells as free if they are connected to enough cells,
        //occupied otherwise
        for(int k=0; k<connected_list.size(); k++)
        {
          if(connected_count > min_connected)
            map->data.at(connected_list.at(k)) = CellFree; //robot could fit there
          else
            map->data.at(connected_list.at(k)) = CellOccupied; //unknown space
        }
      }
    }
  }
}



// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OccupancyMapFromWorld)

}  // namespace gazebo
