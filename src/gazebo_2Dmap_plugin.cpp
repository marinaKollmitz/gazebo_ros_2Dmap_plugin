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

#include <octomap_msgs/conversions.h>
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

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gazebo_2Dmap_plugin/map", 1);
  map_service_ = nh_.advertiseService(
        "gazebo_2Dmap_plugin/generate_map", &OccupancyMapFromWorld::ServiceCallback, this);

  //  std::string service_name = "world/get_octomap";
  //  std::string octomap_pub_topic = "world/octomap";
  //  getSdfParam<std::string>(_sdf, "octomapPubTopic", octomap_pub_topic,
  //                           octomap_pub_topic);
  //  getSdfParam<std::string>(_sdf, "octomapServiceName", service_name,
  //                           service_name);

  //  gzlog << "Advertising service: " << service_name << std::endl;
  //  srv_ = node_handle_.advertiseService(
  //      service_name, &OctomapFromGazeboWorld::ServiceCallback, this);
  //  octomap_publisher_ =
  //      node_handle_.advertise<octomap_msgs::Octomap>(octomap_pub_topic, 1, true);
}

//bool OctomapFromGazeboWorld::ServiceCallback(
//    robotino_sim::Octomap::Request& req, robotino_sim::Octomap::Response& res) {
//  std::cout << "Creating octomap with origin at (" << req.bounding_box_origin.x
//        << ", " << req.bounding_box_origin.y << ", "
//        << req.bounding_box_origin.z << "), and bounding box lengths ("
//        << req.bounding_box_lengths.x << ", " << req.bounding_box_lengths.y
//        << ", " << req.bounding_box_lengths.z
//        << "), and leaf size: " << req.leaf_size << ".\n";
//  CreateOctomap(req);
//  if (req.filename != "") {
//    if (octomap_) {
//      std::string path = req.filename;
//      octomap_->writeBinary(path);
//      std::cout << std::endl << "Octree saved as " << path << std::endl;
//    } else {
//      ROS_ERROR("The octree is NULL. Will not save that.");
//    }
//  }
//  common::Time now = world_->GetSimTime();
//  res.map.header.frame_id = "world";
//  res.map.header.stamp = ros::Time(now.sec, now.nsec);

//  if (!octomap_msgs::binaryMapToMsg(*octomap_, res.map)) {
//    ROS_ERROR("Error serializing OctoMap");
//  }

//  if (req.publish_octomap) {
//    gzlog << "Publishing Octomap." << std::endl;
//    octomap_publisher_.publish(res.map);
//  }

//  common::SphericalCoordinatesPtr sphericalCoordinates = world_->GetSphericalCoordinates();
//#if GAZEBO_MAJOR_VERSION >= 6
//  ignition::math::Vector3d origin_cartesian(0.0, 0.0, 0.0);
//  ignition::math::Vector3d origin_spherical = sphericalCoordinates->
//      SphericalFromLocal(origin_cartesian);

//  res.origin_latitude = origin_spherical.X();
//  res.origin_longitude = origin_spherical.Y();
//  res.origin_altitude = origin_spherical.Z();
//  return true;
//#else
//  math::Vector3 origin_cartesian(0.0, 0.0, 0.0);
//  math::Vector3 origin_spherical = sphericalCoordinates->
//         SphericalFromLocal(origin_cartesian);

//  res.origin_latitude = origin_spherical.x;
//  res.origin_longitude = origin_spherical.y;
//  res.origin_altitude = origin_spherical.z;
//  return true;
//#endif
//}

//void OctomapFromGazeboWorld::FloodFill(
//    const math::Vector3& seed_point, const math::Vector3& bounding_box_origin,
//    const math::Vector3& bounding_box_lengths, const double leaf_size) {
//  octomap::OcTreeNode* seed =
//      octomap_->search(seed_point.x, seed_point.y, seed_point.z);
//  // do nothing if point occupied
//  if (seed != NULL && seed->getOccupancy()) return;

//  std::stack<octomath::Vector3> to_check;
//  to_check.push(octomath::Vector3(seed_point.x, seed_point.y, seed_point.z));

//  while (to_check.size() > 0) {
//    octomath::Vector3 p = to_check.top();

//    if ((p.x() > bounding_box_origin.x - bounding_box_lengths.x / 2) &&
//        (p.x() < bounding_box_origin.x + bounding_box_lengths.x / 2) &&
//        (p.y() > bounding_box_origin.y - bounding_box_lengths.y / 2) &&
//        (p.y() < bounding_box_origin.y + bounding_box_lengths.y / 2) &&
//        (p.z() > bounding_box_origin.z - bounding_box_lengths.z / 2) &&
//        (p.z() < bounding_box_origin.z + bounding_box_lengths.z / 2) &&
//        (!octomap_->search(p))) {
//      octomap_->setNodeValue(p, 0);
//      to_check.pop();
//      to_check.push(octomath::Vector3(p.x() + leaf_size, p.y(), p.z()));
//      to_check.push(octomath::Vector3(p.x() - leaf_size, p.y(), p.z()));
//      to_check.push(octomath::Vector3(p.x(), p.y() + leaf_size, p.z()));
//      to_check.push(octomath::Vector3(p.x(), p.y() - leaf_size, p.z()));
//      to_check.push(octomath::Vector3(p.x(), p.y(), p.z() + leaf_size));
//      to_check.push(octomath::Vector3(p.x(), p.y(), p.z() - leaf_size));

//    } else {
//      to_check.pop();
//    }
//  }
//}

bool OccupancyMapFromWorld::ServiceCallback(std_srvs::Empty::Request& req,
                                            std_srvs::Empty::Response& res)
{
  CreateOccupancyMap();
  return true;
}

bool OccupancyMapFromWorld::worldCellIntersection(const math::Vector3& cell_center,
                                                  const double cell_length,
                                                  gazebo::physics::RayShapePtr ray)
{
  //check for collisions with rays surrounding the cell
  //    ---
  //   | + |
  //    ---

  double dist;
  std::string entity_name;

  for(int i=-1; i<2; i+=2)
  {
    double start_x = cell_center.x + i * cell_length/2;
    double start_y = cell_center.y - i * cell_length/2;

    for(int j=-1; j<2; j+=2)
    {
      double end_x = cell_center.x + j * cell_length/2;
      double end_y = cell_center.y + j * cell_length/2;

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

  return false;
}

void OccupancyMapFromWorld::CreateOccupancyMap()
{
  double map_height = 0.3;

  math::Vector3 map_origin(0,0,map_height);

  double map_size_x = 20;
  double map_size_y = 20;

  double map_resolution = 0.1;

  unsigned int cells_size_x = map_size_x / map_resolution;
  unsigned int cells_size_y = map_size_y / map_resolution;

  occupancy_map_ = new nav_msgs::OccupancyGrid();
  occupancy_map_->data.resize(cells_size_x * cells_size_y);

  gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  std::cout << "Rasterizing world and checking collisions" << std::endl;

  int ind = 0;
  for (unsigned int cell_y = 0; cell_y < cells_size_y; cell_y += 1)
  {
    for (unsigned int cell_x = 0; cell_x < cells_size_x; cell_x += 1)
    {
      //cell origin
      double world_x = cell_x * map_resolution - map_origin.x - map_size_x/2 + map_resolution/2;
      double world_y = cell_y * map_resolution - map_origin.y - map_size_y/2 + map_resolution/2;

      if (worldCellIntersection(math::Vector3(world_x, world_y, map_height),
                                map_resolution, ray))
      {
        occupancy_map_->data.at(ind) = 100;
      }
      ind += 1;
    }
  }

  occupancy_map_->header.stamp = ros::Time::now();
  occupancy_map_->header.frame_id = "odom";
  occupancy_map_->info.map_load_time = ros::Time(0);
  occupancy_map_->info.resolution = map_resolution;
  occupancy_map_->info.width = cells_size_x;
  occupancy_map_->info.height = cells_size_y;
  occupancy_map_->info.origin.position.x = map_origin.x - map_size_x / 2;
  occupancy_map_->info.origin.position.y = map_origin.y - map_size_y / 2;
  occupancy_map_->info.origin.position.z = map_origin.z;
  occupancy_map_->info.origin.orientation.w = 1;

//  // set unknown to filled
//  for (double x =
//       leaf_size / 2 + bounding_box_origin.x - bounding_box_lengths.x / 2;
//       x < bounding_box_origin.x + bounding_box_lengths.x / 2; x += leaf_size) {
//    int progress =
//        round(100 * (x + bounding_box_lengths.x / 2 - bounding_box_origin.x) /
//              bounding_box_lengths.x);
//    std::cout << "\rFilling closed spaces... " << progress << "%              ";

//    for (double y =
//         leaf_size / 2 + bounding_box_origin.y - bounding_box_lengths.y / 2;
//         y < bounding_box_origin.y + bounding_box_lengths.y / 2;
//         y += leaf_size) {
//      for (double z = leaf_size / 2 + bounding_box_origin.z -
//           bounding_box_lengths.z / 2;
//           z < bounding_box_origin.z + bounding_box_lengths.z / 2;
//           z += leaf_size) {
//        octomap::OcTreeNode* seed = octomap_->search(x, y, z);
//        if (!seed) octomap_->setNodeValue(x, y, z, 1);
//      }
//    }
//  }

  std::cout << "\rOccupancy Map generation completed                  " << std::endl;
  map_pub_.publish(*occupancy_map_);
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OccupancyMapFromWorld)

}  // namespace gazebo
