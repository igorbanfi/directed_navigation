/*
 * Copyright (c) 2020, Igor Banfi
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <directed_layer/directed_layer.h>

PLUGINLIB_EXPORT_CLASS(directed_layer::DirectedLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace directed_layer
{

DirectedLayer::DirectedLayer() : dsrv_(NULL) {}

DirectedLayer::~DirectedLayer()
{
  if (dsrv_)
    delete dsrv_;
}

void DirectedLayer::initMaps(unsigned int size_x, unsigned int size_y)
{
  Costmap2D::initMaps(size_x, size_y);
  directedMapXu = new unsigned char[size_x * size_y];
  directedMapXd = new unsigned char[size_x * size_y];
  directedMapYu = new unsigned char[size_x * size_y];
  directedMapYd = new unsigned char[size_x * size_y];
}

void DirectedLayer::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  Costmap2D::resetMap(x0, y0, xn, yn);
  memset(directedMapXu, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  memset(directedMapXd, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  memset(directedMapYu, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  memset(directedMapYd, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void DirectedLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("first_map_only", first_map_only_, false);
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);

  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  // Only resubscribe if topic has changed
  if (map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map...");
    map_sub_ = g_nh.subscribe(map_topic, 1, &DirectedLayer::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    while (!map_received_ && g_nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Received a %d X %d directed map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

    if (subscribe_to_updates_)
    {
      ROS_INFO("Subscribing to updates");
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &DirectedLayer::incomingUpdate, this);

    }
  }
  else
  {
    has_updated_data_ = true;
  }

  if (dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &DirectedLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void DirectedLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void DirectedLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling())
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    ROS_INFO("matchSize");
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
}

unsigned char DirectedLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void DirectedLayer::incomingMap(const directed_msgs::DirectedMapConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d directed map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() &&
      (master->getSizeInCellsX() != size_x ||
       master->getSizeInCellsY() != size_y ||
       master->getResolution() != new_map->info.resolution ||
       master->getOriginX() != new_map->info.origin.position.x ||
       master->getOriginY() != new_map->info.origin.position.y))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y,
                                true );
  }
  if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing directed layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  }

  unsigned int index = 0;
  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {

      unsigned char value = new_map->data[index];
      unsigned char valuexu = new_map->dataXu[index];
      unsigned char valuexd = new_map->dataXd[index];
      unsigned char valueyu = new_map->dataYu[index];
      unsigned char valueyd = new_map->dataYd[index];

      costmap_[index] = interpretValue(value);
      directedMapXu[index] = interpretValue(valuexu);
      directedMapXd[index] = interpretValue(valuexd);
      directedMapYu[index] = interpretValue(valueyu);
      directedMapYd[index] = interpretValue(valueyd);

      ++index;
    }
  }
  map_frame_ = new_map->header.frame_id;

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_)
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }
}


void DirectedLayer::incomingUpdate(const directed_msgs::DirectedMapUpdateConstPtr& update)
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;

      costmap_[index] = interpretValue(update->data[di]);
      directedMapXu[index] = interpretValue(update->dataXu[di]);
      directedMapXd[index] = interpretValue(update->dataXd[di]);
      directedMapYu[index] = interpretValue(update->dataYu[di]);
      directedMapYd[index] = interpretValue(update->dataYd[di]);
      di = di + 1;

    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void DirectedLayer::activate()
{
  onInitialize();
}

void DirectedLayer::deactivate()
{
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
}

void DirectedLayer::reset()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}

void DirectedLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{

  if( !layered_costmap_->isRolling() ){
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
      return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;
}

void DirectedLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;

  if (!enabled_)
    return;

  if (!layered_costmap_->isRolling())
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    if (!use_maximum_)
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  else
  {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::convert(transform.transform, tf2_transform);
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform*p;
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          if (!use_maximum_)
            master_grid.setCost(i, j, getCost(mx, my));
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }
}

unsigned char* DirectedLayer::getDirectedMapXu(){
  return directedMapXu;
};

unsigned char* DirectedLayer::getDirectedMapXd(){
  return directedMapXd;
};
unsigned char* DirectedLayer::getDirectedMapYu(){
  return directedMapYu;
};
unsigned char* DirectedLayer::getDirectedMapYd(){
  return directedMapYd;
};

}  // namespace costmap_2d
