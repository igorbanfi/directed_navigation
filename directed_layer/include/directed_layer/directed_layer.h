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
#ifndef DIRECTED_LAYER_DIRECTED_LAYER_H_
#define DIRECTED_LAYER_DIRECTED_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>
#include <directed_msgs/DirectedMap.h>
#include <directed_msgs/DirectedMapUpdate.h>

namespace directed_layer
{

/*
 * @class DirectedLayer
 * @brief A costmap layer that provides data regarding directional cost
 */

class DirectedLayer : public costmap_2d::CostmapLayer
{
public:
  /**
   * Default constructor
   */
  DirectedLayer();

  /**
   * deconstructor
   */
  virtual ~DirectedLayer();

  /**
   * gets called at initalization of costmap.
   * defines reconfig callback functions, gets parameters from param server.
   */
  virtual void onInitialize();

  /**
   * Activates directed layer.
   */
  virtual void activate();

  /**
   * Deactivates directed layer
   */
  virtual void deactivate();

  /**
   * Resets directed layer
   */
  virtual void reset();

  /**
   * Gets called by layered costmap..
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);

  /**
   * Updates values of costmap.
   */
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /**
   * Matches layer size with costmap.
   */
  virtual void matchSize();

  /**
   * Allocates memory for master map and for 4 directed maps.
   */
  virtual void initMaps(unsigned int size_x, unsigned int size_y);

  /**
   * Resets values of maps.
   */
  virtual void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);

  /**
   * @brief Returns pointer to directed map xu
   */
  virtual unsigned char* getDirectedMapXu();

  /**
   * @brief Returns pointer to directed map xd
   */
  virtual unsigned char* getDirectedMapXd();

  /**
   * @brief Returns pointer to directed map yu
   */
  virtual unsigned char* getDirectedMapYu();

  /**
   * @brief Returns pointer to directed map yd
   */
  virtual unsigned char* getDirectedMapYd();

  /**
   * Returns resolution of directed maps.
   */
  virtual double getDirectedMapResolution();

  /**
   * Given 2 directed map coordinates, computes associated directed map index.
   */
  virtual unsigned int getDirectedMapIndex(unsigned int mx, unsigned int my);

  /**
   * Convert from directed map cooridinates to world coordinates.
   */
  virtual void directedMapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy);

  /**
   * Converts world coordinates to directed map coordinates.
   */
  virtual bool worldToDirectedMap(double wx, double wy, unsigned int& mx, unsigned int& my);

private:
  /**
   * Callback to update directed map.
   */
  void incomingMap(const directed_msgs::DirectedMapConstPtr& new_map);
  void incomingUpdate(const directed_msgs::DirectedMapUpdateConstPtr& update);

  /**
   * Callback function for dynamic reconfigure.
   */
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  unsigned char interpretValue(unsigned char value);

  // Parameters
  std::string global_frame_;
  std::string map_frame_;
  bool subscribe_to_updates_;
  bool map_received_;
  bool has_updated_data_;
  unsigned int x_, y_, width_, height_;
  bool track_unknown_space_;
  bool use_maximum_;
  bool first_map_only_;
  bool trinary_costmap_;
  double directed_map_resolution_;
  unsigned int directed_map_size_x_;
  unsigned int directed_map_size_y_;
  unsigned char lethal_threshold_, unknown_cost_value_;

  // subscribers for directed_msgs
  ros::Subscriber map_sub_, map_update_sub_;

  // dynamic reconfigure server
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

protected:
  // Pointers of individual directed maps
  unsigned char* directedMapXu;
  unsigned char* directedMapXd;
  unsigned char* directedMapYu;
  unsigned char* directedMapYd;
};

}  // namespace directed_layer

#endif  // DIRECTED_LAYER_DIRECTED_LAYER_H_
