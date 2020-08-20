#ifndef DWA_DIRECTED_LOCAL_PLANNER_DWA_DIRECTED_LOCAL_PLANNER_H_
#define DWA_DIRECTED_LOCAL_PLANNER_DWA_DIRECTED_LOCAL_PLANNER_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include <directed_local_planner/DirectedLocalPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <dwa_local_planner/dwa_planner_ros.h>

#include "directed_msgs/DirectedMap.h"
#include "directed_layer/directed_layer.h"

namespace dwa_directed_local_planner {

  class DWADirectedLocalPlanner : public dwa_local_planner::DWAPlannerROS {
    public:
      DWADirectedLocalPlanner();

      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    private:
      costmap_2d::Costmap2DROS * 	costmap_ros_;
      bool determineRightOfWay();

      costmap_2d::LayeredCostmap* layered_costmap_;
      boost::shared_ptr<directed_layer::DirectedLayer> directed_layer_;

      bool right_of_way;

      unsigned char * cost_static;
      unsigned char * cost_directed_xu;
      unsigned char * cost_directed_xd;
      unsigned char * cost_directed_yu;
      unsigned char * cost_directed_yd;
      float * node_direction_vectors;

      std::string directed_layer_name;

  };
};
#endif
