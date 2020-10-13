#ifndef TRAJECTORY_DIRECTED_LOCAL_PLANNER_TRAJECTORY_DIRECTED_LOCAL_PLANNER_H_
#define TRAJECTORY_DIRECTED_LOCAL_PLANNER_TRAJECTORY_DIRECTED_LOCAL_PLANNER_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <base_local_planner/footprint_helper.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <dwa_local_planner/dwa_planner_ros.h>
#include "directed_msgs/DirectedMap.h"
#include "directed_layer/directed_layer.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Point.h>


namespace trajectory_directed_local_planner {
  /**
   * @class DWAPlannerROS
   * @brief ROS Wrapper for the DWAPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class TrajectoryDirectedLocalPlanner : public base_local_planner::TrajectoryPlannerROS {

    public:
      /**
       * Constructor
       */
      TrajectoryDirectedLocalPlanner();

      /**
       * Deconstructor
       */
      ~TrajectoryDirectedLocalPlanner();

      /**
       * Initializes directed local planner
       *
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * Computes velocity commands that are sent to robot.
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * Sets plan.
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      bool isInitialized() {
        return initialized_;
      }

      bool isGoalReached();

      /**
       * Velocity 0 is sent to robot.
       */
      bool setStationaryVelocity(geometry_msgs::Twist& cmd_vel);

    private:

      /**
       * Generate node direction vectors.
       * @param  cost_directed_xu       Directed costmap for movement in x+ direction
       * @param  cost_directed_xd       Directed costmap for movement in x- direction
       * @param  cost_directed_yu       Directed costmap for movement in y+ direction
       * @param  cost_directed_yd       Directed costmap for movement in y- direction
       * @param  node_direction_vectors Array thats going to be filled by direction vectors
       * @param  directed_size_x        X size of directed map
       * @param  directed_size_y        Y size of directed map
       * @return                        Return true if succesful, false otherwise
       */
      bool generateDirectionVectors(unsigned char * cost_directed_xu, unsigned char * cost_directed_xd,
                                    unsigned char * cost_directed_yu, unsigned char * cost_directed_yd,
                                    double * node_direction_vectors, unsigned int directed_size_x, unsigned int directed_size_y );

      /**
       * Calculates priority a robot has to move from current position to curent goal.
       * @param  start_mx current mx of robot on directed map.
       * @param  start_my current my of roboy on directed map.
       * @param  goal_mx  mx of current goal on directed map
       * @param  goal_my  my of current goal on directed map
       * @return          value representing priority val
       */
      double getPriorityVal(unsigned int start_mx, unsigned int start_my,
                            unsigned int goal_mx, unsigned int goal_my);

      /**
       * Position on directed map is checked if anything is positioned in it.
       * Occupation is determined by looking at local costmap.
       * @param  mx mx of position on directed map
       * @param  my my of position on directed map
       * @return    bool value: true if occupied, false if free
       */
      bool checkOccupation(unsigned int mx, unsigned int my);

      /**
       * Position on directed map is checked if anything is positioned on it.
       * Occupation is determined by looking at local costmap.
       * @param  wx wx of position in world
       * @param  wy wy of position in world
       * @param footprint Polygon of points that are checked around position
       * @return    bool value: true if occupied, false if free
       */
      bool checkOccupation(double wx, double wy, std::vector< geometry_msgs::Point > footprint);

      /**
       * TODO:
       * Checks if my current position blockes a robot in wx wy going to goal.
       * @param  wx                  wx position of blocked robot
       * @param  wy                  wy position of blocked robot
       * @param  current_goal_wx     wx position of current goal
       * @param  current_goal_wy     wy position of current goal
       * @param  current_position_wx wx of current position
       * @param  current_position_wy wy of current position
       * @return                     bool value: true if blocked, false if not blocked
       */
      bool checkIfBlocked(double wx, double wy, double current_goal_wx, double current_goal_wy, double current_position_wx, double current_position_wy);

      /**
       * Determine right of way of robot based on directed map, local costmap.
       * @return bool value: true if have right of way, false if do not have right of way
       */
      bool determineRightOfWay();

      // Parameterss
      unsigned int directed_size_x_, directed_size_y_;

      costmap_2d::Costmap2DROS * 	costmap_ros_;
      costmap_2d::LayeredCostmap* layered_costmap_;
      boost::shared_ptr<directed_layer::DirectedLayer> directed_layer_;

      bool right_of_way;
      bool setup_;

      unsigned char * cost_static;
      unsigned char * cost_directed_xu;
      unsigned char * cost_directed_xd;
      unsigned char * cost_directed_yu;
      unsigned char * cost_directed_yd;
      double * node_direction_vectors;

      double check_occupation_size;

      std::vector<geometry_msgs::PoseStamped> global_plan_;
      costmap_2d::Costmap2D* costmap_;
      std::string global_frame_;
      std::string robot_base_frame_;

      ros::Time no_priority_time;
      ros::Time no_priority_time_2;

      bool priority, path_blocked;

      std::string directed_layer_name;

      bool initialized_;
      base_local_planner::LocalPlannerUtil planner_util_;
      geometry_msgs::PoseStamped current_pose_;
      base_local_planner::LatchedStopRotateController latchedStopRotateController_;
      base_local_planner::OdometryHelperRos odom_helper_;
      tf2_ros::Buffer* tf_;

      base_local_planner::FootprintHelper footprint_helper_;

      std::vector< geometry_msgs::Point > check_occupation_footprint;

      int wait_time;

  };
};
#endif // namespace trajectory_directed_local_planner
