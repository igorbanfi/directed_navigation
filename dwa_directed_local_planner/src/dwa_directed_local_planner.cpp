#include <dwa_directed_local_planner/dwa_directed_local_planner.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_directed_local_planner::DWADirectedLocalPlanner, nav_core::BaseLocalPlanner)

namespace dwa_directed_local_planner {

  void DWADirectedLocalPlanner::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
        if (! isInitialized()) {
          costmap_ros_ = costmap_ros;
          right_of_way = false;
          layered_costmap_ = costmap_ros_->getLayeredCostmap();
          std::vector< boost::shared_ptr <costmap_2d::Layer> >* plugini_;
          plugini_ = layered_costmap_->getPlugins();

          ros::NodeHandle private_nh("~/" + name);
          private_nh.param<std::string>("directed_layer_name", directed_layer_name, "global_costmap/directed_map");


          int plugin_num = (*plugini_).size();
          for (int i = 0; i < plugin_num; i++)
          {
            std::string ime = (*plugini_)[i]->getName();
            if (ime == directed_layer_name)
            {
              directed_layer_ = boost::static_pointer_cast<directed_layer::DirectedLayer>((*plugini_)[i]);
            }
          }

          cost_static = (costmap_ros_->getCostmap())->getCharMap();
          cost_directed_xu = directed_layer_->getDirectedMapXu();
          cost_directed_xd = directed_layer_->getDirectedMapXd();
          cost_directed_yu = directed_layer_->getDirectedMapYu();
          cost_directed_yd  = directed_layer_->getDirectedMapYd();

          unsigned int size_x_ = directed_layer_->getSizeInCellsX();
          unsigned int size_y_ = directed_layer_->getSizeInCellsY();

          node_direction_vectors = new float[size_x_*size_y_];

          unsigned int x0, xn, y0, yn;
          xn = (costmap_ros_->getCostmap())->getSizeInCellsX();
          x0 = 0;
          yn = (costmap_ros_->getCostmap())->getSizeInCellsY();
          y0 = 0;

          unsigned int index;
          unsigned int index_xu;
          unsigned int index_xd;
          unsigned int index_yu;
          unsigned int index_yd;
          int difference_xu;
          int difference_xd;
          int difference_yu;
          int difference_yd;

          for (unsigned int i = 0; i<size_x_; i++){
            for (unsigned int j = 0; j<size_y_; j++){

              difference_xu = 0;
              difference_xd = 0;
              difference_yu = 0;
              difference_yd = 0;

              index = costmap_ros_->getCostmap()->getIndex(i, j);
              if (i+1<=xn){
                index_xu = costmap_ros_->getCostmap()->getIndex(i+1, j);
                difference_xu = cost_directed_xd[index] - cost_directed_xu[index_xu];
              }
              if (i>0){
                index_xd = costmap_ros_->getCostmap()->getIndex(i-1, j);
                difference_xd = cost_directed_xu[index] - cost_directed_xd[index_xd];
              }
              if (j+1<=yn){
                index_yu = costmap_ros_->getCostmap()->getIndex(i, j+1);
                difference_yu = cost_directed_yd[index] - cost_directed_yu[index_yu];
              }
              if (j>0){
                index_yd = costmap_ros_->getCostmap()->getIndex(i, j-1);
                difference_yd = cost_directed_yu[index] - cost_directed_yd[index_yd];
              }
              node_direction_vectors[index] = atan2(difference_yu-difference_yd,
                                                    difference_xd-difference_xu);
            }
          }
          DWAPlannerROS::initialize(name, tf, costmap_ros);
        }
      }

    bool DWADirectedLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
      if (right_of_way) DWAPlannerROS::computeVelocityCommands(cmd_vel);
      return true;
    }

    bool DWADirectedLocalPlanner::determineRightOfWay(){
      return true;
    }

};
