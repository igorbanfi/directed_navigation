#include <trajectory_directed_local_planner/trajectory_directed_local_planner.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>
#include <Eigen/Dense>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(trajectory_directed_local_planner::TrajectoryDirectedLocalPlanner, nav_core::BaseLocalPlanner)

namespace trajectory_directed_local_planner {

  TrajectoryDirectedLocalPlanner::TrajectoryDirectedLocalPlanner() : initialized_(false),
    odom_helper_("odom"), setup_(false) {

    }
  TrajectoryDirectedLocalPlanner::~TrajectoryDirectedLocalPlanner(){
    //make sure to clean things up
    //delete dsrv_;
    }

  void TrajectoryDirectedLocalPlanner::initialize(
    std::string name,
    tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* costmap_ros) {
      if (! isInitialized()) {
        ros::NodeHandle private_nh("~/" + name);

        priority = true;
        path_blocked = true;


        TrajectoryPlannerROS::initialize(name, tf, costmap_ros);

        tf_ = tf;

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();

        right_of_way = false;
        layered_costmap_ = costmap_ros_->getLayeredCostmap();
        std::vector< boost::shared_ptr <costmap_2d::Layer> >* plugini_;
        plugini_ = layered_costmap_->getPlugins();

        private_nh.param<std::string>("directed_layer_name", directed_layer_name, "local/directed_map");
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

        directed_size_x_ = directed_layer_->getSizeInCellsX();
        directed_size_y_ = directed_layer_->getSizeInCellsY();

        if (!private_nh.getParam("check_occupation_size", check_occupation_size)) {
          check_occupation_size = directed_layer_->getDirectedMapResolution()/2.0;
        }
        geometry_msgs::Point p1, p2, p3, p4;
        p1.x = -check_occupation_size/2.0;
        p1.y = -check_occupation_size/2.0;
        p2.x = -check_occupation_size/2.0;
        p2.y = check_occupation_size/2.0;
        p3.x = check_occupation_size/2.0;
        p3.y = check_occupation_size/2.0;
        p4.x = check_occupation_size/2.0;
        p4.y = -check_occupation_size/2.0;

        check_occupation_footprint.push_back(p1);
        check_occupation_footprint.push_back(p2);
        check_occupation_footprint.push_back(p3);
        check_occupation_footprint.push_back(p4);

        node_direction_vectors = new double[directed_size_x_*directed_size_y_];

        generateDirectionVectors(cost_directed_xu, cost_directed_xd, cost_directed_yu,
                                 cost_directed_yd, node_direction_vectors, directed_size_x_, directed_size_y_);

        initialized_ = true;
      }
    }

  bool TrajectoryDirectedLocalPlanner::generateDirectionVectors(unsigned char * cost_directed_xu, unsigned char * cost_directed_xd,
                                unsigned char * cost_directed_yu, unsigned char * cost_directed_yd,
                                double * node_direction_vectors, unsigned int directed_size_x, unsigned int directed_size_y ) {
    unsigned int x0, xn, y0, yn;
    xn = directed_size_x;
    x0 = 0;
    yn = directed_size_y;
    y0 = 0;

    unsigned int index;
    unsigned int index_xu;
    unsigned int index_xd;
    unsigned int index_yu;
    unsigned int index_yd;
    double difference_xu;
    double difference_xd;
    double difference_yu;
    double difference_yd;

    for (unsigned int i = 0; i<directed_size_x; i++){
      for (unsigned int j = 0; j<directed_size_y; j++){

        difference_xu = 0.;
        difference_xd = 0.;
        difference_yu = 0.;
        difference_yd = 0.;

        index = j * directed_size_x + i;
        if (i+1<=xn){
          index_xu = j * directed_size_x + i + 1;
          difference_xu = (double)(cost_directed_xd[index] - cost_directed_xu[index_xu]);
        }
        if (i>0){
          index_xd = j * directed_size_x + i -1;
          difference_xd = (double)(-cost_directed_xu[index] + cost_directed_xd[index_xd]);
        }
        if (j+1<=yn){
          index_yu = (j + 1) * directed_size_x + i;
          difference_yu = (double)(cost_directed_yd[index] - cost_directed_yu[index_yu]);
        }
        if (j>0){
          index_yd = (j - 1) * directed_size_x + i;
          difference_yd = (double)(-cost_directed_yu[index] + cost_directed_yd[index_yd]);
        }
        node_direction_vectors[index] = atan2(difference_yu+difference_yd,
                                              difference_xd+difference_xu);
      }
    }
    return true;
  }

  bool TrajectoryDirectedLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    //if (right_of_way) DWAPlannerROS::computeVelocityCommands(cmd_vel);
    if (determineRightOfWay()) {
      no_priority_time_2 = ros::Time::now();
      wait_time = rand()%6 + 4;
      return TrajectoryPlannerROS::computeVelocityCommands(cmd_vel);
    }
    else if ((ros::Time::now() - no_priority_time_2) >  ros::Duration(wait_time)) {
      return TrajectoryPlannerROS::computeVelocityCommands(cmd_vel);
    }
    else return setStationaryVelocity(cmd_vel);
  }

  bool TrajectoryDirectedLocalPlanner::setStationaryVelocity(geometry_msgs::Twist& cmd_vel){
    //ROS_INFO("Seting 0 vel");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }

  bool TrajectoryDirectedLocalPlanner::determineRightOfWay(){

    if (! isInitialized()) {
     ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
     return false;
   }

     right_of_way = true;

     std::vector<geometry_msgs::PoseStamped> local_plan;

     geometry_msgs::PoseStamped global_pose_;
     if (!costmap_ros_->getRobotPose(global_pose_)) {
       return false;
     }

     std::vector<geometry_msgs::PoseStamped> transformed_plan;
     //get the global plan in our frame
     if (!base_local_planner::transformGlobalPlan(*tf_, global_plan_, global_pose_, *costmap_, global_frame_, transformed_plan)) {
       ROS_WARN("Could not transform the global plan to the frame of the controller");
       return false;
     }

     //if the global plan passed in is empty... we won't do anything
     if(transformed_plan.empty())
       return false;

     tf::Stamped<tf::Pose> goal_point;
     double goal_x = goal_point.getOrigin().getX();
     double goal_y = goal_point.getOrigin().getY();
     int closest_point_index;
     double closest_point_dist = std::numeric_limits<double>::max();

     for (int i=0; i < transformed_plan.size(); i++) {
       double goal_x = transformed_plan[i].pose.position.x;
       double goal_y = transformed_plan[i].pose.position.y;
       double dist_ = base_local_planner::getGoalPositionDistance(global_pose_, goal_x, goal_y);
       if (dist_ <= closest_point_dist) {
         closest_point_dist = dist_;
         closest_point_index = i;
       }
     }

     double current_directed_map_x = transformed_plan[closest_point_index].pose.position.x;
     double current_directed_map_y = transformed_plan[closest_point_index].pose.position.y;

     unsigned int current_position_directed_map_mx = (unsigned int) (current_directed_map_x / directed_layer_->getDirectedMapResolution());
     unsigned int current_position_directed_map_my = (unsigned int) (current_directed_map_y / directed_layer_->getDirectedMapResolution());

     double current_goal_x, current_goal_y;
     unsigned int current_goal_directed_map_mx, current_goal_directed_map_my;

     int current_goal_index = 0;
     for (int i=closest_point_index; i<transformed_plan.size(); i++) {
       current_goal_index = i;
       current_goal_x = transformed_plan[current_goal_index].pose.position.x;
       current_goal_y = transformed_plan[current_goal_index].pose.position.y;

       current_goal_directed_map_mx = (unsigned int) (current_goal_x / directed_layer_->getDirectedMapResolution());
       current_goal_directed_map_my = (unsigned int) (current_goal_y / directed_layer_->getDirectedMapResolution());

       if (current_goal_directed_map_mx!=current_position_directed_map_mx || current_goal_directed_map_my!=current_position_directed_map_my) {
            break;
          }

     }

     float current_direction_vector = atan2(transformed_plan[current_goal_index].pose.position.y-transformed_plan[closest_point_index].pose.position.y,
                                          transformed_plan[current_goal_index].pose.position.x-transformed_plan[closest_point_index].pose.position.x);
      double priority_val = getPriorityVal(current_position_directed_map_mx, current_position_directed_map_my,
                                          current_goal_directed_map_mx, current_goal_directed_map_my);


      //bool occupied = checkOccupation(current_position_directed_map_mx, current_position_directed_map_my);
      bool occupied = checkOccupation(current_directed_map_x, current_directed_map_y, check_occupation_footprint);

      bool occupied_xd = false;
      bool occupied_xu = false;
      bool occupied_yd = false;
      bool occupied_yu = false;

      bool right_of_way_xd = true;
      bool right_of_way_xu = true;
      bool right_of_way_yd = true;
      bool right_of_way_yu = true;

      double priority_val_xd = std::numeric_limits<double>::max()*(-1.);
      double priority_val_xu = std::numeric_limits<double>::max()*(-1.);
      double priority_val_yd = std::numeric_limits<double>::max()*(-1.);
      double priority_val_yu = std::numeric_limits<double>::max()*(-1.);

      if (checkOccupation(current_goal_directed_map_mx, current_goal_directed_map_my)) {
        right_of_way = false;
        path_blocked = true;
        priority = true;
        no_priority_time = ros::Time::now();
        //ROS_INFO("Path occupied");
        return right_of_way;
      }

      path_blocked = false;

      if (current_goal_directed_map_mx>0){
        unsigned int mx_ = current_goal_directed_map_mx-1;
        unsigned int my_ = current_goal_directed_map_my;
        double wx_, wy_;
        directed_layer_->directedMapToWorld(mx_, my_, wx_, wy_);
        if (!(mx_==current_position_directed_map_mx && my_==current_position_directed_map_my)) {
            if (checkOccupation(wx_, wy_, check_occupation_footprint)) {
              priority_val_xd = getPriorityVal(mx_, my_, current_goal_directed_map_mx, current_goal_directed_map_my);
          }
        }
      }
      if (current_goal_directed_map_mx<directed_size_x_-1){
        unsigned int mx_ = current_goal_directed_map_mx+1;
        unsigned int my_ = current_goal_directed_map_my;
        double wx_, wy_;
        directed_layer_->directedMapToWorld(mx_, my_, wx_, wy_);
        if (!(mx_==current_position_directed_map_mx && my_==current_position_directed_map_my)) {
            if (checkOccupation(wx_, wy_, check_occupation_footprint)) {
              priority_val_xu = getPriorityVal(mx_, my_, current_goal_directed_map_mx, current_goal_directed_map_my);
          }
        }
      }
      if (current_goal_directed_map_my>0){
        unsigned int mx_ = current_goal_directed_map_mx;
        unsigned int my_ = current_goal_directed_map_my-1;
        double wx_, wy_;
        directed_layer_->directedMapToWorld(mx_, my_, wx_, wy_);
        if (!(mx_==current_position_directed_map_mx && my_==current_position_directed_map_my)) {
            if (checkOccupation(wx_, wy_, check_occupation_footprint)) {
            priority_val_yd = getPriorityVal(mx_, my_, current_goal_directed_map_mx, current_goal_directed_map_my);
          }
        }
      }
      if (current_goal_directed_map_my<directed_size_y_-1){
        unsigned int mx_ = current_goal_directed_map_mx;
        unsigned int my_ = current_goal_directed_map_my+1;
        double wx_, wy_;
        directed_layer_->directedMapToWorld(mx_, my_, wx_, wy_);
        if (!(mx_==current_position_directed_map_mx && my_==current_position_directed_map_my)) {
            if (checkOccupation(wx_, wy_, check_occupation_footprint)) {
              priority_val_yu = getPriorityVal(mx_, my_, current_goal_directed_map_mx, current_goal_directed_map_my);
          }
        }
      }

      if (priority_val_xd > priority_val) {
        //ROS_INFO("priority");
        right_of_way = false;
        priority = false;
        return right_of_way;
      }
      if (priority_val_xu > priority_val) {
        //ROS_INFO("priority");
        right_of_way = false;
        priority = false;
        return right_of_way;
      }
      if (priority_val_yd > priority_val) {
        //ROS_INFO("priority");
        right_of_way = false;
        priority = false;
        return right_of_way;
      }
      if (priority_val_yu > priority_val) {
        //ROS_INFO("priority");
        right_of_way = false;
        priority = false;
        return right_of_way;

      }
      return right_of_way;
  }

    bool TrajectoryDirectedLocalPlanner::checkOccupation(double wx, double wy, std::vector< geometry_msgs::Point > footprint) {

      Eigen::Vector3f position_of_cell(wx, wy, 0.0);
      std::vector <base_local_planner::Position2DInt > cells_to_check = footprint_helper_.getFootprintCells(position_of_cell, footprint, *costmap_, true);

      unsigned int occupation_val = 0;
      for (int i = 1; i < cells_to_check.size(); i++){
        unsigned int val = (unsigned int)(costmap_->getCost(cells_to_check[i].x, cells_to_check[i].y));
        occupation_val = occupation_val + val;
      }

      if (occupation_val) {
        return true;
      }

      return false;
    }

    bool TrajectoryDirectedLocalPlanner::checkOccupation(unsigned int mx, unsigned int my)
    {
      double wx, wy;
      directed_layer_->directedMapToWorld(mx, my, wx, wy);
      Eigen::Vector3f position_of_cell(wx, wy, 0.0);
      std::vector <base_local_planner::Position2DInt > cells_to_check = footprint_helper_.getFootprintCells(position_of_cell, check_occupation_footprint, *costmap_, true);

      unsigned int occupation_val = 0;
      for (int i = 1; i < cells_to_check.size(); i++){
        unsigned int val = (unsigned int)(costmap_->getCost(cells_to_check[i].x, cells_to_check[i].y));
        occupation_val = occupation_val + val;
      }
      if (occupation_val) return true;

      return false;
    }

    bool TrajectoryDirectedLocalPlanner::checkIfBlocked(double wx, double wy, double current_goal_wx, double current_goal_wy, double current_position_wx, double current_position_wy){

      geometry_msgs::Point p1_, p2_, p3_, p4_;
      p1_.x = 0.0;
      p1_.y = 0.0;
      p2_.x = current_goal_wx - wx;
      p2_.y = current_goal_wy - wy;

      std::vector< geometry_msgs::Point > priority_val_occupation_footprint;

      priority_val_occupation_footprint.push_back(p1_);
      priority_val_occupation_footprint.push_back(p2_);

      Eigen::Vector3f position_of_cell(wx, wy, 0.0);
      std::vector <base_local_planner::Position2DInt > cells_to_check_ = footprint_helper_.getFootprintCells(position_of_cell, priority_val_occupation_footprint, *costmap_, true);

      p1_.x = -0.2;
      p1_.y = -0.2;
      p2_.x = -0.2;
      p2_.y = 0.2;
      p3_.x = 0.2;
      p3_.y = 0.2;
      p4_.x = 0.2;
      p4_.y = -0.2;

      std::vector< geometry_msgs::Point > my_footprint;

      my_footprint.push_back(p1_);
      my_footprint.push_back(p2_);
      my_footprint.push_back(p3_);
      my_footprint.push_back(p4_);

      Eigen::Vector3f my_position(current_position_wx, current_position_wy, 0.0);
      std::vector <base_local_planner::Position2DInt > my_cells = footprint_helper_.getFootprintCells(my_position, my_footprint, *costmap_, true);

      for (int i = 1; i < cells_to_check_.size(); i++){
        for (int j = 1; i< my_cells.size(); j++) {
          if (cells_to_check_[i].x == my_cells[j].x && cells_to_check_[i].y == my_cells[j].y){
            return true;
            break;
          }
        }
      }
      return false;
    }



    bool TrajectoryDirectedLocalPlanner::isGoalReached(){
      return TrajectoryPlannerROS::isGoalReached();
    }

    bool TrajectoryDirectedLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
      if (! isInitialized()) {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
      }
     global_plan_.clear();
     global_plan_ = orig_global_plan;
     return TrajectoryPlannerROS::setPlan(orig_global_plan);
    }

    double TrajectoryDirectedLocalPlanner::getPriorityVal(unsigned int start_mx, unsigned int start_my,
                                                          unsigned int goal_mx, unsigned int goal_my)
    {
      double start_wx, start_wy, goal_wx, goal_wy;
      directed_layer_->directedMapToWorld(start_mx, start_my, start_wx, start_wy);
      directed_layer_->directedMapToWorld(goal_mx, goal_my, goal_wx, goal_wy);

      double current_direction_vector = atan2(goal_wy-start_wy, goal_wx-start_wx);

      int current_goal_index_directed_map = directed_layer_->getDirectedMapIndex(goal_mx, goal_my);

      double priority_val = cos (current_direction_vector - node_direction_vectors[current_goal_index_directed_map]);

      return priority_val;
    }


}; // namespace trajectory_directed_local_planner
