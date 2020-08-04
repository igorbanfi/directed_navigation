/*
 * Copyright (c) 2020, Igor Banfi
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
#include <directed_planner/directed_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <limits>


namespace directed_planner {

bool DirectedPlanner::astar(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

  if(!initialized_){
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

  plan.clear();
  costmap_ = costmap_ros_->getCostmap();

  // Do posameznih layeryev costmapa dostopamo preko pluginov
  layered_costmap_ = costmap_ros_->getLayeredCostmap();
  std::vector< boost::shared_ptr <costmap_2d::Layer> >* plugini_;
  plugini_ = layered_costmap_->getPlugins();

  boost::shared_ptr<directed_layer::DirectedLayer> directed_layer;

  int plugin_num = (*plugini_).size();
  for (int i = 0; i < plugin_num; i++)
  {
    std::string ime = (*plugini_)[i]->getName();
    if (ime == directed_layer_name)
    {
      directed_layer = boost::static_pointer_cast<directed_layer::DirectedLayer>((*plugini_)[i]);
    }
  }

  unsigned char * cost_static = costmap_->getCharMap();
  unsigned char * cost_directed_xu = directed_layer->getDirectedMapXu();
  unsigned char * cost_directed_xd = directed_layer->getDirectedMapXd();
  unsigned char * cost_directed_yu = directed_layer->getDirectedMapYu();
  unsigned char * cost_directed_yd  = directed_layer->getDirectedMapYd();

  // Pogledamo kakšne so velikosti gridov
  int x0, xn, y0, yn;
  xn = directed_layer->getSizeInCellsX();
  x0 = 0;
  yn = directed_layer->getSizeInCellsY();
  y0 = 0;


  // Pripravimo si arraye, kamor bomo shranjevali potrebne podatke

    // cost: Array, kamor bomo zapisovali ceno za potovanje do točke
    // Na začetku jo napolnimo z največjim številom
    unsigned int cost[xn*yn];
    std::fill(cost, cost+xn*yn, std::numeric_limits<unsigned int>::max()-1);

    // visited: Array, kamor bomo zapisali, ali smo določeno točko že obiskali z pathplanning algoritmom
    // 0 -> točka ni bila obiskana
    // 1 -> točko smo obiskali
    unsigned char visited[xn*yn];
    std::fill(visited, visited+xn*yn, 0);

    // path: Array vectorjev. Vectorji predstavljajo zaporedje vozlišč po katerih pridemo do vozlišča
    std::vector <unsigned int> path[xn*yn];

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    // Spremenljivke za shranjevanje cilja, začetka in trenutne lokacije
    unsigned int current_index;
    unsigned int current_mx;
    unsigned int current_my;
    unsigned int goal_index;

    unsigned int start_mx;
    unsigned int start_my;

    directed_layer->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);

    unsigned int goal_mx;
    unsigned int goal_my;

    directed_layer->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my);

    current_index = directed_layer->getIndex(start_mx, start_my);
    goal_index = directed_layer->getIndex(goal_mx, goal_my);

    // Začetno lokacijo označimo, kot obiskano, s ceno 0
    visited[current_index] = 1;
    path[current_index].push_back(current_index);
    cost[current_index] = 0;

    current_mx = start_mx;
    current_my = start_my;

    // Spremenljivke za shranjevanje premike, in ceno premikov
    unsigned int xu, xd, yu, yd;
    unsigned int movement_costs[4];


    unsigned int new_index;
    unsigned int dx, dy;
    float heuristic;
    unsigned int heuristic_;

  // Loop dokler ne dosežemo cilja
  while (current_index!=goal_index){
    // Preverimo da nismo ob robu grida
    if (current_mx+1<=xn-1){
    // Izračunamo kakšna je cena premika v desno
    xu = directed_layer->getIndex(current_mx+1, current_my);

    if (current_mx+1>goal_mx){
      dx = current_mx+1-goal_mx;
    }
    else dx=goal_mx-current_mx-1;
    if (current_my>goal_my){
      dy = current_my-goal_my;
    }
    else dy=goal_my-current_my;

    heuristic = std::pow((double)dx, 2.0) + std::pow((double)dy, 2.0);
    heuristic_ = (unsigned int)heuristic;

    // Preverimo, če je cena 100 (tam je ovira)
    if (cost_static[xu]>250){
      movement_costs[0] = std::numeric_limits<unsigned int>::max();
    }
    else{
    movement_costs[0] = cost[current_index]+directed_costmap_influence*cost_directed_xu[xu]+cost_static[xu]+heuristic_function_influence*heuristic_;
  }
    // Če je cena boljša od prejšne, cene updatamo
    if (movement_costs[0] < cost[xu]){
      cost[xu] = movement_costs[0];
      path[xu] = path[current_index];
      path[xu].push_back(xu);
    }
    }
    else{
      movement_costs[0] = std::numeric_limits<unsigned int>::max();
    }

    // Ponovimo za ostale smeri
    if (current_mx>0){
    xd = directed_layer->getIndex(current_mx-1, current_my);

    if (current_mx-1>goal_mx){
      dx = current_mx-1-goal_mx;
    }
    else dx=goal_mx-current_mx+1;
    if (current_my>goal_my){
      dy = current_my-goal_my;
    }
    else dy=goal_my-current_my;

    heuristic = std::pow((double)dx, 2.0) + std::pow((double)dy, 2.0);
    heuristic_ = (unsigned int)heuristic;


    if (cost_static[xd]>250){
      movement_costs[1] = std::numeric_limits<unsigned int>::max();
    }
    else{
    movement_costs[1] = cost[current_index]+directed_costmap_influence*cost_directed_xd[xd]+cost_static[xd]+heuristic_function_influence*heuristic_;
    }
    if (movement_costs[1] < cost[xd]){
      cost[xd] = movement_costs[1];
      path[xd] = path[current_index];
      path[xd].push_back(xd);
    }
    }
    else{
      movement_costs[1] = std::numeric_limits<unsigned int>::max();
    }
    if (current_my+1<=yn-1){
    yu = directed_layer->getIndex(current_mx, current_my+1);

    if (current_mx>goal_mx){
      dx = current_mx-goal_mx;
    }
    else dx=goal_mx-current_mx;
    if (current_my+1>goal_my){
      dy = current_my+1-goal_my;
    }
    else dy=goal_my-current_my-1;

    heuristic = std::pow((double)dx, 2.0) + std::pow((double)dy, 2.0);
    heuristic_ = (unsigned int)heuristic;

    if (cost_static[yu]>250){
      movement_costs[2] = std::numeric_limits<unsigned int>::max();
    }
    else{
    movement_costs[2] = cost[current_index]+directed_costmap_influence*cost_directed_yu[yu]+cost_static[yu]+heuristic_function_influence*heuristic_;
  }
    if (movement_costs[2] < cost[yu]){
      cost[yu] = movement_costs[2];
      path[yu] = path[current_index];
      path[yu].push_back(yu);
    }
    }
    else{
      movement_costs[2] = std::numeric_limits<unsigned int>::max();
    }
    if (current_my>0){
    yd = directed_layer->getIndex(current_mx, current_my-1);

    if (current_mx>goal_mx){
      dx = current_mx-goal_mx;
    }
    else dx=goal_mx-current_mx;
    if (current_my-1>goal_my){
      dy = current_my-1-goal_my;
    }
    else dy=goal_my-current_my+1;

    heuristic = std::pow((double)dx, 2.0) + std::pow((double)dy, 2.0);
    heuristic_ = (unsigned int)heuristic;
    if (cost_static[yd]>250){
      movement_costs[3] = std::numeric_limits<unsigned int>::max();
    }
    else{
    movement_costs[3] = cost[current_index]+directed_costmap_influence*cost_directed_yd[yd]+cost_static[yd]+heuristic_function_influence*heuristic_;
  }
    if (movement_costs[3] < cost[yd]){
      cost[yd] = movement_costs[3];
      path[yd] = path[current_index];
      path[yd].push_back(yd);
    }
    }
    else{
      movement_costs[3] = std::numeric_limits<unsigned int>::max();
    }

    // Najdemo naslednje vozlišče (tisto, ki ima najnižjo ceno)
    new_index = 0;
    unsigned int best_cost = std::numeric_limits<unsigned int>::max();
    for (int i = 0; i<xn*yn; i++){
      if (!visited[i])
        if (cost[i]<best_cost){
          new_index=i;
          best_cost = cost[i];
        }
    }
    //ROS_INFO_STREAM(best_cost);
    visited[new_index]=1;
    current_index = new_index;
    directed_layer->indexToCells(current_index, current_mx, current_my);
  }

// vhodno spremenljivko plan napolnimo z najdeno potjo
plan.push_back(start);
geometry_msgs::PoseStamped pose = goal;
tf2::Quaternion goal_quat;
goal_quat.setRPY(0, 0, 0.0);

double pose_x;
double pose_y;

for(std::vector<unsigned int>::iterator it = path[goal_index].begin(); it != path[goal_index].end(); ++it) {
    directed_layer->indexToCells(*it, current_mx, current_my);

    directed_layer->mapToWorld(current_mx, current_my, pose_x, pose_y);
    pose.pose.position.x = (float) pose_x;
    pose.pose.position.y = (float) pose_y;

    pose.pose.orientation.x = goal_quat.x();
    pose.pose.orientation.y = goal_quat.y();
    pose.pose.orientation.z = goal_quat.z();
    pose.pose.orientation.w = goal_quat.w();

    plan.push_back(pose);
}

// Ker je bila pot uspešno najdena vrnemo true
bool done = true;
return (done);
}

}
