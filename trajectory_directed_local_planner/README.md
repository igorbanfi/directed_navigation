# trajectory_directed_local_planner package

trajectory_directed_local_planner package provides local navigation of robot
considering local costmap and directed directed map.
trajectory_directed_local_planner::TrajectoryDirectedLocalPlanner adheres to
nav_core::BaseLocalPlanner interface found in [nav_core](http://wiki.ros.org/nav_core) package.

## Behavior

trajectory_directed_local_planner::TrajectoryDirectedLocalPlanner is a modified version of
[base_local_planner::TrajectoryPlannerROS](http://wiki.ros.org/base_local_planner#TrajectoryPlannerROS).
It works by determining if robot has a right of way. If Robot has a right of way
velocity computed by base_local_planner::TrajectoryPlannerROS is sent to robot, otherwise
0 velocity is sent to robot.

### Right of way

#### Path obstructed

If next point on global costmap is obstructed the robot will wait until it clears up.
Obstruction is determined by considering local costmap.

#### Path priority

In case next point is clear but its neighbours have a robot present right of way
is determined by priority. For each position on directed costmap a direction of
priority vector is calculated. Priority vector represents the least expensive
direction of movement from the node by considering directed costmap.

Path priority is calculated by determining direction of robot's approach vector
into node, and then calculating scalar product of robot's approach vector and
node's priority vector. If robot determines that another robot is present at
neighbouring node, it's approach vector is determined and subsequently it's
priority val is calculated.

Robot has right of way if it's priority val is the highest of all robots present.

##### Example
In the intersection red turtle has priority.
<img src="priority_example.gif" />

NOTE: due to possibility of deadlock in high density of robots, the timeout is implemented. Robot will
continue driving after random timeout even if has no right of way.

## Parameters
Additionally to parameters of base_local_planner::TrajectoryPlannerROS, parameters are:
* "check_occupation_size" (double, default: directed_map::resolution_)
* "directed_layer_name" (string, default: local/directed_map)

Directed costmap values are obtained from directed_layer. Directed_layer name must
be set accordingly.

## Example

Working example of trajectory_directed_local_planner::TrajectoryDirectedLocalPlanner working with directed_planner::DirectedPlanner.
<img src="example.gif" width="500"/>

NOTE: This local planner was created as a proof of concept. It only works, if
individual poses of global plan and directed costmap are spaced by
robots width + inflation. Global plan must not allow diagonal travel for
determining priority to work correctly.
