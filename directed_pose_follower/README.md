# trajectory_directed_local_planner package

directed_pose_follower package provides local navigation of robot
considering local costmap and directed directed map.
directed_pose_follower::DirectedPoseFollower adheres to
nav_core::BaseLocalPlanner interface found in [nav_core](http://wiki.ros.org/nav_core) package.

## Behavior

directed_pose_follower::DirectedPoseFollower is a modified version of
[pose_follower::PoseFollower](http://wiki.ros.org/pose_follower). Behavior is
identical to one described in [trajectory_directed_local_planner](trajectory_directed_local_planner/README.md).

NOTE: This local planner was created as a proof of concept. It only works, if
individual poses of global plan and directed costmap are spaced by
robots width + inflation. Global plan must not allow diagonal travel for
determining priority to work correctly.
