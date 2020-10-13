# directed_planner package

directed_planner package provides global path planning using directed map. For
pathfinding dijkstra or astar algorithms can be chosen.

### Parameters

* "use_dijkstra" (bool, default: true)
* "heuristic_function_influence" (double, default: 1.0)
* "directed_costmap_influence" (double, default: 1.0)
* "directed_layer_name" (string, default: "global_costmap/directed_map")

Directed costmap values are obtained from directed_layer. Directed_layer name must
be set accordingly.
