# directed_layer package

Costmap_2d layer that stores information about direction movement bias.
DirectedLayer was created with option to be used with other conventional
planners and controllers without having to remove it from costmap. Individual
directed costmaps are stored in separate pointers and do not interfere with
Costmap2d::costmap_

DirectedLayer stores 4 directed costmaps with movement bias data:
 - DirectedMapXu : movement cost into position, if new position has increased mx
 - DirectedMapXd : movement cost into position, if new position has decreased mx
 - DirectedMapYu : movement cost into position, if new position has increased my
 - DirectedMapYd : movement cost into position, if new position has decreased my


## unique methods

For planners and controllers to access directed costmaps, methodes unique to DirectedLayer are used

### unsigned char* getDirectedMapXu

Returns pointer to directed map xu

### unsigned char* getDirectedMapXd

Returns pointer to directed map xd

### unsigned char* getDirectedMapYu

Returns pointer to directed map yu

### unsigned char* getDirectedMapYd

Returns pointer to directed map yd

### double getDirectedMapResolution

Returns resolution of directed map in meter/cell

### unsigned int getDirectedMapIndex

Returns index of directed map

### directedMapToWorld

Transforms directed map coordinates to world coordinates
