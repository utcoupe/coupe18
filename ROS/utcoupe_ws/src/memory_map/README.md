# Description

This is the Map Manager package. It is essentially the database that holds all the information, characteristics and statuses of all the elements
in the robot's environment. This includes the objects' positions, containers and the robot's containers.

This package temporarily holds the RViz config file. In order to use it, open the file from RViz __before__ launching the node itself
 (`rosrun memory_map map_node.py`). If not launched, the node will not publish its markers.

# Services

The package is aimed to offer three services called `memory/map/get`, `memory/map/set` and `memory/map/conditions`:
- The `get` service will let other packages retrieve the statuses of all elements in the database. This includes position, number of sub-elements
in a container, current speed of the enemy and so on.
- The `set` service is the way to update the database. For instance, a recognizer from the `Perception` Namespace will repeatedly
update the statuses of its designated objects.
- The `conditions` service will let packages easily test pre-implemented conditions on objects, containers and entities. For instance, the AI could
use this service to check if there are enough cubes in the robot's container before unloading them on the table.

### Getting a value

The node can only return a full object's dict or a single attribute value for now. In order to use it, create a path string defining the path in the database
until the specified attribute or object. For instance, in order to get the X position of the `cube_1` object, create a `MapGet` service request with 
`/objects/cube_1/position/x` as the `request_path`. Typing `/objects/cube_1/position` will return the full position dict.

__NOTE__ : For now, you can not get a dict with other dicts inside. For instance, you cannot get a full JSON description of a Map object (i.e. its position, 
shape, visual, etc). You must ask for a dict which only has simple values, not dicts (e.g. the position attribute).

### Setting a value

For now, one SET request can only set one specific value at a time. It will soon support replacing multiple values inside a dict, or even automatic modifications 
based on filters (e.g. change the color of all blue balls).

Use the same request path structure as the GET service. The last key you indicate will be the one replaced by your `new_value`. Trying to set a new value to an
entire dict will return an error (for now).

# Topics

This package will soon publish an image feed of the OccupancyGrid. `navigation_pathfinder` will be the main subscriber package. (__TODO__ or publish
a dictionary dedicated to physical obstacles. This would reduce the amount of traffic and memory usage if `navigation_pathfinder` is the only
subscriber).
