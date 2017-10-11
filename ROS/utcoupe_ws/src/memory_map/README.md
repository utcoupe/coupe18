# Description

This is the Map Manager package. It is essentially the database that holds all the information, characteristics and statuses of all the elements
in the robot's environment. This includes the objects' positions, containers as well as the ones defined in the robot itself.

This package temporarily holds the RViz config file. In order to use it, open the file from RViz before launching the node itself
 (`rosrun memory_map map_node.py`).

__NOTE__ : This package currently requires the installation of `pyclipper` (`pip install pyclipper`), that lets us inflate 2D shapes by a desired amount.
This is useful for generating an OccupancyGrid, that will later be sent to the pathfinder.

# Services

The package is aimed to offer three services called `memory/map/get`, `memory/map/set` and `memory/map/conditions`:
- The `get` service will let other packages retrieve the statuses of all elements in the database. This includes position, number of sub-elements
in a container, current speed of the enemy and so on.
- The `set` service is the way to update the database. For instance, a recognizer from the `Perception` Namespace will repeatedly
update the statuses of its designated objects.
- The `conditions` service will let packages easily test pre-implemented conditions on objects, containers and entities. For instance, the AI could
use this service to check if there are enough cubes in the robot's container before unloading them on the table.

# Topics

This package will soon publish an image feed of the OccupancyGrid. `navigation_pathfinder` will be the main subscriber package. (__TODO__ or publish
a dictionary dedicated to physical obstacles. This would reduce the amount of traffic and memory usage if `navigation_pathfinder` is the only
subscriber).
