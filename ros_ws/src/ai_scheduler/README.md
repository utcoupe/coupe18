# Description : AIScheduler

This package takes care of choosing what actions to take based on an `actions` definition list.
The definition files are located under `src/Definitions`.

This package also contains the main launch files that start all system nodes.
In order to launch the main program with all system nodes, run `roslaunch robot_ai_scheduler main.launch` after sourcing the workspace.

## AI Scheduler

The scheduler is a fully-recursive task manager and executer. Calling the `findNext()` function in the `Strategy` class
will recursively call the ActionLists and Actions until an Order is returned. This Order is then triggered with its function
`execute()`, which will send an `action` message to the node specified in the XML definition file.

__NOTE 1__ : The Definition files and main launch files are currently located in this package, but will soon be moved to the `Definitions` package.
The AI will retrieve these files during initialization.

__NOTE 2__ : For now, the orders are implemented as services. They will soon be sent as `action` messages. This will enable the AI to execute several
Orders at the same time.

__NOTE 3__ : Conditions (e.g. ask for the Map server if there are enough elements in a container) and parameters (e.g. position for a generic `goto` order) are
yet to be implemented.

### Order definitions
The orders are defined in the file `3_Orders.xml`. The `<order>` node has to have a `ref` attribute, in order for it to be referenced from the outside. It can take an optional `duration` attribute, which is a manual estimation. This node must have a `<message>` child node, with a `dest` attributes, representing the service or action the message will be sent to. A message node holds multiple `<param>`, mirroring the structure of the ROS request message sent. Each `<param>` has a `name`, matching the name of the parameter in the ROS message, and a `type`, used to parse the parameter. A parameter can be `optional`, which means it doesn't have to be filled when the order is called (the default value will be put in by ROS). It can be `preset` : in that case, the parameter cannot be set when the order is called, the value is constant and cannot be changed. To finish, default values can be set in the order definition. Note that a param with a default value is therefore optional, and a param cannot be preset and optional.

#### Example
Let's assume we want to send a request to `/asserv/goto`, with the following ROS definition :
```
geometry_msgs/Pose2D position
float64 number
uint8 command
string message
```
Here is a valid example of the order definition, with all the elements seen above :
```xml
<order ref="goto" duration="1">
  <message dest="/asserv/goto">
    <param name="position" type="pose2d" />                   <!-- regular parameter, required -->
    <param name="number" type="float">42.8</param>            <!-- parameter with a default value -->
    <param name="command" type="int" preset="true">5</param>  <!-- preset parameter -->
    <param name="message" type="string" optional="true"/>     <!-- optional parameter -->
  </message>
</order>
```

### Order references
The orders can be referenced from the actions or strategies definition files. To reference an order, a node `<orderref>` has to have its `ref` attribute matching the `ref` attribute of the referenced order. As childs of the node, the parameters (non-preset) are set, with the tag of the child matching the name of the parameter.

#### Example
Let's continue with our defined order from above. Suppose we want to reference it, here is a valid way to do it :
```xml
<orderref ref="goto">
  <position>
    <x>55.2</x>
    <y>57.1</y>
    <theta>3.14159</theta>
  </position>
  <message>hello world!</message> <!-- this node is optional -->
</orderref>
```

### Adding new parameter types
In order to parse correctly all wanted type, one has to add a parser class, child of the `Param` class, for each type of parameter. Check out `ai_params.py` for more details.

