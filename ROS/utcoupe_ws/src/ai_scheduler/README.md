#Description : AIScheduler

This package takes care of choosing what actions to take based on an `actions` definition list.
The definition files are located under `src/Definitions`.

This package also contains the main launch files that start all system nodes.
In order to launch the main program with all system nodes, run `roslaunch robot_ai_scheduler main.launch` after sourcing the workspace.

#AI Scheduler

The scheduler is a fully-recursive task manager and executer. Calling the `findNext()` function in the `Strategy` class
will recursively call the ActionLists and Actions until an Order is returned. This Order is then triggered with its function
`execute()`, which will send an `action` message to the node specified in the XML definition file.

__NOTE 1__ : The Definition files and main launch files are currently located in this package, but will soon be moved to the `Definitions` package.
The AI will retrieve these files during initialization.

__NOTE 2__ : For now, the orders are implemented as services. They will soon be sent as `action` messages. This will enable the AI to execute several
Orders at the same time.

__NOTE 3__ : Conditions (e.g. ask for the Map server if there are enough elements in a container) and parameters (e.g. position for a generic `goto` order) are
yet to be implemented.
