Inputs :

- robot's current position and speed *from drivers/ard_asserv or navigation/navigator*
- robot's state (driving, idle) and current path *from memory/map or navigation/navigator*
- belt dangerous objects and static objects *from processing/qmsk*
- enemies positions *from recognition/enemy_finder*

Outputs :

- publishes on a topic when a collision is predicted. navigator will subscribe to it and stop the robot when this message is published.

global GET request string must start with '/'.