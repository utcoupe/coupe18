uint8 LEVEL_STOP      = 0  # Collision will happen very soon, stop the robot!
uint8 LEVEL_DANGER    = 1  # Path needs to be modified or a collision will happen.
uint8 LEVEL_POTENTIAL = 2  # A dynamic object will probably pass in some time, maybe not (replan optional).
uint8 danger_level

# to be continued