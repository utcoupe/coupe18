uint8 STATUS_INITIALIZING = 0
uint8 STATUS_INITIALIZED  = 1
uint8 STATUS_INGAME       = 2
uint8 STATUS_HALT         = 3
uint8 game_status

string[] pending_nodes # Node names that didn't notify they are init or not yet.
string[] ready_nodes   # Node names that responded they initialized correctly.
string[] failed_nodes  # Node names that responded they couldn't init.