# Description : AITimer

This simple node keeps track of the match's time.
Once triggered, publishes the time elapsed, time left and whether the game is finished in a single message.

## Service commands

The timer can handle these service requests :
- `timer_set_duration` : An external node sets the math duration (__NOTE__ may be changed : the timer would get the duration alone).
- `timer_start` : Start the timer.

## Publisher

The timer publishes the `ai_timer.msg` topic message with the following fields :
- `time_elapsed`
- `time_left`
- `is_finished`
