#!/usr/bin/python
import time


class GameTimer():
    def __init__(self, duration):
        self.game_duration = duration  # Holds the match duration.
        self.started = False           # Set to true when Timer is active.

        self._start_time = -1

    def reset(self):
        self._start_time = time.time() * 1000

    def start(self):
        self.reset()
        self.started = True

    def stop(self):
        self._start_time = -1
        self.started = False

    def elapsed_time(self):
        return (time.time() * 1000 - self._start_time) / 1000.0 # return elapsed time in seconds
    def time_left(self):
        return self.game_duration - self.elapsed_time()
    def is_finished(self):
        return True if self.time_left() < 0 else False
