#!/usr/bin/python
import time


class GameTimer():
    def __init__(self):
        self.game_duration = -1  # Holds the match duration.
        self.started = False     # Set to true when Timer is triggered.

        self._start_time = -1

    def start(self):
        self._start_time = time.time() * 1000
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
