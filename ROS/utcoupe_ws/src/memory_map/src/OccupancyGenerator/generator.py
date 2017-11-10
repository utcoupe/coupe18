#!/usr/bin/python


class OccupancyGenerator():
    def __init__(self):
        pass

    def generateTerrainImages(self, world):
        layers = world.get("/terrain/walls").toDict(recursive = True)
        for layer in layers:
            pass

