#!/usr/bin/python
import time

class Map(object):
    OBSTACLES_LIFESPAN = 0.5 # max time before in seconds before being considered too old
    Robot = None

    Enemies = []
    BeltPoints = []
    LidarObjects = []

    @staticmethod
    def updateEnemies(new_enemies):
        Map.Enemies = new_enemies

    @staticmethod
    def updateBeltPoints(new_belt_points):
        Map.BeltPoints = new_belt_points

    @staticmethod
    def updateLidarObjects(new_lidar_objects):
        Map.LidarObjects = new_lidar_objects

    @staticmethod
    def toList():
        return Map.Enemies + Map.BeltPoints + Map.LidarObjects

    @staticmethod
    def garbageCollect():
        current_time = time.time()
        for obstacle in Map.BeltPoints:
            if current_time - obstacle.SpawnTime > Map.OBSTACLES_LIFESPAN:
                Map.BeltPoints.remove(obstacle)

        for obstacle in Map.LidarObjects:
            if current_time - obstacle.SpawnTime > Map.OBSTACLES_LIFESPAN:
                Map.BeltPoints.remove(obstacle)

        for obstacle in Map.Enemies:
            if current_time - obstacle.SpawnTime > Map.OBSTACLES_LIFESPAN:
                Map.BeltPoints.remove(obstacle)
