from map_loader import MapLoader, LoadChecker
from config import Config
from robot import Robot
from zone import Zone
from feature import Feature
from object import Object

class Map(object):
    CONFIG = Config()

    MAP_FEATURE = None

    ROBOTS      = []
    WAYPOINTS   = []
    ZONES       = []
    OBJECTS     = []

    @staticmethod
    def load_config():
        config_xml = MapLoader.loadFile(MapLoader.CONFIG_FILE)
        Map.CONFIG.load(config_xml)

        LoadChecker.checkNodesExist(config_xml, "map")
        Map.MAP_FEATURE = Feature(config_xml.find("map").find("feature"))

    @staticmethod
    def load_robots(robot_name):
        robots_xml = MapLoader.loadFile(MapLoader.ROBOTS_FILE)

        for robot in robots_xml.findall("robot"):
            r = Robot(robot)
            if r.name == robot_name:
                r.active = True
            Map.ROBOTS.append(r)


    @staticmethod
    def load_objects(team_name):
        classes_xml = MapLoader.loadFile(MapLoader.CLASSES_FILE)
        objects_xml = MapLoader.loadFile(MapLoader.OBJECTS_FILE)

        for z in classes_xml.find("zones").findall("zone"):
            Map.ZONES.append(Zone(z))

        classes = []
        for c in classes_xml.find("objects").findall("object"):
            classes.append(Object(c))
