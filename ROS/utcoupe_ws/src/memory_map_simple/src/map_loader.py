#!/usr/bin/python
import rospy, os


class MapLoader():
    def load(self, filename):
        return self.loadYamlFromFile(filename)

    def loadYamlFromFile(self, filename):
        import yaml
        with open(os.path.dirname(__file__) + "/" + filename, 'r') as stream:
            try:
                return yaml.load(stream)
            except yaml.YAMLError as exc:
                print exc

    def loafYamlFromDescriptionModule(self, path):
        # TODO use this when Definitions package is ready instead of loading from file.
        pass