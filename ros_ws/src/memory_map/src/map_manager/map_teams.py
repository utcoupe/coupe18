import rospy
import map


class Team(object):
    def __init__(self, name, initdict):
        self.name = name
        self.default = bool(initdict["default"])

        self.transforms = []
        if "transforms" in initdict:
            self.transforms = [t for t in initdict["transforms"]]

    def swap(self):
        success = True
        if map.Map.CurrentTeam != self.name:
            for code in self.transforms:
                success = min(success, Transformer.transform(code))

        if success: rospy.loginfo("Swapped map to team '{}' successfully.".format(self.name))
        else: rospy.logerr("Couldn't swap map to team '{}'.".format(self.name))
        return success


class Transformer(object):
    @staticmethod
    def transform(code):
        codes = {
            "map_x_mirror": Transformer._map_x_mirror
        }
        return codes[code]()

    @staticmethod
    def _map_x_mirror():
        print "transforming x mirror"
        return False
