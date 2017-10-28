#!/usr/bin/python
import rospy

class MapPath(object):
    '''
    Class that manages a path given from a get or set request.
    '''

    def __init__(self, stringpath):
        self.i = -1
        self.Keys = [self.parseKey(k) for k in stringpath.split('/') if k != '']

    def parseKey(self, keystring):
        return MapPathKey(keystring)  # TODO implement string to python object

    def getNextKey(self):
        self.i += 1
        if self.i < len(self.Keys):
            return self.Keys[self.i]
        else:
            rospy.logerr("[memory/map] MapPath too short ! Asked to access inexistent key")
            return None

# TODO not implemented yet
class MapPathKey(object):
    def __init__(self, keystring):
        print keystring
        if keystring.count('.') != 1:
            pass#rospy.logerr("Request path invalid ! '{}' needs to be of type 'key.extension'.".format(keystring))
            #raise ValueError
        self.KeyName, self.Extension = keystring.split('.') if keystring.count('.') == 1 else (keystring, None)

    def __repr__(self):
        return str(self.KeyName) + '.' + str(self.Extension)


class GetFilter(object):
    '''
    Base class for handling get filters.
    Filters are useful for conducting complex get and set requests.abs
        E.g. get the closest red cube from the robot's gripper position.
    '''

    def __init__(self):
        pass
