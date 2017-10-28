#!/usr/bin/python
import rospy

class MapPath(object):
    '''
    Class that manages a path given from a get or set request.
    '''

    def __init__(self, stringpath):
        self.i = 0
        self.Keys = [self.parseKey(k) for k in stringpath.split('/') if k != '']

    def parseKey(self, keystring):
        return keystring  # TODO implement string to python object

    def getNext(self):
        self.i += 1
        return self.Keys[self.i]

# TODO not implemented yet
class MapPathKey(object):
    def __init__(self, keystring):
        if keystring.count('.') == 1:
            rospy.logerr("Request path invalid ! '{}' needs to be of type 'key.extension'.".format(keystring))
            raise ValueError
        self.KeyName, self.Extension = keystring.split('.')


class GetFilter(object):
    '''
    Base class for handling get filters.
    Filters are useful for conducting complex get and set requests.abs
        E.g. get the closest red cube from the robot's gripper position.
    '''

    def __init__(self):
        pass
