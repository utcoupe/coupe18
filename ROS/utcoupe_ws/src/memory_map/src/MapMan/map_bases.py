#!/usr/bin/python
import rospy

class MapElement(object):
    def get(self, requestpath):
        raise NotImplementedError("This is the super class. Needs to be overwritten from the child class.")
    def set(self, parameter_list):
        raise NotImplementedError("This is the super class. Needs to be overwritten from the child class.")


class ListManager(MapElement):
    def __init__(self, classdef, initdict):
        self.Elements = []
        for k in initdict.keys():
            self.Elements.append(classdef(initdict[k]))


class DictManager(MapElement):
    def __init__(self, elemdict):
        self.Dict = elemdict
        for k in self.Dict.keys():
            if isinstance(k, dict):
                raise ValueError("Inner dicts as values NOT allowed. '{}' has a dict inside.".format(k))

    def get(self, requestpath):
        keyname = requestpath.getNextKey()
        if keyname in self.Dict.keys():
            if requestpath.isLast(): return self.Dict[keyname]
            else: return self.Dict[keyname].get(requestpath)
        rospy.logerr("    ERROR Couldn't find request path key '{}'.".format(keyname))


'''
    Simple Python dict manager with get and set methods for accessing it from the map.
    CAUTION : The dict is NOT allowed to have inner dicts as values. This is a python
    restriction that does not allow us to set an inner dict from the set method.
'''
'''
class NakedDictManager(MapElement):
    def __init__(self, nakeddict):
        self.Dict = nakeddict
        # TODO Check if no inner dicts
'''


class RequestPath():
    def __init__(self, pathstring):
        self.pathstring = pathstring
        if pathstring[0] != '/':
            raise ValueError("ERROR Request path must start with '/' by convention.")
        if "/" in pathstring:
            self.Keys = [p for p in pathstring.split('/') if p != '']
            self.Counter = -1
        else:
            raise ValueError("ERROR Request path must contain at least one separator '/'.")

    def getNextKey(self):
        if self.Counter < len(self.Keys) - 1:
            self.Counter += 1
            return self.Keys[self.Counter]
        else:
            raise ValueError("ERROR Not enough levels in path ! Trying to reach farther than the last path key.")

    def isLast(self):
        return self.Counter == len(self.Keys) - 1

    def __repr__(self):
        return self.pathstring
