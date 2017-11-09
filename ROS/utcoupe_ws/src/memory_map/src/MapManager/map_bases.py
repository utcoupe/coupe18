#!/usr/bin/python
import rospy

class MapElement(object):
    def get(self, requestpath):
        raise NotImplementedError("This is the super class. Needs to be overwritten from the child class.")
    def set(self, requestpath, new_value):
        raise NotImplementedError("This is the super class. Needs to be overwritten from the child class.")


class ListManager(MapElement): # Not used for now
    def __init__(self, classdef, initdict):
        self.Elements = []
        for k in initdict.keys():
            self.Elements.append(classdef(initdict[k]))


class DictManager(MapElement):
    def __init__(self, elemdict):
        self.Dict = elemdict
        for k in self.Dict.keys():
            if isinstance(k, dict):
                raise ValueError("Inner dicts as values NOT allowed. '{}' has a dict inside. Must be initialized beforehand.".format(k))

    def toList(self):
        return self.Dict.values()

    def toDict(self, recursive = False):
        if recursive:
            d = {}
            for item in self.Dict:
                if isinstance(self.Dict[item], DictManager):
                    d[item] = self.Dict[item].toDict(recursive = True)
                else:
                    d[item] = self.Dict[item]
            return d
        else:
            for k in self.Dict.values():
                if isinstance(k, DictManager):
                    rospy.logerr(("    ERROR Trying to transform a DictManager that has other DictManagers inside."
                                " Use '*' at the end to get a recursively-constructed subdict."))
                    return None
            return self.Dict

    def get(self, requestpath):
        if isinstance(requestpath, str):
            requestpath = RequestPath(requestpath)
            if not requestpath.Valid: return None
        keyname = requestpath.getNextKey()

        if requestpath.isLast():
            if keyname in self.Dict.keys():
                return self.Dict[keyname]
            elif keyname == '*':
                return self.toDict(recursive = True)
            rospy.logerr("    GET Request failed : Unrecognized last key dict operator '{}'.".format(keyname))
        else:
            if keyname in self.Dict.keys():
                if isinstance(self.Dict[keyname], DictManager):
                    return self.Dict[keyname].get(requestpath)
                rospy.logerr("    GET Request failed : '{}' key points to '{}' object, not dict. Can't reach the next key(s).".format(keyname, type(self.Dict[keyname])))
            else:
                rospy.logerr("    GET Request failed : Couldn't find request path key '{}'.".format(keyname))

    def set(self, requestpath, new_value):
        if isinstance(requestpath, str): # TODO remove for set ?
            requestpath = RequestPath(requestpath)
            if not requestpath.Valid: return None
        keyname = requestpath.getNextKey()
        if keyname in self.Dict.keys():
            if requestpath.isLast():
                if not isinstance(self.Dict[keyname], DictManager):
                    self.Dict[keyname] = new_value
                    return True
                else:
                    raise ValueError("    SET Request failed : Can't SET a whole DictManager to a new value. Aborting.")
            else: return self.Dict[keyname].set(requestpath, new_value)
        rospy.logerr("    SET Request failed : Couldn't find request path key '{}'.".format(keyname))


class RequestPath():
    def __init__(self, pathstring):
        self.pathstring = pathstring

        self.Valid = True
        if not len(pathstring):
            rospy.logerr("Invalid path : empty path.")
            self.Valid = False
        if pathstring[-1] == '/':
            rospy.logerr("Invalid path : must not end with '/'.")
            self.Valid = False
        self.Keys = [p for p in pathstring.split('/') if p != '']
        self.Counter = -1

    def getNextKey(self):
        if self.Counter < len(self.Keys) - 1:
            self.Counter += 1
            return self.Keys[self.Counter]
        else:
            raise ValueError("ERROR Not enough levels in path ! Can't reach farther than the last path key.")

    def isLast(self):
        return self.Counter == len(self.Keys) - 1

    def __repr__(self):
        return self.pathstring
