#!/usr/bin/python


class MapElement(object):
    def get(self, parameter_list):
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
        self.Elements = elemdict


class NakedDictManager(MapElement):
    '''
    Simple Python dict manager with get and set methods for accessing it from the map.
    CAUTION : The dict is NOT allowed to have inner dicts as values. This is a python
    restriction that does not allow us to set an inner dict from the set method.
    '''
    def __init__(self, nakeddict):
        self.Dict = nakeddict
        # TODO Check if no inner dicts
