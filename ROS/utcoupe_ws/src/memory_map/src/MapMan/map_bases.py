#!/usr/bin/python


class ListManager(object):
    def __init__(self, classdef, initdict):
        self.Elements = []
        for k in initdict.keys():
            self.Elements.append(classdef(k, initdict[k]))


class DictManager(object):
    def __init__(self, elemdict):
        self.Elements = elemdict
