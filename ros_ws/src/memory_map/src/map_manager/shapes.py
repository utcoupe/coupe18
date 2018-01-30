class Shape(object):
    def __init__(self, xml):
        self.type = xml.attrib["type"]
