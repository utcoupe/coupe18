from feature import Feature


class Object(object):
    def __init__(self, xml):
        self.features = []
        for feature in xml.find("features").findall("feature"):
            self.features.append(Feature(feature))
