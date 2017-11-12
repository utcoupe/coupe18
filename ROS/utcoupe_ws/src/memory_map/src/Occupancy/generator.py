#!/usr/bin/python
import os
import time
import rospy
from PIL import Image, ImageDraw

class OccupancyGenerator():
    def __init__(self, world):
        self.ImgWidth = 500 # Width of the generated images. Height will be calculated based on the map aspect ratio.
        self.WorldSize = (float(world.get("/terrain/shape/width")), float(world.get("/terrain/shape/height")))
        self.ImgSize = (self.ImgWidth, int(self.ImgWidth * (self.WorldSize[1] / self.WorldSize[0])))

    def generateTerrainImages(self, world):
        s = time.time() * 1000
        layers = world.get("/terrain/walls/^").toList()
        for layer in layers:
            self.generateStaticOccupancy(layer)
        rospy.loginfo("Generated static terrain images in {0:.2f}ms.".format(time.time() * 1000 - s))

    def generateStaticOccupancy(self, layer):
        img = Image.new("RGB", self.ImgSize)

        draw = ImageDraw.Draw(img)

        for wall in layer.toList():
            position, shape = wall.get("position/^").toDict(), wall.get("shape/^").toDict()

            pos = self.world_to_img_pos((position["x"], position["y"]))
            if shape["type"] == "rect":
                w, h = self.world_to_img_scale(shape["width"]), self.world_to_img_scale(shape["height"])
                draw.rectangle((pos[0], pos[1], pos[0] + w, pos[1] - h), fill=(255, 255, 0))
            elif shape["type"] == "circle":
                r = self.world_to_img_scale(shape["radius"])
                draw.ellipse((pos[0] - r, pos[1] - r, pos[0] + r, pos[1] + r), fill = (255, 0, 0))
            elif shape["type"] == "polygon":
                pass
            elif shape["type"] == "line":
                pass
            else:
                rospy.logerr("Occupancy generator could not recognize shape type '{}'.".format(shape["type"]))
        del draw

        img.save(os.path.dirname(__file__) + "/test.png", "PNG")

    def world_to_img_scale(self, world_coord):
        return world_coord * (self.ImgSize[0] / (self.WorldSize[0]))
    def world_to_img_pos(self, world_pos):
        return (self.world_to_img_scale(world_pos[0]), self.ImgSize[1] - self.world_to_img_scale(world_pos[1]))
