#!/usr/bin/python
import os
from PIL import Image, ImageDraw

class OccupancyGenerator():
    def __init__(self):
        self.ImgWidth = 400 # Width of the generated images. Height will be calculated based on the map aspect ratio.

    def generateTerrainImages(self, world):
        layers = world.get("/terrain/walls/^").toDict(recursive = True)
        self.generateStaticOccupancy(layers["layer_ground"])

    def generateStaticOccupancy(self, layer):
        img = Image.new("RGB", (400, 400))

        draw = ImageDraw.Draw(img)
        draw.line((10, 10, 100, 100), fill=(255, 0, 0), width=5)
        del draw

        img.save(os.path.dirname(__file__) + "/test.png", "PNG")


if __name__ == "__main__":
    generator = OccupancyGenerator()
    generator.generateStaticOccupancy("layer_ground")
