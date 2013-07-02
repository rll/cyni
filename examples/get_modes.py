import cyni
import numpy as np
import Image

cyni.initialize()
device = cyni.getAnyDevice()
device.open()
print "Depth:"
print device.getSupportedVideoModes("depth")
print "Color:"
print device.getSupportedVideoModes("color")
