import cyni
import numpy as np
import Image
import time

cyni.initialize()
device = cyni.getAnyDevice()
device.open()
depthStream = device.createStream("depth", fps=30)
depthStream.start()

depthStream.setEmitterState(False)
depthFrame = depthStream.readFrame()
while np.sum(depthFrame.data) != 0:
    depthFrame = depthStream.readFrame()
Image.fromarray(cyni.depthMapToImage(depthFrame.data)).save("depth_off.png")

depthStream.setEmitterState(True)
depthFrame = depthStream.readFrame()
while np.sum(depthFrame.data) == 0:
    depthFrame = depthStream.readFrame()
# Read one more just to ensure the emitter is totally on
depthFrame = depthStream.readFrame()
Image.fromarray(cyni.depthMapToImage(depthFrame.data)).save("depth_on.png")
