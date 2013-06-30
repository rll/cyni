import cyni
import numpy as np
import Image

cyni.initialize()
device = cyni.getAnyDevice()
device.open()
depthStream = device.createStream("depth", fps=30)
colorStream = device.createStream("color", fps=30)
depthStream.start()
colorStream.start()
depthFrame = depthStream.readFrame()
colorFrame = colorStream.readFrame()
Image.fromarray(colorFrame.data).save("color.png")
Image.fromarray(cyni.depthMapToImage(depthFrame.data)).save("depth.png")
