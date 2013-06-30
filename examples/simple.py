import cyni2
import numpy as np
import Image

def depthMapToImage(image):
    return np.uint8(image / (np.max(image)*1.0/255))

cyni2.initialize()
deviceList = cyni2.enumerateDevices()
device = cyni2.Device(deviceList[0]['uri'])
device.open()
depthStream = device.createStream("depth", fps=30)
colorStream = device.createStream("color", fps=30)
depthStream.start()
colorStream.start()
depthFrame = depthStream.readFrame()
colorFrame = colorStream.readFrame()
Image.fromarray(colorFrame.data).save("color.png")
Image.fromarray(depthMapToImage(depthFrame.data)).save("depth.png")
