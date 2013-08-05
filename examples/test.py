import cyni
import numpy as np
import Image

cyni.initialize()
device = cyni.getAnyDevice()
device.open()
depthStream = device.createStream("depth", fps=30, width=640, height=480)
colorStream = device.createStream("color", fps=30, width=1280, height=960)
#colorStream = device.createStream("color", fps=30, width=640, height=480)
#device.setImageRegistrationMode("depth_to_color")
device.setDepthColorSyncEnabled(on=True)
depthStream.start()
colorStream.start()
colorFrame = colorStream.readFrame()
colorFrame = colorStream.readFrame()
colorFrame = colorStream.readFrame()
colorFrame = colorStream.readFrame()
depthFrame = depthStream.readFrame()
registered = cyni.registerColorImage(depthFrame.data, colorFrame.data, depthStream, colorStream)
Image.fromarray(colorFrame.data).save("color.png")
Image.fromarray(registered).save("registered.png")
Image.fromarray(cyni.depthMapToImage(depthFrame.data)).save("depth.png")
