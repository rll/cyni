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
colorFrame = colorStream.readFrame()
depthFrame = depthStream.readFrame()
cloud = cyni.depthMapToPointCloud(depthFrame.data, depthStream, colorFrame.data)
cyni.writePCD(cloud, "cloud.pcd")
readCloud = cyni.readPCD("cloud.pcd")
cyni.writePCD(readCloud, "cloud2.pcd", ascii=True)
