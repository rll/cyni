from libcpp.string cimport string
from libcpp.vector cimport vector
from cython.operator cimport dereference as drf
from cython cimport sizeof
cimport c_openni2
include "config.pxi"

cimport numpy as np
import numpy as np
import sys

class OpenNIException(Exception):
  pass

def warning(*args):
  sys.stderr.write("Warning: " + ' '.join(map(str,args)) + '\n')

def error(*args):
  #sys.stderr.write(' '.join(map(str,args)) + '\n')
  raise OpenNIException(' '.join(map(str,args)))

def initialize():
  return c_openni2.initialize()

def enumerateDevices():
  cdef c_openni2.Array[c_openni2.DeviceInfo] c_devices 
  c_openni2.enumerateDevices(&c_devices)
  devices = []
  for i in range(c_devices.getSize()):
    d = {}
    d['name'] = c_devices[i].getName()
    d['uri'] = c_devices[i].getUri()
    d['vendor'] = c_devices[i].getVendor()
    d['usbVendorId'] = c_devices[i].getUsbVendorId()
    d['usbProductId'] = c_devices[i].getUsbProductId()
    devices.append(d)
  return devices

cdef class Device(object):

  cdef c_openni2.Device _device
  cdef bytes _uri
  cdef vector[c_openni2.VideoStream*] _streams

  def __dealloc__(self):
    for _stream in self._streams:
      _stream.stop()
      _stream.destroy()
    if self._device.isValid():
      self._device.close()

  def __init__(self, uri):
    self._uri = uri
 
  def open(self, syncFrames=None):
   self._device.open(self._uri)
   if syncFrames is not None:
     self.setDepthColorSyncEnabled(syncFrames)

  def setDepthColorSyncEnabled(self, on=True):
    status = self._device.setDepthColorSyncEnabled(on)
    if status != c_openni2.STATUS_OK:
      error("Couldn't set Depth and Color syncrhonization to %s" % on)
      return False
    return True
  
  def getDepthColorSyncEnabled(self):
    return self._device.getDepthColorSyncEnabled()

  def setImageRegistrationMode(self, mode):
      if mode == "off":
          if self._device.isImageRegistrationModeSupported(c_openni2.IMAGE_REGISTRATION_OFF):
              status = self._device.setImageRegistrationMode(c_openni2.IMAGE_REGISTRATION_OFF)
          else:
              warning("ImageRegistrationMode %s isn't supported." % mode)
              return False
      elif mode == "depth_to_color":
          if self._device.isImageRegistrationModeSupported(c_openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR):
              status = self._device.setImageRegistrationMode(c_openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
          else:
              warning("ImageRegistrationMode %s isn't supported." % mode)
              return False
      return status == c_openni2.STATUS_OK

  def getImageRegistrationMode(self):
      mode = self._device.getImageRegistrationMode()
      if mode == c_openni2.IMAGE_REGISTRATION_OFF:
          return "off"
      elif mode == c_openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR:
          return "depth_to_color"
 
  def createStream(self, streamType, x=None, y=None, fps=None, format=None):
    stream = VideoStream() 
    stream.create(self._device, streamType, x, y, fps, format)
    self._streams.push_back(&(stream._stream))
    return stream
 
  def getSerial(self):
    cdef int size = 1024
    cdef char serial[1024]
    self._device.getProperty(c_openni2.ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &serial, &size)
    return <bytes> serial
  
  def close(self):
    if self._device.isValid():
      self._device.close()

class Frame(object):
  def __init__(self, data, metadata, sensorType):
    self.data = data
    self.metadata = metadata
    self.sensorType = sensorType

cdef class VideoStream(object):
  cdef c_openni2.VideoStream _stream
  cdef string _streamType
  cdef string _pixelFormat
  cdef int frameSize

  cdef readonly int frameWidth
  cdef readonly int frameHeight
  cdef readonly int fps

  def __dealloc__(self):
    self._stream.stop()
    self._stream.destroy()

  pixelFormats = { 
                  "rgb": c_openni2.PIXEL_FORMAT_RGB888,
                  "yuv422": c_openni2.PIXEL_FORMAT_YUV422,
                  "gray16": c_openni2.PIXEL_FORMAT_GRAY16,
                  "depth1mm": c_openni2.PIXEL_FORMAT_DEPTH_1_MM,
                  "depth100um": c_openni2.PIXEL_FORMAT_DEPTH_100_UM,
                 }

  pixelFormatsReverse = dict([[v,k] for k,v in pixelFormats.items()])

  cdef create(self, 
              c_openni2.Device& _device, 
              streamType, 
              frameWidth,
              frameHeight,
              fps,
              pixelFormat):
    self._streamType = streamType
    if streamType == 'color':
      status = self._stream.create(_device, c_openni2.SENSOR_COLOR)
    elif streamType == 'depth':
      status = self._stream.create(_device, c_openni2.SENSOR_DEPTH)
    if status != c_openni2.STATUS_OK:
      error("Error opening %s stream." % self.streamType)

    cdef const c_openni2.SensorInfo* _info = &self._stream.getSensorInfo()
    cdef const c_openni2.Array[c_openni2.VideoMode]* _modes = &(_info.getSupportedVideoModes())
    foundMode = False
    if self._streamType == b"color" and pixelFormat != "rgb":
      if pixelFormat is None:
        pixelFormat = "rgb"
      else:
        error("Only RGB currently supported for color streams.")
        self.destroy()
        return

    for i in range(_modes.getSize()):
      mode = drf(_modes)[i]
      if frameWidth is not None and frameWidth != mode.getResolutionX():
        continue
      else:
        chosenX = mode.getResolutionX()
      if frameHeight is not None and frameHeight != mode.getResolutionY():
        continue
      if fps is not None and fps != mode.getFps():
        continue
      if (pixelFormat is not None and 
          VideoStream.pixelFormats[pixelFormat] != mode.getPixelFormat()):
        continue

      # Set the pixel format in case it was None
      pixelFormat = VideoStream.pixelFormatsReverse[mode.getPixelFormat()]

      if pixelFormat == "rgb":
        pixelSize = sizeof(c_openni2.RGB888Pixel)
      elif pixelFormat == "yuv422":
        pixelSize = sizeof(c_openni2.YUV422DoublePixel)
      elif pixelFormat == "depth1mm":
        pixelSize = sizeof(c_openni2.DepthPixel)
      elif pixelFormat == "depth100um":
        pixelSize = sizeof(c_openni2.DepthPixel)
      elif pixelFormat == "gray16": 
        pixelSize = sizeof(c_openni2.Grayscale16Pixel)

      self._pixelFormat = pixelFormat

      self.frameWidth = mode.getResolutionX()
      self.frameHeight = mode.getResolutionY()
      self.fps = mode.getFps()

      self.frameSize = self.frameWidth * self.frameHeight * pixelSize

      self._stream.setVideoMode(mode)

      foundMode = True

      break

    if not foundMode:
      error("Couldn't find supported mode for %s stream." % streamType)
      self.destroy()

  def start(self):
    status = self._stream.start()
    if status != c_openni2.STATUS_OK:
      error("Error starting %s stream." % self.streamType)
      self.destroy()
      return False
    return True

  def readFrame(self):
    if not self._stream.isValid():
      error("Stream is invalid.")
      return None

    cdef c_openni2.VideoFrameRef _frame
    self._stream.readFrame(&_frame)
    if not _frame.isValid():
      error("Invalid frame read.")
      return None

    if _frame.getDataSize() != self.frameSize:
      error("Read frame with wrong size. Got height: %d, width: %d" 
            % (_frame.getHeight(), _frame.getWidth()))
      return None

    metadata = {}
    metadata["timestamp"] = _frame.getTimestamp()
    metadata["frameIndex"] = _frame.getFrameIndex()

    if self._streamType == b"color":
      if self._pixelFormat == b"rgb":
        data = self.convertRGBFrame(_frame)
        return Frame(data, metadata, self._streamType)
      else:
        error("Only RGB currently supported for color streams.")
        return None
    elif self._streamType == b"depth":
      data = self.convertDepthFrame(_frame)
      return Frame(data, metadata, self._streamType)

  cdef convertRGBFrame(self, c_openni2.VideoFrameRef _frame):
    _imageData = <const c_openni2.RGB888Pixel*> _frame.getData()
    cdef np.ndarray[np.uint8_t, ndim=3] image 
    image = np.empty((self.frameHeight, self.frameWidth, 3), dtype=np.uint8)
    for y in range(self.frameHeight):
      for x in range(self.frameWidth):
        _pixel = <const c_openni2.RGB888Pixel> _imageData[y*self.frameWidth + x]
        image[y,x,0] = _pixel.r
        image[y,x,1] = _pixel.g
        image[y,x,2] = _pixel.b
    return image

  cdef convertDepthFrame(self, c_openni2.VideoFrameRef _frame):
    _imageData = <const c_openni2.DepthPixel*> _frame.getData()
    cdef np.ndarray[np.uint16_t, ndim=2] image 
    image = np.empty((self.frameHeight, self.frameWidth), dtype=np.uint16)
    for y in range(self.frameHeight):
      for x in range(self.frameWidth):
        image[y,x] = _imageData[y*self.frameWidth + x]
    return image

  def stop(self):
    self._stream.stop()

  def destroy(self):
    self._stream.stop()
    self._stream.destroy()

  def setMirroring(self, on=True):
    self._stream.setMirroringEnabled(on)

  IF HAS_EMITTER_CONTROL == 1:

    def setEmitterState(self, on=True):
      if self._streamType == b"depth":
        self._stream.setEmitterEnabled(on)
      else:
        warning("Can only control emitter for depth sensors.")

def getAnyDevice():
    deviceList = enumerateDevices()
    return Device(deviceList[0]['uri'])

def depthMapToImage(image):
    return np.uint8(image / (np.max(image)*1.0/255))
