from libcpp cimport bool
from libc.stdint cimport uint8_t, uint16_t, uint32_t, uint64_t 

cdef extern from "OniCProperties.h":
    ctypedef enum:
        ONI_DEVICE_PROPERTY_FIRMWARE_VERSION = 0 # By implementation
        ONI_DEVICE_PROPERTY_DRIVER_VERSION = 1 # OniVersion
        ONI_DEVICE_PROPERTY_HARDWARE_VERSION = 2 # int
        ONI_DEVICE_PROPERTY_SERIAL_NUMBER = 3 # string
        ONI_DEVICE_PROPERTY_ERROR_STATE = 4, # ??
        ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION = 5 # OniImageRegistrationMode

cdef extern from "OniEnums.h" namespace "openni":
    ctypedef enum Status:
        STATUS_OK = 0
        STATUS_ERROR = 1
        STATUS_NOT_IMPLEMENTED = 2
        STATUS_NOT_SUPPORTED = 3
        STATUS_BAD_PARAMETER = 4
        STATUS_OUT_OF_FLOW = 5
        STATUS_NO_DEVICE = 6
        STATUS_TIME_OUT = 102

    ctypedef enum PixelFormat:
        # Depth
        PIXEL_FORMAT_DEPTH_1_MM = 100
        PIXEL_FORMAT_DEPTH_100_UM = 101
        PIXEL_FORMAT_SHIFT_9_2 = 102
        PIXEL_FORMAT_SHIFT_9_3 = 103

        # Color
        PIXEL_FORMAT_RGB888 = 200
        PIXEL_FORMAT_YUV422 = 201
        PIXEL_FORMAT_GRAY8 = 202
        PIXEL_FORMAT_GRAY16 = 203
        PIXEL_FORMAT_JPEG = 204
        PIXEL_FORMAT_YUYV = 205

    ctypedef enum SensorType:
        SENSOR_IR = 1
        SENSOR_COLOR = 2
        SENSOR_DEPTH = 3

    ctypedef enum ImageRegistrationMode:
        IMAGE_REGISTRATION_OFF = 0
        IMAGE_REGISTRATION_DEPTH_TO_COLOR = 1

cdef extern from "OpenNI.h" namespace "openni":

    ctypedef struct RGB888Pixel:
        uint8_t r
        uint8_t g
        uint8_t b

    ctypedef struct YUV422DoublePixel:
        uint8_t u
        uint8_t y1
        uint8_t v
        uint8_t y2

    ctypedef uint16_t DepthPixel
    ctypedef uint16_t Grayscale16Pixel

    cdef cppclass Array[T]:
        Array()
        int getSize()
        const T& operator[](int)

    cdef cppclass DeviceInfo:
        const char* getUri()
        const char* getVendor()
        const char* getName()
        uint16_t getUsbVendorId()
        uint16_t getUsbProductId()

    cdef cppclass SensorInfo:
        SensorType getSensorType()
        const Array[VideoMode]& getSupportedVideoModes()

    cdef cppclass VideoMode:
        VideoMode()
        PixelFormat getPixelFormat()
        int getResolutionX()
        int getResolutionY()
        int getFps()

    cdef cppclass Device:
        Device()
        Status open(const char*) nogil
        void close()
        const DeviceInfo& getDeviceInfo() const
        bool hasSensor(SensorType)
        const SensorInfo* getSensorInfo(SensorType)
        Status getProperty(int propertyId, void* data, int* dataSize) const
        Status setProperty(int propertyId, const void* data, int dataSize)
        ImageRegistrationMode getImageRegistrationMode() const
        Status setImageRegistrationMode(ImageRegistrationMode mode)
        bool isImageRegistrationModeSupported(ImageRegistrationMode mode) const
        bool isValid() const
        Status setDepthColorSyncEnabled(bool isEnabled)
        bool getDepthColorSyncEnabled()
        bool isPropertySupported(int propertyId) const
        Status invoke(int commandId, void* data, int dataSize)
        bool isCommandSupported(int commandId) const

    cdef cppclass VideoStream:
            VideoStream()

            bool isValid() nogil const
            Status create(const Device& device, SensorType sensorType) nogil
            inline void destroy()

            const SensorInfo& getSensorInfo() const

            Status start() nogil
            void stop() nogil

            Status readFrame(VideoFrameRef* pFrame)
            CameraSettings* getCameraSettings()

            VideoMode getVideoMode() const
            Status setVideoMode(const VideoMode& videoMode)

            int getMaxPixelValue() const
            int getMinPixelValue() const

            bool isCroppingSupported() const
            bool getCropping(int* pOriginX, int* pOriginY, int* pWidth, int* pHeight) const
            Status setCropping(int originX, int originY, int width, int height)
            Status resetCropping()

            bool getMirroringEnabled() const
            Status setMirroringEnabled(bool isEnabled)

            Status setEmitterEnabled(bool isEnabled)

            float getHorizontalFieldOfView() const
            float getVerticalFieldOfView() const

    cdef cppclass VideoFrameRef:
        VideoFrameRef()
        int getDataSize() const
        const void* getData() const
        SensorType getSensorType() const
        const VideoMode& getVideoMode() const
        uint64_t getTimestamp() const
        inline int getFrameIndex() const
        inline int getWidth() const
        inline int getHeight() const
        inline bool getCroppingEnabled() const
        inline int getCropOriginX() const
        inline int getCropOriginY() const
        inline int getStrideInBytes() const
        inline bool isValid() const
        void release()

    cdef cppclass CameraSettings:
        Status setAutoExposureEnabled(bool enabled)
        Status setAutoWhiteBalanceEnabled(bool enabled)
        bool getAutoExposureEnabled() const
        bool getAutoWhiteBalanceEnabled() const
        Status setGain(int gain)
        Status setExposure(int exposure)
        int getGain()
        int getExposure()

cdef extern from "OpenNI.h" namespace "openni::CoordinateConverter":
     Status convertDepthToColor(const VideoStream& depthStream,
                                const VideoStream& colorStream,
                                int depthX,
                                int depthY,
                                DepthPixel depthZ,
                                int* pColorX,
                                int* pColorY)
     Status convertDepthToWorld(const VideoStream& depthStream,
                                int depthX,
                                int depthY,
                                DepthPixel depthZ,
                                float* pWorldX,
                                float* pWorldY,
                                float* pWorldZ)
     Status	convertDepthToWorld(const VideoStream &depthStream,
                                float depthX,
                                float depthY,
                                float depthZ,
                                float *pWorldX,
                                float *pWorldY,
                                float *pWorldZ)
     Status	convertWorldToDepth(const VideoStream &depthStream,
                                float worldX,
                                float worldY,
                                float worldZ,
                                int *pDepthX,
                                int *pDepthY,
                                DepthPixel *pDepthZ)
     Status convertWorldToDepth(const VideoStream &depthStream,
                                float worldX,
                                float worldY,
                                float worldZ,
                                float *pDepthX,
                                float *pDepthY,
                                float *pDepthZ)

cdef extern from "OpenNI.h" namespace "openni::OpenNI":
    Status initialize()
    void enumerateDevices(Array[DeviceInfo]*)
