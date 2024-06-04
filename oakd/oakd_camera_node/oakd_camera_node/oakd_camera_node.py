import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np  
import depthai as dai  
import logging
import time
from collections import deque 

class CameraError(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

    def __str__(self):
        return f'{self.message}'

class OakDCameraNode(Node):

    def __init__(self):
        #Call default node constructor
        super().__init__('oakd_camera_node')
        self.logger = self.get_logger()
        self.bridge = CvBridge()

        def declare_parameters():
            self.declare_parameter('width', 640)
            self.declare_parameter('height', 480)
            self.declare_parameter('depth', 3)
            self.declare_parameter('isp_scale', None)
            self.declare_parameter('framerate', 30)
            self.declare_parameter('enable_depth', False)
            self.declare_parameter('enable_obstacle_dist', False)
            self.declare_parameter('rgb_resolution', '1080p')
            self.declare_parameter('rgb_apply_cropping', False)
            self.declare_parameter('rgb_sensor_crop_x', 0.0)
            self.declare_parameter('rgb_sensor_crop_y', 0.125)
            self.declare_parameter('rgb_video_size', (1280, 600))
            self.declare_parameter('rgb_apply_manual_conf', False)
            self.declare_parameter('rgb_exposure_time', 2000)
            self.declare_parameter('rgb_sensor_iso', 1200)
            self.declare_parameter('rgb_wb_manual', 2800)

            
        self.declare_parameters()
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        depth = self.get_parameter('depth').get_parameter_value().integer_value
        isp_scale = self.get_parameter('isp_scale').get_parameter_value().string_value
        framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        enable_depth = self.get_parameter('enable_depth').get_parameter_value().bool_value
        enable_obstacle_dist = self.get_parameter('enable_obstacle_dist').get_parameter_value().bool_value
        rgb_resolution = self.get_parameter('rgb_resolution').get_parameter_value().string_value
        rgb_apply_cropping = self.get_parameter('rgb_apply_cropping').get_parameter_value().bool_value
        rgb_sensor_crop_x = self.get_parameter('rgb_sensor_crop_x').get_parameter_value().double_value
        rgb_sensor_crop_y = self.get_parameter('rgb_sensor_crop_y').get_parameter_value().double_value
        rgb_video_size = self.get_parameter('rgb_video_size').get_parameter_value().string_value
        rgb_apply_manual_conf = self.get_parameter('rgb_apply_manual_conf').get_parameter_value().bool_value
        rgb_exposure_time = self.get_parameter('rgb_exposure_time').get_parameter_value().integer_value
        rgb_sensor_iso = self.get_parameter('rgb_sensor_iso').get_parameter_value().integer_value
        rgb_wb_manual = self.get_parameter('rgb_wb_manual').get_parameter_value().integer_value
        #Initialize OakD camera object with parameters
        self.oakd_camera = OakDCamera(
            width, height, depth, isp_scale, framerate, enable_depth, enable_obstacle_dist,
            rgb_resolution, rgb_apply_cropping, rgb_sensor_crop_x, rgb_sensor_crop_y,
            rgb_video_size, rgb_apply_manual_conf, rgb_exposure_time, rgb_sensor_iso, rgb_wb_manual
        )

        #Create Publishers for RGB, Depth
        #sets size of queue where messages can be buffered by publisher
        self.queue_size = 10
        self.rgb_pub = self.create_publisher(Image, 'oakd_camera/rgb', self.queue_size)
        self.depth_pub = self.create_publisher(Image, 'oakd_camera/depth', self.queue_size)

        #ros2 timer to trigger timer_callback on intervals
        self.timer = self.create_timer(1.0/ framerate, self.timer_callback)

    #node responsible for executing code to capture and process frames and publish them
    def timer_callback(self):
        try:
            if self.oakd_camera.enable_depth:
                #get rgb, depth frames
                frame_rgb, frame_depth = self.oakd_camera.run_threaded()
                if frame_rgb is not None:
                    rgb_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
                    self.rgb_pub.publish(rgb_msg)
                if frame_depth is not None:
                    depth_msg = self.bridge.cv2_to_imgmsg(frame_depth, encoding='mono16')
                    self.depth_pub.publish(depth_msg)
            else:
                #get rgb frames
                frame_rgb = self.oakd_camera.run_threaded()
                if frame_rgb is not None:
                    rgb_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
                    self.rgb_pub.publish(rgb_msg)
        
        except CameraError as e:
            #if error, log with logger
            self.logger.error(f"Camera error: {e}")


class OakDCamera():
    def __init__(self, width, height, depth=3, isp_scale=None, framerate=30,
                 enable_depth=False, enable_obstacle_dist=False,
                 rgb_resolution="1080p", rgb_apply_cropping=False,
                 rgb_sensor_crop_x=0.0, rgb_sensor_crop_y=0.125,
                 rgb_video_size=(1280, 600), rgb_apply_manual_conf=False,
                 rgb_exposure_time=2000, rgb_sensor_iso=1200, rgb_wb_manual=2800):

        self.on = False
        self.device = None
        self.rgb_resolution = rgb_resolution
        #output queue for rgb, depth, spatial data
        self.queue_xout = None
        self.queue_xout_depth = None
        self.queue_xout_spatial_data = None
        self.roi_distances = [] #region of interest distances
        #holds most recent frame captured
        self.frame_xout = None
        self.frame_xout_depth = None
        self.extended_disparity = True
        self.subpixel = False
        #left-right consistency check
        self.lr_check = True
        self.latencies = deque([], maxlen=20)
        self.enable_depth = enable_depth
        self.enable_obstacle_dist = enable_obstacle_dist

        #Create DepthAI pipeline
        self.pipeline = dai.Pipeline()
        self.pipeline.setXLinkChunkSize(0)

        #Create other pipelines
        if self.enable_depth:
            self.create_depth_pipeline()

        if self.enable_obstacle_dist:
            self.create_obstacle_dist_pipeline()

        #Output Stream 
        xout = self.pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("xout")

        #Create different pipelines for Mono and Colored cameras
        if depth == 3:
            camera = self.pipeline.create(dai.node.ColorCamera)
            if self.rgb_resolution == "800p":
                camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
            elif self.rgb_resolution == "1080p":
                camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            else:
                camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            #sets if pixel data is interleaved or planar
            camera.setInterleaved(False)
            camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

            #adjust resolution to isp scale
            if isp_scale:
                camera.setIspScale(isp_scale)
            if rgb_apply_cropping:
                camera.setSensorCrop(rgb_sensor_crop_x, rgb_sensor_crop_y)
                camera.setVideoSize(rgb_video_size)

            camera.setPreviewKeepAspectRatio(False)
            camera.setPreviewSize(width, height)
            camera.setIspNumFramesPool(1)
            camera.setVideoNumFramesPool(1)
            camera.setPreviewNumFramesPool(1)

            if rgb_apply_manual_conf:
                #manual exposure and white balance for bad ligthing
                camera.initialControl.setManualExposure(rgb_exposure_time, rgb_sensor_iso)
                camera.initialControl.setManualWhiteBalance(rgb_wb_manual)
            else:
                #default scene with fast shutter speeds
                camera.initialControl.SceneMode(dai.CameraControl.SceneMode.SPORTS)
                camera.initialControl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)

            camera.preview.link(xout.input)
        elif depth == 1:
            camera = self.pipeline.create(dai.node.MonoCamera)
            #which camera to use left or right
            camera.setBoardSocket(dai.CameraBoardSocket.LEFT)
            camera.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

            #change the size of the image
            manip = self.pipeline.create(dai.node.ImageManip)
            manip.setMaxOutputFrameSize(width * height)
            manip.initialConfig.setResize(width, height)
            manip.initialConfig.setFrameType(dai.RawImgFrame.Type.GRAY8)

            camera.out.link(manip.inputImage)
            manip.out.link(xout.input)
        else:
            raise ValueError("'depth' parameter must be either '3' (RGB) or '1' (GRAY)")


        camera.initialControl.setManualFocus(self.pipeline)
        camera.initialControl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.FLUORESCENT)
        try:
            self.device = dai.Device(self.pipeline)
            warming_time = time.time() + 5

            if enable_depth:
                #get outputs from RGB, Depth
                self.queue_xout = self.device.getOutputQueue("xout", maxSize=1, blocking=False)
                self.queue_xout_depth = self.device.getOutputQueue("xout_depth", maxSize=1, blocking=False)
                
                #try to get initial frame until warming time runs out
                while (self.frame_xout is None or self.frame_xout_depth is None) and time.time() < warming_time:
                    self.run()
                    time.sleep(0.2)
                if self.frame_xout is None:
                    raise CameraError("Unable to start OAK-D RGB and Depth camera.")
            elif enable_obstacle_dist:
                self.queue_xout = self.device.getOutputQueue("xout", maxSize=1, blocking=False)
                self.queue_xout_spatial_data = self.device.getOutputQueue("spatialData", maxSize=1, blocking=False)
            else:
                self.queue_xout = self.device.getOutputQueue("xout", maxSize=1, blocking=False)
                self.queue_xout_depth = None
                while self.frame_xout is None and time.time() < warming_time:
                    self.run()
                    time.sleep(0.2)
                if self.frame_xout is None:
                    raise CameraError("Unable to start OAK-D camera.")

            self.on = True
        except:
            self.shutdown()
            raise

    #create depth pipeline
    def create_depth_pipeline(self):
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        stereo_manip = self.pipeline.create(dai.node.ImageManip)
        stereo = self.pipeline.create(dai.node.StereoDepth)

        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        stereo.setSubpixel(False)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.initialConfig.setConfidenceThreshold(200)

        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("xout_depth")

        topLeft = dai.Point2f(0.1875, 0.0)
        bottomRight = dai.Point2f(0.8125, 0.25)

        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        stereo_manip.initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

        #link outputs of mono cameras to stereo
        monoRight.out.link(stereo.right)
        monoLeft.out.link(stereo.left)
        #links output of stereo to ImageManip, then to XLinkOut
        stereo.depth.link(stereo_manip.inputImage)
        stereo_manip.out.link(xout_depth.input)
    
    def create_obstacle_dist_pipeline(self):
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        spatialLocationCalculator = self.pipeline.create(dai.node.SpatialLocationCalculator)

        xoutSpatialData = self.pipeline.create(dai.node.XLinkOut)
        xinSpatialCalcConfig = self.pipeline.create(dai.node.XLinkIn)

        xoutSpatialData.setStreamName("spatialData")
        xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        spatialLocationCalculator.inputConfig.setWaitForMessage(False)

        for i in range(4):
            config = dai.SpatialLocationCalculatorConfigData()
            config.depthThresholds.lowerThreshold = 200
            config.depthThresholds.upperThreshold = 10000
            config.roi = dai.Rect(dai.Point2f(i * 0.1 + 0.3, 0.35), dai.Point2f((i + 1) * 0.1 + 0.3, 0.43))
            spatialLocationCalculator.initialConfig.addROI(config)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        stereo.depth.link(spatialLocationCalculator.inputDepth)
        spatialLocationCalculator.out.link(xoutSpatialData.input)
        xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

    #run to get frames from XLinkOut
    def run(self):
        if self.queue_xout is not None:
            #if output is not null, get frame
            data_xout = self.queue_xout.get()
            image_data_xout = data_xout.getFrame()
            self.frame_xout = np.moveaxis(image_data_xout, 0, -1)

        if self.queue_xout_depth is not None:
            #if depth not null, get depth frame
            data_xout_depth = self.queue_xout_depth.get()
            self.frame_xout_depth = data_xout_depth.getFrame()

        if self.queue_xout_spatial_data is not None:
            xout_spatial_data = self.queue_xout_spatial_data.get().getSpatialLocations()
            self.roi_distances = []
            for depthData in xout_spatial_data:
                roi = depthData.config.roi
                coords = depthData.spatialCoordinates
                self.roi_distances.append([round(roi.topLeft().x, 2), round(roi.topLeft().y, 2),
                                           round(roi.bottomRight().x, 2), round(roi.bottomRight().y, 2),
                                           int(coords.x), int(coords.y), int(coords.z)])

        if self.enable_depth:
            return self.frame_xout, self.frame_xout_depth
        elif self.enable_obstacle_dist:
            return self.frame_xout, np.array(self.roi_distances)
        else:
            return self.frame_xout

    def run_threaded(self):
        return self.run()
    
    def shutdown(self):
        self.on = False
        time.sleep(.5)
        if self.device is not None:
            self.device.close()
        self.device = None
        self.queue = None
        self.pipeline = None


def main(args=None):
    rclpy.init(args=args)
    node = OakDCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()