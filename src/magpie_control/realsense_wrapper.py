########## INIT ####################################################################################

import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from PIL import Image
import time
from collections import deque
import asyncio


from magpie_control.realsense_device_manager import DeviceManager, Device



########## HELPER FUNCTIONS ########################################################################

def poll_devices():
    ctx = rs.context()
    devices = ctx.query_devices()
    models = [d.get_info(rs.camera_info.name).split(' ')[-1] for d in devices]
    serial = [d.get_info(rs.camera_info.serial_number) for d in devices]
    info = {i:j for i,j in zip(models, serial)}
    return info


def make_o3d_cpcd( points : list, colors : list ):
    cpcd = o3d.geometry.PointCloud()
    if len( points ):
        cpcd.points = o3d.utility.Vector3dVector( np.array( points ) )
        cpcd.colors = o3d.utility.Vector3dVector( np.array( colors ) )
    return cpcd


def get_oPCD_aabb( cpcd ) -> np.ndarray:
    """ Compute the Axis-Aligned Bounding Box """
    rtnBB = np.zeros( (2,3,) )
    try:
        arr = np.array( cpcd.points ).reshape((-1,3,))
        # print( arr.size )
        ptMin = arr.min( axis = 0 )
        ptMax = arr.max( axis = 0 )
        print( f"AABB: {[ptMin, ptMax,]}" )
        rtnBB[0,:] = ptMin
        rtnBB[1,:] = ptMax
    except ValueError:
        print( f"`get_oPCD_aabb`: Array size error! {arr.shape}" )
    return rtnBB


def get_oPCD_aabb_volume( cpcd ) -> float:
    """ Compute the volume of the Axis-Aligned Bounding Box """
    rtnVol = 1.0
    aabb   = get_oPCD_aabb( cpcd )
    for i in range(3):
        rtnVol *= (aabb[1,i]-aabb[0,i])
    return rtnVol


########## HELPER CLASSES ##########################################################################

class MPCD:
    """ Container class for 3D data """

    def __init__( self, rgbd, xyzArr : np.ndarray, rgbArr : np.ndarray ):
        self.rgbd   = rgbd
        self.xyzArr = xyzArr
        self.rgbArr = rgbArr
        self.aabb   = None


    # def get_aabb( self ):
    #     
    #     rtnBB = np.zeros( (2,3,) )
    #     rtnBB[0,:] = self.xyzArr.min( axis=(0,1,) )
    #     rtnBB[1,:] = self.xyzArr.max( axis=(0,1,) )
    #     print( f"AABB: {rtnBB}" )
    #     self.aabb = rtnBB.copy()
    #     return rtnBB


    # def get_aabb_volume( self ):
    #     
    #     


    def get_full_cpcd( self, minVal = 0.070, maxVal = 18.00 ):
        """ Mask only desired points """
        M, N    = self.xyzArr.shape[:2]
        xyz     = deque()
        rgb     = deque()
        for i in range(M):
            for j in range(N):
                if minVal <= sum( self.xyzArr[i,j,:] ) <= maxVal:
                    xyz.append( self.xyzArr[i,j,:] )
                    rgb.append( self.rgbArr[i,j,:] )
        return make_o3d_cpcd( list( xyz ), list( rgb ) )


    def get_masked_cpcd( self, mask : np.ndarray, NB = 50, zClip = 0.500 ):
        """ Mask only desired points """
        epsilon = 0.0005
        M, N    = self.xyzArr.shape[:2]
        xyz     = deque()
        rgb     = deque()
        for i in range(M):
            for j in range(N):
                if (mask[i,j] > epsilon) and (self.xyzArr[i,j,2] <= zClip):
                    xyz.append( self.xyzArr[i,j,:] )
                    rgb.append( self.rgbArr[i,j,:] )
        cpcd = make_o3d_cpcd( list( xyz ), list( rgb ) )

        # denoise pcd
        _, ind = cpcd.remove_statistical_outlier( nb_neighbors = NB, std_ratio = 0.01 )
        return cpcd.select_by_index( ind )









########## REALSENSE WRAPPER #######################################################################

class RealSense():
    def __init__(self, w=1280, h=720, zMax=0.5, voxelSize=0.001, fps=5, device_serial=None, device_name='D405'):
        self.pinholeIntrinsics = None  # set in self.takeImages()
        self.zMax = zMax  # max distance for objects in depth images (m)
        # downsample point cloud with voxel size = 1 mm (0.001 m / 0.04 in)
        self.voxelSize = voxelSize
        self.pcd = o3d.geometry.PointCloud()  # current pcd from realsense
        self.extrinsics = np.eye(4)  # extrinsic parameters of the camera frame 4 x 4 numpy array
        self.cameraFrameTransform = np.eye(4)
        self.pipe, self.config, self.device = None, None, None
        self.recording      = False
        self.recording_task = None
        self.fps            = fps # fps can only be: 5, 15, 30, 60, 90, or 6 for the wrist D405
        self.device_name    = device_name        
        if self.device_name is not None:
            try:
                self.device_serial = poll_devices()[self.device_name]
            except KeyError:
                print("Device not found. Please check device name. Default is D405")
                self.device_serial = None
        else:
            self.device_serial = device_serial
        self.w = w
        self.h = h
        self.buffer_dict = {}
        self.deviceManager  = DeviceManager()
        self.rerun_viz = None


    def initConnection(self, device_serial=None, enable_depth=True, enable_color=True):
        # Initializes connection to realsense, sets pipe,config values
        self.pipe = rs.pipeline()
        self.config = rs.config()

        # enable specific device, used if multiple devices connected
        # THIS CALL MUST HAPPEN BEFORE EVERYTHING ELSE
        # Especially before calling get_device() on config.resolve(...), duh...
        if self.device_serial is None and device_serial is not None:
            self.device_serial = device_serial
        self.config.enable_device(self.device_serial)

        # Getting information about the connected realsense model (device object) - D405
        pipeProfile = self.config.resolve(rs.pipeline_wrapper(self.pipe))
        self.device = device = pipeProfile.get_device()
        depth_sensor = device.first_depth_sensor()
        ''' trying to improve FPS, not needed anymore
        depth_sensor.set_option(rs.option.enable_auto_exposure, False)
        set exposure
        depth_sensor.set_option(rs.option.exposure, 400)

        if self.device_name != 'D405':
            color_sensor = device.first_color_sensor()
            color_sensor.set_option(rs.option.enable_auto_exposure, False)
            color_sensor.set_option(rs.option.exposure, 400)
        '''
        self.depthScale = depth_sensor.get_depth_scale()
        # print(depth_scale)

        # 1 - default, 2 - hand, 3 - high accuracy, 4 - high density, 5 - medium density
        depth_sensor.set_option(rs.option.visual_preset, 4)  # 4 corresponds to high-density option

        # Setting attributes for stream
        # Depth Stream (1280 x 720) 5 fps - D405 Sensor has max 1280 x 720
        # (Minimum z depth is between 55-70 mm)
        if enable_depth:
            self.config.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, self.fps)

        # Color and Infrared D405 Streams Available (1280 x 720) 5 fps - D405 Sensor has max 1280 x 720
        if enable_color:
            self.config.enable_stream(rs.stream.color, self.w, self.h, rs.format.rgb8, self.fps)

        # Starting the pipeline based on the specified configuration
        profile = self.pipe.start(self.config)

        product_line = None
        for device_info in self.deviceManager._available_devices:
            device_serial_i = device_info[0]
            if device_serial_i == device_serial:
                product_line = device_info[1]

        self.deviceManager._enabled_devices[ self.device_serial ] = Device( self.pipe, profile, product_line)


    def getPinholeInstrinsics(self, frame):
        # frame is a subclass of pyrealsense2.video_frame (depth_frame,etc)

        if 0:
            intrinsics = frame.profile.as_video_stream_profile().intrinsics
        else:
            # Get the intrinsics of the realsense device
            # Source: https://github.com/isl-org/Open3D/issues/473#issuecomment-408017937
            frames_devices     = self.deviceManager.poll_frames()
            # print( f"There were {len(frames_devices)} frames!" )
            intrinsics_devices = self.deviceManager.get_device_intrinsics( frames_devices )
            # print( f"{intrinsics_devices.keys()}" )
            intrinsics         = intrinsics_devices[self.device_serial][rs.stream.depth]


        return o3d.camera.PinholeCameraIntrinsic( intrinsics.width, intrinsics.height, 
                                                  intrinsics.fx, intrinsics.fy, 
                                                  intrinsics.ppx, intrinsics.ppy )
    
    # def get_custom_D405_intrinsics( self ):
    #     """ People on the internet say that the correct intrinsics don't come from the API? """
    #     return o3d.camera.PinholeCameraIntrinsic( 1288, 808, 
    #                                               641.341675, 640.458862, 
    #                                               intrinsics.ppx, intrinsics.ppy )


    def write_buffer(self):
        for ts, image_info in self.buffer_dict.items():
            colorIM = Image.fromarray(image_info["rgb"])
            colorIM.save(image_info["rgb_path"])
            if "depth" in image_info:
                np.save(image_info["depth_path"], image_info["depth"])
        self.buffer_dict = {}

    def flush_buffer(self, t=3):
        start = time.time()
        while time.time() - start < t:
            frames = self.pipe.wait_for_frames()

    async def take_image(self, save=False, filepath="", buffer=False, depth=False):
        # Takes RGBD Image using Realsense
        # intrinsic and extrinsic parameters are NOT applied only in getPCD()
        # out: Open3D RGBDImage
        pipe, config = self.pipe, self.config

        frames = pipe.wait_for_frames()
        colorFrame = frames.get_color_frame()
        rawColorImage = np.asanyarray(colorFrame.get_data()).copy()
        timestamp = frames.get_timestamp() / 1000

        if not depth: 
            subFix = str(timestamp)
            image_info = {
                "rgb": rawColorImage,
                "rgb_path": f"{filepath}{subFix}.jpeg",
            }
            self.buffer_dict[timestamp] = image_info
            if self.rerun_viz is not None: self.rerun_viz.log_camera_data(image_info, timestamp, self.device_name)
            return rawColorImage

        rgbd = None
        # Sets class value for intrinsic pinhole parameters
        self.pinholeInstrinsics = self.getPinholeInstrinsics(colorFrame)
        # asign extrinsics here if the camera pose is known
        # alignOperator maps depth frames to color frames
        # alignment halves fps: https://github.com/IntelRealSense/realsense-ros/issues/1632
        alignOperator = rs.align(rs.stream.color)
        alignOperator.process(frames)
        alignedDepthFrame, alignedColorFrame = frames.get_depth_frame(), frames.get_color_frame()

        # unmodified rgb and z images as numpy arrays of 3 and 1 channels
        rawColorImage = np.array(alignedColorFrame.get_data()).copy()
        rawDepthImage = np.asarray(alignedDepthFrame.get_data()).copy()

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rawColorImage),
            o3d.geometry.Image(rawDepthImage.astype('uint16')),
            depth_scale=1.0 / self.depthScale,
            depth_trunc=self.zMax,
            convert_rgb_to_intensity=False)
        
        if buffer:
            subFix = str(timestamp)
            image_info = {
                "rgb": np.array(rgbd.color).copy(),
                "depth":np.array(rgbd.depth).copy(),
                "rgb_path": f"{filepath}{subFix}.jpeg",
                "depth_path": f"{filepath}{subFix}.npy"
            }
            self.buffer_dict[timestamp] = image_info
            if self.rerun_viz is not None: self.rerun_viz.log_camera_data(image_info, timestamp, self.device_name)

        return rgbd

    def take_image_blocking(self, filepath="", buffer=False, depth=False):
        # Takes RGBD Image using Realsense
        # intrinsic and extrinsic parameters are NOT applied only in getPCD()
        # out: Open3D RGBDImage
        pipe, config = self.pipe, self.config

        frames = pipe.wait_for_frames()
        colorFrame = frames.get_color_frame()
        rawColorImage = np.asanyarray(colorFrame.get_data()).copy()
        timestamp = frames.get_timestamp() / 1000

        if not depth: 
            subFix = str(timestamp)
            image_info = {
                "rgb": rawColorImage,
                "rgb_path": f"{filepath}{subFix}.jpeg",
            }
            self.buffer_dict[timestamp] = image_info
            if self.rerun_viz is not None: self.rerun_viz.log_camera_data(image_info, timestamp, self.device_name)
            return rawColorImage

        rgbd = None
        # Sets class value for intrinsic pinhole parameters
        self.pinholeInstrinsics = self.getPinholeInstrinsics(colorFrame)
        # asign extrinsics here if the camera pose is known
        # alignOperator maps depth frames to color frames
        alignOperator = rs.align(rs.stream.color)
        alignOperator.process(frames)
        alignedDepthFrame, alignedColorFrame = frames.get_depth_frame(), frames.get_color_frame()

        # unmodified rgb and z images as numpy arrays of 3 and 1 channels
        rawColorImage = np.array(alignedColorFrame.get_data()).copy()
        rawDepthImage = np.asarray(alignedDepthFrame.get_data()).copy()

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rawColorImage),
            o3d.geometry.Image(rawDepthImage.astype('uint16')),
            depth_scale=1.0 / self.depthScale,
            depth_trunc=self.zMax,
            convert_rgb_to_intensity=False)
        
        if buffer:
            subFix = str(timestamp)
            image_info = {
                "rgb": np.array(rgbd.color).copy(),
                "depth":np.array(rgbd.depth).copy(),
                "rgb_path": f"{filepath}{subFix}.jpeg",
                "depth_path": f"{filepath}{subFix}.npy"
            }
            self.buffer_dict[timestamp] = image_info
            if self.rerun_viz is not None: self.rerun_viz.log_camera_data(image_info, timestamp, self.device_name)

        return rgbd


    async def _record_images(self, filepath="", record_depth=False):
        # records images to a specified filepath
        try:
            while self.recording:
                await self.take_image(save=True, filepath=filepath, buffer=True, depth=record_depth)
                await asyncio.sleep(0.01)
        except asyncio.CancelledError:
            print(f"{self.device_name} Recording Task Cancelled")

    def begin_record(self, filepath="", record_depth=False):
        if not self.recording:
            self.recording = True
            self.recording_task = asyncio.create_task(self._record_images(filepath=filepath, record_depth=record_depth))
            print(f"{self.device_name} Recording Started")

    async def stop_record(self, flush_time=2, write=True):
        if self.recording:
            self.recording = False
            if self.recording_task is not None:
                self.recording_task.cancel()
                try:
                    await self.recording_task
                except asyncio.CancelledError:
                    pass
                print(f"{self.device_name} Recording Stopped")
            if write:
                self.write_buffer()
                print(f"{self.device_name} Buffer written")
            # self.flush_buffer(t=flush_time)
            # print("Buffer flushed")
        else:
            print("Recording inactive")

    def takeImages(self, save=False, filepath=""):
        # Takes RGBD Image using Realsense
        # intrinsic and extrinsic parameters are NOT applied only in getPCD()
        # out: Open3D RGBDImage
        pipe, config = self.pipe, self.config

        frames = pipe.wait_for_frames()
        depthFrame = frames.get_depth_frame()  # pyrealsense2.depth_frame
        colorFrame = frames.get_color_frame()

        # Sets class value for intrinsic pinhole parameters
        self.pinholeInstrinsics = self.getPinholeInstrinsics(colorFrame)
        # asign extrinsics here if the camera pose is known
        # alignOperator maps depth frames to color frames
        alignOperator = rs.align(rs.stream.color)
        alignOperator.process(frames)
        alignedDepthFrame, alignedColorFrame = frames.get_depth_frame(), frames.get_color_frame()

        # unmodified rgb and z images as numpy arrays of 3 and 1 channels
        rawColorImage = np.array(alignedColorFrame.get_data())
        rawDepthImage = np.asarray(alignedDepthFrame.get_data())

        rawRGBDImage = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rawColorImage),
            o3d.geometry.Image(rawDepthImage.astype('uint16')),
            depth_scale=1.0 / self.depthScale,
            depth_trunc=self.zMax,
            convert_rgb_to_intensity=False)

        if save:
            subFix = str(time.time())
            np.save(f"{filepath}depthImage{subFix}", rawRGBDImage.depth)
            np.save(f"{filepath}colorImage{subFix}", rawRGBDImage.color)
            colorIM = Image.fromarray(rawColorImage)
            colorIM.save(f"{filepath}colorImage{subFix}.jpeg")
        return rawRGBDImage
    

    def getPCD_alt( self ):
        """ What if Open3D were not my friend? """
        # Source: https://github.com/dorodnic/binder_test/blob/master/pointcloud.ipynb
        
        frameset    = self.pipe.wait_for_frames()
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()

        alignOperator = rs.align(rs.stream.color)
        alignOperator.process(frameset)
        alignedDepthFrame, alignedColorFrame = frameset.get_depth_frame(), frameset.get_color_frame()

        # unmodified rgb and z images as numpy arrays of 3 and 1 channels
        rawColorImage = np.array(alignedColorFrame.get_data()).copy()
        rawDepthImage = np.asarray(alignedDepthFrame.get_data()).copy()

        rawRGBDImage = rawRGBDImage = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image( rawColorImage ),
            o3d.geometry.Image( rawDepthImage.astype('uint16') ),
            depth_scale = 1.0 / self.depthScale,
            depth_trunc = self.zMax,
            convert_rgb_to_intensity = False
        )

        pc = rs.pointcloud()
        pc.map_to( color_frame )
        pointcloud = pc.calculate( depth_frame )
        # This is stupid, but it WORKS
        vtx = np.array( np.array( pointcloud.get_vertices() ).tolist() ).reshape( rawColorImage.shape )

        return MPCD( rawRGBDImage, vtx, rawColorImage )
        

    def getPCD(self, save=False, adjust_extrinsics=False):
        # Takes images and returns a PCD and RGBD Image
        # Applies extrinsics and zMax
        # Downsamples PCD based on self.voxelSize
        # :save boolean that toggles whether to save data
        # out: tuple of (open3d point cloud (o3d.geometry.PointCloud),RGBDImage)
        rawRGBDImage = self.takeImages()
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rawRGBDImage,
            self.pinholeInstrinsics,
            project_valid_depth_only=True,
            extrinsic=self.extrinsics
        )

        # Don't downsample
        # downsampledPCD = pcd.voxel_down_sample(voxel_size=self.voxelSize)
        if save:
            subFix = time.time()
            np.save(f"colorImage{subFix}", np.array(rawRGBDImage.color))
            np.save(f"depthImage{subFix}", np.array(rawRGBDImage.depth))
            # o3d.io.write_point_cloud(f"pcd{subFix}.pcd", downsampledPCD)

        if adjust_extrinsics:
            # create rotation matrix of -pi/2 about z-axis
            rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
            tmat_gripper = np.array([[1, 0, 0, -1.15 / 100],
                            [0, 1, 0, 1.3 / 100],
                            [0, 0, 1, (309.63 - 195.0) / 1000],
                            [0, 0, 0, 1]])
            pcd.rotate(rot) # account for camera orientation, which is -pi/2 about z-axis relative to ur5 wrist
            pcd.transform(tmat_gripper) # account for camera position relative to ur5 wrist

        return pcd, rawRGBDImage
        # return (downsampledPCD,rawRGBDImage)

    def apply_extrinsics(self, pcd):
        # Applies extrinsics to the camera frame, returning pcd back to wrist frame
        # pose is a 4x4 numpy array
        # pcd is an open3d point cloud
        # create rotation matrix of -pi/2 about z-axis
        rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        tmat_gripper = np.array([[1, 0, 0, -1.15 / 100],
                        [0, 1, 0, 1.3 / 100],
                        [0, 0, 1, (309.63 - 195.0) / 1000],
                        [0, 0, 0, 1]])
        pcd.rotate(rot) # account for camera orientation, which is -pi/2 about z-axis relative to ur5 wrist
        pcd.transform(tmat_gripper) # account for camera position relative to ur5 wrist
        return pcd


    def displayImages(self, depthImg, colorImg):
        # Displays a depth and color image given the rgbdImage
        plt.subplot(1, 2, 1)
        plt.title("RealSense Color Image")
        plt.imshow(depthImg)
        plt.subplot(1, 2, 2)
        plt.title("RealSense Depth Image")
        plt.imshow(colorImg)
        plt.show()

    def displayPCD(self, pcds):
        # Displays a list of point clouds given an array of pcd's. Displays camera frame if self.extrinsics != None
        # flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
        # pcd.transform(flip_transform)
        worldFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.075, origin=[0, 0, 0])
        if (self.extrinsics is None) == False:
            cameraFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.075)
            cameraFrame.transform(self.cameraFrameTransform)
            res = [worldFrame, cameraFrame]
            res.extend(pcds)
            baseSphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.025)
            res.append(baseSphere)
            o3d.visualization.draw_geometries(res)
        else:
            res = [worldFrame].extend(pcds)
            o3d.visualization.draw_geometries(res)

    def display_world(self, world_pcd):
        coordFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        geometry = [coordFrame]
        geometry.append(world_pcd)
        o3d.visualization.draw_geometries(geometry)

    def display_world_nb(self, world_pcd):
        coordFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        geometry = [coordFrame]
        geometry.append(world_pcd)
        o3d.web_visualizer.draw(geometry)

    def displayStream(self):
        # streams and displays the point cloud data in open3d
        # pipe,config are stream properties set in the earlier cells
        # Streaming loop
        pipe, config = self.pipe, self.config
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        framesTaken = 0
        displayedPCD = o3d.geometry.PointCloud()
        try:
            while True:
                temp = self.getPCD()[0]
                displayedPCD.points = temp.points
                displayedPCD.colors = temp.colors
                if framesTaken == 0:
                    vis.add_geometry(displayedPCD)
                    worldFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin=[0, 0, 0])
                    vis.add_geometry(worldFrame)
                vis.update_geometry(displayedPCD)
                framesTaken += 1
                t0 = time.time()
                vis.poll_events()
                vis.update_renderer()
                # time.sleep(5)
        except Exception as e:
            print(f"Stream Issue. Exception Raised")
            # raise(e)
        # closes window when cell is stopped (exception raised)
        finally:
            vis.destroy_window()
            # pipe.stop()

    def disconnect(self):
        self.pipe.stop()