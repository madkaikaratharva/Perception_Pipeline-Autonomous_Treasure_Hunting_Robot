#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String

class RealSenseCamera:
    def __init__(self,
                 depth_width,
                 depth_height,
                 color_width,
                 color_height):

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        self.bridge = CvBridge()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, 30)

        # Enable depth to color spatial alignment
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # Start streaming
        self.pipeline.start(config)

        # Initialize Publisher nodes
        self.color_publisher = rospy.Publisher('/camera/color/image_raw', Image, queue_size=0)
        self.depth_publisher = rospy.Publisher('/camera/depth/image_rect_raw', Image, queue_size=0)
        self.color_intrinsics_publisher = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=10)

        # Get Camera Intrinsics
        self.ret, self.depth_intrinsics, self.color_intrinsics = self.get_intrinsics()

    def publish_frames(self):
        color_msg = Image()
        depth_msg = Image()
        ret, depth_frame, color_frame = self.get_frame()

        if ret:
            color_msg = self.bridge.cv2_to_imgmsg(color_frame, encoding='bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='passthrough')
            self.color_publisher.publish(color_msg)
            self.depth_publisher.publish(depth_msg)
    
    def publish_intrinsics(self):
        color_intrinsics_msg = CameraInfo()
        depth_intrinsics_msg = CameraInfo()
        if self.ret:
            width = self.color_intrinsics.width
            height = self.color_intrinsics.height
            ppx = self.color_intrinsics.ppx
            ppy = self.color_intrinsics.ppy
            fx = self.color_intrinsics.fx
            fy = self.color_intrinsics.fy
            color_intrinsics_msg.width = width
            color_intrinsics_msg.height = height
            color_intrinsics_msg.K = [fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0]
            color_intrinsics_msg.D = self.color_intrinsics.coeffs
            color_intrinsics_msg.distortion_model = "Inverse Brown Conrady"

            self.color_intrinsics_publisher.publish(color_intrinsics_msg)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()

        # Align depth frame to color frame
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image
    
    def get_intrinsics(self):
        frames = self.pipeline.wait_for_frames()

        # Align depth frame to color frame
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if depth_frame and color_frame:
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            return True, depth_intrinsics, color_intrinsics
        else:
            return False, None, None
        
    def release(self):
        self.pipeline.stop()

def main():
    rospy.init_node("realsense_ros_wrapper")

    # Read Parameters from the Server
    depth_width = rospy.get_param("~depth_width", 640)
    depth_height = rospy.get_param("~depth_height", 480)
    color_width = rospy.get_param("~color_width", 640)
    color_height = rospy.get_param("~color_height", 480)

    realsense_node = RealSenseCamera(depth_width, depth_height, color_width, color_height)
    rospy.loginfo('Realsense ROS node activated')
    try:
        while not rospy.is_shutdown():
            realsense_node.publish_frames()
            realsense_node.publish_intrinsics()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Shutting Down Node")
        realsense_node.release()

if __name__ == '__main__':
    main()