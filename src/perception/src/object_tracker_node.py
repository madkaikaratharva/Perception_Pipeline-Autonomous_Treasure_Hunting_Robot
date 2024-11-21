#!/usr/bin/env python3

# Python Imports
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from concurrent.futures import ThreadPoolExecutor
import datetime
from deep_sort_realtime.deepsort_tracker import DeepSort

# ROS Imports
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped

# Camera intrinsics
WIDTH = 640
HEIGHT = 480
K = np.array([[615.353, 0.0, 315.423], [0.0, 615.405, 236.470], [0.0, 0.0, 1.0]])
distortion_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_intrinsics = rs.intrinsics()
camera_intrinsics.width = WIDTH
camera_intrinsics.height = HEIGHT
camera_intrinsics.fx = K[0][0]
camera_intrinsics.fy = K[1][1]
camera_intrinsics.ppx = K[0][2]
camera_intrinsics.ppy = K[1][2]
camera_intrinsics.model = rs.distortion.inverse_brown_conrady
camera_intrinsics.coeffs = distortion_coeffs


# Distance scaling (Convert from mm to m)
SCALE = 0.001 # (m)


class ObjectTracker:
    def __init__(self):

        # Initialize DeepSort Tracker
        self.tracker = DeepSort(max_age=30, n_init=2,
                                embedder='mobilenet', half=True,
                                bgr=True, embedder_gpu=True,
                                polygon=False, today=None)
        
        # Initialize Cv-Bridge
        self.bridge = CvBridge()

        # Initialize Subscribers
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        self.tracking_sub = rospy.Subscriber('/track/bounding_box', Detection2DArray, self.track_callback)

        # Initialize Publisher
        self.goal_cam_pub = rospy.Publisher('/goal/camera', PointStamped, queue_size=10)

        self.color_image = None
        self.depth_image = None
        self.track_msg = None
        self.trackings = []
    
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except:
            rospy.logerr("Depth frame not received")
    
    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except:
            rospy.logerr("Color frame not received")
    
    def track_callback(self, msg):
        try:
            # Check if the msg is empty
            if len(msg.detections) == 0:
                self.trackings = []
            else:
                # If not empty, pass tracking boxes to DeepSort
                # Box format: [([x, y, w, h], score, class_id)]
                self.trackings = []
                for detection_msg in msg.detections:
                    result = detection_msg.results.pop()
                    class_id = result.id
                    score = result.score

                    x_center = detection_msg.bbox.center.x
                    y_center = detection_msg.bbox.center.y
                    size_x = detection_msg.bbox.size_x
                    size_y = detection_msg.bbox.size_y

                    # Get the top left corner (x1, y1)
                    x1 = x_center - (size_x * 0.5)
                    y1 = y_center - (size_y * 0.5)

                    self.trackings.append(([int(x1), int(y1), int(size_x), int(size_y)], score, class_id))
        except:
            rospy.logerr("Error processing data")
        
    def track_objects(self, trackings, color_frame):
        # Get trackings and pass to DeepSort to track
        bboxes = []
        tracks = self.tracker.update_tracks(trackings, frame=color_frame)

        # From results, get the bounding boxes
        for track in tracks:
            if not track.is_confirmed():
                continue
            track_id = track.track_id
            bbox = track.to_ltrb()
            bboxes.append(bbox)
        
        return bboxes
    
    def get_3d_point(self, bbox, depth_frame):
        global camera_intrinsics
        global SCALE

        x1 = bbox[0]
        y1 = bbox[1]
        x2 = bbox[2]
        y2 = bbox[3]

        # Evaluate Object Center and depth
        x_center = x1 + ((x2 - x1) * 0.5)
        y_center = y1 + ((y2 - y1) * 0.5)
        depth = depth_frame[int(y_center), int(x_center)]

        # Get co-ordinates in Camera Frame 1
        x_cam_1, y_cam_1, z_cam_1 = rs.rs2_deproject_pixel_to_point(camera_intrinsics, [x_center, y_center], depth)

        # Convert co-ordinates in Frame 2
        x_cam_2 = z_cam_1
        y_cam_2 = -1.0 * (x_cam_1)
        z_cam_2 = -1.0 * (y_cam_1)

        return x_cam_2*SCALE, y_cam_2*SCALE, z_cam_2*SCALE

    def generate_point_message(self, x, y, z):
        msg = PointStamped()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/camera'

        msg.header = header
        msg.point.x = np.float64(x)
        msg.point.y = np.float64(y)
        msg.point.z = np.float64(z)

        return msg

def main():

    # Initialize ROS node
    rospy.init_node("object_tracker_node")

    # Read Params from the server
    tracker = rospy.get_param("~tracker", False)

    # Initialize the tracker node
    tracker_node = ObjectTracker()
    rospy.loginfo("Object Tracking Node Started")

    try:
        while not rospy.is_shutdown():
            # Timeout function
            timeout = rospy.Duration(5) # Set the timeout to 5 secs
            start_time = rospy.Time.now()

            # Wait until both color and depth images are received
            while tracker_node.depth_image is None or tracker_node.color_image is None:
                rospy.sleep(0.1)

            # Check if timeout occured
            if rospy.Time.now() - start_time >= timeout:
                rospy.logerr("Timed out waiting for images. Skipping this iteration.")
                continue

            # Get depth and color frames from callbacks    
            depth_frame = tracker_node.depth_image.copy()
            color_frame = tracker_node.color_image.copy()

            # Get trackings 
            trackings = tracker_node.trackings

            with ThreadPoolExecutor(max_workers=4) as executor:
                bboxes = executor.submit(tracker_node.track_objects, trackings, color_frame).result()

            if len(bboxes) > 0:
                for bbox in bboxes:

                    # Get 3d coordinates of the center point of the object
                    x, y, z = tracker_node.get_3d_point(bbox, depth_frame)

                    # Generate Point Message
                    with ThreadPoolExecutor(max_workers=4) as executor:
                        #goal_camera = detector_node.generate_point_message(x, y, z)
                        future = executor.submit(tracker_node.generate_point_message, x, y, z)                    
                    goal_camera = future.result()

                    # If tracker is enabled, publish goal point
                    if tracker:
                        tracker_node.goal_cam_pub.publish(goal_camera)
    

                    # Display the bounding box
                    cv2.putText(color_frame, "Tracking object", (int(bbox[0]), int(bbox[1])-5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (200,0,0),1)
                    cv2.rectangle(color_frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (200,0,0), 2)
            
            cv2.imshow("Tracking Frame", color_frame)
            key = cv2.waitKey(1)

            if key == 27:
                break

    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
        rospy.loginfo("Shutting Down Node")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

