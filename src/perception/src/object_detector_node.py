#!/usr/bin/env python3

# Python imports
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from concurrent.futures import ThreadPoolExecutor
from perception.utils import*
import datetime
from perception.object_detector import ObjectDetector

# ROS imports
from cv_bridge import CvBridge
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import PointStamped, PoseArray, Pose

# Define Camera Intrinsics
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

# Object Recognition Flags
object_id = None

# BackProjection Params
DIST_THRESHOLD = 0.8
HIST_THRESHOLD = 0.7

class ObjectDetectorNode:
    def __init__(self,
                 model_name='yolov8m.pt'):
        
        # Set Params
        self.model_name = model_name

        # Initialize object detector
        try:
            self.detector = ObjectDetector(model_name=self.model_name)
            self.class_names = self.detector.detected_classes
        except:
            rospy.loginfo("Model not found. Using default model")
            self.detector = ObjectDetector()
            self.class_names = self.detector.detected_classes
        
        # Initailize Cv-Bridge
        self.bridge = CvBridge()

        # Initialize Depth and Color Subscribers
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)

        # Initialize Publishers
        self.detections_cam_pub = rospy.Publisher('/detections/camera', PoseArray, queue_size=1)
        self.goal_cam_pub = rospy.Publisher('/goal/camera', PointStamped, queue_size=10)
        self.status_pub = rospy.Publisher('/goal/confirmation', Bool, queue_size=10)
        self.tracking_pub = rospy.Publisher('/track/bounding_box', Detection2DArray, queue_size=10)
        self.disp_publisher = rospy.Publisher('/rviz/visualization', Image, queue_size=5)

        self.color_image = None 
        self.depth_image = None
    
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
    
    def compute_histogram(self, frame):
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Generate a Mask
        mask = cv2.inRange(hsv, np.array((0., 60.,32.)), np.array((180.,255.,255.)))

        # Compute Histogram
        hist = cv2.calcHist([hsv],[0,1],mask,[180,256],[0,180,0,256])

        # Normalize
        cv2.normalize(hist,hist,0,255,cv2.NORM_MINMAX)

        return hist
    
    def get_3d_point(self, x1, y1, x2, y2, depth_frame):
        global camera_intrinsics
        global SCALE

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
    
    def generate_pose_message(self, x, y, z):
        msg = Pose()

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        msg.orientation.w = 0.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        return msg
    
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
    
    def generate_tracking_msg(self, x1, y1, x2, y2, score, class_id):
        msg = Detection2D()
        header = Header()
        result = ObjectHypothesisWithPose()

        header.stamp = rospy.Time.now()
        header.frame_id = '/camera/image'

        result.id = class_id
        result.score = score
        msg.results.append(result)

        x_center = x1 + (x2 - x1) * 0.5
        y_center = y1 + (y2 - y1) * 0.5
        size_x = np.float64(x2 - x1)
        size_y = np.float64(y2 - y1)

        msg.header = header
        msg.bbox.center.x = np.float64(x_center)
        msg.bbox.center.y = np.float64(y_center)
        msg.bbox.size_x = size_x
        msg.bbox.size_y = size_y

        return msg
    
    def publish_disp(self, frame):
        msg = Image()
        header = Header()

        header.frame_id = '/camera'
        header.stamp = rospy.Time.now()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.disp_publisher.publish(msg)


def main():

    # Set global flags
    global object_id
    global DIST_THRESHOLD
    global HIST_THRESHOLD

    # Initialize ROS node
    rospy.init_node("object_detector_node")

    # Read Params from the server
    tracker = rospy.get_param("~tracker", False)
    model_name = rospy.get_param("~model_name", 'yolov8m.pt')

    # Initialize the detector
    detector_node = ObjectDetectorNode(model_name)
    rospy.loginfo("Objected Detection Node Started")

    try:
        while not rospy.is_shutdown():
            # Set local flags
            object_in_frame = False

            # Timeout function
            timeout = rospy.Duration(5) # Set the timeout to 5 secs
            start_time = rospy.Time.now()

            # Wait until both color and depth images are received
            while detector_node.depth_image is None or detector_node.color_image is None:
                rospy.sleep(0.1)

            # Check if timeout occured
            if rospy.Time.now() - start_time >= timeout:
                rospy.logerr("Timed out waiting for images. Skipping this iteration.")
                continue
            
            # Get Color and Depth frames
            depth_frame = detector_node.depth_image
            color_frame = detector_node.color_image
            disp_frame = color_frame.copy()

            # Get Detections
            detections = detector_node.detector.get_detections(color_frame)

            # Tracking boxes array
            tracking_boxes = Detection2DArray()
            tracking_header = Header()

            # Detections Array
            detections_array = PoseArray()
            detections_header = Header()

            # Process the detections
            for detection in detections:
                x1, y1, x2, y2, score, class_id = detection

                # Get co-ordinates in camera frame
                x, y, z = detector_node.get_3d_point(x1, y1, x2, y2, depth_frame)

                # Check for selected object
                if class_id == object_id:

                    # Modify flags
                    object_in_frame = True

                    # Generate Bounding Box for tracker node
                    tracking_msg = detector_node.generate_tracking_msg(x1, y1, x2, y2, score, class_id)
                    tracking_boxes.detections.append(tracking_msg)

                    # Generate Point Message
                    with ThreadPoolExecutor(max_workers=4) as executor:
                        #goal_camera = detector_node.generate_point_message(x, y, z)
                        future = executor.submit(detector_node.generate_point_message, x, y, z)                    
                    goal_camera = future.result()

                    if x < DIST_THRESHOLD:
                        # Run Back Projection  
                        # ROI in the frame
                        frame_roi = color_frame[y1:y2, x1:x2]
                        frame_hist = detector_node.compute_histogram(frame_roi)

                        # Compare Histograms
                        dst = cv2.compareHist(roi_hist, frame_hist, cv2.HISTCMP_BHATTACHARYYA)

                        if dst < HIST_THRESHOLD:
                            rospy.loginfo("Object Confirmed")

                            # Publish the confirmation message
                            status_msg = Bool()
                            status_msg.data = True
                            detector_node.status_pub.publish(status_msg)

                            # Display the bounding box
                            cv2.putText(disp_frame, "Confirmed", (x1, y1 - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 200, 0), 1)
                            cv2.rectangle(disp_frame, (x1, y1), (x2, y2), (0, 200, 0), 2)
                        else:
                            cv2.rectangle(disp_frame, (x1, y1), (x2, y2), (200, 0, 0), 2)
                            cv2.putText(disp_frame, detector_node.class_names[object_id], (x1, y1 - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (200, 0, 0), 1)
                    else:
                        rospy.loginfo("Similar object in frame")
                        cv2.rectangle(disp_frame, (x1, y1), (x2, y2), (200, 0, 0), 2)
                        cv2.putText(disp_frame, detector_node.class_names[object_id], (x1, y1 - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (200, 0, 0), 1)

                    continue

                # Generate Pose message for detection
                with ThreadPoolExecutor(max_workers=4) as executor:
                    future = executor.submit(detector_node.generate_pose_message, x, y, z)                
                detection_pose_msg = future.result()

                # Add Pose Message to Pose Array
                detections_array.poses.append(detection_pose_msg)


                cv2.rectangle(disp_frame, (x1, y1), (x2, y2), (0, 0, 200), 2)
                cv2.putText(disp_frame, detector_node.class_names[class_id], (x1, y1 - 5),
                            cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 200), 1)


            # Publish Detections
            detections_header.stamp = rospy.Time.now()
            detections_header.frame_id = '/camera'
            detections_array.header = detections_header
            detector_node.detections_cam_pub.publish(detections_array)

            # Publish Tracking boxes
            tracking_header.stamp = rospy.Time.now()
            tracking_header.frame_id = '/camera/image'
            tracking_boxes.header = tracking_header
            detector_node.tracking_pub.publish(tracking_boxes)

            # If tracker is set to False, publish goal point from this node
            if not tracker:
                if object_in_frame:
                    detector_node.goal_cam_pub.publish(goal_camera)

            # Publish the display frame for visualization
            detector_node.publish_disp(disp_frame)

            # Display the frame
            cv2.imshow("Color Frame", disp_frame)
            key = cv2.waitKey(1)

            if key == 27:   # Press escape to shut down node
                break
            elif key == ord('p'):
                # Open object selection window
                object_id = show_detection_buttons_modified(detections, detector_node.class_names)

                # Check if object is selected
                if object_id == None:
                    continue
                
                rospy.loginfo(detector_node.class_names[object_id]+" selected")

                # Get the ROI from the frame
                roi_box = [lst for lst in detections if lst[5] == object_id][0]
                x1, y1, x2, y2, _, _ = roi_box
                roi = color_frame[y1 + 5: y2 - 5, x1 + 5: x2 - 5]

                # Compute Histogram
                roi_hist = detector_node.compute_histogram(roi)

    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
        rospy.loginfo("Shutting Down Node")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()