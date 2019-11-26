#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('object_tracker')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose

IMAGE_TOPIC = "/camera/color/image_raw"
WIDTH_TO_DEPTH_CONVERSION_FACTOR = 1.0
DEPTH_TO_XPOS_CONVERSION_FACTOR = 1.0
DEPTH_TO_YPOS_CONVERSION_FACTOR = 1.0
IMG_WIDTH = 640.0
IMG_HEIGHT = 480.0

class image_converter:

    def __init__(self):
        # Initialze global constants
        self.duration = 3.0 # Will get a new bbox every BBOX_REFRESH seconds

        # Set up ros nodes, publishers, subsribers
        self.bridge = CvBridge()

        # Initialize the tracker
        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'CSRT', 'MOSSE']
        self.tracker_type = tracker_types[6]

        self.tracker_pub = rospy.Publisher('/object_tracker/pose',Pose)
        self.bottle_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.bbox_callback)
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC,Image,self.image_callback)
        self.curr_dark_bbox = None
        self.prev_dark_bbox = None
        self.bbox = None
        self.tracker = self.create_tracker(self.tracker_type)
    
    def create_tracker(self,tracker_type):
        if self.tracker_type == 'BOOSTING':
            ret = cv2.TrackerBoosting_create()
        if self.tracker_type == 'MIL':
            ret = cv2.TrackerMIL_create()
        if self.tracker_type == 'KCF':
            ret = cv2.TrackerKCF_create()
        if self.tracker_type == 'TLD':
            ret = cv2.TrackerTLD_create()
        if self.tracker_type == 'MEDIANFLOW':
            ret = cv2.TrackerMedianFlow_create()
        if self.tracker_type == 'CSRT':
            ret = cv2.TrackerCSRT_create()
        if self.tracker_type == 'MOSSE':
            ret = cv2.TrackerMOSSE_create()
        return ret

    def bbox_callback(self,data):
        bboxes = data.bounding_boxes
        bottle = next(iter(list(filter(lambda bbox: bbox.Class == "bottle",bboxes))),None)
        if bottle:
            self.curr_dark_bbox = (bottle.xmin, bottle.ymin, bottle.xmax-bottle.xmin, bottle.ymax-bottle.ymin)

    def image_callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.curr_dark_bbox:
            if (not self.prev_dark_bbox) or (self.prev_dark_bbox != self.curr_dark_bbox):
                self.prev_dark_bbox = self.curr_dark_bbox
                self.tracker = self.create_tracker(self.tracker_type)
                self.tracker.init(frame,self.curr_dark_bbox)
            
            # Start timer
            timer = cv2.getTickCount()

            # Update tracker
            ok, self.bbox = self.tracker.update(frame)

            if ok:
                if self.bbox[2] != 0:
                    zloc = WIDTH_TO_DEPTH_CONVERSION_FACTOR/self.bbox[2]
                    xprime = float(self.bbox[2]/2.0) + float(self.bbox[0]) - IMG_WIDTH/2.0
                    yprime = float(self.bbox[3]/2.0) + float(self.bbox[1]) - IMG_HEIGHT/2.0
                    xloc = DEPTH_TO_XPOS_CONVERSION_FACTOR*zloc*xprime
                    yloc = DEPTH_TO_YPOS_CONVERSION_FACTOR*zloc*yprime

                    print("x: {}\ny: {}\nz: {}".format(xloc,yloc,zloc),"\n\n")

                    new_pose_stamp = Pose()
                    new_pose_stamp.position.x = xloc
                    new_pose_stamp.position.y = yloc
                    new_pose_stamp.position.z = zloc

                    self.tracker_pub.publish(new_pose_stamp)

                # Calculate Frames per second (FPS)
                fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

                # Draw bounding box
                if ok:
                    # Tracking success
                    p1 = (int(self.bbox[0]), int(self.bbox[1]))
                    p2 = (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3]))
                    cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
                else :
                    # Tracking failure
                    cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

                # Display tracker type on frame
                cv2.putText(frame, self.tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
            
                # Display FPS on frame
                cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

        cv2.imshow("Object Tracker", frame)
        cv2.waitKey(3)

def main(args):
    ic = image_converter()
    rospy.init_node('object_tracker', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
