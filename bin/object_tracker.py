#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from cv_bridge import CvBridge, CvBridgeError

class Tracker:
    
    """
    Tracker class
    """
    def __init__(self):
        self.frame = None
        self.darknet_bbox = None
        self.bbox = (50,50,50,50)
        self.bridge = CvBridge()
        self.new_bbox = False
        rospy.init_node('tracker', anonymous=True)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.update_bounding_box)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.update_raw_image)
        rospy.Publisher('/tracker_bounging_boxes', BoundingBoxes, queue_size=10)
        self.start_tracking()

    def update_raw_image(self,img_msg):
        self.frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    
    def update_bounding_box(self,bbox_msg):
        bottles =  list(filter(lambda x: x.Class == "bottle", bbox_msg.bounding_boxes))
        if bottles:
            self.darknet_bbox = bottles[0]
            self.new_bbox = True
    
    def update_bbox(self):
        if self.new_bbox:
            x = self.darknet_bbox.xmin
            y = self.darknet_bbox.ymin
            width = self.darknet_bbox.xmax - x
            height = self.darknet_bbox.ymax - y
            self.bbox = (x,y,width,height)
            self.new_bbox = False
      
    
    def start_tracking(self):
        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'CSRT', 'MOSSE']
        tracker_type = tracker_types[6]

        if tracker_type == 'BOOSTING':
            tracker = cv2.TrackerBoosting_create()
        if tracker_type == 'MIL':
            tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv2.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv2.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
        if tracker_type == 'CSRT':
            tracker = cv2.TrackerCSRT_create()
        if tracker_type == 'MOSSE':
            tracker = cv2.TrackerMOSSE_create()
            
        # Initialize tracker with first frame and bounding box
        while(self.frame == None):
            try:
                print("Waiting for image ...")
                rospy.spin()
            except Exception:
                break
        print("Got image...")
        cv2.imshow("Tracking", self.frame)


        print("Got bottle...")
        self.update_bbox()
        ok = tracker.init(self.frame, self.bbox)
    
        while True:
            # Start timer
            timer = cv2.getTickCount()
    
            # Update tracker
            
            ok,  self.bbox = tracker.update(self.frame)
    
            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    
            # Draw bounding box
            if ok:
                # Tracking success
                p1 = (int(self.bbox[0]), int(self.bbox[1]))
                p2 = (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3]))
                cv2.rectangle(self.frame, p1, p2, (255,0,0), 2, 1)
            else :
                # Tracking failure
                cv2.putText(self.frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    
            # Display tracker type on frame
            cv2.putText(self.frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
        
            # Display FPS on frame
            cv2.putText(self.frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    
            # Display result
            cv2.imshow("Tracking", self.frame)

            rospy.spin()

        

if __name__ == '__main__':
    try:
        t = Tracker()
    except rospy.ROSInterruptException:
        pass
