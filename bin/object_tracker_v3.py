#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('object_tracker')
import sys
import rospy
import cv2
import time
import concurrent.futures
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes, _BoundingBoxes

# Initialze global constants
IMAGE_TOPIC = "/camera/color/image_raw" 

class Tracker:

    def __init__(self):
        # Set up ros nodes, publishers, subsribers
        rospy.init_node('object_tracker', anonymous=True)
        self.bridge = CvBridge()

        # Initialize the tracker
        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'CSRT', 'MOSSE']
        self.tracker_type = tracker_types[6]

        self.start_tracking()

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
        


    def track(self,frame,bbox):
        start = time.time()
        print("Tracking bottle...")
        tracker = self.create_tracker(self.tracker_type)
        tracker.init(frame,bbox)
        bottle_loc = bbox
        while(bottle_loc == bbox):
            bboxes = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes).bounding_boxes
            bottle = next(iter(list(filter(lambda bbox: bbox.Class == "bottle",bboxes))),None)
            if bottle:
                bbox = (bottle.xmin, bottle.ymin, bottle.xmax-bottle.xmin, bottle.ymax-bottle.ymin)
            # Read a new frame
            frame = self.bridge.imgmsg_to_cv2(rospy.wait_for_message(IMAGE_TOPIC, Image),'bgr8')
            
            # Start timer
            timer = cv2.getTickCount()
    
            # Update tracker
            ok, bbox = tracker.update(frame)

            if not ok:
                break
    
            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    
            # Draw bounding box
            if ok:
                # Tracking success
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
            else :
                # Tracking failure
                cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    
            # Display tracker type on frame
            cv2.putText(frame, self.tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
        
            # Display FPS on frame
            cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    
            # Display result
            cv2.imshow("Tracking", frame)
    
            # Exit if ESC pressed
            k = cv2.waitKey(1) & 0xff
            if k == 27 : break

        print("Done tracking bottle")
    
    def start_tracking(self):
        
        while(True):
            bboxes = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes).bounding_boxes
            bottle = next(iter(list(filter(lambda bbox: bbox.Class == "bottle",bboxes))),None)
            print(bottle)
            if bottle != None:
                print("I see a bottle!")
                bbox = (bottle.xmin, bottle.ymin, bottle.xmax-bottle.xmin, bottle.ymax-bottle.ymin)
                frame = self.bridge.imgmsg_to_cv2(rospy.wait_for_message(IMAGE_TOPIC, Image),'bgr8')
                self.track(frame,bbox)
        
def main(args):
    tracker = Tracker()
    

if __name__ == '__main__':
    main(sys.argv)