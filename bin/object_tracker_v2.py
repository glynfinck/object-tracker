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

IMAGE_TOPIC = "/camera/color/image_raw"

# class Tracker():
#     """
#     Tracker class
#     """
#     def __init__(self):
#         self.frame = None
#         self.darknet_bbox = None
#         self.bbox = (50,50,50,50)
#         self.new_bbox = False

#         rospy.init_node('tracker', anonymous=True)
#         rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.update_bounding_box)
        
#         self.start_tracking()

#     def update_raw_image(self,img_msg):
#         self.frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    
#     def update_bounding_box(self,bbox_msg):
#         bottles =  list(filter(lambda x: x.Class == "bottle", bbox_msg.bounding_boxes))
#         if bottles:
#             self.darknet_bbox = bottles[0]
#             self.new_bbox = True
    
#     def update_bbox(self):
#         if self.new_bbox:
#             x = self.darknet_bbox.xmin
#             y = self.darknet_bbox.ymin
#             width = self.darknet_bbox.xmax - x
#             height = self.darknet_bbox.ymax - y
#             self.bbox = (x,y,width,height)
#             self.new_bbox = False
      
    
#     def start_tracking(self):
#         tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'CSRT', 'MOSSE']
#         tracker_type = tracker_types[6]

#         if tracker_type == 'BOOSTING':
#             tracker = cv2.TrackerBoosting_create()
#         if tracker_type == 'MIL':
#             tracker = cv2.TrackerMIL_create()
#         if tracker_type == 'KCF':
#             tracker = cv2.TrackerKCF_create()
#         if tracker_type == 'TLD':
#             tracker = cv2.TrackerTLD_create()
#         if tracker_type == 'MEDIANFLOW':
#             tracker = cv2.TrackerMedianFlow_create()
#         if tracker_type == 'CSRT':
#             tracker = cv2.TrackerCSRT_create()
#         if tracker_type == 'MOSSE':
#             tracker = cv2.TrackerMOSSE_create()
            
#         # Initialize tracker with first frame and bounding box
#         while(self.frame == None):
#             try:
#                 print("Waiting for image ...")
#                 rospy.spin()
#             except Exception:
#                 break
#         print("Got image...")
#         cv2.imshow("Tracking", self.frame)


#         print("Got bottle...")
#         self.update_bbox()
#         ok = tracker.init(self.frame, self.bbox)
    
#         while True:
#             # Start timer
#             timer = cv2.getTickCount()
    
#             # Update tracker
            
#             ok,  self.bbox = tracker.update(self.frame)
    
#             # Calculate Frames per second (FPS)
#             fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    
#             # Draw bounding box
#             if ok:
#                 # Tracking success
#                 p1 = (int(self.bbox[0]), int(self.bbox[1]))
#                 p2 = (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3]))
#                 cv2.rectangle(self.frame, p1, p2, (255,0,0), 2, 1)
#             else :
#                 # Tracking failure
#                 cv2.putText(self.frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    
#             # Display tracker type on frame
#             cv2.putText(self.frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
        
#             # Display FPS on frame
#             cv2.putText(self.frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    
#             # Display result
#             cv2.imshow("Tracking", self.frame)

#             rospy.spin()

class Tracker:

    def __init__(self):
        # Initialze global constants
        self.duration = 2.0 # Will get a new bbox every BBOX_REFRESH seconds

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
        


    def track(self,frame,bbox,duration):
        start = time.time()
        print("Tracking bottle...")
        tracker = self.create_tracker(self.tracker_type)
        tracker.init(frame,bbox)
        while(time.time() - start < self.duration):
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
                self.track(frame,bbox,self.duration)
        
def main(args):
    tracker = Tracker()
    

if __name__ == '__main__':
    main(sys.argv)