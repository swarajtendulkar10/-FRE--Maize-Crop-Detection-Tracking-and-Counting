#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from src.sort import Sort  # Ensure you have the SORT tracker in a file named sort.py



# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/user1/git2_ws/src/obj_track/src/maize.pt', force_reload=True)
color_box = False

# Set up the RealSense D455 camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Set your YOLOv5 depth scale here
depth_scale = 0.0010000000474974513
palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'count_publisher', 10)
        self.counter=[]
        self.timer_period = 0.05   # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.subscription = self.create_subscription(String,'customtopic',self.listener_callback,10)
        self.i = 0
        self.sort_max_age = 20
        self.sort_min_hits = 1
        self.sort_iou_thresh = 0.3
        self.tracker = Sort(max_age=self.sort_max_age, min_hits=self.sort_min_hits, iou_threshold=self.sort_iou_thresh)
        self.row='right'
        self.start='start'
        self.row_number = 1
        self.msg=String()
        self.stop_flag=''

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #self.get_logger().info(f'Received parameter: "{msg.data}"')
        message = msg.data
        if 'start' in message:
            self.start=message
        
        
        if '+' in message:
            parts = message.split('+', 1)
            self.start = parts[0]
            self.row = parts[1]
            
        else:
            self.start = message
            if 'stop' in message:
                if self.stop_flag != 'stop':
                    self.stop_flag=message
                    self.msg.data=str(self.row_number)+'+'+str(len(self.counter))
                    self.row_number +=1
                    self.counter=[]
                    self.publisher_.publish(self.msg)
                


        # Logging for debugging purposes
        #self.get_logger().info(f'Received message: "{msg.data}"')
        #self.get_logger().info(f'Start: "{self.start}"')
       # self.get_logger().info(f'Row: "{self.row}"')

    def compute_color_for_labels(self, label):
        color = [int(int(p * (label ** 2 - label + 1)) % 255) for p in palette]
        return tuple(color)

    def draw_boxes(self, img, bbox, identities=None, categories=None, names=None, color_box=False, offset=(0, 0)):
        for i, box in enumerate(bbox):
            x1, y1, x2, y2 = [int(i) for i in box]
            x1 += offset[0]
            x2 += offset[0]
            y1 += offset[1]
            y2 += offset[1]
            cat = int(categories[i]) if categories is not None else 0
            id = int(identities[i]) if identities is not None else 0
            data = (int((box[0] + box[2]) / 2), (int((box[1] + box[3]) / 2)))
            label = str(id)
            if data[0]>620 and self.row=='right':
                if id not in self.counter:
                    self.counter.append(id)   
            if data[0]<620 and self.row=='right':
                if id in self.counter:
                    self.counter.remove(id)  
            if data[0]<20 and self.row=='left':
                if id not in self.counter:
                    self.counter.append(id)     
            if data[0]>20 and self.row=='left':
                if id in self.counter:
                    self.counter.remove(id) 

            if color_box:
                color = self.compute_color_for_labels(id)
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                cv2.rectangle(img, (x1, y1 - 20), (x1 + w, y1), (255, 191, 0), -1)
                cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [255, 255, 255], 1)
                cv2.circle(img, data, 3, color, -1)
            else:
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 191, 0), 2)
                cv2.rectangle(img, (x1, y1 - 20), (x1 + w, y1), (255, 191, 0), -1)
                cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [255, 255, 255], 1)
                cv2.circle(img, data, 3, (255, 191, 0), -1)
        return img

    def timer_callback(self):
        if self.start=='start':
        # Get the latest frame from the camera
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return

            # Convert the frames to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Convert the depth image to meters
            depth_image = depth_image * depth_scale

            # Detect objects using YOLOv5
            results = model(color_image)

            # Prepare detections for SORT [x1, y1, x2, y2, confidence, class_id]
            dets_to_sort = np.empty((0, 6))
            for result in results.xyxy[0]:
                x1, y1, x2, y2, confidence, class_id = result.cpu().numpy()
                if confidence>0.8:
                    dets_to_sort = np.vstack((dets_to_sort, np.array([x1, y1, x2, y2, confidence, class_id])))

            # Update tracker with the detections
            tracked_dets = self.tracker.update(dets_to_sort)

            # draw boxes for visualization
            if len(tracked_dets) > 0:
                bbox_xyxy = tracked_dets[:, :4]
                identities = tracked_dets[:, 8]
                categories = tracked_dets[:, 4]
                self.draw_boxes(color_image, bbox_xyxy, identities, categories, names=None, color_box=color_box)

            # Show the image
            print(len(self.counter))
            cv2.imshow("Color Image", color_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        minimal_publisher.destroy_node()
        rclpy.shutdown()
        pipeline.stop()  # Release the pipeline
        cv2.destroyAllWindows()  # Close any OpenCV windows


if __name__ == '__main__':
    main()
