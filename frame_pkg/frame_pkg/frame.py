import pyrealsense2 as rs
import cv2
import numpy as np
from . import pid
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from math import sqrt, pow,atan2
import rclpy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32

import math
# Global variables
# lowhuepadi, sminpadi, vminpadi = 164, 105, 0
# upperhuepadi, smaxpadi, vmaxpadi = 178, 255, 255
# minDistance, maxDistance = 500, 1500
# threshold_value = 255

# pBackSub = cv2.createBackgroundSubtractorKNN()

# def setup_trackbars():
#     cv2.namedWindow("Trackbars", cv2.WINDOW_AUTOSIZE)
#     cv2.createTrackbar("Low Hue", "Trackbars", lowhuepadi, 255, lambda x: None)
#     cv2.createTrackbar("High Hue", "Trackbars", upperhuepadi, 255, lambda x: None)
#     cv2.createTrackbar("Min Distance", "Trackbars", minDistance, 2000, lambda x: None)
#     cv2.createTrackbar("Max Distance", "Trackbars", maxDistance, 2000, lambda x: None)
#     cv2.createTrackbar("Threshold", "Trackbars", threshold_value, 255, lambda x: None)


# bounding_box_publisher = None


# def process_frame(frame, depth):
#     global lowhuepadi, sminpadi, vminpadi, upperhuepadi, smaxpadi, vmaxpadi, minDistance, maxDistance

#       # Update trackbar values
#     lowhuepadi = cv2.getTrackbarPos("Low Hue", "Trackbars")
#     upperhuepadi = cv2.getTrackbarPos("High Hue", "Trackbars")
#     minDistance = cv2.getTrackbarPos("Min Distance", "Trackbars")
#     maxDistance = cv2.getTrackbarPos("Max Distance", "Trackbars")

#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


#     lower = np.array([lowhuepadi, sminpadi, vminpadi])
#     upper = np.array([upperhuepadi, smaxpadi, vmaxpadi])
#     mask = cv2.inRange(hsv, lower, upper)


#      # Create depth masks
#     object_mask = (depth > minDistance).astype(np.uint8)
#     object_mask2 = (depth < maxDistance).astype(np.uint8)

#      # Combine masks
#     combined_mask = cv2.bitwise_and(mask, mask, mask=object_mask2)

#     # Morphological operations
#     kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
#     combined_mask = cv2.erode(combined_mask, kernel)
#     combined_mask = cv2.dilate(combined_mask, kernel)

#      # Draw centerlines
#     y_line = 240  # Frame height / 2
#     x_line = 320  # Frame width / 2
#     cv2.line(frame, (0, y_line), (frame.shape[1], y_line), (0, 0, 0), 2)
#     cv2.line(frame, (x_line, 0), (x_line, frame.shape[0]), (0, 0, 0), 2)

#     # Find contours
#     contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     for contour in contours:
#         area = cv2.contourArea(contour)
#         if 40 < area < 10000:
#             bounding_rect = cv2.boundingRect(contour)
#             cv2.rectangle(frame, bounding_rect, (0, 255, 0), 2)

#             center = (bounding_rect[0] + bounding_rect[2] // 2, bounding_rect[1] + bounding_rect[3] // 2)
#             cv2.circle(frame, center, 3, (255, 255, 255), -1)

#             dist = depth[center[1], center[0]]
#             # print(f"Bounding Box Center: {center}")

#             if bounding_box_publisher is not None:
#                 msg = Int32MultiArray()
#                 msg.data = [int(center[0]),int (center[1]), int (dist)]
#                 bounding_box_publisher.publish(msg)

#     # Display results
#     cv2.imshow("Combined Mask", combined_mask)
#     cv2.imshow("Frame", frame)


# def frame_detection(node):

#     global bounding_box_publisher

#     bounding_box_publisher = node.create_publisher(Int32MultiArray, 'bounding_box_center',10)
    
#     pipeline = rs.pipeline()
#     config = rs.config()
#     config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
#     pipeline.start(config)
#     # setup_trackbars()

#     try:
#         while rclpy.ok():


#             if node.button2_pressed:
#                 print("cancelframe")
#                 break

#             frames = pipeline.wait_for_frames()
#             color_frame = frames.get_color_frame()
#             depth_frame = frames.get_depth_frame()

#             if not color_frame or not depth_frame:
#                 print("Error: Frames not captured correctly!")
#                 continue
            
#             # Convert frames to numpy arrays
#             depth = np.asanyarray(depth_frame.get_data())
#             frame = np.asanyarray(color_frame.get_data())

#             process_frame(frame, depth)

#             key = cv2.waitKey(1) & 0xFF
#             if key == ord('q'):
#                 break

#     except Exception as e:
#         print(f"Error: {e}")
#     finally:
#         pipeline.stop()
#         cv2.destroyAllWindows()
        

# class MechaControl:
    # def __init__(self):
    #     self.k_p = 0
    #     self.k_i = 0
    #     self.k_d = 0

    # def setBaseParam(self, k_p, k_i,k_d):
    #     self.k_p = k_p
    #     self.k_i = k_i
    #     self.k_d = k_d

    # def getBaseParam(self):
    #     return{"k_p" : self.k_p, "k_i": self.k_i, "k_d": self.k_d}

# class PID:
#     # Membuat objek mecha_linear dan mecha_angular
#     mecha_linear = MechaControl()
#     mecha_angular = MechaControl()

# class MechaControl_:
    # def __init__(self):
        # self.max_linear_vel = 1.0
        # self.max_angular_vel = 1.0
        # 
    # def control_base_(self, error, desired_vel):
        # return desired_vel * (1- abs(error) / self.max_linear_vel)

class semiauto(Node):
    def __init__(self):
        super().__init__('semiauto_node')

        self.lowhuepadi,self.sminpadi, self.vminpadi = 164, 105, 0
        self.upperhuepadi, self.smaxpadi,self.vmaxpadi = 178, 255 , 255
        self.minDistance , self.maxDistance, self.threshold_value = 200, 1500, 255
        self.threshold_value = 255

        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.0

        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0

        self.max_linear_vel = 1.0
        self.max_angular_vel = 1.0

        self.bounding_box_center = None
        self.error_x = 0
        self.error_y = 0
        self.error_theta = 0
        self.error_distance = 0
        self.error_angle = 0
        self.tolerance_x = 10
        self.tolerance_y = 10
        self.button2_pressed = False
        # self.Mecha_linear_ = MechaControl_()
        # self.Mecha_angular_= MechaControl_()

        # pBackSub = cv2.createBackgroundSubtractorKNN()
        # setup_trackbars()
        # global bounding_box_publisher

        self.bounding_box_publisher = self.create_publisher(Int32MultiArray, 'bounding_box_center', 10)
        self.publisher_auto = self.create_publisher(Twist,'cmd_vel_defense',10)

        self.subscription_auto = self.create_subscription(Pose2D,'robot_position',self.target_callback,10)
        self.subscription_bounding_box = self.create_subscription(Int32MultiArray,'bounding_box_center', self.bounding_box_callback,10)
        self.subscription_cancelF = self.create_subscription(Int32, 'button_triangle', self.trifram_callback,10)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        # self.setup_trackbars()

        self.timer = self.create_timer(0.1, self.process_camera)



    def process_camera(self):
        if self.button2_pressed:
            return

        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return

            depth = np.asanyarray(depth_frame.get_data())
            frame = np.asanyarray(color_frame.get_data())
            self.process_frame(frame, depth)

        except Exception as e:
            self.get_logger().error(f"error:{e}")



    # def setup_trackbars(self):
        # cv2.namedWindow("Trackbars", cv2.WINDOW_AUTOSIZE)
        # cv2.createTrackbar("Low Hue", "Trackbars", self.lowhuepadi, 255, lambda x: None)
        # cv2.createTrackbar("High Hue", "Trackbars", self.upperhuepadi, 255, lambda x: None)
        # cv2.createTrackbar("Min Distance", "Trackbars", self.minDistance, 2000, lambda x: None)
        # cv2.createTrackbar("Max Distance", "Trackbars", self.maxDistance, 2000, lambda x: None)
        # cv2.createTrackbar("Threshold", "Trackbars", self.threshold_value, 255, lambda x: None)

        # bounding_box_publisher = None

    def process_frame(self,frame, depth):
        # global lowhuepadi, sminpadi, vminpadi, upperhuepadi, smaxpadi, vmaxpadi, minDistance, maxDistance

        # lowhuepadi   = cv2.getTrackbarPos("Lowhue", "Trackbars")
        # upperhuepadi = cv2.getTrackbarPos("highhue", "Trackbars")
        # minDistance = cv2.getTrackbarPos("minDistance", "Trackbars")
        # maxDistance = cv2.getTrackbarPos("maxDistance", "Trackbars")

        # if frame is None or depth is None:
        #     self.get_logger().error("frame or depth is None")
        #     return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([self.lowhuepadi, self.sminpadi, self.vminpadi])
        upper = np.array([self.upperhuepadi, self.smaxpadi, self.vmaxpadi])
        mask = cv2.inRange(hsv, lower, upper)


        object_mask = (depth > self.minDistance).astype(np.uint8)
        object_mask2 = (depth < self.maxDistance).astype(np.uint8)
        combined_mask = cv2.bitwise_and(mask,mask, mask = object_mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
        combined_mask = cv2.erode(combined_mask, kernel)
        combined_mask = cv2.dilate(combined_mask, kernel)



        y_line = 240
        x_line = 320
        cv2.line(frame, (0, y_line), (frame.shape[1], y_line), (0,0,0), 2)
        cv2.line(frame, (x_line, 0), (x_line, frame.shape[0]), (0,0,0), 2)

        contours,_ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if 40 < area < 10000:
                bounding_rect = cv2.boundingRect(contour)
                cv2.rectangle(frame, bounding_rect, (0,255,0),2)
                center = (bounding_rect[0] + bounding_rect[2] // 2, bounding_rect[1] + bounding_rect[3] // 2)
                dist = depth[center[1], center[0]]
                #print(f"bounding box center: {center}")

                # if self.bounding_box_publisher is not None:
                msg = Int32MultiArray()
                msg.data = [int(center[0]),int(center[1]), int (dist)]
                self.bounding_box_publisher.publish(msg)


        cv2.imshow("Combined Mask", combined_mask)
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

    


    def set_base_param(self, kp, ki, kd, mode="linear"):
        if mode == "linear":
            self.kp_linear = kp
            self.ki_linear = ki
            self.kd_linear = kd
        elif mode == "angular":
            self.kp_angular = kp
            self.ki_angular = ki
            self.kd_angular = kd

    def control_base(self, error,desired_vel, mode="linear"):
        if mode == "linear":
            return desired_vel * (1 - abs(error) / self.max_linear_vel)
        elif mode == "angular":
            return desired_vel * (1 - abs(error) / self.max_angular_vel)

    # def control_base(self, error, desired_vel, mode="linear"):
    #     if mode == "linear":
    #      return self.kp_linear * error
    #     elif mode == "angular":
    #      return self.kp_angular * error


    def trifram_callback(self, msg):
        if msg.data > 0 and not self.button2_pressed:
            print("triangle 1")
            self.button2_pressed =  True


    def bounding_box_callback(self, msg):
        if msg.data:
            self.bounding_box_center = [msg.data[0], msg.data[1]]
            # self.get_logger().info(f"bounding box center : {self.bounding_box_center}")

            # return

        
    def target_callback(self, msg):

        self.get_logger().info("📡 target_callback() triggered!")       
        
        if self.bounding_box_center is None:
                self.get_logger().warn("bounding box not detected yet!!")
                return

        target_x ,target_y = [320,240]
        robot_x ,robot_y = self.bounding_box_center

        self.error_x = target_x - robot_x
        self.error_y = target_y - robot_y
        self.error_theta = msg.theta - 0

        # if abs (self.error_x - 320) <= self.tolerance_x:
        #     self.error_x = 0
        # if abs (self.error_y - 250) <= self.tolerance_y:
        #     self.error_y = 0

        self.error_distance = sqrt(pow(self.error_x,2) + pow(self.error_y,2))
        self.error_angle = atan2(self.error_y, self.error_x)

        # PID.mecha_linear.setBaseParam(1,0,0)
        # PID.mecha_angular.setBaseParam(1,0,0)

        self.set_base_param(1,0,0, mode= "linear")
        self.set_base_param(1,0,0, mode= "angular")

        desired_linear_vel = 0.5
        desired_angular_vel = 0.5

        controlled_distance = self.control_base(self.error_distance,desired_linear_vel , mode="linear")
        controlled_angle = self.control_base(self.error_theta, desired_angular_vel, mode="angular")

        twist = Twist()
        twist.linear.x = controlled_distance * math.cos(self.error_theta)
        twist.linear.y = controlled_distance * math.sin(self.error_theta)
        twist.angular.z = controlled_angle
        self.get_logger().info(f"🚀 Sending cmd_vel_defense: Linear (x={twist.linear.x}, y={twist.linear.y}), Angular={twist.angular.z}")

        self.publisher_auto.publish(twist)


        # self.get_logger().info(f" {controlled_angle} \n  {controlled_distance}\n{self.error_x}")
        # self.get_logger().info(f"Publishing Twist: Linear Velocity (x, y): ({twist.linear.x}, {twist.linear.y}), Angular Velocity: {twist.angular.z}")
        
      
def main(args=None):


    rclpy.init(args=args)
    node = semiauto()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # rclpy.init()
# 
    # node = semiauto()
    # try:
        # rclpy.spin(node)
    # except KeyboardInterrupt:
        # pass
# 
    # finally:
        # node.destroy_node()
        # rclpy.shutdown()
        # cv2.destroyAllWindows()


if __name__ == "__main__":
    main()