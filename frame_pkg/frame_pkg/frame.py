import pyrealsense2 as rs
import cv2
import numpy as np
from .pid import PID
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from math import sqrt, pow,atan2
import rclpy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
import time



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


#  def frames_callback(self, msg):

#         if msg.data in [1,2] and not self.button2_pressed:
#             self.camera_active = not self.camera_active 
#             self.button2_pressed = True

#             if self.camera_active:
             
#                 self.pipeline = rs.pipeline()
#                 self.config = rs.config()
#                 self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#                 self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#                 self.pipeline.start(self.config)    
#                 self.timer = self.create_timer(0.1, self.process_camera)

#             else:
#                 self.get_logger().info(f'cancel camera')

#             try:
#                 self.pipeline.stop()
#                 self.get_logger().info(f'stop succseessss')


#                 if self.timer is not None:
#                     self.timer.cancel()
#                     self.timer = None
                
#             except Exception as e:
#                 self.get_logger().error(f'Error stopping camera: {str(e)}')
                
#         if msg.data == 0:
#             self.button2_pressed = False






 # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower_red1 = np.array([self.lowhue_red1, self.smin_red1, self.vmin_red1])
        # upper_red1 = np.array([self.upperhue_red1, self.smax_red1, self.vmax_red1])
        # mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)

        # lower_red2 = np.array([self.lowhue_red2, self.smin_red2, self.vmin_red2])
        # upper_red2 = np.array([self.upperhue_red2, self.smax_red2, self.vmax_red2])
        # mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # mask_red = cv2.bitwise_or(mask_red1, mask_red2)



    #    self.lowhue_blue1, self.smin_blue1,self.vmin_blue1 = 97 , 161 , 38
    #     self.upperhue_blue1, self.smax_blue1, self.vmax_blue1 = 122 ,255 , 255



#   contours,_ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         new_centroids = {}


#         for contour in contours:
#             area = cv2.contourArea(contour)
#             if 40 < area < 10000:
#                 bounding_rect = cv2.boundingRect(contour)

#                 x, y, w, h = bounding_rect  # Pecah tuple menjadi variabel terpisah
#                 cx,cy = x + w // 2, y + h // 2
#                 # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

#                 center = (x + w // 2, y + h // 2)
#                 cv2.circle(frame, center, 3, (255, 255, 255), -1)


#                 assigned_id = None
#                 for obj_id, prev_centroid in self.object_centroids.items():
#                     dist = np.linalg.norm(np.array([cx,cy]) - np.array(prev_centroid))

#                     if assigned_id is None:
#                         assigned_id = self.next_id
#                         self.next_id += 1

#                     new_centroids[assigned_id] = (cx, cy)


#                 cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 cv2.putText(frame, f"ID {assigned_id}" , (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255, 0) , 2)



#                 # bounding_rect = cv2.boundingRect(contour)

#                 # cv2.rectangle(frame, bounding_rect, (0,255,0),2)

#                 # center = (bounding_rect[0] + bounding_rect[2] // 2, bounding_rect[1] + bounding_rect[3] // 2)
#                 # cv2.circle(frame,center,3,(255,255,255), -1)
#                 dist = depth[center[1], center[0]]
                
#                 msg = Int32MultiArray()
#                 msg.data = [int(center[0]),int(center[1]), int (dist)]
#                 self.bounding_box_publisher.publish(msg)


#             self.object_centroids = new_centroids
            

class semiauto(Node):
    def __init__(self):
        super().__init__('semiauto_node')

        # self.lowhuepadi,self.sminpadi, self.vminpadi = 0, 163, 92
        # self.upperhuepadi, self.smaxpadi,self.vmaxpadi = 224, 255 , 255
        # self.minDistance , self.maxDistance = 200, 1500
        # self.threshold_value = 255

        # # Rentang HSV untuk warna merah
        # self.lowhue_red1, self.smin_red1, self.vmin_red1 = 0, 100, 100
        # self.upperhue_red1, self.smax_red1, self.vmax_red1 = 3, 255, 255
# 
        self.lowhue_red2, self.smin_red2, self.vmin_red2 = 175, 95, 95
        self.upperhue_red2, self.smax_red2, self.vmax_red2 = 180, 255, 255
# 
        self.minDistance, self.maxDistance = 200, 1500


        # self.lowhue_blue1, self.smin_blue1,self.vmin_blue1 = 100 , 161 , 38
        # self.upperhue_blue1, self.smax_blue1, self.vmax_blue1 = 122 ,255 , 255


        # self.minDistance, self.maxDistance = 200, 1500 



        self.bounding_box_center = None
        self.error_x = 0
        self.error_y = 0
        self.error_theta = 0
        self.error_distance = 0
        self.error_angle = 0
        self.button2_pressed = False
        self.button9_pressed = False
        self.frame_proses = True 

        self.object_centroids = {}   
        self.next_id  = 1   
        
        self.bounding_box_publisher = self.create_publisher(Int32MultiArray, 'bounding_box_center', 10)
        self.publisher_auto = self.create_publisher(Twist,'cmd_vel_defense',10)
        

        self.subscription_auto = self.create_subscription(Odometry,'odom',self.target_callback,10)
        self.subscription_bounding_box = self.create_subscription(Int32MultiArray,'bounding_box_center', self.bounding_box_callback,10)
        self.subscription_frame = self.create_subscription(Int32, 'button_triangle' , self.frames_callback , 10)
        self.subscription_Options = self.create_subscription(Int32, 'button_option' , self.OPtion_callback , 10)

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        # self.setup_trackbars()

        self.timer = self.create_timer(0.1, self.process_camera)


    def frames_callback(self, msg):

        if msg.data  == 1 and not self.button2_pressed:
            self.frame_proses = not self.frame_proses 
            self.button2_pressed = True

            if self.frame_proses:
             
                self.get_logger().info(f'obejct detection start')

            else:
            
                self.get_logger().info(f'obeject detection stop')
               
        elif msg.data == 0:
            self.button2_pressed = False


    def process_camera(self):

        if not self.frame_proses:

            self.bounding_box_center = None

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
    #     cv2.namedWindow("Trackbars", cv2.WINDOW_AUTOSIZE)
    #     cv2.createTrackbar("Low Hue", "Trackbars", self.lowhue_blue1, 255, lambda x: None)
    #     cv2.createTrackbar("High Hue", "Trackbars", self.upperhue_blue1, 255, lambda x: None)
    #     cv2.createTrackbar("Low Sat", "Trackbars", self.smin_blue1, 255, lambda x: None)
    #     cv2.createTrackbar("High Sat", "Trackbars", self.smax_blue1, 255, lambda x: None)
    #     cv2.createTrackbar("Min Distance", "Trackbars", self.minDistance, 2000, lambda x: None)
    #     cv2.createTrackbar("Max Distance", "Trackbars", self.maxDistance, 2000, lambda x: None)
    #     cv2.createTrackbar("Threshold", "Trackbars", self.threshold_value, 255, lambda x: None)

    
    def process_frame(self,frame, depth):
 
        # global lowhue_blue1, smin_blue1, vminpadi, upperhue_blue1, smax_blue1, vmaxpadi, minDistance, maxDistance

        # lowhue_blue1  = cv2.getTrackbarPos("Lowhue", "Trackbars")
        # upperhue_blue1 = cv2.getTrackbarPos("highhue", "Trackbars")
        # smin_blue1   = cv2.getTrackbarPos("Lowsat", "Trackbars")
        # smax_blue1 = cv2.getTrackbarPos("highsat", "Trackbars")
        # minDistance = cv2.getTrackbarPos("minDistance", "Trackbars")
        # maxDistance = cv2.getTrackbarPos("maxDistance", "Trackbars")

        # if frame is None or depth is None:
        #     self.get_logger().error("frame or depth is None")
        #     return


        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower_red1 = np.array([self.lowhue_red1, self.smin_red1, self.vmin_red1])
        # upper_red1 = np.array([self.upperhue_red1, self.smax_red1, self.vmax_red1])
        # mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([self.lowhue_red2, self.smin_red2, self.vmin_red2])
        upper_red2 = np.array([self.upperhue_red2, self.smax_red2, self.vmax_red2])
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # mask_red = cv2.bitwise_or(mask_red1, mask_red2)



        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower_blue1 = np.array([self.lowhue_blue1, self.smin_blue1, self.vmin_blue1])
        # upper_blue1 = np.array([self.upperhue_blue1, self.smax_blue1, self.vmax_blue1])
        # mask_blue1 = cv2.inRange(hsv, lower_blue1, upper_blue1)


        object_mask = (depth > self.minDistance).astype(np.uint8)
        object_mask2 = (depth < self.maxDistance).astype(np.uint8)
        # combined_mask = cv2.bitwise_and(mask_red,mask_red, mask = object_mask2)
        combined_mask = cv2.bitwise_and(mask_red2,mask_red2 , mask = object_mask2) # red
        # combined_mask = cv2.bitwise_and(mask_blue1,mask_blue1,mask = object_mask2) # blue

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
                bounding_rect= cv2.boundingRect(contour)
                x, y, w, h = bounding_rect  # Pecah tuple menjadi variabel terpisah 
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                center = (x + w // 2, y + h // 2)
                cv2.circle(frame, center, 3, (255, 255, 255), -1)

                # bounding_rect = cv2.boundingRect(contour)

                # cv2.rectangle(frame, bounding_rect, (0,255,0),2)

                # center = (bounding_rect[0] + bounding_rect[2] // 2, bounding_rect[1] + bounding_rect[3] // 2)
                # cv2.circle(frame,center,3,(255,255,255), -1)
                dist = depth[center[1], center[0]]
                
                msg = Int32MultiArray()
                msg.data = [int(center[0]),int(center[1]), int (dist)]
                self.bounding_box_publisher.publish(msg)

        cv2.imshow("Combined Mask", combined_mask)
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

    def bounding_box_callback(self, msg):
        if msg.data:
            self.bounding_box_center = [msg.data[0], msg.data[1]]
            # self.get_logger().info(f"bounding box center : {self.bounding_box_center}")


    def OPtion_callback(self , msg):
        if msg.data == 1 and not self.button9_pressed:

            self.button9_pressed = True

        
    def toDeg(self, radian):
        return radian * 180 / math.pi

   
        
    def target_callback(self, msg):

        if self.bounding_box_center is None:

                self.get_logger().warn("bounding box not detected yet!!")
                
                return
        
        quaternion = (

            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _,_,yaw = euler_from_quaternion(quaternion)
        target_x, target_y = 0,0
        robot_x ,robot_y = self.bounding_box_center
        new_x_pose_frame = (robot_y - 240) 
        new_y_pose_frame = (robot_x - 320) * -1

        self.error_x = target_x - new_x_pose_frame 
        self.error_y = target_y - new_y_pose_frame
        self.error_theta = 0 - self.toDeg(yaw)
        
        self.error_distance = sqrt(pow(self.error_x,2) + pow(self.error_y,2))
        self.error_angle = atan2(self.error_y, self.error_x)

        pid = PID()
        pid.set_base_params(0.005,0,0)
        pid.set_heading_params(0.1,0.0003,0)

        desired_linear_vel = 3.0
        desired_angular_vel = 3.0

        # controlled_distance = PID.controlling(self.error_distance,desired_linear_vel)
        # controlled_angle = pid.control_base(self.error_theta, desired_angular_vel, mode="angular")

        controlled_distance = pid.controlling(self.error_distance,desired_linear_vel)
        controlled_angle = pid.control_base(self.error_theta,desired_angular_vel)

        twist = Twist()
        twist.linear.x = controlled_distance * math.cos(self.error_angle)
        twist.linear.y = controlled_distance * math.sin(self.error_angle)
        twist.angular.z = controlled_angle
        # self.get_logger().info(f"🚀 Sending cmd_vel_defense: Linear (x={twist.linear.x}, y={twist.linear.y}), Angular={twist.angular.z}")
        self.get_logger().info(f"\n pos_x:{new_x_pose_frame}\n pos_y:{new_y_pose_frame}\nposangle:{self.toDeg(yaw)}\n out_X:{twist.linear.x}\n out_y:{twist.linear.y}\nout_angle:{twist.angular.z}\n error_x{self.error_x}\n error_y:{self.error_y}\nerror_angle:{self.error_angle}\nreturn linear:{pid.u}\nreturn angular:{pid.uT}")

        self.publisher_auto.publish(twist)


        # self.get_logger().info(f" {controlled_angle} \n  {controlled_distance}\n{self.error_x}")
        # self.get_logger().info(f"Publishing Twist: Linear Velocity (x, y): ({twist.linear.x}, {twist.linear.y}), Angular Velocity: {twist.angular.z}")

      
def main(args=None):


    rclpy.init(args=args)
    node = semiauto()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()