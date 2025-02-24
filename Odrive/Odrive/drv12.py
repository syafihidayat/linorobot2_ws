import odrive
from odrive.enums import *
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class OdriveControllerNode(Node):
    def __init__(self):
        super().__init__('odrive_controller_node')
        
        self.get_logger().info("Finding ODrive...")
        self.odrv0 = odrive.find_any()
        self.get_logger().info("ODrive found!")
        
        self.speed = 0.0

        self.odrv0.axis0.motor.config.pre_calibrated = True
        self.odrv0.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.input_vel = self.speed
        
        self.odrv0.axis1.motor.config.pre_calibrated = True
        self.odrv0.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.controller.input_vel = self.speed

        self.subscription_drive= self.create_subscription(Int32MultiArray, 'hats', self.button_callback,10)

        self.button12_pressed = False
        self.button13_pressed = False

    def button_callback(self, msg):

        if msg.data[1] > 0 and not self.button12_pressed:
            self.speed += 10.0
            self.button12_pressed = True
        elif not msg.data[1]:
            self.button12_pressed = False

        if msg.data[1] < 0 and not self.button13_pressed:
            self.speed -= 10.0
            self.button13_pressed = True
        elif not msg.data[1]:
            self.button13_pressed = False

        self.speed = max(0.0, min(self.speed, 40.0))

        
        self.odrv0.axis1.controller.input_vel = self.speed
        self.odrv0.axis0.controller.input_vel = self.speed
        self.get_logger().info(f"\nCurrent speed: {self.speed}")



def main(args=None):
    rclpy.init(args=args)
    node = OdriveControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors on shutdown
        node.get_logger().info("Stopping motors...")
        node.odrv0.axis0.controller.input_vel = 0
        node.odrv0.axis1.controller.input_vel = 0
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# odrv0.axis0.motor.config.pre_calibrated = True
# odrv0.axis0.controller.input_vel = 30
# odrv0.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

# odrv0.axis1.motor.config.pre_calibrated = True
# odrv0.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
# odrv0.axis1.controller.input_vel = 30
# while True:
#     # time.sleep(5)
#     odrv0.axis0.controller.input_vel = -40   #maksimal dengan beban speed 60-an       
#     odrv0.axis1.controller.input_vel = -40   #maksimal dengan beban speed 60-an   
#     # time.sleep(5)
#     # odrv0.axis0.controller.input_vel = 0
#     # time.sleep(5)
#     # odrv0.axis0.controller.input_vel = 50
#     # time.sleep(5)
#     # odrv0.axis0.controller.input_vel = 0
