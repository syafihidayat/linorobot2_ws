import sys
import pygame
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import rclpy
from rclpy.node import Node
import time

class GamePad(Node):
    def __init__(self):
        super().__init__("game_pad")
        
        self.publisher_axis = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        self.publisher_button = self.create_publisher(Int32MultiArray, 'button', 10)
        self.publisher_hats = self.create_publisher(Int32MultiArray, 'hats', 10)
        self.publisher_R1 = self.create_publisher(Int32, 'button_R1' , 10)
        self.publisher_L1 = self.create_publisher(Int32, 'button_L1', 10)
        self.publisher_Triangle = self.create_publisher(Int32,'button_triangle',10)
        self.publisher_Option = self.create_publisher(Int32, 'button_option', 10)
       
        
        # Inisialisasi pygame
        pygame.init()
        pygame.joystick.init()

        
        self.connect_joystick()

        self.get_logger().info(f"Controller connected: {self.joystick.get_name()}")

        # Inisialisasi atribut tambahan 
        self.speed = 0.0  # Set default speed
        self.button6_pressed = False
        self.button7_pressed = False

        # Timer untuk memanggil callback axis dan button
        self.create_timer(0.1, self.axis_callback)
        self.create_timer(0.1, self.button_callback)
        self.create_timer(0.1, self.micro_callback)
        self.create_timer(0.1, self.micro2_callback)
        self.create_timer(0.1, self.frame_callback )
        self.create_timer(0.1, self.options_callback)
    
    def connect_joystick(self):
        """Attempts to connect to the joystick. Retries if not available."""
        while True:
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"Controller connected: {self.joystick.get_name()}")
                break
            else:
                self.get_logger().warn("No joystick connected. Retrying in 2 seconds...")
                time.sleep(2)

    def check_joystick_connection(self):
        """Checks if the joystick is still connected and attempts to reconnect if necessary."""
        try:
            self.joystick.get_name()  # Will throw an exception if disconnected
        except pygame.error:
            self.get_logger().warn("Joystick disconnected. Attempting to reconnect...")
            self.connect_joystick()

    def button_callback(self):
        pygame.event.pump()

        msg = Int32MultiArray()
        hats_msg = Int32MultiArray()

        # Mengakses status tombol dan hats joystick
        button_states = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        hat_states = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]

        # Mengisi pesan ROS 2 dengan data joystick
        msg.data = button_states
        hats_msg.data = [state for hat in hat_states for state in hat]  # Flatten tuple to list

        # Publish data tombol dan hats
        self.publisher_button.publish(msg)
        self.publisher_hats.publish(hats_msg)

        # Menampilkan log kecepatan saat ini
        self.get_logger().info(f'Current Speed: {self.speed}')
        # self.get_logger().info(f'Button states: {button_states}')
        # self.get_logger().info(f'Hats states: {hat_states}')


    def axis_callback(self):
        pygame.event.pump()

        # Mengubah kecepatan dengan tombol 6 dan 7
        if self.joystick.get_button(6) and not self.button6_pressed:
            self.speed -= 1.0
            self.button6_pressed = True  # Pastikan tombol tidak tertekan berulang kali
        elif not self.joystick.get_button(6):
            self.button6_pressed = False

        if self.joystick.get_button(7) and not self.button7_pressed:
            self.speed += 1.0
            self.button7_pressed = True
        elif not self.joystick.get_button(7):
            self.button7_pressed = False

        # Batas minimum kecepatan
        if self.speed < 0.0:
            self.speed = 0.0

        # Mengambil nilai dari axis joystick
        linear_axis_Y = self.joystick.get_axis(0) * self.speed
        linear_axis_X = -self.joystick.get_axis(1) * self.speed
        angular_axis_Z = self.joystick.get_axis(3) * self.speed

        # Menghilangkan noise dari joystick
        if abs(linear_axis_X) < 0.03:
            linear_axis_X = 0.0
        if abs(linear_axis_Y) < 0.03:
            linear_axis_Y = 0.0
        if abs(angular_axis_Z) < 0.03:
            angular_axis_Z = 0.0
        if linear_axis_X == 0.0 and linear_axis_Y == 0.0 and angular_axis_Z == 0.0:
            return 
        # Membuat pesan Twist
        twist = Twist()
        twist.linear.x = linear_axis_X
        twist.linear.y = linear_axis_Y
        twist.angular.z = angular_axis_Z 

        # Mempublikasikan ke cmd_vel
        self.publisher_axis.publish(twist)

        # Logging informasi
        self.get_logger().info(f"\nLinear Velocity X: {linear_axis_X}\nLinear Velocity Y: {linear_axis_Y}\nAngular Velocity: {angular_axis_Z}\n")
        self.get_logger().info(f"\nCurrent speed: {self.speed}")

    def micro_callback(self):
        pygame.event.pump()

        msg_R1= Int32()
       
        msg_R1.data = self.joystick.get_button(5)

        self.get_logger().info(f"R1: {msg_R1.data}")
    
        self.publisher_R1.publish(msg_R1)
     

    def micro2_callback(self):
        pygame.event.pump()

        msg_L1 = Int32()

        msg_L1.data = self.joystick.get_button(4)

        self.get_logger().info(f"L1: {msg_L1.data}")

        self.publisher_L1.publish(msg_L1)


    def frame_callback(self):
        pygame.event.pump()

        msg_Tring = Int32()

        msg_Tring.data = self.joystick.get_button(2)

        self.get_logger().info(f"triangle: {msg_Tring.data}")
        
        self.publisher_Triangle.publish(msg_Tring)

    def options_callback(self):
        pygame.event.pump()

        msg_option = Int32()

        msg_option.data = self.joystick.get_button(9)

        self.get_logger().info(f"option : {msg_option.data}")

        self.publisher_Option.publish(msg_option)


def main(args=None):
    rclpy.init(args=args)

    game_pad = GamePad()

    rclpy.spin(game_pad)

    game_pad.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
