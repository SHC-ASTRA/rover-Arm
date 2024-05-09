import rclpy
from rclpy.node import Node

import pygame

import time

import serial
import sys
import threading
import glob

from std_msgs.msg import String
from interfaces_pkg.msg import ControllerState

class Headless(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("headless_arm_ctrl")

        self.create_timer(1, self.send_controls)#read and send controls every 0.1 seconds


        # Create a publisher to publish any output the pico sends
        self.publisher = self.create_publisher(ControllerState, '/astra/arm/control', 10) 

        # Create a subscriber to listen to any commands sent for the pico
        self.subscriber = self.create_subscription(String, '/astra/arm/feedback', self.read_feedback, 10)
        #self.subscriber


        #self.lastMsg = String() #Used to ignore sending controls repeatedly when they do not change


        # Initialize pygame
        pygame.init()

        # Initialize the gamepad module
        pygame.joystick.init()

        # Check if any gamepad is connected
        if pygame.joystick.get_count() == 0:
            print("No gamepad found.")
            pygame.quit()
            exit()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        # Initialize the first gamepad, print name to terminal
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        print(f'Gamepad Found: {self.gamepad.get_name()}')
        #
        #


    def run(self):
        # This thread makes all the update processes run in the background
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()
        

        try:
            while rclpy.ok():
                #Check the pico for updates
                #self.send_controls()

                self.read_feedback()
                if pygame.joystick.get_count() == 0: #if controller disconnected, wait for it to be reconnected
                    print(f"Gamepad disconnected: {self.gamepad.get_name()}")
                    
                    while pygame.joystick.get_count() == 0:
                        self.send_controls()
                        self.read_feedback()
                    self.gamepad = pygame.joystick.Joystick(0)
                    self.gamepad.init() #re-initialized gamepad
                    print(f"Gamepad reconnected: {self.gamepad.get_name()}")
                    

        except KeyboardInterrupt:
            sys.exit(0)
        

    def send_controls(self):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
        input = ControllerState()

        input.lt = self.gamepad.get_axis(2)#left trigger
        input.rt = self.gamepad.get_axis(5)#right trigger
        
        #input.lb = self.gamepad.get_button(9)#Value must be converted to bool
        if(self.gamepad.get_button(4)):#left bumper
            input.lb = True
        else:
            input.lb = False

        #input.rb = self.gamepad.get_button(10)#Value must be converted to bool
        if(self.gamepad.get_button(5)):#right bumper
            input.rb = True
        else:
            input.rb = False
        
        #input.plus = self.gamepad.get_button(6)#plus button
        if(self.gamepad.get_button(7)):#plus button
            input.plus = True
        else:
            input.plus = False

        #input.minus = self.gamepad.get_button(4)#minus button
        if(self.gamepad.get_button(6)):#minus button
            input.minus = True
        else:
            input.minus = False

        input.ls_x = round(self.gamepad.get_axis(0),2)#left x-axis
        input.ls_y = round(self.gamepad.get_axis(1),2)#left y-axis
        input.rs_x = round(self.gamepad.get_axis(3),2)#right x-axis
        input.rs_y = round(self.gamepad.get_axis(4),2)#right y-axis  

        #input.a = self.gamepad.get_button(1)#A button
        if(self.gamepad.get_button(0)):#A button
            input.a = True
        else:
            input.a = False
        #input.b = self.gamepad.get_button(0)#B button
        if(self.gamepad.get_button(1)):#B button
            input.b = True
        else:
            input.b = False
        #input.x = self.gamepad.get_button(3)#X button
        if(self.gamepad.get_button(2)):#X button
            input.x = True
        else:
            input.x = False
        #input.y = self.gamepad.get_button(2)#Y button
        if(self.gamepad.get_button(3)):#Y button
            input.y = True
        else:
            input.y = False


        dpad_input = self.gamepad.get_hat(0)#D-pad input

        #not using up/down on DPad
        input.d_up = False
        input.d_down = False


        if(dpad_input[0] == 1):#D-pad right
            input.d_right = True
        else:
            input.d_right = False
        if(dpad_input[0] == -1):#D-pad left
            input.d_left = True
        else:
            input.d_left = False
       

        if pygame.joystick.get_count() != 0:
        
            self.get_logger().info(f"[Ctrl] Updated Controller State\n")

            self.publisher.publish(input)
        else:
            pass


            


    def read_feedback(self, msg):
        
        # Create a string message object
        #msg = String()

        # Set message data
        #msg.data = output

        # Publish data
        #self.publisher.publish(msg.data)
        
        print(f"[MCU] {msg.data}", end="")
        #print(f"[Pico] Publishing: {msg}")

        

def main(args=None):
    rclpy.init(args=args)

    node = Headless()
    
    rclpy.spin(node)
    rclpy.shutdown()

    #tb_bs = BaseStation()
    #node.run()


if __name__ == '__main__':
    main()
