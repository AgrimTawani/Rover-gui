#!/usr/bin/env python3

import rospy
import pygame
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np
from threading import Lock

class CameraGUI:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('gui', anonymous=True)
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((1400, 1000))
        pygame.display.set_caption("ROS GUI")
        
        # Initialize font
        self.font = pygame.font.SysFont('Arial', 26)
        
        # Camera toggles and mutex
        self.toggles = {1: True, 2: True, 3: True}
        self.image_lock = Lock()
        
        # Store the latest frames
        self.latest_frames = {1: None, 2: None, 3: None}
        
        # Camera regions
        self.cam_rects = {
            1: pygame.Rect(20, 20, 670, 480),
            2: pygame.Rect(710, 20, 670, 480),
            3: pygame.Rect(20, 520, 670, 480)
        }
        
        # Button setup
        button_width, button_height = 200, 50
        self.button_rects = {
            1: pygame.Rect(1055, 540, button_width, button_height),
            2: pygame.Rect(1055, 600, button_width, button_height),
            3: pygame.Rect(1055, 660, button_width, button_height)
        }
        self.button_texts = {
            i: self.font.render(f"Toggle Cam {i}", True, (255, 255, 255))
            for i in range(1, 4)
        }
        
        self.button_color = (0, 255, 0)
        self.hover_color = (255, 0, 0)
        
        # Set up ROS subscribers
        self.setup_subscribers()

    def setup_subscribers(self):
        """Initialize ROS subscribers with proper queue_size"""
        topics = {
            1: '/usb_cam/image_raw/compressed',
            2: '/logi_1/image_raw/compressed',
            3: '/logi_2/image_raw/compressed'
        }
        
        for cam_num, topic in topics.items():
            rospy.Subscriber(
                topic,
                CompressedImage,
                self.create_callback(cam_num),
                queue_size=1
            )

    def create_callback(self, cam_number):
        """Create a callback closure for each camera"""
        def callback(msg):
            try:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                
                with self.image_lock:
                    self.latest_frames[cam_number] = cv_image
            except Exception as e:
                rospy.logerr(f"Error processing image for camera {cam_number}: {e}")
        return callback

    def handle_button(self, event):
        """Handle button click events"""
        if event.type == pygame.MOUSEBUTTONDOWN:
            for cam_num, rect in self.button_rects.items():
                if rect.collidepoint(event.pos):
                    self.toggles[cam_num] = not self.toggles[cam_num]
                    rospy.loginfo(f"Camera {cam_num} Toggled: {self.toggles[cam_num]}")

    def update_display(self):
        """Update the display with the latest frames"""
        with self.image_lock:
            for cam_num, frame in self.latest_frames.items():
                rect = self.cam_rects[cam_num]
                
                if not self.toggles[cam_num] or frame is None:
                    # Draw black rectangle and "Camera Off" text
                    pygame.draw.rect(self.screen, (0, 0, 0), rect)
                    text = self.font.render(f"Camera {cam_num} Off", True, (255, 255, 255))
                    text_rect = text.get_rect(center=rect.center)
                    self.screen.blit(text, text_rect)
                else:
                    # Convert numpy array to pygame surface
                    try:
                        frame_surface = pygame.surfarray.make_surface(frame)
                        rotated_frame = pygame.transform.rotate(frame_surface, -90)
                        self.screen.blit(rotated_frame, rect.topleft)
                    except Exception as e:
                        rospy.logerr(f"Error displaying frame for camera {cam_num}: {e}")

    def draw_buttons(self):
        """Draw the toggle buttons"""
        mouse_pos = pygame.mouse.get_pos()
        
        for cam_num, rect in self.button_rects.items():
            color = self.hover_color if rect.collidepoint(mouse_pos) else self.button_color
            pygame.draw.rect(self.screen, color, rect)
            self.screen.blit(
                self.button_texts[cam_num],
                (rect.x + 20, rect.y + 10)
            )

    def run(self):
        """Main loop"""
        rate = rospy.Rate(30)  # 30 Hz refresh rate
        
        try:
            while not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                    self.handle_button(event)
                
                self.update_display()
                self.draw_buttons()
                pygame.display.flip()
                rate.sleep()
                
        except rospy.ROSInterruptException:
            pass
        finally:
            pygame.quit()

if __name__ == '__main__':
    try:
        gui = CameraGUI()
        gui.run()
    except Exception as e:
        rospy.logerr(f"GUI crashed: {e}")
        pygame.quit()