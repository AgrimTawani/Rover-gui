#!/usr/bin/env python3

import rospy
import pygame
import subprocess  # For running scripts in the terminal
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraGUI:
    def __init__(self):
        rospy.init_node('camera_gui', anonymous=True)
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((1880, 950))
        pygame.display.set_caption("DJS ANTARIKSH ROVER GUI")
        
        # Initialize font and cv_bridge
        self.font = pygame.font.SysFont('Arial', 24)
        self.bridge = CvBridge()
        
        # Navbar configuration
        self.navbar_height = 50
        self.navbar_color = (211, 211, 211)  # Light gray
        self.button_color = (0, 0, 255)  # Blue
        self.button_hover_color = (0, 100, 255)  # Lighter blue
        self.close_button_color = (255, 0, 0)  # Red for the Close button
        
        # Adjusted button positions with 250px width and gaps
        self.buttons = {
            1: pygame.Rect(100, 10, 250, 30),   # Button 1
            2: pygame.Rect(360, 10, 250, 30),   # Button 2
            3: pygame.Rect(620, 10, 250, 30),   # Button 3
            4: pygame.Rect(880, 10, 250, 30),   # Sensor Data Button
            5: pygame.Rect(1600, 10, 250, 30)   # Close Button (Right-most)
        }
        
        # Adjusted button labels
        self.button_labels = {
            1: "Combined Launch",
            2: "Teensy Launch",
            3: "Cams Launch",
            4: "Sensor Data",  # Added the Sensor Data button
            5: "Close"  # Added the Close button
        }
        
        # Camera configuration
        self.latest_frames = {i: None for i in range(1, 7)}
        self.cam_rects = {
            1: pygame.Rect(20, 70, 600, 450),    # Top-left
            2: pygame.Rect(640, 70, 600, 450),   # Top-center
            3: pygame.Rect(1260, 70, 600, 450),  # Top-right
            4: pygame.Rect(20, 540, 600, 450),   # Bottom-left
            5: pygame.Rect(640, 540, 600, 450),  # Bottom-center
            6: pygame.Rect(1260, 540, 600, 450)  # Bottom-right
        }
        
        # Subscribe to camera topics
        self.subscribe_to_topics()
    
    def subscribe_to_topics(self):
        """Subscribe to camera topics."""
        topics = {
            1: '/logi0/image_raw/compressed',
            2: '/logi1/image_raw/compressed',
            3: '/logi2/image_raw/compressed',
            4: '/logi3/image_raw/compressed',
            5: '/logi4/image_raw/compressed',
            6: '/logi5/image_raw/compressed'
        }
        for cam, topic in topics.items():
            rospy.Subscriber(topic, CompressedImage, lambda msg, cam=cam: self.image_callback(cam)(msg))

    def image_callback(self, cam):
        """Callback for camera topics to process and store images."""
        def callback(msg):
            try:
                # Use cv_bridge to convert the image
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="rgb8")
                self.latest_frames[cam] = cv_image
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge error for camera {cam}: {e}")
        return callback

    def draw_navbar(self):
        """Draw the navigation bar and buttons."""
        # Draw the navbar
        pygame.draw.rect(self.screen, self.navbar_color, (0, 0, 1880, self.navbar_height))
        
        # Draw the buttons
        mouse_pos = pygame.mouse.get_pos()
        for button_id, rect in self.buttons.items():
            if button_id == 5:  # Close button (Red)
                color = self.close_button_color if rect.collidepoint(mouse_pos) else self.close_button_color
            else:
                color = self.button_hover_color if rect.collidepoint(mouse_pos) else self.button_color
            
            pygame.draw.rect(self.screen, color, rect)
            label = self.font.render(self.button_labels[button_id], True, (255, 255, 255))
            label_rect = label.get_rect(center=rect.center)
            self.screen.blit(label, label_rect)

    def draw_camera(self, cam):
        """Draw the camera feed on the screen."""
        frame = self.latest_frames[cam]
        rect = self.cam_rects[cam]
        
        if frame is not None:
            try:
                surface = pygame.surfarray.make_surface(frame)
                self.screen.blit(pygame.transform.rotate(surface, -90), rect.topleft)
            except Exception as e:
                rospy.logerr(f"Error rendering camera {cam}: {e}")
        else:
            pygame.draw.rect(self.screen, (0, 0, 0), rect)
            text = self.font.render(f"No Feed from Camera {cam}", True, (255, 255, 255))
            self.screen.blit(text, text.get_rect(center=rect.center))

    def handle_events(self):
        """Handle pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown("Pygame closed")
            elif event.type == pygame.MOUSEBUTTONDOWN:
                for button_id, rect in self.buttons.items():
                    if rect.collidepoint(event.pos):
                        if button_id == 1:
                            rospy.loginfo("Combined Launch clicked!")
                            self.run_script('./scripts/combined_launch.sh')  # Replace with your script
                        elif button_id == 2:
                            rospy.loginfo("Teensy Launch clicked!")
                            self.run_script('./scripts/teensy_launch.sh')  # Replace with your script
                        elif button_id == 3:
                            rospy.loginfo("Cams Launch clicked!")
                            self.run_script('./scripts/cams_launch.sh')  # Replace with your script
                        elif button_id == 4:
                            rospy.loginfo("Sensor Data clicked!")
                            self.run_script('./scripts/sensor_data.sh')  # Replace with your script
                        elif button_id == 5:
                            rospy.loginfo("Close button clicked!")
                            self.run_script('./scripts/close.sh') 
                            pygame.quit()  # Quit the pygame window

    def run_script(self, script_name):
        """Run a terminal script."""
        try:
            # Replace 'script_name' with the path to your script
            subprocess.Popen(['bash', script_name])  # This runs the script in a new terminal
        except Exception as e:
            rospy.logerr(f"Failed to run script {script_name}: {e}")

    def run(self):
        """Main loop to render the interface."""
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.handle_events()
            self.screen.fill((0, 0, 0))
            self.draw_navbar()  # Draw the navbar
            for cam in self.cam_rects:
                self.draw_camera(cam)
            pygame.display.flip()
            rate.sleep()
        pygame.quit()

if __name__ == '__main__':
    try:
        gui = CameraGUI()
        gui.run()
    except Exception as e:
        rospy.logerr(f"Camera GUI Error: {e}")
        pygame.quit()
