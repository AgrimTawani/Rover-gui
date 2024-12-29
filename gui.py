#!/usr/bin/env python3

import rospy
import pygame
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np

# Global variables
screen = None
toggle1 = True
toggle2 = True
toggle3 = True

# Camera regions
cam1_rect = pygame.Rect(20, 20, 670, 480)
cam2_rect = pygame.Rect(710, 20, 670, 480)
cam3_rect = pygame.Rect(20, 520, 670, 480)

# Button regions
button_width, button_height = 200, 50
button1_rect = pygame.Rect(1055, 540, button_width, button_height)
button2_rect = pygame.Rect(1055, 600, button_width, button_height)
button3_rect = pygame.Rect(1055, 660, button_width, button_height)
button_color = (0, 255, 0)
hover_color = (255, 0, 0)

# Initialize pygame
pygame.init()

# Initialize font after pygame.init() call
font = pygame.font.SysFont('Arial', 26)

button1_text = font.render("Toggle Cam 1", True, (255, 255, 255))
button2_text = font.render("Toggle Cam 2", True, (255, 255, 255))
button3_text = font.render("Toggle Cam 3", True, (255, 255, 255))

def handle_button(mouse_event):
    global toggle1, toggle2, toggle3

    if mouse_event.type == pygame.MOUSEBUTTONDOWN:
        if button1_rect.collidepoint(mouse_event.pos):  
            toggle1 = not toggle1
            rospy.loginfo("Camera 1 Toggled: {}".format(toggle1))
        elif button2_rect.collidepoint(mouse_event.pos): 
            toggle2 = not toggle2
            rospy.loginfo("Camera 2 Toggled: {}".format(toggle2))
        elif button3_rect.collidepoint(mouse_event.pos): 
            toggle3 = not toggle3
            rospy.loginfo("Camera 3 Toggled: {}".format(toggle3))


def display_image(image_data, cam_number):    
    try:
        global toggle1, toggle2, toggle3
        # Convert ROS CompressedImage message to a NumPy array
        np_arr = np.frombuffer(image_data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode the compressed image
        
        # Convert the image from BGR to RGB for pygame
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Create Pygame surface from the NumPy array
        frame = pygame.surfarray.make_surface(cv_image) 
        
        if cam_number == 1 and toggle1:
            rotated_frame = pygame.transform.rotate(frame, -90)
            screen.blit(rotated_frame, cam1_rect.topleft)
        elif cam_number == 2 and toggle2:
            rotated_frame = pygame.transform.rotate(frame, -90)
            screen.blit(rotated_frame, cam2_rect.topleft)
        elif cam_number == 3 and toggle3:
            rotated_frame = pygame.transform.rotate(frame, -90)
            screen.blit(rotated_frame, cam3_rect.topleft)
        else:
            # Clear the camera region if toggled off
            if cam_number == 1:
                pygame.draw.rect(screen, (0, 0, 0), cam1_rect)
                text = font.render("Camera 1 Off", True, (255, 255, 255))  # White text
                text_rect = text.get_rect(center=cam1_rect.center)  # Center the text in the camera region
                screen.blit(text, text_rect)
            elif cam_number == 2:
                pygame.draw.rect(screen, (0, 0, 0), cam2_rect)
                text = font.render("Camera 2 Off", True, (255, 255, 255))
                text_rect = text.get_rect(center=cam2_rect.center)
                screen.blit(text, text_rect)
            elif cam_number == 3:
                pygame.draw.rect(screen, (0, 0, 0), cam3_rect)
                text = font.render("Camera 3 Off", True, (255, 255, 255))
                text_rect = text.get_rect(center=cam3_rect.center)
                screen.blit(text, text_rect)

        pygame.display.update()

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")


def main():    
    rospy.init_node('gui', anonymous=True)
    
    global screen
    screen = pygame.display.set_mode((1400, 1000))  
    pygame.display.set_caption("ROS GUI")

    # Subscribe to the image topics (assuming different cameras are publishing to different topics)
    rospy.Subscriber('/web_cam/image_raw/compressed', CompressedImage, lambda msg: display_image(msg, 1))
    rospy.Subscriber('/logi_1/image_raw/compressed', CompressedImage, lambda msg: display_image(msg, 2))
    rospy.Subscriber('/logi_2/image_raw/compressed', CompressedImage, lambda msg: display_image(msg, 3))

    rospy.loginfo("Image Viewer Node Started...")

    try:
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown("Pygame window closed")
                    break
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    handle_button(event)

            # Draw buttons
            pygame.draw.rect(screen, button_color if not button1_rect.collidepoint(pygame.mouse.get_pos()) else hover_color, button1_rect)
            pygame.draw.rect(screen, button_color if not button2_rect.collidepoint(pygame.mouse.get_pos()) else hover_color, button2_rect)
            pygame.draw.rect(screen, button_color if not button3_rect.collidepoint(pygame.mouse.get_pos()) else hover_color, button3_rect)

            # Blit button text
            screen.blit(button1_text, (button1_rect.x + 20, button1_rect.y + 10))
            screen.blit(button2_text, (button2_rect.x + 20, button2_rect.y + 10))
            screen.blit(button3_text, (button3_rect.x + 20, button3_rect.y + 10))   

            pygame.display.update()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()

if __name__ == '__main__':
    main()
