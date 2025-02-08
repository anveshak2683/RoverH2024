import pygame
import pygame.camera

# Initialize Pygame and the camera
pygame.init()
pygame.camera.init()

# Get a list of available cameras
camlist = pygame.camera.list_cameras()
print(camlist)
if not camlist:
    raise ValueError("No cameras detected.")

# Select the first camera and set its resolution
cam = pygame.camera.Camera(camlist[0], (640, 480))
cam.start()

# Set up the Pygame display in fullscreen mode
screen_width, screen_height = pygame.display.Info().current_w, pygame.display.Info().current_h
gameDisplay = pygame.display.set_mode((screen_width, screen_height), pygame.FULLSCREEN)
pygame.display.set_caption("Camera Feed in Fullscreen")

# Define the size and position of the camera feed
camera_width, camera_height = 320, 240  # Resize the camera feed
camera_position = (1200, 200)  # Top-left corner

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False
    
    # Get the camera image
    img = cam.get_image()
    
    # Scale the camera image to fit the defined size
    img_resized = pygame.transform.scale(img, (camera_width, camera_height))
    
    # Fill the screen with a background color
    gameDisplay.fill((0, 0, 0))  # Black background
    
    # Blit the resized camera feed to the screen
    gameDisplay.blit(img_resized, camera_position)
    
    # Update the display
    pygame.display.update()

# Stop the camera and quit Pygame
cam.stop()
pygame.quit()
exit()

