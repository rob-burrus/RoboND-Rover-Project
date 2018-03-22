import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh_min=(160, 160, 160), rgb_thresh_max=(255,255,255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] >= rgb_thresh_min[0]) \
                & (img[:,:,1] >= rgb_thresh_min[1]) \
                & (img[:,:,2] >= rgb_thresh_min[2])
    below_thresh = (img[:,:,0] <= rgb_thresh_max[0]) \
                & (img[:,:,1] <= rgb_thresh_max[1]) \
                & (img[:,:,2] <= rgb_thresh_max[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh & below_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

def is_valid_rotation(Rover):
    abs_roll = 360 - Rover.roll if Rover.roll > 180 else Rover.roll
    abs_pitch = 360 - Rover.pitch if Rover.pitch > 180 else Rover.pitch
    return abs_roll < 1.0 and abs_pitch < 1.0


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    
    image = Rover.img
    
    # 1) Apply color threshold to identify navigable terrain/obstacles/rock samples
    obstacle_thresh = color_thresh(image, (0,0,0), (160,160,160))
    navigable_thresh = color_thresh(image, (160,160,160))
    rock_thresh = color_thresh(image, (100,100,0), (255, 255, 20))
    
    # 3) Define source and destination points for perspective transform
    
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                    [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                    [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                    [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                    ])
    # 3) Apply perspective transform
    obstacle_warped = perspect_transform(obstacle_thresh, source, destination)
    navigable_warped = perspect_transform(navigable_thresh, source, destination)
    rock_warped = perspect_transform(rock_thresh, source, destination)
    


    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacle_warped*255
    Rover.vision_image[:,:,1] = rock_warped*255
    Rover.vision_image[:,:,2] = navigable_warped*255


    # 5) Convert map image pixel values to rover-centric coords
    obstacle_x_rover, obstacle_y_rover = rover_coords(obstacle_warped)
    navigable_x_rover, navigable_y_rover = rover_coords(navigable_warped)
    rock_x_rover, rock_y_rover = rover_coords(rock_warped)

    # 6) Convert rover-centric pixel values to world coordinates
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x_rover, obstacle_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    navigable_x_world, navigable_y_world = pix_to_world(navigable_x_rover, navigable_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    rock_x_world, rock_y_world = pix_to_world(rock_x_rover, rock_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if is_valid_rotation(Rover):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 10
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
        Rover.worldmap[rock_y_world, rock_x_world, :] = 255

    # 8) Convert rover-centric pixel positions to polar coordinates
    navigable_dist, navigable_angles = to_polar_coords(navigable_x_rover, navigable_y_rover)
    # Update Rover pixel distances and angles
    Rover.nav_dists = navigable_dist
    Rover.nav_angles = navigable_angles
    
    return Rover