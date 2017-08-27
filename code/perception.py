import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def find_rock(img, rgb_thresh=(110, 110, 40)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
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
    mask_img = np.ones_like(img[:,:,0])
    mask = cv2.warpPerspective(mask_img, M, (img.shape[1], img.shape[0]))
    # Add a round filter
    #mask2 = np.ones_like(mask)
    #for i in range(mask.shape[0]):
    #    for j in range(mask.shape[1]):
    #        dist = np.sqrt((i - img.shape[1]/2)**2 + (j - img.shape[0])**2)
    #        if dist >= img.shape[1] * 1/3:
    #            mask2[i,j] = 0
    #mask = mask * mask2
    
    return warped, mask


# Define a function to convert to radial coords in rover space with weighting factor
def to_polar_coords_w(x_pixel, y_pixel, x_pix_world, y_pix_world):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


def world_to_pix(x_pix_world, y_pix_world, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(x_pix_world, y_pix_, -yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, -xpos, -ypos, 1/scale)
    # Return the result
    return xpix, ypix


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    image = Rover.img
    dst_size = 5
    bottom_offset = 10
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable = color_thresh(warped, rgb_thresh=(160, 160, 160)) * Rover.round_mask * mask
    obstacles = (-1 * np.float32(navigable) + 1) * Rover.round_mask * mask
    rock = find_rock(warped, rgb_thresh=(110, 110, 50))

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles * 255 # obstacle color-thresholded binary image
    Rover.vision_image[:,:,1] = rock * 255 # rock_sample color-thresholded binary image
    Rover.vision_image[:,:,2] = navigable * 255 # navigable terrain color-thresholded binary image
    #Rover.vision_image = navigable * 255

    # 5) Convert map image pixel values to rover-centric coords
    o_xpix, o_ypix = rover_coords(obstacles)
    r_xpix, r_ypix = rover_coords(rock)
    n_xpix, n_ypix = rover_coords(navigable)

    #rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(n_xpix, n_ypix)

    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    roll = Rover.roll
    pitch = Rover.pitch
    o_x_world, o_y_world = pix_to_world(o_xpix, o_ypix, xpos, ypos, yaw, world_size, scale)
    r_x_world, r_y_world = pix_to_world(r_xpix, r_ypix, xpos, ypos, yaw, world_size, scale)
    n_x_world, n_y_world = pix_to_world(n_xpix, n_ypix, xpos, ypos, yaw, world_size, scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    Rover.colormap[o_y_world, o_x_world, 0] += 1
    #Rover.colormap[r_y_world, r_x_world, 1] += 1
    Rover.colormap[n_y_world, n_x_world, 2] += 1
    if roll > 180:
        roll = roll - 360
    if pitch > 180:
        pitch = pitch - 360
    roll_abs = np.absolute(roll)
    pitch_abs = np.absolute(pitch)
    if (roll_abs < 3) and (pitch_abs < 3):
        Rover.worldmap[o_y_world, o_x_world, 0] = 255
        Rover.worldmap[n_y_world, n_x_world, 2] = 255# += 1
        # nav_pix = Rover.worldmap[:, :, 2] > 0
        nav_pix = 5 * Rover.colormap[:, :, 2] > Rover.colormap[:, :, 0]
        obs_pix = 5 * Rover.colormap[:, :, 2] < Rover.colormap[:, :, 0]
        Rover.worldmap[nav_pix, 0] = 0
        Rover.worldmap[obs_pix, 2] = 0
        Rover.worldmap[r_y_world, r_x_world, :] = 255
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(n_xpix, n_ypix)
        
    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles
    Rover.mean_dist = len(Rover.nav_dists)
    Rover.mean_angle = len(Rover.nav_angles)
    
    
    return Rover