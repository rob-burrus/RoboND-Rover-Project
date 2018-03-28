# Search and Sample Return Project

This project is modeled after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html) and covers the three essential elements of robotics - perception, decision making and actuation.  This project runs in simulator environment built with the Unity game engine.  

## The Simulator
Lnks for [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip), [Mac](	https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip), or [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip). 

## Dependencies
Python 3 and Jupyter Notebooks. Udacity has provided an Anaconda environment with all dependencies [RoboND-Python-Starterkit](https://github.com/ryan-keenan/RoboND-Python-Starterkit). 

## Notebook Data Analysis

#### Color Thresholding
Modify color_thresh to accept minimum and maximum thresholds as parameters. Trial and error used to find appropriate min/max thresholds for the rocks. 
```
threshed_rock = color_thresh(img, (100,100,0), (255, 255, 30))
threshed_nav = color_thresh(img, (160,160,160))
threshed_obs = color_thresh(img, (0,0,0), (160, 160, 160))
```
![thresholds][./output/Thresholds.png]

#### Process Images
There are 7 steps to in the perception pipeline to process each image. Most steps use the functions derived in the lessons:
  1. Apply color thresholds to create binary image identifying pixels corresponding to navigable terrain, obstacles, and rocks
  ```
  obstacle_thresh = color_thresh(img, (0,0,0), (160,160,160))
  navigable_thresh = color_thresh(img, (160,160,160))
  rock_thresh = color_thresh(img, (100,100,0), (255, 255, 20))
  ```
  2. Define source and destination points for perspective transform
  ```
  dst_size = 5 
  bottom_offset = 6
  source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
  destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
  ```
  3. Apply perspective transform
  ```
  obstacle_warped = perspect_transform(obstacle_thresh, source, destination)
  navigable_warped = perspect_transform(navigable_thresh, source, destination)
  rock_warped = perspect_transform(rock_thresh, source, destination)
  ```
  4. Convert thresholded and warped image pixel values to rover-centric coordinates
  ```
  obstacle_x_rover, obstacle_y_rover = rover_coords(obstacle_warped)
  navigable_x_rover, navigable_y_rover = rover_coords(navigable_warped)
  rock_x_rover, rock_y_rover = rover_coords(rock_warped)
  ```
  5. Convert rover-centric coordinates to world coordinates
  ```
  obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x_rover, obstacle_y_rover, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], data.worldmap.shape[0], 10)
  navigable_x_world, navigable_y_world = pix_to_world(navigable_x_rover, navigable_y_rover, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], data.worldmap.shape[0], 10)
  rock_x_world, rock_y_world = pix_to_world(rock_x_rover, rock_y_rover, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], data.worldmap.shape[0], 10)
  ```
  6. Update world map by setting the detected pixels to their respective color channel (obstacles = red, navigable terrain = blue, rocks = white)
  ```
  data.worldmap[obstacle_y_world, obstacle_x_world, 0] = 255
  data.worldmap[navigable_y_world, navigable_x_world, 2] = 255
  data.worldmap[rock_y_world, rock_x_world, :] = 255
  ```
  7. Create output image for the video (unmodified)
  

## Autonomous Navigation and Mapping






