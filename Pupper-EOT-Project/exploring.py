#!/usr/bin/env python3

# This assignment lets you both define a strategy for picking the next point to explore and determine how you
#  want to chop up a full path into way points. You'll need path_planning.py as well (for calculating the paths)
#
# Note that there isn't a "right" answer for either of these. This is (mostly) a light-weight way to check
#  your code for obvious problems before trying it in ROS. It's set up to make it easy to download a map and
#  try some robot starting/ending points
#
# Given to you:
#   Image handling
#   plotting
#   Some structure for keeping/changing waypoints and converting to/from the map to the robot's coordinate space
#
# Slides

# The ever-present numpy
import numpy as np

import path_planning

import math

# Putting this in here to avoid messing up ROS
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import random

# -------------- Showing start and end and path ---------------
def plot_with_explore_points(im, im_threshhold, zoom=1.0, robot_loc=None, explore_points=None, groupings=None, intersections=None, lines=None, best_pt=None):
    """Show the map plus, optionally, the robot location and points marked as ones to explore/use as end-points
    @param im - the image of the SLAM map
    @param im_threshhold - the image of the SLAM map
    @param robot_loc - the location of the robot in pixel coordinates
    @param best_pt - The best explore point (tuple, i,j)
    @param explore_points - the proposed places to explore, as a list"""

    fig, axs = plt.subplots(1, 2)
    axs[0].imshow(im, origin='lower', cmap="gist_gray")
    axs[0].set_title("original image")
    axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[1].set_title("threshold image")
    """
    # Used to double check that the is_xxx routines work correctly
    for i in range(0, im_threshhold.shape[1]-1, 10):
        for j in range(0, im_threshhold.shape[0]-1, 2):
            if is_reachable(im_thresh, (i, j)):
                axs[1].plot(i, j, '.b')
    """

    # Show original and thresholded image
    if explore_points is not None:
        for p in explore_points:
            axs[1].plot(p[0], p[1], '.b', markersize=2)

    if groupings != None:
        for group in groupings:
            if len(group) > 25:
                random_hex_color = '#' + ('%06x' % random.randrange(16**6))
                axs[1].plot([group[0][0], group[-1][0]], [group[0][1], group[-1][1]], marker='.', color=f'{random_hex_color}')
            # for p in group:
            #     axs[1].plot(p[0], p[1], marker='.', color=f'{random_hex_color}')

    if intersections != None:
        for intersection in intersections:
            random_hex_color = '#' + ('%06x' % random.randrange(16**6))
            axs[1].plot(intersection[0], intersection[1], marker='.', color=f'{random_hex_color}')
    
    if lines != None:
        for line in lines:
            random_hex_color = '#' + ('%06x' % random.randrange(16**6))
            axs[1].plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], marker='.', color=f'{random_hex_color}')

    for i in range(0, 2):
        if robot_loc is not None:
            axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
        if best_pt is not None:
            axs[i].plot(best_pt[0], best_pt[1], '*y', markersize=10)
        axs[i].axis('equal')

    for i in range(0, 2):
        # Implements a zoom - set zoom to 1.0 if no zoom
        width = im_threshhold.shape[1]
        height = im_threshhold.shape[0]

        axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
        axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)

# -------------- For converting to the map and back ---------------
def convert_pix_to_x_y(im_size, pix, size_pix):
    """Convert a pixel location [0..W-1, 0..H-1] to a map location (see slides)
    Note: Checks if pix is valid (in map)
    @param im_size - width, height of image
    @param pix - tuple with i, j in [0..W-1, 0..H-1]
    @param size_pix - size of pixel in meters
    @return x,y """
    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Pixel {pix} not in image, image size {im_size}")

    return [size_pix * pix[i] / im_size[1-i] for i in range(0, 2)]

def convert_x_y_to_pix(im_size, x_y, size_pix):
    """Convert a map location to a pixel location [0..W-1, 0..H-1] in the image/map
    Note: Checks if x_y is valid (in map)
    @param im_size - width, height of image
    @param x_y - tuple with x,y in meters
    @param size_pix - size of pixel in meters
    @return i, j (integers) """
    pix = [int(x_y[i] * im_size[1-i] / size_pix) for i in range(0, 2)]

    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Loc {x_y} not in image, image size {im_size}")
    return pix

def is_reachable(im, pix) -> bool:
    """ Is the pixel reachable, i.e., has a neighbor that is free?
    Used for
    @param im - the image
    @param pix - the pixel i,j"""

    # Returns True (the pixel is adjacent to a pixel that is free)
    #  False otherwise
    # You can use four or eight connected - eight will return more points
    
    neighbors = path_planning.eight_connected(pix)
    reachable = False
    counter = 0

    for neighbor in neighbors:
        if neighbor[0] >= 0 and neighbor[0] < im.shape[1] and neighbor[1] >= 0 and neighbor[1] < im.shape[0] and path_planning.is_unseen(im, (neighbor)):
            reachable = True
        
        if path_planning.is_wall(im, (neighbor)):
            counter += 1

        if counter >= 3:
            reachable = False
            break
        
    return reachable

def find_all_possible_goals(im):
    """ Find all of the places where you have a pixel that is unseen next to a pixel that is free
    It is probably easier to do this, THEN cull it down to some reasonable places to try
    This is because of noise in the map - there may be some isolated pixels
    @param im - thresholded image
    @return dictionary or list or binary image of possible pixels"""
    possible_goals = []
    rep_map = np.zeros((im.shape[1], im.shape[0]))

    ys, xs = np.where(im > 170)
    for y_index, y in enumerate(ys):
        x = xs[y_index]
        
        point = (x, y)
        if is_reachable(im, point) and rep_map[point] == 0:
            possible_goals.append(point)

            point_x = point[0]
            point_y = point[1]

            for x in range(math.floor(point_x - im.shape[1]/400), point_x + math.floor(im.shape[1]/400)):
                if x >= 0 and x < im.shape[1]:
                    for y in range(math.floor(point_y - im.shape[0]/400), point_y + math.floor(im.shape[0]/400)):
                        if y >= 0 and y < im.shape[0]:
                            rep_map[(x, y)] = 1

    return possible_goals

def find_all_connected_pix(pixs):
    pixs.sort()
    connected_groups = []

    for pix in pixs:
        connection_found = False
        for connected_group in connected_groups:
            # for connected_group_pix in connected_group:
            if abs(pix[0]-connected_group[-1][0]) <= 1 and abs(pix[1]-connected_group[-1][1]) <= 1:
                connected_group.append(pix)
                connection_found = True
            
            if connection_found:
                break
        
        if not connection_found:
             connected_groups.append([pix])
            
    return connected_groups

def find_intersections(im, connected_groups: list):
    ms = []
    bs = []
    for connected_group in connected_groups:
        if len(connected_group) > 25 and connected_group[-1][0]-connected_group[0][0] > 0:
            ms.append((connected_group[-1][1]-connected_group[0][1])/(connected_group[-1][0]-connected_group[0][0]))
            bs.append(connected_group[-1][1] - ms[-1]*connected_group[-1][0])
    
    intersections = []
    for x in np.arange(0, im.shape[1], .01):
        ys = []
        for i in range(len(ms)):
            y = ms[i]*x+bs[i]

            intersection_detected = False
            for other_y in ys:
                if abs(y - other_y) < .01:
                    intersection_detected = True
                
                if intersection_detected:
                     break
            
            if intersection_detected and not (np.round(x), np.round(y)) in intersections and 0 < np.round(y) < im.shape[0]:
                intersections.append((np.round(x), np.round(y)))
            
            ys.append(y)
    
    lines = []
    for i in range(len(ms)):
        lines.append([(0, bs[i]), (im.shape[1], ms[i]*im.shape[1]+bs[i])])
    
    return intersections, lines

def get_robot_pos(intersections):
    intersections_by_axis = {'x': [], 'y': []}
    for intersection in intersections:
        intersections_by_axis['x'].append(intersection[0])
        intersections_by_axis['y'].append(intersection[1])

    return int(np.mean(intersections_by_axis['x'])), int(np.mean(intersections_by_axis['y']))

def find_best_point(im, possible_points, robot_loc):
    """ Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from
    @param robot_loc - location of the robot (in case you want to factor that in)
    """
    closest_point = robot_loc

    positions = []

    for point in possible_points:
        distance = abs(robot_loc[0] - point[0]) + abs(robot_loc[1] - point[1])

        if distance > 0:
            positions.append(point)
    
    path = path_planning.search_algorithm(im, robot_loc, possible_points)
    if len(path) > 0:
        closest_point = path[-1]

    return closest_point

def find_waypoints(im, path):
	""" Place waypoints along the path
	@param im - the thresholded image
	@param path - the initial path
	@ return - a new path"""
	new_path = []

	prev_i = 1

	if path:
		# adds the first waypoint in the path
		new_path.append(path[0])

		# loops through the original waypoint array
		for i in range(1, len(path) - 1):
			# the previous waypoint
			i1 = path[i - prev_i][0]
			j1 = path[i - prev_i][1]

			# the waypoint we potentially want to get rid of
			i2 = path[i][0]
			j2 = path[i][1]

			# double check that the waypoints are far enough apart. Don't add current waypoint to new array if too close
			if abs(i1 - i2) > 1 or abs(j1 - j2) > 1:
				# the final waypoint
				i3 = path[i + 1][0]
				j3 = path[i + 1][1]

				# create vector from i1, j1 to i2, j2
				v1 = [i2 - i1, j2 - j1]
				v2 = [i3 - i2, j3 - j2]

				# get dot product
				dot = v1[0] * v2[0] + v1[1] * v2[1]

				# get magnitude of v1
				mag1 = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1])

				# get magnitude of v2
				mag2 = math.sqrt(v2[0] * v2[0] + v2[1] * v2[1])

				# putting it all together
				result = dot / (mag1 * mag2)

				# if the dot product is about one, then the angles between the vectors about 0
				# if that is not true, we want to keep the waypoint in the else statement
				if  result < 1.1 and result > 0.9:
					prev_i += 1
				else:
					new_path.append(path[i])
					prev_i = 1

			# a continuation of the distance comparison
			else:
				prev_i += 1

		# adds the final waypoint in the original list
		new_path.append(path[len(path) - 1])
	else:
		print('There is no path.')

	return new_path

def thicken_walls(im, start):
    ys, xs = np.where(im <= 85)

    for y_index, y in enumerate(ys):
        if abs(start[1] - y) < 50:
            x = xs[y_index]

            if abs(start[0] - x) < 50:
                neighbors = path_planning.eight_connected((x, y))

                for neighbor in neighbors:
                    if neighbor[0] >= 0 and neighbor[0] < im.shape[1] and neighbor[1] >= 0 and neighbor[1] < im.shape[0] and neighbor != start:
                        im[neighbor[1], neighbor[0]] = 0

def convert_ft_to_px(ft):
    return np.round(ft*20/3)

def convert_px_to_time(px):
    return np.round(px*10.96/20)

def convert_angle_to_time(angle):
    return np.round(angle*29.33/730)

if __name__ == '__main__':
    im, im_thresh = path_planning.open_image("map_init.pgm")

    all_unseen = find_all_possible_goals(im_thresh)
    connected_groups = find_all_connected_pix(all_unseen)
    intersections, lines = find_intersections(im_thresh, connected_groups)
    robot_start_loc = get_robot_pos(intersections)
    
    thicken_walls(im_thresh, robot_start_loc)

    zoom = 1
    # plot_with_explore_points(im, im_thresh, zoom, robot_loc=robot_start_loc, intersections=intersections, lines=lines)

    im = np.fliplr(im)
    im_thresh = np.fliplr(im_thresh)
    robot_start_loc = (im.shape[1]-robot_start_loc[0], robot_start_loc[1])
    print(robot_start_loc)

    all_unseen = find_all_possible_goals(im_thresh)
    # best_unseen = find_best_point(im_thresh, all_unseen, robot_loc=robot_start_loc)
    best_unseen = (robot_start_loc[0]+5, robot_start_loc[1]+10)
    plot_with_explore_points(im, im_thresh, zoom, robot_loc=robot_start_loc, explore_points=all_unseen, best_pt=best_unseen)

    # path = path_planning.search_algorithm(im_thresh, robot_start_loc, [best_unseen])
    # waypoints = find_waypoints(im_thresh, path)
    # path_planning.plot_with_path(im, im_thresh, zoom, robot_loc=robot_start_loc, goal_loc=best_unseen, path=waypoints)
    
    # instructions = path_planning.get_instructions(path)
    # for instruction in instructions:
    #     print(instruction)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    plt.show()
