#!/usr/bin/env python3

# This assignment implements Dijkstra's shortest path on a graph, finding an unvisited node in a graph,
#   picking which one to visit, and taking a path in the map and generating waypoints along that path
#
# Given to you:
#   Priority queue
#   Image handling
#   Eight connected neighbors
#
# Slides https://docs.google.com/presentation/d/1XBPw2B2Bac-LcXH5kYN4hQLLLl_AMIgoowlrmPpTinA/edit?usp=sharing

# The ever-present numpy
import numpy as np

# Our priority queue
import heapq

# Using imageio to read in the image
import imageio.v2 as imageio

# Putting this in here to avoid messing up ROS
import matplotlib.pyplot as plt

# Needed for reading in map info
# from os import open
from math import floor

# Putting this here because in JN it's yaml
import yaml

# -------------- Showing start and end and path ---------------
def plot_with_path(im, im_threshhold, zoom=1.0, robot_loc=None, goal_loc=None, path=None):
    """Show the map plus, optionally, the robot location and goal location and proposed path
    @param im - the image of the SLAM map
    @param im_threshhold - the image of the SLAM map
    @param zoom - how much to zoom into the map (value between 0 and 1)
    @param robot_loc - the location of the robot in pixel coordinates
    @param goal_loc - the location of the goal in pixel coordinates
    @param path - the proposed path in pixel coordinates"""

    fig, axs = plt.subplots(1, 2)
    axs[0].imshow(im, origin='lower', cmap="gist_gray")
    axs[0].set_title("original image")
    axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[1].set_title("threshold image")
    """
    # Used to double check that the is_xxx routines work correctly
    for i in range(0, im_threshhold.shape[1]-1, 10):
        for j in range(0, im_threshhold.shape[0]-1, 10):
            if is_wall(im_thresh, (i, j)):
                axs[1].plot(i, j, '.b')
    """

    # Double checking lower left corner
    axs[1].plot(10, 5, 'xy', markersize=5)

    # Show original and thresholded image
    for i in range(0, 2):
        if robot_loc is not None:
            axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
        if goal_loc is not None:
            axs[i].plot(goal_loc[0], goal_loc[1], '*g', markersize=10)
        if path is not None:
            for p, q in zip(path[0:-1], path[1:]):
                axs[i].plot([p[0], q[0]], [p[1], q[1]], '-y', markersize=2)
                axs[i].plot(p[0], p[1], '.y', markersize=2)
        axs[i].axis('equal')

    for i in range(0, 2):
        # Implements a zoom - set zoom to 1.0 if no zoom
        width = im.shape[1]
        height = im.shape[0]

        axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
        axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)

# -------------- Thresholded image True/False ---------------
def is_wall(im, pix):
    """ Is the pixel a wall pixel?
    @param im - the image
    @param pix - the pixel i,j"""
    if im[pix[1], pix[0]] <= 85:
        return True
    return False

def is_unseen(im, pix):
    """ Is the pixel one we've seen?
    @param im - the image
    @param pix - the pixel i,j"""
    if im[pix[1], pix[0]] > 85 and im[pix[1], pix[0]] <= 170:
        return True
    return False

def is_free(im, pix):
    """ Is the pixel empty?
    @param im - the image
    @param pix - the pixel i,j"""
    if im[pix[1], pix[0]] > 170:
        return True
    return False

def convert_image(im, wall_threshold, free_threshold):
    """ Convert the image to a thresholded image with not seen pixels marked
    @param im - WXHX ?? image (depends on input)
    @param wall_threshold - number between 0 and 1 to indicate wall
    @param free_threshold - number between 0 and 1 to indicate free space
    @return an image of the same WXH but with 0 (free) 255 (wall) 128 (unseen)"""
    
    im_ret = np.zeros((im.shape[0], im.shape[1]), dtype='uint8') + 128

    im_avg = im
    if len(im.shape) == 3:
        # RGB image - convert to gray scale
        im_avg = np.mean(im, axis=2)
    # Force into 0,1
    im_avg = im_avg / np.max(im_avg)
    # threshold
    #   in our example image, black is walls, white is free
    im_ret[im_avg < wall_threshold] = 0
    im_ret[im_avg > free_threshold] = 255
    return im_ret

# -------------- Getting 4 or 8 neighbors ---------------
def four_connected(pix):
    """ Generator function for 4 neighbors
    @param im - the image
    @param pix - the i, j location to iterate around"""
    for i in [-1, 1]:
        ret = pix[0] + i, pix[1]
        yield ret
    for i in [-1, 1]:
        ret = pix[0], pix[1] + i
        yield ret

def eight_connected(pix):
    """ Generator function for 8 neighbors
    @param im - the image
    @param pix - the i, j location to iterate around"""
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                pass
            ret = pix[0] + i, pix[1] + j
            yield ret

def heuristic(node, goal):
	"""
	@param node - a suggested point (tuple, i,j)
	@param goal - where to go to (tuple, i,j)
	@returns a the estimated distance from node to gaol.
	
	Note: Sqrt is actually computationally complex to compute, so manhatten distance is faster
		overall, even though there are more points to analyze.
	"""
	return abs(node[0]-goal[0]) + abs(node[1]-goal[1])

def search_algorithm(im, robot_loc, goal_locs):
    """ Occupancy grid image, with robot and goal loc as pixels
    @param im - the thresholded image - use is_free(i, j) to determine if in reachable node
    @param robot_loc - where the robot is (tuple, i,j)
    @param goal_loc - where to go to (tuple, i,j)
    @param dijkstra - Use dijkstra as the search algorithm if true otherwise use A*.
    @returns a list of tuples"""

    dijkstra = False

    for goal_loc in goal_locs:
        if not is_free(im, goal_loc):
            print('Goal is not free.')

            return []

    if len(goal_locs) > 1:
        dijkstra = True
	
	# The priority queue itself is just a list, with elements of the form (weight, (i,j))
	#	 - i.e., a tuple with the first element the weight/score, the second element a tuple with the pixel location
    priority_queue = []
	
    #The shorted path and its corresponding location are put at the front of the heap.
    if dijkstra:
        heapq.heappush(priority_queue, (0, robot_loc))
    else:
        heapq.heappush(priority_queue, (0, 0, robot_loc))
	
	#visited will keep track of locations as we go.
	#	0 - Distance, the distance traveled to get to this point.
	#	1 - Parent, the point which resulted in the shorted path so far.
	#	2 - Closed, whether the node has been analyzed already.
    visited = {}
	
	#Starting location has a distance of 0, no path, and is initially not closed.
    visited[robot_loc] = (0, None, False)	# For every other node this will be the current_node, distance
	
    goal_loc_reached = False

	#Not done until the end goal is found or the queue is empty.
    while priority_queue:
	    # Get the current best node off of the list
        current_node = heapq.heappop(priority_queue)

        #Give meaning to the current node.
        node_score = current_node[0 if dijkstra else 1]
        node_ij = current_node[1 if dijkstra else 2]
		
		#Initialize visited on the go to reduce time.
        if not (node_ij in visited):
            visited[node_ij] = (np.inf, None, False)

		#Give meaning to visited.
        visited_distance = visited[node_ij][0]
        visited_parent = visited[node_ij][1]
        visited_closed_yn = visited[node_ij][2]
		
		#Stop when done.
        if node_ij in goal_locs:
            goal_loc_reached = node_ij
            break
	
		#Don't repeat nodes.
        if visited_closed_yn == False:
            #  Step 3: Set the node to closed
            visited[node_ij] = (visited_distance, visited_parent, True)
			
            #	 Now do the instructions from the slide (the actual algorithm)
            adj_nodes = eight_connected(node_ij)
			
			#Check for open nodes with lowerable path weights.
            for adj_node in adj_nodes:
				#Evaluate if in bounds.
                if adj_node[0] >= 0 and adj_node[0] < im.shape[1] and adj_node[1] >= 0 and adj_node[1] < im.shape[0] and is_free(im, adj_node):
					#If a node has not been visited, initialize it.
                    if not (adj_node in visited):
                        visited[adj_node] = (np.inf, None, False)
					
					#Give meaning to visited.
                    visited_distance = visited[adj_node][0]
                    visited_closed_yn = visited[adj_node][2]
					
					#Check if the adj node is closed and has a lower path weight than what it had previously.
                    if (visited_closed_yn == False) and node_score + 1 < visited_distance:
                        visited[adj_node] = (node_score + 1, node_ij, False)

                        if dijkstra:
                            heapq.heappush(priority_queue, (node_score + 1, adj_node))
                        else:
                            goal_loc = goal_locs[0]
                            heapq.heappush(priority_queue, (heuristic(adj_node, goal_loc) + node_score + 1, node_score + 1, adj_node))
	
	# Now check that we actually found the goal node    
    if not goal_loc_reached:
        return []
	
    path = []
    path.append(goal_loc_reached)

	#Build the path by starting at the goal node and working backwards
    while path[0] != robot_loc:
        #Build the path correctly to reduce time.
        path.insert(0, visited[path[0]][1])
	
    return path

def adjust_position(robot_pos, im, resolution):
    return (floor(robot_pos[0]/resolution + im.shape[1]/2), floor(robot_pos[1]/resolution + im.shape[0]/2))

def open_image(im_name):
    """ A helper function to open up the image and the yaml file and threshold
    @param im_name - name of image in Data directory
    @returns image anbd thresholded image"""

    im = imageio.imread("Data/" + im_name)

    wall_threshold = 0.7
    free_threshold = 0.9
    robot_pos = (0, 0)

    try:
        yaml_name = "Data/" + im_name[0:-3] + "yaml"

        with open(yaml_name, "r") as f:
            dict = yaml.safe_load(f)

            # wall_threshold = dict["occupied_thresh"]
            # free_threshold = dict["free_thresh"]
            origin = [1.7, .45]

            robot_pos = adjust_position(origin[:2], im, dict['resolution'])
    except:
        pass

    im_thresh = convert_image(im, wall_threshold, free_threshold)
    return im, im_thresh, robot_pos

def get_instructions(path):
    instructions = []

    mov_dirs = {
        (1, 0): 0,
        (1, 1): 45,
        (0, 1): 90,
        (-1, 1): 135,
        (-1, 0): 180,
        (-1, -1): 225,
        (0, -1): 270,
        (1, -1): 315
    }

    curr_direction = 90
    curr_loc = None
    distance_counter = 0
    for loc_index, loc in enumerate(path):
        if curr_loc == None:
            curr_loc = loc
        
        if heuristic(curr_loc, loc) > 0:
            direction = mov_dirs[(loc[0]-curr_loc[0], loc[1]-curr_loc[1])]

            if direction != curr_direction and distance_counter > 0:
                instructions.append(f'Forward {distance_counter}')
                distance_counter = 0

            distance_counter += 1

            if loc_index == len(path) - 1:
                instructions.append(f'Forward {distance_counter}')
                distance_counter = 0

            if direction != curr_direction:
                direction_text = 'left'
                amount_to_turn = direction-curr_direction

                if amount_to_turn > 180:
                    direction_text = 'right'
                    amount_to_turn = 360-amount_to_turn
                if -180 < amount_to_turn < 0:
                    direction_text = 'right'
                    amount_to_turn = abs(amount_to_turn)
                elif amount_to_turn < -180:
                    amount_to_turn = 360-abs(amount_to_turn)
                    
                instructions.append(f'Turn-{direction_text} {amount_to_turn}')
                curr_direction = direction
            
        curr_loc = loc
    
    return instructions

if __name__ == '__main__':
    # Use one of these

    """ Values for SLAM map"""
    im, im_thresh, robot_start_loc = open_image("map.pgm")

    # robot_start_loc = (100, 55)
    # Closer one to try
    # robot_goal_loc = (315, 250)
    robot_goal_loc = (75, 85)
    zoom = 0.7
    
    """ Values for map.pgm"""
    """im, im_thresh, robot_pos = open_image("map.pgm")
    robot_start_loc = (1940, 1953)
    robot_goal_loc = (2135, 2045)
    zoom = 0.1"""

    """
    print(f"Image shape {im_thresh.shape}")
    for i in range(0, im_thresh.shape[1]-1):
        for j in range(0, im_thresh.shape[0]-1):
            if is_free(im_thresh, (i, j)):
                print(f"Free {i} {j}")
    """
    path = search_algorithm(im_thresh, robot_start_loc, [robot_goal_loc])
    print(path)

    instructions = get_instructions(path)
    for instruction in instructions:
        print(instruction)

    plot_with_path(im, im_thresh, zoom=zoom, robot_loc=robot_start_loc, goal_loc=robot_goal_loc, path=path)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    plt.show()

    print("Done")
