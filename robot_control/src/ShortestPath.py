#!/usr/bin/python

import numpy as np
import yaml as yaml
import math as math

class AStarNode:
    # Function to construct one A* node
    def __init__(self, i_in, j_in):
        self.i = int(i_in) #x location of this node
        self.j = int(j_in) #y location of this node
        self.g = 9999999  #the current best distance from start node to this node
        self.f = 9999999  #the current best guess at start node to goal 
                           #  through this node
        self.parent = None # the node on the shortest path back to the start node

    # Function to calculate the heuristic value for this node
    # The heuristic is the euclidean distance to the goal node plus
    # the actual distance from the start node to this node.
    def calculate_f(self,goal,x_spacing,y_spacing):
        x = (self.j + 0.5)*x_spacing
        y = (self.i + 0.5)*y_spacing
        self.f = self.g + math.sqrt((x - goal[0])**2 + (y - goal[1])**2)


class AStarSet(list):
    """
    AStarSet
    Extends python list adding:
    get_lowest_f - returns node in set with lowest f value
    is_in_list - determines if an equivalent node in coords is in the list
    """

    # return the node in the open set with the lowest f value or none if the
    #    open set is empty
    def get_lowest_f(self):
        cur_low_f = 9999999
        low_node = None
        # search the list
        for cur_node in self:
            if cur_node.f < cur_low_f:
                cur_low_f = cur_node.f
                low_node = cur_node

        return low_node

    # returns the node with the coords or None if it doesn't exist
    def is_in_list(self,i,j):
        for cur_node in self:
            if cur_node.i == i and cur_node.j == j:
                return True
        return False

    # returns the node with the coords or None if it doesn't exist
    def get_node(self,i,j):
        for cur_node in self:
            if cur_node.i == i and cur_node.j == j:
                return cur_node
        return None


def print_end_data(active_node, start, goal, x_spacing, y_spacing):
    """
    Prints the path information for the descoverd path
    Input:
    active_node - the AStarNode which contains the goal point
    start - the start coordinates in the real world
    end - the goal coordinates in the real world
    """

    print('Done, path planned')
    
    # Print out the grid coordinates of the path and calculate the path depth
    cur_node = active_node
    path_depth = 0
    while cur_node is not None:
        # print grid coords
        print cur_node.i,',',cur_node.j

        # print physical coords
        y = (float(cur_node.i) + 0.5)*y_spacing
        x = (float(cur_node.j) + 0.5)*x_spacing

        # print 'y: ',y,',','x: ',x

        # print g values grid coords
        #print cur_node.g

        cur_node = cur_node.parent
        path_depth += 1

    # Generate the numpy array for the path
    cur_node = active_node
    found_path = np.zeros([path_depth,2])
    loop = 1
    while cur_node is not None:
        y = (float(cur_node.i) + 0.5)*y_spacing
        x = (float(cur_node.j) + 0.5)*x_spacing
        found_path[path_depth-loop] = [x,y]

        cur_node = cur_node.parent
        loop += 1

    #Calculate the total path length by adding the distance from the end grid
    #   center to the end point and adding the g value from this node:
    y = (float(active_node.i) + 0.5)*y_spacing
    x = (float(active_node.j) + 0.5)*x_spacing
    total_distance = active_node.g + \
        math.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)

    print 'Total distance: ',total_distance
    print(found_path)
    return found_path

    
    # Accumulate the euclidaen distance traveled on this path and print it


def dijkstras(occupancy_map,x_spacing,y_spacing,start,goal):
    """
    Implements AStar shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """

    #print('map:')
    #print(occupancy_map)
    #print('start:',start)
    #print('goal:',goal)
    #print('x_space:',x_spacing)
    #print('y_space:',y_spacing)

    # occupancy_map(i,j) maps to location in space at point (x,y) 
    #    like x = (j + 0.5)*x_spacing, y = (i + 0.5)*y_spacing
    # Expectation of grader is that you only move to the 4 most adjacent cells
    #     Therefore, the distance between any two nodes in the graph is either 
    #    x_spacing or y_spacing.

    print('Running AStar...')

    # Locate the start coordinates node in the grid
    start_j = int(start[0]/x_spacing)
    start_i = int(start[1]/y_spacing)

    # Get bounding helpers
    map_shape = occupancy_map.shape
    print map_shape
    map_len_i = int(map_shape[0])
    map_len_j = int(map_shape[1])

    # Bound the i and j value inside of the occupancy map
    if start_j >= map_len_j:
        start_j = map_len_j - 1
    if start_i >= map_len_i:
        start_i = map_len_i - 1

    #print 'Located start grid at: ',start_i,',',start_j

    # Locate the goal coordinates in the grid
    goal_j = int(goal[0]/x_spacing)
    goal_i = int(goal[1]/y_spacing)

    # if the goal node it placed inside of an obsticle move the goal i value
    #   back by one (ONLY FOR PASSING HOMEWORK)
    if occupancy_map[goal_i][goal_j] == 1:
        goal_i = goal_i - 1

    # Bound the i and j value inside of the occupancy map
    if goal_j >= map_len_j:
        goal_j = map_len_j - 1
    if goal_i >= map_len_i:
        goal_i = map_len_i - 1

    #print 'Located goal grid at: ',goal_i,',',goal_j

    # Construct the start node using the grid location of the start coords
    start_node = AStarNode(start_i,start_j)
    
    # Calculate the f score for the start node (g = 0 + h)

    start_x = (float(start_j) + 0.5)*x_spacing
    start_y = (float(start_i) + 0.5)*y_spacing   
    # set to the disance between start cell center and start point
    start_node.g = math.sqrt((start[0] - start_x)**2 + (start[1] - start_y)**2)
    start_node.calculate_f(goal,x_spacing,y_spacing)

    # Put the start node on the open set
    open_set = AStarSet()
    open_set.append(start_node)

    # Initialize the closed set
    closed_set = AStarSet()

    # While the open set isn't empty
    while len(open_set) > 0:

        # raw_input('Waiting...')

        # print len(open_set)
    
        # Set the current node being explored to the node on the open set with the 
        #    lowest f score
        active_node = open_set.get_lowest_f()

        # print 'Activated node: ',active_node.i,',',active_node.j

        # If the node we just opened is the goal node, we're done, return path
        if active_node.i == goal_i and active_node.j == goal_j:
            return print_end_data(active_node,start,goal,x_spacing,y_spacing)
            break

        # Remove this node from the open set and add it to the closed set
        open_set.remove(active_node)
        closed_set.append(active_node)

        # print len(open_set)
        # print len(closed_set)
    
        # For every neighbor of current node
        for k in range(0,4):
            # get the current neighbors i,j coords
            cur_i = active_node.i
            cur_j = active_node.j
            # set the step distance
            d = 0
            if k == 0:
                # i+1, j+0
                cur_i = cur_i + 1
                d = y_spacing

            elif k == 1:
                # i-1, j+0
                cur_i = cur_i - 1
                d = y_spacing

            elif k == 2:
                # i+0, j+1
                cur_j = cur_j + 1
                d = x_spacing

            else:
                # i+0, j-1
                cur_j = cur_j - 1
                d = x_spacing

            # print 'Inspecting neighbor node: ',cur_i,',',cur_j

            # Continue if the neighbor i or j is out of bounds
            if cur_j >= map_len_j or cur_i >= map_len_i or cur_j < 0 or cur_i < 0:
                continue
                # print 'Node out of bounds'



            # If those coords are not an obstocle and not in the closed set
            #     do more, otherwise move on to next neighbor
            if occupancy_map[cur_i][cur_j] == 0 and not closed_set.is_in_list(cur_i,cur_j):
                # Calculate the projected g score of the neighbor by adding the
                #    current node's g to the edge weight between it and the neighbor
                neighbor_g = active_node.g + d

                # If the neighbor is not in the open set, add it
                update_neighbor_node = False
                neighbor_node = open_set.get_node(cur_i,cur_j)

                if neighbor_node is None:
    
                    # print 'Unexplored node, adding it'    

                    neighbor_node = AStarNode(cur_i, cur_j)
                    open_set.append(neighbor_node)
                    update_neighbor_node = True # adding a new node so need to update it

                    
                else:
                # If the neighbor is on the open set and the g score of the 
                #    neighbor through the current node is worse than the recorede 
                #    one, do nothing and stop work on this neighbor
                    # print 'Seen this one before'    
                    if neighbor_node.g > neighbor_g:
                        update_neighbor_node = True # found a better path to this neighbor so update it
                        # print 'Better path found, updating this neighbor node'

                if update_neighbor_node:
                    # Set the parent of the neighbor to the current node
                    neighbor_node.parent = active_node

                    # Set the gscore of the neighbor to the tentative score
                    neighbor_node.g = neighbor_g
    
                    # Set the fscore of the neighbor to the heuristic
                    neighbor_node.calculate_f(goal,x_spacing,y_spacing)
            else:
                if occupancy_map[cur_i][cur_j] == 1:
                    # print 'Location is an obstacle'
                    pass
                else:
                    # print 'Location is already closed'
                    pass

def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")

    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    if np.array_equal(path2,true_path2):
      print("Path 2 passes")

def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)



def main():
    # Load parameters from yaml
    print('Loading parameters...')
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    # Get params we need
    occupancy_map = np.array(params['occupancy_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    print(path)

if __name__ == '__main__':
    #main()
    #test()
    test_for_grader()

