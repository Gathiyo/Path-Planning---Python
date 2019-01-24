from waypoint import Waypoint
from path_validator import PathValidator as pv

class PathFinder(object):

    '''Here in this node:
    G is the distance between the current node and the start node,
    H is the heuristic — estimated distance from the current node to the end node,
    F is the total cost of the node,
    in wp we pass the waypoint information,
    and parent helps us keep track of the parent
    '''
    class Node(object):
        def __init__(self, wp, parent=None):
            self.wp = wp
            self.f = 0
            self.g = 0
            self.h = 0
            self.parent = parent

    def get_path(self, grid, start_wp, end_wp):
        """Returns a list of Waypoints from the start Waypoint to the end Waypoint.
:
        :param grid: Grid is a 2D numpy ndarray of boolean values. grid[x, y] == True if the cell contains an obstacle.
        The grid dimensions are exposed via grid.shape
        :param start_wp: The Waypoint that the path should start from.
        :param end_wp: The Waypoint that the path should end on.
        :return: The path from the start waypoint to the end waypoint that follows the movement model without going
        off the grid or intersecting an obstacle.
        :rtype: A list of Waypoints.

        Here we Create start and end node
        '''
        start_node = self.Node(start_wp, None)
        start_node.f = start_node.g = start_node.h = 0
        end_node = self.Node(end_wp, None)
        end_node.f = end_node.g = end_node.h = 0



        '''
        Initialize both open and closed list
        let the openList equal empty list of nodes
        let the closedList equal empty list of nodes
        '''
        open_list = []
        closed_list = []

        #put the startNode on the openList (leave it's f at zero)

        open_list.append(start_node)

        #Loop until you find the end

        while len(open_list) > 0:

            #Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            open_list.pop(current_index)

            closed_list.append(current_node)

            #Found the goal
            #if currentNode is the goal we've found the end! Backtrack to get path
            if current_node.wp == end_node.wp:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.wp)
                    current = current.parent
                return path[::-1]

            #let the children of the currentNode equal the adjacent nodes
            #we will get valid nodes with the function get_valid_neighbours which gives us 6 neigbours by  checking the possible orientation

            children = self.get_valid_neighbours(current_node.wp, grid)

            new_children = []
            for child in children:
                new_node = self.Node(child, current_node)
                new_children.append(new_node)



            for child in new_children:

                lst = [i.wp for i in closed_list]
                if child.wp in lst:
                    continue
                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = (abs(child.wp.x - end_node.wp.x)) + (abs(child.wp.y - end_node.wp.y))
                child.f = child.g + child.h

                # Child is already in the open list
                if child.wp in [i.wp for i in open_list]:
                    continue
                # Add the child to the open list
                open_list.append(child)

    #This function is used to create the valid_neighbours for the current_node
    #we are using is_valid_waypoint and is_valid_transition to check and authorise the correct orientation
    #The below code might not "LOOK" efficent but it's a contant time operation which check 24 possibolities for each node for possible orientation,
    #thus in A* it's contant time operation

    def get_valid_neighbours(self,current, grid):

        valid_neighbours = []
        for i in [0,1,-1]:
            for j in [0,1,-1]:
                for k in [0,1,2,3]:
                    new_waypoint = Waypoint(current.x + i, current.y + j, (current.orientation + k) % 4)
                    if pv.is_valid_waypoint(new_waypoint, grid) and pv.is_valid_transition(current, new_waypoint):
                        valid_neighbours.append(new_waypoint)
        return valid_neighbours
