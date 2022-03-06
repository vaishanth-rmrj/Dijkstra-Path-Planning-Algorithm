import cv2
import matplotlib.pyplot as plt

from generate_map_utils import *

"""
Class to process map image

"""
class MazeMap:    

    def __init__(self, map_img):
        self.map_img = map_img
        self.obstacle_coordinates = self.obstacle_constructor()

    def obstacle_constructor(self):
        """
        method to construct a boundary

        Returns:
            list: list of all obstacle co-ordinates
        """
        obstacle_coordinates = list()
        # constructing from map
        for row in range(self.map_img.shape[0]):
            for col in range(self.map_img.shape[1]):
                if np.array_equal(self.map_img[row, col], [47, 47, 211]) or np.array_equal(self.map_img[row, col], [154, 154, 239]):
                    obstacle_coordinates.append((row, col))
        return obstacle_coordinates        

    def check_if_obstacle(self, coordinate):
        """
        method to check if a coordinate is obstacle

        Args:
            coordinate (tuple): cordinate to check

        Returns:
            bool: True if detected
        """
        for x_obs,y_obs in self.obstacle_coordinates:
            if coordinate[0] == x_obs and coordinate[1] == y_obs:
                return True
        
        return False

"""
class to store node data
"""
class Node:

    def __init__(self, unique_key, coordinates, parent_node_key, cost2come):
        self.unique_key = unique_key
        self.parent_node_key = parent_node_key
        self.cost2come = cost2come
        self.coordinates = coordinates

        
class NodeCatlogue:   
    """
    class to keep track of all nodes created
    """ 

    def __init__(self):
        self.unique_key = 1
        self.node_catalogue = dict()
        

    def create_node(self, coordinate, parent_node_coordinate, cost2come):
        """
        method to create a new node

        Args:
            coordinate (tuple): node coordinate
            parent_node_coordinate (tuple): parent node coordinate
            cost2come (float): cost 2 come to this node

        Returns:
            Node: newly created node object
        """

        for node_obj in self.node_catalogue.values():
            if coordinate[0] == node_obj.coordinates[0] and coordinate[1] == node_obj.coordinates[1]:
                return node_obj  

        parent_node_key = 0
        for node_key, node_obj in self.node_catalogue.items():
            if parent_node_coordinate[0] == node_obj.coordinates[0] and parent_node_coordinate[1] == node_obj.coordinates[1]:
                parent_node_key =  node_key;           

        node_obj = Node(self.unique_key, coordinate, parent_node_key, cost2come)
        self.node_catalogue[node_obj.unique_key] = node_obj
        self.unique_key +=1
        return node_obj

    def fetch_node(self, coordinate):
        """
        method to fetch the node give coordinate

        Args:
            coordinate (tuple): the node coordinate

        Returns:
            Node: requested node object
        """
        for node_obj in self.node_catalogue.values():
            if coordinate[0] == node_obj.coordinates[0] and coordinate[1] == node_obj.coordinates[1]:
                return node_obj
        return False

    def fetch_parent_node(self, node):
        """
        method to fetch the parent node for a given node

        Args:
            node (Node): Node object

        Returns:
            Node: parent node object
        """
        return self.node_catalogue[node.parent_node_key]

class DijikstraPlanner:
    """
    class to perform Dijkstra planning
    """

    def __init__(self, start_coordinate, goal_coordinate, map_obj):
        self.start_coordinate = start_coordinate
        self.goal_coordinate = goal_coordinate
        self.map = map_obj

        self.map.map_img[start_coordinate[0], start_coordinate[1]] = [47, 139, 85]
        self.map.map_img[goal_coordinate[0], goal_coordinate[1]] = [47, 139, 85]

        self.is_plan_feasible = self.check_start_goal_feasibility()

        self.open_node = []
        self.closed_node = []

        # action set
        self.action_set = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)] 
        self.action_cost = [1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4]
        self.node_catalogue = NodeCatlogue()



    def check_start_goal_feasibility(self):
        """
        method to check if the start and goal node are not obstacle

        Returns:
            bool: False if not obstacle
        """
        if self.map.check_if_obstacle(self.start_coordinate) or self.map.check_if_obstacle(self.goal_coordinate):
            return False

        if self.start_coordinate > self.map.map_img.shape or self.goal_coordinate > self.map.map_img.shape:
            return False

        return True        

    def check_if_visited(self, coordinate):
        """
        method to check if the coordinate is already visited

        Args:
            coordinate (tuple): coordinate to check

        Returns:
            bool: False if not visited
        """
        for node_obj in self.node_catalogue.node_catalogue.values():
            if coordinate[0][0] == node_obj.coordinates[0] and coordinate[0][1] == node_obj.coordinates[1]:
                return True 
        return False

    def get_next_move(self, current_coordinate):
        """
        method to get next move coordinates

        Args:
            current_coordinate (tuple): current coordinate

        Returns:
            list: list of next move coordinates
        """

        neighbour_coordinates = [ (current_coordinate[0]+move[0], current_coordinate[1]+move[1]) for move in self.action_set]
        neighbour_coordinates_with_cost = [ [neighbour_coordinates[index], cost] for index, cost  in enumerate(self.action_cost)]

        checked_neighbour_coordinates_with_cost = list()
        for coordinate_with_cost in neighbour_coordinates_with_cost:
            if not self.map.check_if_obstacle(coordinate_with_cost[0]):
                checked_neighbour_coordinates_with_cost.append(coordinate_with_cost)

        return checked_neighbour_coordinates_with_cost

    def low_cost_node_index(self, open_nodes):
        """
        method to get the index of the low cost node

        Args:
            open_nodes (list): list of all the open nodes

        Returns:
            int: index of the element in the open node with low cost
        """
        min_cost = float('inf')    
        min_cost_node = []
        for node in open_nodes:
            if node.cost2come < min_cost:
                min_cost = node.cost2come
                min_cost_node.clear()
                min_cost_node.append(node)

        for index, node in enumerate(open_nodes):
            if node.coordinates[0] == min_cost_node[0].coordinates[0] and min_cost_node[0].coordinates[1] == node.coordinates[1]:
                return index        
        

    def search_for_path(self):
        """
        main method to search through the map

        Returns:
            list, int: list of the planned path, length of visited nodes
        """

        if self.is_plan_feasible:

            start_node = self.node_catalogue.create_node(self.start_coordinate, 1, 0)
            self.open_node.append(start_node)# adding start node to the open queue

            iteration_count = 0
            while len(self.open_node):                
                
                current_node = self.open_node.pop(self.low_cost_node_index(self.open_node))

                self.closed_node.append(current_node)

                if current_node.coordinates[0] == self.goal_coordinate[0] and current_node.coordinates[1] == self.goal_coordinate[1]:
                    print("Hurray!! Puzzle has been solved")
                    break
                
                neighbour_coordinates_with_cost = self.get_next_move(current_node.coordinates)

                for coordinate in neighbour_coordinates_with_cost: 
                    if not self.check_if_visited(coordinate):
                        
                        new_node = self.node_catalogue.create_node(coordinate[0], current_node.coordinates, coordinate[1]+current_node.cost2come)
                        already_exist = False
                        for index, node in enumerate(self.open_node):
                            if new_node.coordinates[0] == node.coordinates[0] and new_node.coordinates[1] == node.coordinates[1] and new_node.cost2come < node.cost2come:
                                self.open_node.pop(index)
                                self.open_node.append(new_node)
                                already_exist = True      
                        
                        if not already_exist:
                            self.open_node.append(new_node)

                for node in self.closed_node:
                    try:
                        self.map.map_img[node.coordinates[0], node.coordinates[1]] = [156, 144, 120]
                    except:
                        print("Frame error detected!")

                iteration_count += 1
                cv2.imshow('Frame',self.map.map_img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break                

            cv2.destroyAllWindows()
            return self.plan_path(), len(self.closed_node)

        else:        
            print("Plan is not feasible because either start or goal coordinate is inside an obstacle")

         
    
    def plan_path(self):
        """
        method to back track through the nodes to get the optimal path

        Returns:
            list: list of planned path
        """

        planned_path = []              
        planned_path.append(self.goal_coordinate)

        current_node = self.node_catalogue.fetch_node(self.goal_coordinate)

        while True:
            current_node = self.node_catalogue.fetch_parent_node(current_node)

            planned_path.append(current_node.coordinates)

            if current_node.coordinates[0] == self.start_coordinate[0] and current_node.coordinates[1] == self.start_coordinate[1]:
                break
        
        return planned_path[::-1]

if __name__ == "__main__":    
    # numpy image canvas
    image = np.zeros((250, 400))
    hex_img = draw_hexagon((200, 100), 35, image)

    xpts = [36, 115, 80, 105]
    ypts = [185, 210, 180, 100]

    poly_img = draw_polygon(xpts, ypts, image)
    circle_img = draw_circle((300, 185), 40, image)

    # building map image by bitwise adding separate shape images
    map_image = cv2.bitwise_or(hex_img, poly_img)
    map_image = cv2.bitwise_or(map_image, circle_img)

    map_image = cv2.bitwise_or(map_image, draw_boundary(image))
    map_image = cv2.flip(map_image, 0)
    map_image = colorize_image(map_image, [47, 47, 211])

    circle_boundary = cv2.bitwise_xor(draw_circle((300, 185), 40, image), draw_circle((300, 185), 45, image))
    hex_boundary = cv2.bitwise_xor(draw_hexagon((200, 100), 35, image), draw_hexagon((200, 100), 40, image))

    xpts_boundary = [26, 130, 90, 115]
    ypts_boundary = [185, 220, 180, 80]

    poly_boundary = cv2.bitwise_xor(draw_polygon(xpts, ypts, image), draw_polygon(xpts_boundary, ypts_boundary, image))

    # building boundary map image by bitwise adding separate shape images
    boundary_map_image = cv2.bitwise_or(circle_boundary, hex_boundary)
    boundary_map_image = cv2.bitwise_or(boundary_map_image, poly_boundary)
    boundary_map_image = cv2.flip(boundary_map_image, 0)

    added_img = overlay_boundary(map_image, boundary_map_image, [154, 154, 239])
    
    # taking user input for start matrix and converting to np array
    print("Enter the start coordinate x y:  ", end="")
    start_coordinate_input = input().split(" ")
    start_coordinate = (int(start_coordinate_input[0]), int(start_coordinate_input[1]))
    print("Your start coordinate",start_coordinate)
    

    # taking user input for start matrix and converting to np array
    print("Enter the goal coordinate x y:  ", end="")
    goal_coordinate_input = input().split(" ")
    goal_coordinate = (int(goal_coordinate_input[0]), int(goal_coordinate_input[1]))
    print("Your goal coordinate",goal_coordinate)

    # start_coordinate = (210, 54)
    # goal_coordinate = (210, 80)    

    map_obj = MazeMap(added_img.copy())

    planner = DijikstraPlanner(start_coordinate, goal_coordinate, map_obj)
    planned_path, visited_node_count = planner.search_for_path()

    if type(planned_path) == type(list()):
        for x,y in planned_path:
            map_image[x,y] = [47, 139, 85]

        cv2.imshow("Final planned path",map_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    