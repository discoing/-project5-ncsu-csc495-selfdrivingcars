import heapq
import math
from typing import List


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.destination = []
        #A*
        self.g = math.inf
        self.parent = None
    
    def getPoint(self):
        return self.x, self.y
    
    def addConnection(self, other_point):
        self.destination.append(other_point)

    # Euclidean distance of given point
    def getDistance(self, other):
        # print(self.x, ", ", self.y)
        # print(other.x, ", ", other.y)
        return math.hypot((self.x - other.x), (self.y - other.y))
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __str__(self) -> str:
        return "(" + str(self.x) + ", " + str(self.y) + ")" 

def astar(start : Point, end : Point, graph : List[Point]):
    # astar: F=G+H, we name F as f_distance, G as g_distance, 
    # H as heuristic
    
    queue = [(0,start)]
    
    # Reset gs
    for pt in graph:
        pt.g = math.inf

    start.g = 0

    while queue:
        curr_f_dist, current_node = heapq.heappop(queue)

        # Success
        if current_node == end:
            
            # Walk back to start and return the path
            came_from = []
            curr = end
            while curr is not start:
                came_from.append(curr)
                curr = curr.parent
            came_from.append(start)
            came_from.reverse()

            return came_from
        
        for next_node in current_node.destination:
            temp_g_distance = current_node.g + current_node.getDistance(next_node)
            if temp_g_distance < next_node.g:
                
                next_node.g = temp_g_distance
                next_node.parent = current_node
                
                heuristic = next_node.getDistance(end)

                # Store F cost in the heap
                # The heap will automatically pop out the lowest FCost
                # F = G + H where H = heuristic
                heapq.heappush(queue, (temp_g_distance + heuristic, next_node))
    
    # Worst case scenario where we didnt find a path
    return None