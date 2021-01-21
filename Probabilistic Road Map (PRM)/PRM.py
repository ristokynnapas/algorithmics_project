## Reference:
# https://github.com/KaleabTessera/PRM-Path-Planning
# https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ProbabilisticRoadMap/probabilistic_road_map.py
# https://theclassytim.medium.com/robotic-path-planning-prm-prm-b4c64b1f5acb




import math
import time
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from scipy.spatial import cKDTree
from matplotlib.animation import PillowWriter



#    _____  _____  __  __                                        _                
#   |  __ \|  __ \|  \/  |                                      | |               
#   | |__) | |__) | \  / |  _ __   __ _ _ __ __ _ _ __ ___   ___| |_ ___ _ __ ___ 
#   |  ___/|  _  /| |\/| | | '_ \ / _` | '__/ _` | '_ ` _ \ / _ \ __/ _ \ '__/ __|
#   | |    | | \ \| |  | | | |_) | (_| | | | (_| | | | | | |  __/ ||  __/ |  \__ \
#   |_|    |_|  \_\_|  |_| | .__/ \__,_|_|  \__,_|_| |_| |_|\___|\__\___|_|  |___/
#                          | |                                                    
#                          |_|                                                    




sample_points_nr = 500 # number of sample_points
KNN_edges_nr = 20  # number of edge from one sampled point
KNN_edges_len = 50  # [m] Maximum edge length



#    _____  _____  __  __   ______                _   _                 
#   |  __ \|  __ \|  \/  | |  ____|              | | (_)                
#   | |__) | |__) | \  / | | |__ _   _ _ __   ___| |_ _  ___  _ __  ___ 
#   |  ___/|  _  /| |\/| | |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
#   | |    | | \ \| |  | | | |  | |_| | | | | (__| |_| | (_) | | | \__ \
#   |_|    |_|  \_\_|  |_| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
#                                                                       
#                                                                       





class initialize:
    
    
    ## Self-callable class for Dijkstra

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.cost) + "," + str(self.parent_index)




def prm_plan(start_x, start_y, goal_x, goal_y, obstacle_x,obstacle_y, rr):

    obstacle_kd_tree = cKDTree(np.vstack((obstacle_x,obstacle_y)).T)

    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
                                       rr, obstacle_x,obstacle_y, obstacle_kd_tree)
    
    plt.plot(sample_x, sample_y, ".b") 

    road_map = road_map_check(sample_x, sample_y, rr, obstacle_kd_tree)

    rx, ry = dijkstra_algo(
        start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)

    return rx, ry





def collision_detect(start_x, start_y, goal_x, goal_y, rr, obstacle_kd_tree):
    x = start_x
    y = start_y
    dx = goal_x - start_x
    dy = goal_y - start_y
    yaw = math.atan2(goal_y - start_y, goal_x - start_x)
    d = math.hypot(dx, dy)

    if d >= KNN_edges_len:
        return True

    D = rr
    n_step = round(d / D)

    for i in range(n_step):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    dist, _ = obstacle_kd_tree.query([goal_x, goal_y])
    if dist <= rr:
        return True  # collision

    return False  # OK


def road_map_check(sample_x, sample_y, rr, obstacle_kd_tree):
    
    """
    Road map generation
    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: Robot Radius[m]
    obstacle_kd_tree: KDTree object of obstacles
    """

    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = cKDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not collision_detect(ix, iy, nx, ny, rr, obstacle_kd_tree):
                edge_id.append(indexes[ii])

            if len(edge_id) >= KNN_edges_nr:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_algo(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y):

    start_node = initialize(start_x, start_y, 0.0, -1)
    goal_node = initialize(goal_x, goal_y, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True

    while True:
        if not open_set:
            print("Path not foundable")
            path_found = False
            break

        c_id = min(open_set, key=lambda o: open_set[o].cost)
        current = open_set[c_id]

        # graph showing
        if len(closed_set.keys()) % 2 == 0:
            plt.plot(current.x, current.y, "xy")
            plt.pause(0.002) ## for slowing down plotting

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            print("%s seconds" % (time.time() - start_time))
            break

        # Remove the item from the open set
        del open_set[c_id]
        # Add it to the closed set
        closed_set[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)
            node = initialize(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closed_set:
                continue
            # Otherwise if it is already in the open set
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

    if path_found is False:
        return [], []

    # generate final course
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):  

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(start_x, start_y, goal_x, goal_y, rr, obstacle_x,obstacle_y, obstacle_kd_tree):
    max_x = max(obstacle_x)
    max_y = max(obstacle_y)
    min_x = min(obstacle_x)
    min_y = min(obstacle_y)

    sample_x, sample_y = [], []

    while len(sample_x) <= sample_points_nr:
        tx = (random.random() * (max_x - min_x)) + min_x
        ty = (random.random() * (max_y - min_y)) + min_y

        dist, index = obstacle_kd_tree.query([tx, ty])

        if dist >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(start_x)
    sample_y.append(start_y)
    sample_x.append(goal_x)
    sample_y.append(goal_y)

    return sample_x, sample_y









def main():
    
    start_x = 100  
    start_y = 100  
    goal_x = 350  
    goal_y = 350  
    path_finder = 20.0  

    obstacle_x = []
    obstacle_y = []
    
    for i in range(300):
        obstacle_x.append(i)
        obstacle_y.append(650.0)
        
    for i in range(500):
        obstacle_x.append(600.0)
        obstacle_y.append(i)
        
    for i in range(300):
        obstacle_x.append(i)
        obstacle_y.append(400.0)
      
    for i in range(650):
        obstacle_x.append(0.0)
        obstacle_y.append(i)
         
    for i in range(650, 500, -1):
        obstacle_x.append(300)
        obstacle_y.append(i)
    
    for i in range(500, 400, -1):
        obstacle_x.append(400)
        obstacle_y.append(i)
        
    for i in range(600):
        obstacle_x.append(i)
        obstacle_y.append(0)
    
    for i in range(300, 600, 2):
        obstacle_x.append(i)
        obstacle_y.append(500)
    
    for i in range(300, 600, 2):
        obstacle_x.append(i)
        obstacle_y.append(650)
        
    for i in range(650, 000, -1):
        obstacle_x.append(600)
        obstacle_y.append(i)
        
   

#    _____                       _       _   _   _             
#   |  __ \                     | |     | | | | (_)            
#   | |__) | __ ___ ______ _ __ | | ___ | |_| |_ _ _ __   __ _ 
#   |  ___/ '__/ _ \______| '_ \| |/ _ \| __| __| | '_ \ / _` |
#   | |   | | |  __/      | |_) | | (_) | |_| |_| | | | | (_| |
#   |_|   |_|  \___|      | .__/|_|\___/ \__|\__|_|_| |_|\__, |
#                         | |                             __/ |
#                         |_|                            |___/ 



    print("Just a moment")
    plt.pause(1)
    print("Do not rush me or ... I`ll do it")

    fig, ax = plt.subplots()
    im = plt.imread("Figure1.png");
    plt.imshow(im);
    
    ax.set_aspect('auto')
    ax.set_adjustable('box')
    ax.set_xlim(left=0,right=100, auto=False)  
    ax.set_ylim(ymin=650,ymax= 0, auto=False)
    ax.set_xbound(lower=0.0, upper=100)
    #ax.invert_xaxis()
    ax.invert_yaxis()
    plt.grid(True)
    plt.axis("equal")
    
    
#    _____  _       _   _   _             
#   |  __ \| |     | | | | (_)            
#   | |__) | | ___ | |_| |_ _ _ __   __ _ 
#   |  ___/| |/ _ \| __| __| | '_ \ / _` |
#   | |    | | (_) | |_| |_| | | | | (_| |
#   |_|    |_|\___/ \__|\__|_|_| |_|\__, |
#                                    __/ |
#                                   |___/     
    
    
    plt.text(100, 100, ("Start"), fontsize=30)
    plt.text(350, 350, ("Finiš"), fontsize=30)
    plt.plot(start_x, start_y, "^c", markersize=30)
    plt.plot(goal_x, goal_y, "Dg", markersize=30)
    
    rx, ry = prm_plan(start_x, start_y, goal_x, goal_y, obstacle_x,obstacle_y, path_finder)
    plt.plot(rx, ry, "-r")
    plt.show()
        

    
start_time = time.time()
main()
 













