"""
Path planning - BFS, DFS & A star
"""

from matplotlib import pyplot as plt
import numpy as np


def plot_planner(grid_size, start_pose, goal_pose, obstacles, path, nodes_explored, animation=False):
    """
    Plot the grid map. Show start and goal post, path, obstacles, etc.
    :param grid_size: list(2) - grid size x & y
    :param start_pose: list(2) - x & y pose of start position
    :param goal_pose: list (2) - x & y pose of goal position
    :param obstacles: list (n, 2) - x & y pose of multiple obstacles
    :param path: list (n, 2) - x & y pose of each cell in the path
    :param nodes_explored: dict (n) - index of nodes explored
    :param animation: bool - to show animation in plot or not
    :return:
    """
    # print(obstacles)
    plt.show()

    fig, ax = plt.subplots()
    ax.set(
        xlim=(0, grid_size[0]), xticks=np.arange(1, grid_size[1]),
        ylim=(0, grid_size[0]), yticks=np.arange(1, grid_size[1])
    )
    plt.grid()

    # Plot start and goal pose
    plt.fill_between([start_pose[1], start_pose[1]+1], start_pose[0], start_pose[0]+1, color='white',
                     edgecolor='green', linewidth=5)
    plt.fill_between([goal_pose[1], goal_pose[1] + 1], goal_pose[0], goal_pose[0] + 1, color='white',
                     edgecolor='red', linewidth=5)

    # Plot obstacles
    for each_ob in obstacles:
        plt.fill_between([each_ob[1], each_ob[1]+1], each_ob[0], each_ob[0]+1, color='black')
     # Plot nodes explored
    for each_node in nodes_explored:
        x, y = list(each_node)
        plt.fill_between([y, y+1], x, x+1, color='yellow')
        if animation:
            plt.pause(0.05)

    for each_pa in path:
        plt.fill_between([each_pa[1], each_pa[1] + 1], each_pa[0], each_pa[0] + 1, color='blue')
        if animation:
            plt.pause(0.2)

    plt.show()



def get_node_children(present_node):

    children = []
    
    children.append( [present_node[0] + 1,present_node[1]] )
    children.append( [present_node[0] - 1,present_node[1]] )
    
    children.append( [present_node[0] ,present_node[1] + 1] )
    children.append( [present_node[0] ,present_node[1] - 1] )

    children.append( [present_node[0]+1 ,present_node[1] - 1] )
    children.append( [present_node[0]-1 ,present_node[1] + 1] )

    return children


def valid_child(child):
    
    if all([val >= 0 for val in child]):
        return True 
    
    return False

def construct_path(parent_lib,goal,start_pos):

    path_is = []

    while goal:
        
        link_node = parent_lib[tuple(goal)] 
        path_is.append(link_node)
        goal = link_node

        print(link_node)

        if goal == start_pos:
            break

    return path_is[::-1]
    
def compute_path_bfs(grid, start, goal, obs):
    
    """
    Compute path using breadth first search
    :param grid: list (2) - x y grid size
    :param start: list (2) - x y pose for start position
    :param goal: list (2) - x y pose for goal position
    :param obs: list(n, 2) - x y pose of multiple obstacles
    :return: path - list (n, 2) - path for robot from start to goal
    """

    path = []
    nodes_explored = {}
    queue = [start]
    goal_reached = False
    parents_lib = {tuple(start):None}

    while not goal_reached:

        cur_node = queue.pop(0)

        if cur_node == goal:
            goal_reached = True
            print("--------------------------------Loop break-----------------------------------")
            break

        for each_child in get_node_children(cur_node):

            if tuple(each_child) not in nodes_explored:

                if valid_child(each_child):

                    if each_child not in obs:

                        queue.append(each_child)
                        parents_lib[tuple(each_child)] = cur_node
            
        nodes_explored[tuple(cur_node)] = None

    print("parents lib",parents_lib)

    # Search until we find the goal OR until we ran out of nodes to explore

    # Look at a node from queue and explore it

    # List of moves from this position

    # for each move

        # move leads to goal?
    
        # (1) within grid size?
        # (2) Obstacle?
        # (3) Visited before?

        # Add node to explore in future.

    # mark the node as explored
    
    path = construct_path(parents_lib,goal,start)

    return path, nodes_explored


if __name__ == "__main__":
    
    animation_flag = True

    obs_positions = [[1,2],[2,2],[3,3], [5,1],[6,2],[7,1],
        [8,2],[1,7],[2,8],
        [3,7],[4,8],[5,7],
        [6,8],[7,9] ]
    
    grid_limits = [20, 20]
    start_position = [1, 11]
    goal_position = [15,15]

    path_bfs, nodes_bfs = compute_path_bfs(grid_limits, start_position, goal_position, obs_positions)
    print("Path: {}".format(path_bfs))
    print("Length of path: {}".format(len(path_bfs)))
    print("Number of nodes explored: {}".format(len(nodes_bfs)))
    
    plot_planner(grid_limits, start_position, goal_position, obs_positions, path_bfs, nodes_bfs, animation_flag)