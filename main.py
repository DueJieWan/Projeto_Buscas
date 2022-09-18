from os import minor
from typing import Literal
import numpy as np
import matplotlib.pyplot as plt
from path_planner import PathPlanner
from grid import CostMap
from math import inf


#Plot options
save_fig = True
show_fig = True
fig_format = 'png'



def plot_path(cost_map, start, goal, path, filename, save_fig=True, show_fig = True, fig_format='png'):
    
    plt.matshow(cost_map.grid)
    
    x = []
    y = []
    for point in path:
        x.append(point[1])
        y.append(point[0])
    plt.plot(x, y, linewidth=2)
    plt.plot(start[1], start[0], 'y*', markersize=8)
    plt.plot(goal[1], goal[0], 'rx', markersize=8)
    plt.grid(visible=True,which='both')

    plt.xlabel('x / j')
    plt.ylabel('y / i')

    plt.title('A*')

    if save_fig:
        plt.savefig('%s.%s' % (filename, fig_format), format=fig_format)

    if show_fig:
        plt.show()

    
# Creating the enviroment
start_position = (0, 0)
goal_position = (20, 23)

cost_map = CostMap()

# Create the path planner using the cost map
path_planner = PathPlanner(cost_map)

path, cost = path_planner.a_star(start_position, goal_position)

# Plot the path planned
plot_path(cost_map, (0, 0), (20, 23), path, 'a_star' , save_fig, show_fig, fig_format)


