import numpy as np
import pandas as pd
import math

from Astar import Astar
from Astar_left_biased import Astar_left_biased
from Astar_right_biased import Astar_right_biased
from plot_anim import plot_anim


print('reading image data ...')
data= pd.read_csv("eroded.csv")

M = data.to_numpy()

print('Data loaded into matrix')
(nrows,ncols) = M.shape


skip = 10
offset = 2

small_rows = math.floor(nrows/skip)
small_cols = math.floor(ncols/skip)

smallerM = np.zeros((small_rows,small_cols))

for i in range(small_rows):
    for j in range(small_cols):
        smallerM[i][j] = M[skip*(i-1)+offset][skip*(j-1)+offset]



#print(smallerM)
R1_obstacles = np.zeros((small_rows,small_cols))
R1_obstacles_x = []
R1_obstacles_y = []
for i in range(small_rows):
    for j in range(small_cols):
        if smallerM[i][j] == 0:
            R1_obstacles[i][j] = 1
            R1_obstacles_x.append(i)
            R1_obstacles_y.append(j)


#print(R1_obstacles)
#Plan route

start =[[6, 57],
         [6, 62],
         [6, 67],
         [6, 72]]
goal = [[53, 16],
        [58, 16],
        [58, 110],
        [53, 110]]

planned_paths = []
path_ls= []

grid_size = 5


#bot 1
path = Astar(R1_obstacles_x,R1_obstacles_y,grid_size,2)
rx, ry = path.planning(start[0][0], start[0][1], goal[0][0], goal[0][1])
path_ls.append((start[0][0], start[0][1]))
for i in range(len(rx)):
        a = (rx[i],ry[i])
        path_ls.append(a)
path_ls.append((goal[0][0], goal[0][1]))
planned_paths.append(path_ls)

path_ls= []

#bot2
path = Astar_right_biased(R1_obstacles_x,R1_obstacles_y,grid_size,2)
rx, ry = path.planning(start[1][0], start[1][1], goal[1][0], goal[1][1])
path_ls.append((start[1][0], start[1][1]))
for i in range(len(rx)):

        a = (rx[i],ry[i])
        path_ls.append(a)
path_ls.append((goal[1][0], goal[1][1]))
planned_paths.append(path_ls)
path_ls= []

#bot3
path = Astar_left_biased(R1_obstacles_x,R1_obstacles_y,grid_size,2)
rx, ry = path.planning(start[2][0], start[2][1], goal[2][0], goal[2][1])
path_ls.append((start[2][0], start[2][1]))
for i in range(len(rx)):

        a = (rx[i],ry[i])
        path_ls.append(a)
path_ls.append((goal[2][0], goal[2][1]))
planned_paths.append(path_ls)
path_ls =[]

#bot 4 
path = Astar(R1_obstacles_x,R1_obstacles_y,grid_size,2)
rx, ry = path.planning(start[3][0], start[3][1], goal[3][0], goal[3][1])
path_ls.append((start[3][0], start[3][1]))
for i in range(len(rx)):

        a = (rx[i],ry[i])
        path_ls.append(a)
path_ls.append((goal[3][0], goal[3][1]))
planned_paths.append(path_ls)



plot_anim(R1_obstacles_x, R1_obstacles_y, start, goal, planned_paths)
#print(planned_paths)





