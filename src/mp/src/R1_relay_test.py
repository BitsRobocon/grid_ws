import numpy as np
import pandas as pd
import math

from Astar import Astar
from Astar_left_biased import Astar_left_biased
from Astar_right_biased import Astar_right_biased
from plot_anim import plot_anim
from vectorize import vectorize


print('reading image data ...')
data= pd.read_csv("raw_data.csv")

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
        if smallerM[i][j] == 100:
            R1_obstacles[i][j] = 1
            R1_obstacles_x.append(i)
            R1_obstacles_y.append(j)

#print(R1_obstacles_x)

R1_obstacles_1 = np.zeros((nrows,ncols))
R1_obstacles_x_1 = []
R1_obstacles_y_1 = []
for i in range(nrows):
    for j in range(ncols):
        if M[i][j] == 100:
            R1_obstacles_1[i][j] = 1
            R1_obstacles_x_1.append(i)
            R1_obstacles_y_1.append(j)



#print(R1_obstacles)
#Plan route

start =[[58, 56],
         [58, 62],
         [58, 66],
         [58, 72]]
goal = [[11, 16],
        [7, 18],
        [7, 112],
        [11, 112]]



offset_x = 0#(start[0][0]%5)/2
offset_y = 0#(start[0][1]%5)/2

planned_paths = []
path_ls= []
return_planned_paths = []
return_path_ls= []

grid_size = 2


#bot 1
path = Astar(R1_obstacles_x,R1_obstacles_y,grid_size,2.25)
rx, ry = path.planning(start[0][0], start[0][1], goal[0][0], goal[0][1])

for i in range(len(rx)-1):
        a = ((rx[i]+offset_x-1)*skip+offset,(ry[i] +offset_y-1)*skip+offset)
        path_ls.append(a)
path_ls.append(((start[0][0]-1)*skip+offset, (start[0][1]-1)*skip+offset))
planned_paths.append(path_ls)

path_ls= []

#bot2
path = Astar_right_biased(R1_obstacles_x,R1_obstacles_y,grid_size,2.25)
rx, ry = path.planning(start[1][0], start[1][1], goal[1][0], goal[1][1])

for i in range(len(rx)-1):

        a = ((rx[i]+offset_x-1)*skip+offset,(ry[i] +offset_y-1)*skip+offset)
        path_ls.append(a)
path_ls.append(((start[1][0]-1)*skip+offset, (start[1][1]-1)*skip+offset))
planned_paths.append(path_ls)
path_ls= []

#bot3
path = Astar_left_biased(R1_obstacles_x,R1_obstacles_y,grid_size,2.4)
rx, ry = path.planning(start[2][0], start[2][1], goal[2][0], goal[2][1])

for i in range(len(rx)-1):

        a = ((rx[i]+offset_x-1)*skip+offset,(ry[i] +offset_y-1)*skip+offset)
        path_ls.append(a)
path_ls.append(((start[2][0]-1)*skip+offset, (start[2][1]-1)*skip+offset))
planned_paths.append(path_ls)
path_ls =[]

#bot 4 
path = Astar(R1_obstacles_x,R1_obstacles_y,grid_size,2.4)
rx, ry = path.planning(start[3][0], start[3][1], goal[3][0], goal[3][1])

for i in range(len(rx)-1):

        a = ((rx[i]-offset_x-1)*skip+offset,(ry[i] +offset_y-1)*skip+offset)
        path_ls.append(a)
path_ls.append( ((start[3][0]-1)*skip+offset, (start[3][1]-1)*skip+offset))
planned_paths.append(path_ls)

for j in range(4):
        b= planned_paths[j]
        le = len(b)
        for i in range(le):
                return_path_ls.append(b[le-i-1])
        return_planned_paths.append(return_path_ls)
        return_path_ls= []

            

## vectors 

vec_x_1,vec_y_1 = vectorize(planned_paths[0])

vec_x_2,vec_y_2 = vectorize(planned_paths[1])

vec_x_3,vec_y_3 = vectorize(planned_paths[2])

vec_x_4,vec_y_4 = vectorize(planned_paths[3])
#return vectors
re_vec_x_1,re_vec_y_1 = vectorize(return_planned_paths[0])

re_vec_x_2,re_vec_y_2 = vectorize(return_planned_paths[1])

re_vec_x_3,re_vec_y_3 = vectorize(return_planned_paths[2])

re_vec_x_4,re_vec_y_4 = vectorize(return_planned_paths[3])



plot_anim(R1_obstacles_x_1, R1_obstacles_y_1, start, goal, planned_paths,offset,skip)
#print(planned_paths[0])
