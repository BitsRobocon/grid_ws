import matplotlib.pyplot as plt
import time

def plot_anim(R1_obstacles_x, R1_obstacles_y, start, goal, planned_paths,offset,skip):

    plt.grid(True)
    plt.plot(R1_obstacles_y, R1_obstacles_x, ".k")
    
    for i in range(4):
        #plt.plot(start[i][1], start[i][0], "og")
        #plt.plot(goal[i][1], goal[i][0], "xb")
        plt.plot((start[i][1]-1)*skip+offset, (start[i][0]-1)*skip+offset, "og")
        plt.plot((goal[i][1]-1)*skip+offset, (goal[i][0]-1)*skip+offset, "xb")
    
    plt.axis("equal")
    
    colors = ["-r","-b","-g","-m"]
    rxx= []
    ryy = []
    for j in range(4):
        b= planned_paths[j]
        #print(b)
        rxx= []
        ryy = []
        for i in range(len(b)):
            rxx.append(b[i][0])
            ryy.append(b[i][1])
        le = len(rxx)
        for k in range(1,len(rxx)):
            plt.plot([ryy[le - k-1],ryy[le - k]], [rxx[le - k-1],rxx[le -k]],colors[j])
            # plt.pause(0.00001)
            
        for m in range(le):
            p,= plt.plot(ryy[le -m-1],rxx[le-m-1],"ok",markersize=6)
            
            plt.pause(0.000001)
            # time.sleep(0.0002)
            p.remove()
        for m in range(le):
            p,= plt.plot(ryy[m],rxx[m],"ok",markersize=6)
            plt.pause(0.000001)
            # time.sleep(0.0002)
            p.remove()
    plt.show()