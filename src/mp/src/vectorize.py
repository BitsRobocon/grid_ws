import math


def vectorize(planned_paths):
    rxx= []
    ryy = []
    vec_x = []
    vec_y = []
    
    b= planned_paths
    #print(b)
    rxx= []
    ryy = []
    for i in range(0,len(b)):
        rxx.append(b[i][0])
        ryy.append(b[i][1])
    le = len(rxx)
    for k in range(1,le):
        i=  ryy[le - k-1]- ryy[le - k]
        j=  rxx[le - k-1]-rxx[le -k]
        h = math.hypot(i,j)
        vec_x.append(i/h)
        vec_y.append(j/h)
    
    vec_x.append(vec_x[-1])
    vec_y.append(vec_y[-1])
    return vec_x,vec_y