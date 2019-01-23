from . import *

def homo(mat3d):
    mat4d = np.vstack((mat3d,[0,0,0]))
    return np.hstack((mat4d,[[0],[0],[0],[1]]))

def projective2euklidean(xh):
    return (xh / xh[-1, :])[0:-1, :]

def euklidean2projective(xe):
    return np.vstack((xe, np.ones((1, xe.shape[1]))))
 
def ZXYdeg(a,b,g):
    a,b,g = np.radians(a),np.radians(b),np.radians(g)
    return np.array([[c(a)*c(g)-s(a)*s(b)*s(g), -c(b)*s(a), c(a)*s(g)+c(g)*s(a)*s(b)],
            [c(g)*s(a)+c(a)*s(b)*s(g), c(a)*c(b), s(a)*s(g)-c(a)*c(g)*s(b)],
            [-c(b)*s(g), s(b), c(b)*c(g)]])

def ZYXdeg(a,b,g): #Standard Aerospace Navigation (yaw,pitch,roll)
    a,b,g = np.radians(a),np.radians(b),np.radians(g)
    return np.array([[c(a)*c(b), c(a)*s(b)*s(g)-c(g)*s(a), s(a)*s(g)+c(a)*c(g)*s(b)],
            [c(b)*s(a), c(a)*c(g)+s(a)*s(b)*s(g), c(g)*s(a)*s(b)-c(a)*s(g)],
            [-s(b), c(b)*s(g), c(b)*c(g)]])

