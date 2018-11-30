from __future__ import division, print_function
import numpy as np

__version__ = "0.32"

def homo(mat3d):
    mat4d = np.vstack((mat3d,[0,0,0]))
    return np.hstack((mat4d,[[0],[0],[0],[1]]))

def projective2euklidean(xh):
    return (xh / xh[-1, :])[0:-1, :]

def euklidean2projective(xe):
    return np.vstack((xe, np.ones((1, xe.shape[1]))))
        
 
class CamModel(object):
    NOCAM = 0
    """
    when there is no camera, then this is not a central projection.
    """
    PINHOLE = 1
    """
    the idealized camera is a pinhole camera. Here no distortion occurs. 
    """
    
    BROWN = 2
    """
    a real camera has distortions that can be described by some radial and tangential distortion parameters. 
    Duane C. Brown has introduced this very popular camera model. 
    """
    LUT = 3
    """
    if brown's camera model is not fitting good enough, the lookuptable could do the job. Every sharp pixel projects 
    rays just from one direction. the ray direction can be described with x and y angles to the optical axis.
    """
    
class Camera(object):
    """
    this class provides methods to calculate the projection of a point of interest (poi) from the scene onto the image plane and reverse (reprojection). 
    
    If you have a camera and you know a scene objects world position coordinates and the cameras position and viewing angles, 
    then with this tool you can calculate the scene object's pixelposition on your image plane.
    Okay, this is maybe a special case and not so often used, but the reverse projection from the image to the world coordinates you might find useful.
    But this is a bit tricky, because reprojection needs additional information about the depth.
    
    I needed this kind of image reprojection for detecting and rescueing roe deer fawns in meadows from beeing killed by mowing machines. 
    A UAV mounted thermal camera captured images. The fawns are hot spots with just few pixel size.  
    
    this lets you get a world map position of a point of interest on your image. 
    """
    K = None
    S = np.eye(4) # transformationsmatrix
    def __init__(self):
        """
        standalone means the camera is not mounted on a uav. 
        the coordinate system is then the camera coordinate system (x points right, y points down on the image and z points to the scene)
        if standalone is false we assume that the camera is mounted on a uav
        and the coordinate system is the uav-coordinate system (the geocoordinate system / left handed, see pose for more details)
                 
            
               _z  right handed camera cosy  
               /|
              /______\ x        
             |   ____/____          
            \|/ |         |
              y |   Image | 
        """
        self.distortionmodel = CamModel.NOCAM
        
    
    def attitudeMat(self,mat):#yaw,pitch,roll):
        """
            defines the outer orientation of the camera.  
                   
           :param mat: affine transformation matrix
           :type mat: numpy array with shape (4, 4)
           :rtype: void
        """
        self.S = mat
        self.Si = np.linalg.inv(self.S)
        
    def position(self):
        """
           :return: the position of the camera in world coordinates 
           :rtype: 3D numpy Array 
        """
        return self.S[0:3,[3]].ravel()
            
    #def transform(self):
    #    self.S = self.cosymat.dot(self.R_boresight).dot(self.T_boresight).dot(self.R_gimbal).dot(self.T_gimbal).dot(self.R_uav).dot(self.T_uav)
    #    self.Si = self.Ti_uav.dot(self.Ri_uav).dot(self.Ti_gimbal).dot(self.Ri_gimbal).dot(self.Ti_boresight).dot(self.Ri_boresight).dot(self.cosymat.T)
    
    def intrinsics(self,width,height,fx,cx,cy,ar=1.0,skew=0.0):
        """
            defines the inner orientation of the camera.  
                   
           :param width: image width
           :param height: image height
           :param fx: focal length in pixel units (size: dx)
           :param cx: optical center (x component)
           :param cy: optical center (y component)
           :param ar: aspect ratio. fy = fx * ar 
           :param skew: shear coefficient between x and y. 
           :type width: int
           :type height: int
           :type fx: float
           :type cx: float
           :type cy: float
           :type ar: float
           :type skew: float
           :rtype: void
        """
        self.fx = fx
        self.ar = ar
        self.fy = ar*fx
        self.cx = cx
        self.cy = cy
        self.skew = skew
        self.K = np.array([[fx,skew,cx,0],[0,self.fy,cy,0],[0,0,1,0],[0,0,0,1]])
        self.P = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,-1.0/self.fx-1,1]])
        self.Pi = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,1.0/self.fx+1,1]])
        
        self.imgwidth = width
        self.imgheight = height
        self.distortionmodel = CamModel.PINHOLE if self.distortionmodel == CamModel.NOCAM else self.distortionmodel
    
    def __Kinv__(self):
        """
            :rtype: the inverse of the matrix K. shape: (4,4) 
            
        """
        return np.array([[1.0/self.fx,-float(self.skew)/(self.fx*self.fy),
                -(self.cx*self.fy-self.skew*self.cy)/(self.fx*self.fy),0],
                [0,1.0/self.fy,-float(self.cy)/self.fx,0],[0,0,1,0],[0,0,0,1]])    
    

        
    def project(self,X):
        """
        projects a world 3d-point on the camera plane
        returns a 2d point on the imageplane
        (except for NOCAM: then its just a 3d-ray )
        """
        if X.shape in [(4,1),(3,1)]: #converts column vector to flat row vector 
            X = np.ravel(X)
        if X.shape[-1] == 3: # appends homogenious elements, if input is just 3D world coordinates
            X = np.append(X,np.ones((X.shape[0],1)),1)
        if self.distortionmodel == CamModel.NOCAM:
            x = self.S.dot(X.T)
            return x.T 
        if self.distortionmodel == CamModel.PINHOLE:
            x = self.P.dot(self.K).dot(self.S).dot(X.T)
            x_norm = x/x[3]
            return np.array([x_norm[0]/x_norm[2],x_norm[1]/x_norm[2]]).T
        elif self.distortionmodel == CamModel.BROWN:
            pass
        elif self.distortionmodel == CamModel.LUT:
            pass
    
    def reproject(self,x,distance=1):
        """
        reprojects a pixel to a 3d-ray
        """
        xt = x.T    
        if self.distortionmodel == CamModel.NOCAM:
            Xi = self.Si.dot(xt)
            return Xi.T 
        elif self.distortionmodel == CamModel.PINHOLE:
            X3d = np.array([xt[0],xt[1],np.ones_like(xt[0]),np.ones_like(xt[0])])
            Xi = self.Si.dot(self.__Kinv__()).dot(self.Pi).dot(X3d)
            return (Xi/Xi[3]).T
            
        elif self.distortionmodel == CamModel.BROWN:
            pass
        elif self.distortionmodel == CamModel.LUT:
            pass
    
    def reprojectToPlane(self,x,distance):
        """
          this is a wrapper for reproject() to get real world coordinates instead of a direction vector.
          the direction vector of reproject() intersects a virtual plane with the given distance.
          the plane lies in front of the camera and parallel to its focal plane array.
          the intersection point will be returned. 
                   
           :param x: a numpy 2D-Point (x,y) coordinates of an image 
           :param distance: the distance of the parallel virual intersection plane
           :type x: 2D numpy array [[x],[y]] (in pixel coordinates)
           :type distance: float (in world coordinates e.g. in meter)
           :return: the reprojected Point in 3D world coordinates. returns [[None],[None],[None]] 
                   if no intersection or plane is not in front of the camera.
           :rtype: 3D numpy Array [[X][Y][Z]] (in world coordinates)
        """
        if self.distortionmodel == CamModel.NOCAM:
            raise("Error- with NOCAM this method doeas not work!")
        rp = self.reproject(x)
        raydir = np.add(rp.T[0:3].T, - self.position()).T
        n_plane = np.array([0,0,-1])
        plane = n_plane * (- distance)
        rd_n = np.dot(raydir.T, n_plane)
        pd = np.dot(plane, n_plane)
        p0_n = np.dot(self.position(), n_plane)
        t = (pd - p0_n) / np.ma.masked_where(rd_n >= 0, rd_n)
        return np.add(self.position(), (raydir * t).T)
        
           
#
# BROWN
#
        
    def lensdistortion(self,k1=0.0,k2=0.0,p1=0.0,p2=0.0,k3=0.0):
        """
        sets the radial and tangential parameters for the brown camera model. 
        
        
        """
        self.k1 = k1
        self.k2 = k2
        self.p1 = p1
        self.p2 = p2
        self.k3 = k3
        self.distortionmodel = CamModel.BROWN

    def brown(self,X):
        x,y  =  X[0]/X[2] , X[1]/X[2]
        r = np.sqrt(x*x+y*y)
        a = 1 + self.k1*r*r + self.k2*r*r*r*r + self.k3*r*r*r*r*r*r
        xi = x*a + self.p2*(r*r+2*x*x) + 2*self.p1*x*y
        yi = y*a + self.p1*(r*r+2*y*y) + 2*self.p2*x*y
        u = self.cx + xi*self.fx + yi*self.skew
        v = self.cy + yi*self.fy
        return np.array([u, v])#.ravel()
    





    
if __name__ == "__main__":

# usecase:
    import transforms3d
    from math import pi
    
    # wichtige Erkenntnisse fuer die Eingangsparameter:
    # - die vierte Koordinate von Xw muss ungleich 0 sein (wir verwenden immer 1!)
    # - die Rotation und Translation der Kamera erfolgt im rechtsseitigen Koordinaten System als passive !!! Rotation und Translation
    #   dies wird erreicht indem die homogene Matrix einer aktiven Rotation und Translation in der gewuenschten Richtung erzeugt wird und dann die Transponierte dieser Matrix  
    #   (mat.T) an die Camera-Klasse uebergeben wird (dadurch wird aus einer aktiven Transformation eine passive).
    Xw = np.array([1,0,10,1])     
    rot = transforms3d.euler.euler2mat(0,pi/12,0,"szyx")
    mat = transforms3d.affines.compose([0,0,-10],rot,np.ones(3))
    inv = np.linalg.inv(mat)
    print(mat.T, inv)
    #print(np.equal(inv,mat.T))
    cam = Camera() #
    
    cam.attitudeMat(inv)# z y' x''
    print ("S",cam.S)
    cam.intrinsics(640,512,1000,320,260)# all in pixel coordinates
    #cam.distortion(0,0,0,0)
    print("====================\nXw:", Xw)
    Xc = cam.project(np.expand_dims(Xw,0).T)
    print("Xc:", Xc)
    
    #Xc : [x,y]
    Xcr = cam.reproject(Xc)
    # Xcr: [x,y,z]
    print("Xcr:", Xcr)
    
    
    
    
    
    
    
    
    #trans = Transform()
    #trans.position(X, Y, Z) # orthogonal coordinates
    #trans.attitude(yaw, pitch, roll, "zy'x''")
    #trans.attitudeQ(w,x,y,z)
    #trans.addjoint(dx,dy,dz,"roll") 
    #trans.pose(0,90,0,1,2,5,"Euler")
    #trans.boresight(10,-2,3,0,1,0,"Euler") 
    #trans.gimbal(0,0,20,0,0,0,"Euler") 
    
    #cam.attitudeQ(w,x,y,z)
    #cam.attitude(yaw,pitch,roll)# z y' x''
    
    # print("===Cam_position===")
    
    # print(cam.position())
    # Xc = cam.project(Xw)
    # print("Xc", Xc)
    # Xcr = cam.reproject(Xc)
    # print("Xcr", Xcr)
    # print(Xc[0][0],Xc[1][0])
    
    
    
    


    # from visual import *
    # from rotation import euler,quaternions
    # import numpy as np


    # ax = arrow(pos=(0,0,0),axis=(0,0,-1),color=(1,0,0))
    # ay = arrow(pos=(0,0,0),axis=(1,0,0),color=(0,1,0))
    # az = arrow(pos=(0,0,0),axis=(0,1,0),color=(0,0,1))


    # for i in range(-5,5):
        # for j in range(-5,5):
            # sphere(pos=(i,0,j), radius=0.02,color=(0,0,1))
            # sphere(pos=(0,i,j), radius=0.02,color=(0,1,0))
        # sphere(pos=(i,0,0), radius=0.05)
        # sphere(pos=(0,i,0), radius=0.05)
        # sphere(pos=(0,0,i), radius=0.05)

    # def pt(pos):
        # sphere(pos=(pos[1],pos[2],-pos[0]), radius=0.1)
        # cylinder(pos=(0,0,0),  axis=(0,0,-pos[0]), radius=0.02,color=(1,0,0))
        # cylinder(pos=(0,0,-pos[0]),  axis=(pos[1],0,0), radius=0.02,color=(0,1,0))
        # cylinder(pos=(pos[1],0,-pos[0]),  axis=(0,pos[2],0), radius=0.02,color=(0,0,1))
        # X3 = np.array(pos).reshape((3,1))
        # return np.vstack((X3,1))
        
    # def camcosy(pos, xxx_todo_changeme):
        # (roll,pitch,yaw) = xxx_todo_changeme
        # left = np.array([[0,0,-1],[1,0,0],[0,1,0]])
        # #right = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
        # R = euler.ZYXdeg(yaw,pitch,roll).T.dot(left)
        # Rc = np.array([[0,1,0],[0,0,-1],[1,0,0]]).dot(R)
        # #ux = arrow(pos=(pos[1],pos[2],-pos[0]),axis=R[0],color=(1,0,0))
        # #uy = arrow(pos=(pos[1],pos[2],-pos[0]),axis=R[1],color=(0,1,0))
        # #uz = arrow(pos=(pos[1],pos[2],-pos[0]),axis=R[2],color=(0,0,1))
        # cx = arrow(pos=(pos[1],pos[2],-pos[0]),axis=0.5*Rc[0],color=(1,0,0))
        # cy = arrow(pos=(pos[1],pos[2],-pos[0]),axis=0.5*Rc[1],color=(0,1,0))
        # cz = arrow(pos=(pos[1],pos[2],-pos[0]),axis=0.5*Rc[2],color=(0,0,1))

        
    # Xw = pt((1,0.5,0))

    # Xw2 = pt((-2,-3,0))
    
    
    # #Xw = np.array([1,0.5,0]).reshape((3,1))#World Point (X:Hochachse,Y:Rechtsachse,Z:Elevation nach oben)
    # #Xw = np.vstack((Xw,1))#make it homogenious 
    # print("Xw",Xw)    
    # #Xw2 = np.array([-2,-3,0]).reshape((3,1))#World Point (X:Hochachse,Y:Rechtsachse,Z:Elevation nach oben)
    # #Xw2 = np.vstack((Xw2,1))#make it homogenious 
    # print("Xw2",Xw2)    
    # #cam = Camera(CoSy.UAV)#Camera schaut standardmaessig nach Norden und befindet sich im Ursprung
   # # cam.pose(90,90,0,0,0,5)
   # # cam.boresight(0,0,0,0,0,0)
    
   # # print "uav",np.around(cam.P_uav.dot(Xw).ravel()) # im 3D-Kamerakoordinatensystem( X zeigt Richtung Vorne)
   # # proj = np.around(cam.project(Xw).ravel())
   # # print "cam ",proj
   # # cam.intrinsics(640,512,1160,320,256)
    
    # #cam.distortion(0)
    # #import pickle
    # #lut = pickle.load(open("D:\\WILDRETTER\\tau640_pelz_2014.lut","r"))
    # #cam.lookuptable(lut[0],lut[1])
    # #print "distmodel",cam.distortionmodel
    # #print "Xc",cam.project(Xw)

    # print("\nNOCAM-Example")
    # cam = Camera(CoSy.UAV)#Camera schaut standardmaessig nach Norden und befindet sich im Ursprung
    # #camcosy((1,2,5),(0,90,0))    
    # cam.pose(0,90,0,1,2,5)
    # cam.boresight(0,0,0,0,0,0) # (z''/yaw,x''/pitch,y''/roll,X,Y,Z) rechtsseitiger drehsinn, aber bezogen auf kamerakoordinaten
    # cam.gimbal()
    # cam.transform()
   # # print "=======Transforms======"
    # #print "cosymat",cam.cosymat
    # #print "R_bs",  cam.R_boresight
    # #print "T_bs",  cam.T_boresight
    # #print "R_gi",  cam.R_gimbal
    # #print "T_gi",  cam.T_gimbal
    # #print "R_uav", cam.R_uav
    # #print "T_uav", cam.T_uav
    # #print "Ri_uav", cam.Ri_uav
    # #print "Ti_uav", cam.Ti_uav
    # #print "S", cam.S
    # #print "Si", cam.Si
    # print("===Cam_position===")
    # print(cam.position())
    # Xc = cam.project(Xw)
    # print("Xc", Xc)
    # Xcr = cam.reproject(Xc)
    # print("Xcr", Xcr)
    # print(Xc[0][0],Xc[1][0])
    # #P = cam.poi(Xc[0][0],Xc[1][0],0)
    # #print "Poi",P
    
    # #cylinder(pos=(2,5,-1),  axis=(10*Xcr[0],10*Xcr[2],10*Xcr[1]), radius=0.02,color=(1,0,0))
        
    
    # Xc2 = cam.project(Xw2)
    # print("Xc2", Xc2)
    
    # #print "ALT:"
    # #cam.boresight_alt(0,0,0,0,0,0) # (z''/yaw,x''/pitch,y''/roll,X,Y,Z) rechtsseitiger drehsinn, aber bezogen auf kamerakoordinaten
    # #Xc = cam.project_alt(Xw)
   # # print "P:",cam.P_cam
    # #print "Xc", Xc
    # #Xc2 = cam.project_alt(Xw2)
    # #print "Xc2", Xc2
    
    
   # # Xrepro = cam.reproject(Xc,5)
   # # print "Xrepro",Xrepro
    
    # print("\nPINHOLE-Example")
    # cam = Camera()
    # camcosy((1,2,5),(0,90,0))    
    # cam.pose(0,90,0,1,2,5)
    # cam.boresight(4,10,3,1,0,0)
    # cam.intrinsics(640,512,1160,320,256)
    # cam.gimbal()
    # cam.transform()
    # Xc = cam.project(Xw)
    # print("Xc",Xc)
    # Xrepro = cam.reproject(Xc,5)#,cam.fx)
    # print("Xrepro", Xrepro)
    # P = cam.poi(Xc[0][0],Xc[1][0],0)
    # print("Poi",P)
   
            
