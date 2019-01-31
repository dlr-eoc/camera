import numpy as np
from numpy import sin as s, cos as c
from . import *
      
class Extrinsics(object):
    '''
    is used to set the extrinsic orientation of a camera. output is a homogenious rotation and translation matrix (4x4).
    in camproject we call this attitudeMatrix.
    
    Default Frame References for XYZ coordinates are FRD (front right down) for UAV or Gimbal
    and RDF (right down front) for camera images. That means Z points to the scene. 
    We use right handed coordinate systems for UAV Gimbal and Camera Images.
    For the default 3D World Geo Coordinates we use a lefthanded coordinate system: NEU (north, east, up).
    The reason for this inconvenient system is to have correct (clockwise) magentic field data 
    and intuitively correct positive flight altitude data for Z (0m is start position of the UAV).
    '''
    def __init__(self):
        #NEUtoFRD = homo(-ZXYdeg(180,0,0).T).dot(np.eye(4))
        #FRDtoRDF = homo(ZXYdeg(90,90,0).T).dot(np.eye(4))
        #NEUtoRDF = homo(-ZXYdeg(-90,90,0).T).dot(np.eye(4))
        #we use the transposed Matrix, because the coordinate system is rotated, and not the vector (passive rotation)
        #"-" is used when we convert between left and right hand coordinate system
        #self.Ri_uav = self.R_uav.T # inverse Matrix
        NEUtoRDF = np.array([[0,1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]])
        FRDtoRDF = np.array([[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]])
        NEUtoFRD = np.array([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
        self.cosyWorldToUAV = NEUtoFRD
        self.cosyUAVToCamera = FRDtoRDF
        self.R_uav = np.eye(4)
        self.T_uav = np.eye(4)
        self.R_uav_boresight = np.eye(4)
        self.T_uav_boresight = np.eye(4)
        self.R_gimbal = np.eye(4)
        self.R_cam_boresight = np.eye(4)
        self.T_cam_boresight = np.eye(4)
        self.S = NEUtoRDF
        self.params = {"pose":{},"gimbal":{},"camera_boresight":{},"uav_boresight":{},"cosytransform":{}}
    
        
    def setPose(self,X=0,Y=0,Z=0,roll=0,pitch=0,yaw=0,order="ZYX"): 
        '''
        X,Y and Z are in NEU frame reference (positive geo coordinates), roll, pitch and yaw are in FRD frame reference.
        
        '''
        self.params["pose"]={"roll":roll,"pitch":pitch,"yaw":yaw,"X":X,"Y":Y,"Z":Z,"order":order}
        if order == "ZYX":
            self.R_uav = homo(ZYXdeg(yaw,pitch,roll).T) 
            # transponierte Matrix, weil das Koordinatensystem gedreht wird, nicht der Vektor (passive Rotation)
        elif order == "ZXY":
            self.R_uav = homo(ZXYdeg(yaw,roll,pitch).T) 
        else: 
            raise("Extrinsics Euler Angle Order is not supportet. Choose: ZYX or ZXY")
        self.T_uav = np.array([[1,0,0,-X],[0,1,0,-Y],[0,0,1,-Z],[0,0,0,1]])
        
    def setGimbal(self,roll=0,pitch=0,yaw=0,order="ZYX"):
        '''
        if this data comes from a camera builtin IMU, it's probably "ZYX" order,
        Gimbals used to have "ZXY" order. Reference frame: FRD 
        '''
        self.params["gimbal"]={"roll":roll,"pitch":pitch,"yaw":yaw,"order":order}
        if order == "ZXY":
            R = ZXYdeg(yaw,roll,pitch)
        elif order == "ZYX":
            R = ZYXdeg(yaw,pitch,roll)
        else: 
            raise("Extrinsics Euler Angle Order is not supportet. Choose: ZYX or ZXY")
        self.R_gimbal = homo(R.T)
            
    def setCameraBoresight(self,droll=0,dpitch=0,dyaw=0,dx=0,dy=0,dz=0,order="ZYX"):
        '''
        These boresight angles are mount corrections between camera and gimbal. Reference frame: FRD
        '''
        self.params["camera_boresight"]={"roll":droll,"pitch":dpitch,"yaw":dyaw,"dx":dx,"dy":dy,"dz":dz,"order":order}
        if order == "ZXY":
            R = ZXYdeg(dyaw,droll,dpitch)
        elif order == "ZYX":
            R = ZYXdeg(dyaw,dpitch,droll)
        else: 
            raise("Extrinsics Euler Angle Order is not supportet. Choose: ZYX or ZXY")
        self.R_cam_boresight = homo(R.T)
        self.T_cam_boresight = np.array([[1,0,0,-dx],[0,1,0,-dy],[0,0,1,-dz],[0,0,0,1]])
        
    def setUAVBoresight(self,droll=0,dpitch=0,dyaw=0,dx=0,dy=0,dz=0,order="ZYX"):
        '''
        These boresight angles are mount corrections between gimbal and UAV.
        For brushless gimbals you can ignore droll, dpitch and dyaw. Reference frame: FRD 
        '''
        self.params["uav_boresight"]={"roll":droll,"pitch":dpitch,"yaw":dyaw,"dx":dx,"dy":dy,"dz":dz,"order":order}
        if order == "ZXY":
            R = ZXYdeg(dyaw,droll,dpitch)
        elif order == "ZYX":
            R = ZYXdeg(dyaw,dpitch,droll)
        else: 
            raise("Extrinsics Euler Angle Order is not supportet. Choose: ZYX or ZXY")
        self.R_uav_boresight = homo(R.T)
        self.T_uav_boresight = np.array([[1,0,0,-dx],[0,1,0,-dy],[0,0,1,-dz],[0,0,0,1]])
    
    def getParams(self):
        self.params["cosytransform"]={"world_to_uav":self.cosyWorldToUAV,"uav_to_camera":self.cosyUAVToCamera}
        return self.params
        
    def transform(self):
        '''
        this creates the attitudeMatrix. you can see here the exact order of the transform matrices
        '''
        self.S = self.cosyUAVToCamera.dot(self.R_cam_boresight).dot(self.T_cam_boresight).dot(self.R_gimbal).dot(self.R_uav_boresight).dot(self.T_uav_boresight).dot(self.R_uav).dot(self.cosyWorldToUAV.T).dot(self.T_uav)
        return self.S
    
    
    