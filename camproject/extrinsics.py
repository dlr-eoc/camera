import numpy as np
from numpy import sin as s, cos as c
from . import *
            
class Extrinsics(object):
    '''
    Default Frame References for XYZ coordinates are FRD (front right down) for UAV or Gimbal
    and RDF (right down front) for camera images. That means Z points to the scene. 
    We use right handed coordinate systems for UAV Gimbal and Camera Images.
    For the default 3D World Geo Coordinates we use a lefthanded coordinate system: NEU (north, east, up).
    The reason for this inconvenient system is to have correct (clockwise) magentic field data 
    and intuitively correct positive flight altitude data for Z (0m is start position of the UAV).
    '''
    
    def __init__(self):
        #NEUtoFRD = homo(-ZXYdeg(180,0,0).T).dot(np.eye(4))
        #NEUtoRDF = homo(-ZXYdeg(-90,90,0).T).dot(np.eye(4))
        #transponierte Matrix, weil das Koordinatensystem gedreht wird, nicht der Vektor (passive Rotation)
        #"-" weil konvertierung von linkssystem nach rechtssystem
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
        
        
    def setPose(self,roll=0,pitch=0,yaw=0,X=0,Y=0,Z=0,order="ZYX"): 
        if order == "ZYX":
            self.R_uav = homo(ZYXdeg(yaw,pitch,roll).T) # transponierte Matrix, weil das Koordinatensystem gedreht wird, nicht der Vektor (passive Rotation)
        elif order == "ZXY":
            self.R_uav = homo(ZXYdeg(yaw,roll,pitch).T) 
        else: 
            raise("Extrinsics Euler Angle Order is not supportet. Choose: ZYX or ZXY")
        self.T_uav = np.array([[1,0,0,X],[0,1,0,Y],[0,0,1,Z],[0,0,0,1]])
        
    def setGimbal(self,roll=0,pitch=0,yaw=0,order="ZYX"):
        if order == "ZXY":
            R = ZXYdeg(yaw,roll,pitch)
        elif order == "ZYX":
            R = ZYXdeg(yaw,pitch,roll)
        else: 
            raise("Extrinsics Euler Angle Order is not supportet. Choose: ZYX or ZXY")
        self.R_gimbal = homo(R.T)
            
    def setCameraBoresight(self,droll=0,dpitch=0,dyaw=0,dx=0,dy=0,dz=0,order="ZYX"):
        '''
        These boresight angles are mount corrections between camera and gimbal.
        '''
        if order == "ZXY":
            R = ZXYdeg(yaw,roll,pitch)
        elif order == "ZYX":
            R = ZYXdeg(yaw,pitch,roll)
        else: 
            raise("Extrinsics Euler Angle Order is not supportet. Choose: ZYX or ZXY")
        self.R_cam_boresight = homo(R.T)
        self.T_cam_boresight = np.array([[1,0,0,dx],[0,1,0,dy],[0,0,1,dz],[0,0,0,1]])
        
    def setUAVBoresight(self,droll=0,dpitch=0,dyaw=0,dx=0,dy=0,dz=0,order="ZYX"):
        '''
        These boresight angles are mount corrections between gimbal and UAV.
        For brushless gimbals you can ignore droll, dpitch and dyaw. 
        '''
        if order == "ZXY":
            R = ZXYdeg(yaw,roll,pitch)
        elif order == "ZYX":
            R = ZYXdeg(yaw,pitch,roll)
        else: 
            raise("Extrinsics Euler Angle Order is not supportet. Choose: ZYX or ZXY")
        self.R_uav_boresight = homo(R.T)
        self.T_uav_boresight = np.array([[1,0,0,dx],[0,1,0,dy],[0,0,1,dz],[0,0,0,1]])
        
    def transform(self):
        self.S = self.cosyUAVToCamera.dot(self.R_cam_boresight).dot(self.T_cam_boresight).dot(self.R_gimbal).dot(self.R_uav_boresight).dot(self.T_uav_boresight).dot(self.R_uav).dot(self.T_uav).dot(self.cosyWorldToUAV.T)
        return self.S
    
    