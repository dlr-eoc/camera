Quick Start
===========

the user guide - for the impatient ones
---------------------------------------

this library has two main methods::

    project()
    reproject()
 
to use this library you need just a few lines of code.
This projects the Point P(1,0,10) on to the camera image plane. We call the projected point p(u,v) ::

    import numpy as np
    import camera
    
    P = np.array([[1],[0],[10],[1]])   
    cam = camera.Camera()
    cam.intrinsics(640,512,1000,320,260)
    cam.attitudeMat(np.eye(4))
    p = cam.project(P)

and to reproject it back to the 3D world we use this code ::    

    Q = cam.reprojectToPlane(p,distance=10) 
    
if this easy reprojection does not fit your needs, you can use reproject(p) which returns a direction vector
and write your own reprojection wrapper.   

.. note::

    reprojectToPlane return a 3D Vector [[X][Y][Z]
            
