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
    mycam = camera.Camera()
    mycam.intrinsics(640,512,1000,320,260)
    p = cam.project(P)

and to reproject it back to the 3D world we use this code ::    

    Q = cam.reproject(p) 
    
.. note::

    This is a note box
    
.. warning:: and this is in the first line
    
    This is a warning box
    
.. Tip:: This is a greeen Tip Box
    bla bla bla:: 
    
        bla.open("mystr",12.4)
        
