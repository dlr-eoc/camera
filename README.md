# camproject
a python module for camera projection and reprojection

Maintainer
----------
  * Martin Israel <martin.israel@dlr.de>
  
Requirements
------------

  * python 2.6, 2.7, or 3.x
  * NumPy
  

Installation
------------
The easiest way is to install it from Pypi with:
    `$ pip install camproject`

To install system-wide from source distribution:
   `$ python setup.py install`
   
Quickstart-Usage
----------------

    import numpy as np
    import camproject
    
    P = np.array([[1],[0],[10],[1]]) # this is a point in 3D (e.g. in meters)
    cam = camproject.Camera()  
    cam.intrinsics(640,512,1000,320,260) # inner parameters: (in pixels)
    # (im_width,im_height, focal_length, centerpixel_x, centerpixel_y)
    cam.attitudeMat(np.eye(4))  # outer parameters: point to z-direction
    p = cam.project(P) # gives pixel coordinates on the image

to reproject it back to the 3D world we use this code

    Q = cam.reprojectToPlane(p,distance=10) 

   
   
Documentation
-------------

* documentation can be found at https://martin-israel.de/doc/camera
* If you acquired this code via GitHub, then you can build the documentation using sphinx.
      From the documentation directory, run:
          `$ make html`
   
