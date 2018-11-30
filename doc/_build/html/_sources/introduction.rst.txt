Introduction
============


What is the python camera module?
---------------------------------

camera is a python module that provides functionality for projection
from a 3D-scene to the 2D image plane of a camera. It also provides functionality 
for the reprojection from the 2d image plane to the scene in the 3D world coordinates.
It is commonly required in engineering and science applications for georeferencing images. 




How it works?
-------------

The camera geometry
```````````````````


Lets say, you take a photo of a scene with your camera. Your camera has a lens and a focal plane array (and a lot of other stuff we don't care about).
:numref:`camera_example` shows how a real world point P(X,Y,Z) is being projected through the center of the lens on to the image pixel coordinates (u,v) of the camera's focal plane array.
The optical axis pierces the center of the lens and hits the focal plane in the principle point (cx,cy). 
  
The image on the focal plane array is upside down. This leads to the fact that the axes of the image coordinates (u,v) point always in the reverse direction of the world (respectively the camera) coordinates.

.. _camera_example:
.. figure::  images/pinhole_camera2.png
   :align:   center

   the projection through a camera

To get not that confused with the orientation, the computer vision people always invert the image coordinate system and move the image plane at the same distance (f) in front of the lens. Technically i think it is impossible to realize such a camera, but from the mathematical point of view this leads to the same solution. :numref:`pinhole_cameramodel` shows the simplified model.

.. _pinhole_cameramodel:
.. figure::  images/pinhole_camera.png
   :align:   center

   simplified camera projection model (this image is based on an illustration from the openCV-Documentation)

Now the axes u and v point in the same direction as Xc and Yc. Zc points into the scene. The center of the lens is always the origin of the camera coordinate system. And we have a right sided coordinate system (left sided are used e.g. in geodetic applications). 

Please have a look at the `Open CV Camera Calibration Documentation <https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html>`_
The following documentation extends the OpenCV Docs or writes the same content in different words.

The pinhole camera model
````````````````````````

The most simple camera model is the pinhole camera. It consists of a light-tight hollow body with a very small pinhole and a lightsensitive film or an image detector. Due to the fact that it has no lens there exists no geometric distortion or blurring of unfocused objects. 
.. The pinhole camera can be used as a first order approximation of the mapping from a 3D scene to the image of a real camera.

From the mathematical point of view, the pinhole camera is simply a central projection from 3D to a 2D plane. The projection distance is the focal length of the camera. 

With the aid of homogenious coordinates, projective transformations like the central projection are much easier to describe.
The projection of a 3D point :math:`\mathbf{X} \in \mathbb{R}^3` onto the image plane of a pinhole camera can be described by the equation

.. math::
    \mathbf{\bar{x} = P\bar{X}}.
    
The 3D point is expressed by the homogenious vector :math:`\mathbf{\bar{X}}=[X,Y,Z,W]^T \in \mathbb{P}^3`, while :math:`X,Y` and :math:`Z` are the same as from our real world :math:`\mathbf{X} \in \mathbb{R}^3` and W you can easily set to 1. The resulting image vector :math:`\mathbf{\bar{x}}` has the projective coordinates :math:`[x,y,w]^T`. To get the pixel coordinates :math:`[u,v,1]^T` you have to devide :math:`\mathbf{\bar{x}}` by its third component :math:`w`. :math:`\mathbf{P}` is a :math:`3 \times 4` projection matrix with  

.. math::
    \mathbf{P = K \big{[} R^T|-R^T T\big{]}}.

The rotation matrix :math:`\mathbf{R}` and the translation vector :math:`\mathbf{T} \in \mathbb{R}^3` are the euclidean transformation between the camera and the world coordinate system. We call these parameters the extrinsic camera parameters (or outer orientation). The camera calibration matrix (or inner orientation) 

.. math::
    \mathbf{K} = \begin{bmatrix}
    f_x & s_{xy} & c_x \\
    0   & f_y & c_y \\
    0   & 0 & 1
    \end{bmatrix}
    
holds the intrinsic parameters of the camera. :math:`f_x` and :math:`f_y` are the focal distances, with :math:`f_y = a_r \cdot f_x`. Usually the aspect ratio :math:`a_r` is 1. When you now think: how could there be two focal distances for one lens? The answer is: when your detector elements are not quadratic (:math:`a_r \neq 1`), then you can use the dectector element size in x or y direction as unit to measure the focal distance. 
When your focal plane array is sheared you need to set :math:`s_{xy}` different to 1. The central point of the camera is at :math:`[c_x,c_y]^T` (in pixels).

The whole Equation is then:

.. math::
   \begin{bmatrix}
        x\\
        y\\
        w\\
   \end{bmatrix} =     
   \begin{bmatrix}
    f_x & s_{xy} & c_x \\
    0   & f_y & c_y \\
    0   & 0 & 1
    \end{bmatrix}
    \begin{bmatrix}
    r_{11} & r_{12} & r_{13} & t_{1}\\
    r_{21} & r_{22} & r_{23} & t_{2}\\
    r_{31} & r_{32} & r_{33} & t_{3}\\
    \end{bmatrix}
    \begin{bmatrix}
    X\\
    Y\\
    Z\\
    1
    \end{bmatrix}
    
The image pixel coordinates (u,v) are 

.. math::
   u = \frac{x}{w} , \qquad  v = \frac{y}{w}.
   
Brown's Camera Model
````````````````````

Until now we have ignored the distortion of the lens, but real camera lenses do have distortion. The brown camera model considers radial and tangential lens distortions.


.. math::    
    r = \sqrt{x^2+y^2}
    
    \hat{x} = x (1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_2 (r^2 + 2 x^2) + 2 p_1 x y
    
    \hat{y} = y (1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1 (r^2 + 2 y^2) + 2 p_2 x y
    
    u = c_x + \hat{x} f_x + \hat{y} s_{xy}
    
    v = c_y + \hat{y} f_y
        
        