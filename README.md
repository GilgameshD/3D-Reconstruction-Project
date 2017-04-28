# realsense-project
These are some projects about realsense F200, using SDK and API provided by Intel.
1. get hand gestures and control ppt, do something like page up and page down. (Windows)
2. 3D scan object and scene and reconstruct with two methods (Mac OS)
  - This slam system supports two devices : realsense and primesense. 
      1. If you want to use primesense, you have to install Openni2, and the folder called <OpenNi_test> consists a program to save depth and color images to the disk, which you can use in the slam system.
      2. If you want to use realsense, the library you should use is librealsense, and a program for saving imgaes is in the folder called <realsense_scan>. 
