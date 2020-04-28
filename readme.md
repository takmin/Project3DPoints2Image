# Project 3D Points to Image

Project 3D points to a 2D image and visualize it.

To use this tool, you need to follow the steps below:

1. Plot 2D-3D point pairs between camera image and 3d point clouds with "Plot2D3DPair"
    - https://github.com/takmin/Plot2D3DPair
2. Compute internal camera parameters of 2D Camera with "CameraCalibration"
    - https://github.com/takmin/CameraCalibration
3. Compute relative pose between camera and 3D scanner with "Calibrate2D3D"
    - https://github.com/takmin/Calibrate2D3D
4. To confirm the calibration result, project 3D points to an image with this tools.  You need calibration parameters obtained at the step 2 and 3.

Here is usage:

```
Project3DPoints2Image.exe -pcd=<point cloud> -img=<image> -int=<camera param> -ext=<external param>
  where options are:
                   -h                    = Print this help
                   -pcd file_name        = Input PCD/PLY file name
                   -img file_name        = Input image file name
                   -int file_name        = Internal camera parameter file obtained at step 2.
                   -ext file_name        = External camera parameter file obtained at step 3.
```

