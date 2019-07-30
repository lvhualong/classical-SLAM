# mynteye calibration

1. calibrate left_camera using pinhole model
```
roslaunch camera_models camera_calibr.launch mynteye:=mynteye-s is_left_camera:=true camera_model:=pinhole calibrate_nr:=15
```
calibrate right_camera using pinhole model
```
roslaunch camera_models camera_calibr.launch mynteye:=mynteye-s is_left_camera:=false camera_model:=pinhole calibrate_nr:=15
```

2. calibrate left_camera using kannala-brandt model
```
roslaunch camera_models camera_calibr.launch mynteye:=mynteye-s is_left_camera:=true camera_model:=kannala-brandt calibrate_nr:=15
```
calibrate right_camera using kannala-brandt model
```
roslaunch camera_models camera_calibr.launch mynteye:=mynteye-s is_left_camera:=false camera_model:=kannala-brandt calibrate_nr:=15
```

3. calibrate left_camera using mei model
```
roslaunch camera_models camera_calibr.launch mynteye:=mynteye-s is_left_camera:=true camera_model:=mei  calibrate_nr:=15
```
calibrate right_camera using mei model
```
roslaunch camera_models camera_calibr.launch mynteye:=mynteye-s is_left_camera:=false camera_model:=mei calibrate_nr:=15
```

**note: **mynteye correspond to mynteye-s, mynteye-d, mynteye-avarta, is_left_camera correspond to left_image_topic if true, calibrate_nr correspond to numbers of pictures taken.

---

part of [camodocal](https://github.com/hengli/camodocal)

[Google Ceres](http://ceres-solver.org) is needed.

# Calibration:

Use [intrinsic_calib.cc](https://github.com/dvorak0/camera_model/blob/master/src/intrinsic_calib.cc) to calibrate your camera.

# Undistortion:

See [Camera.h](https://github.com/dvorak0/camera_model/blob/master/include/camodocal/camera_models/Camera.h) for general interface:

 - liftProjective: Lift points from the image plane to the projective space.
 - spaceToPlane: Projects 3D points to the image plane (Pi function)
