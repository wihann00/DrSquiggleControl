# DrSquiggleControl
Scripts to control UFactory XArm 5 for R12860 PMT scanning
(Scripts still need some commenting. I didn't want to upload the entire SDK folder, so R12860_acc_scan.py should be put in the directory below)

Control of the robot arm should only require two python scripts: ArmPMT_class_fixed2.py, which contains the ArmPMT class which describes the geometry of the R12860 PMT and functions to position the robot with respect the PMT and its base, and R12860_acc_scan.py, which connects to the XArm API and describes the movement of the robot.

To run the program to have the arm scan the PMT, simply
```
cd /mnt/c/Users/Belle II Melbourne/Desktop/HyperK/wihann/xArm-Python-SDK-master/example/wrapper/xarm5
```
and run
```
python3 R12860_acc_scan.py
```

Distance from the light to the surface of PMT and distance between base of the robot arm (origin) and the top of the PMT can be adjusted in ArmPMT_class_fixed2.py on lines 37 and 38 respectively. These should not need to be changed unless we find that the accuracy of the XArm 5 is insufficient and distance between the robot and PMT needs to be adjusted.

In R12860_acc_scan.py, together the variables ```azimuthals``` and ```zeniths``` describe how many points will be scanned. This should not need to be changed unless for testing purposes. For Illustration on what azimuthal and zenith angle represents, refer to Shogo Horuichi's presentation on PMT Uniformity Measurement.
