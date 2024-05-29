# Trunk_Segment
--
## Environment
1. ROS
2. Intel RealSense SDK
3. Local/Anaconda pytorch
4. (Optional) If hoping deploy on NVIDIA GPU, NVIDIA Driver, CUDA, cuDNN is necessary
5. Please pay attention to the version matching between the above dependencies

## Tutorial
```
mkdir -p yolov8/src
cd ..
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 (Adjust by yourself)
cd src
git clone https://github.com/Turf1Ctkk/Trunk_Segment.git
```
Rename package Trunk_Segment to `yolov8_ros`
```
cd yolov8_ros
git clone https://github.com/ultralytics/ultralytics.git
cd scripts && chmod +x yolo_v8.py && cd ../../..
catkin_make && source ./devel/setup.bash
roslaunch yolov8_ros yolo_v8.launch
```
