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
cd yolov8
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
cd src
git clone https://github.com/Turf1Ctkk/Trunk_Segment.git
```
Change the package name to `yolov8_ros`
```

```
