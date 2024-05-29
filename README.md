# Trunk_Segment

## Environment
1. ROS
2. Intel RealSense SDK
3. Local/Anaconda pytorch
4. (Optional) If hoping deploy on NVIDIA GPU, NVIDIA Driver, CUDA, cuDNN is necessary
5. Please pay attention to the version matching between the above dependencies

## Tutorial
```
git clone https://github.com/Turf1Ctkk/Trunk_Segment.git
cd Trunk_Segment
chmod +x setup.sh && sh ./setup.sh

# To your package
cd yolov8/src/yolov8_ros
# Anaconda
conda create --name yolov8 python=3.8
conda activate yolov8
pip3 install torch torchvision torchaudio
pip3 install -r requirements.txt
cd ../..
catkin_make && source ./devel/setup.bash

#Launch
roslaunch realsense2_camera rs_aligned_depth.launch
roslaunch yolov8_ros yolo_v8.launch
```
