#!/bin/bash

# Set up workspace directory
WORKSPACE_NAME="yolov8"
PACKAGE_ORIGINAL="Trunk_Segment"
PACKAGE_RENAMED="yolov8_ros"
SCRIPT_NAME="yolo_v8.py"

# Create Catkin workspace
mkdir -p ~/$WORKSPACE_NAME/src
cd ~/$WORKSPACE_NAME

# Build the workspace with the desired Python executable
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# Navigate to source directory
cd src

# Clone the package repository
git clone https://github.com/Turf1Ctkk/$PACKAGE_ORIGINAL.git

# Rename the package directory
mv $PACKAGE_ORIGINAL $PACKAGE_RENAMED

# Navigate to the renamed package directory
cd $PACKAGE_RENAMED

# Clone the ultralytics repository for YOLO
git clone https://github.com/ultralytics/ultralytics.git

# Make the yolo_v8.py script executable
cd scripts
chmod +x $SCRIPT_NAME
cd ../../..

# Build the workspace again
catkin_make

# Source the setup.bash to overlay on the ROS environment
source ./devel/setup.bash

cd ~/$WORKSPACE_NAME/src/$PACKAGE_RENAMED
conda create --name yolov8 python=3.8
conda activate yolov8
pip3 install torch torchvision torchaudio
pip3 install -r requirements.txtcd ../..
catkin_make && source ./devel/setup.bash
