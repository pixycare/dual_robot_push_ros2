# Multi-Robot Cooperation System with ROS2, YOLOv8 and LiDAR

## Overview

This project implements a multi-robot cooperation system based on ROS2, computer vision, and LiDAR sensing. Two autonomous mobile robots are able to detect each other and a target object, navigate in the environment, and collaboratively push a box.

The system integrates a ROS2-based control node with real-time object detection using YOLO and distance estimation from LiDAR. Camera perception and range sensing are fused to enable robust autonomous behavior in simulation and real-world scenarios.

In the real robot implementation, a custom fine-tuned YOLO model was trained on a dataset collected from images of the robots and target objects. This improves detection accuracy compared to the default pretrained model.

In this repository, a standard YOLO model is included by default and should be replaced with a custom-trained model depending on the user’s dataset and application.

---

## Technologies

* ROS2
* Python
* YOLOv8
* Ultralytics
* OpenCV
* LiDAR
* Computer Vision
* Roboflow
* NVIDIA JetBot
* Gazebo
* Docker

---

## Training Custom YOLO Model

### Dataset Preparation with Roboflow

To train a custom YOLO model, prepare a dataset containing images of the robots and target objects.

Recommended steps:

1. Collect images of the robot and target object.
2. Upload the images to Roboflow.
3. Annotate objects using bounding boxes.
4. Split the dataset into training, validation, and test sets.
5. Export the dataset in YOLO format.

---

### Model Training

After exporting the dataset, train the YOLO model using the following command:

```bash
yolo detect train model=yolo8n.pt data=data.yaml epochs=NUMBER_OF_EPOCHS imgsz=640
```

After training, the best model weights are usually saved as:

```bash
runs/detect/train/weights/best.pt
```

This file can then be used in the ROS2 perception node.

---

## Running the System

### 1. Start Docker Container

Start the ROS2 Docker container:

```bash
sh ros2containerStart.sh
```

---

### 2. Open Another Terminal Inside the Container

In a new terminal, enter the running container:

```bash
docker exec -it -e DISPLAY ros2_jazzy_lpa /bin/bash
```

---

### 3. Start ROS2–Gazebo Bridge

Run the bridge script:

```bash
bash bridge.sh
```

---

### 4. Build Workspace

Build the ROS2 workspace:

```bash
bash build.sh
```

---

### 5. Run Simulation

Start the Gazebo simulation:

```bash
gz sim jetbot.sdf --render-engine ogre
```

---

## Model Usage in ROS2

The YOLO model can be loaded and used inside a ROS2 node as follows:

```python
from ultralytics import YOLO

model = YOLO("best.pt")
results = model(frame)
```

The input `frame` is usually an image captured from the robot camera. The model returns detection results, including bounding boxes, class labels, and confidence scores.

These detections can then be combined with LiDAR distance measurements to estimate the relative position of detected robots or target objects.

---

## Notes

* A custom YOLO model was used in the real robot implementation.
* The repository includes a default YOLO model for demonstration purposes.
* Users should retrain the model on their own dataset for best performance.
* Detection performance strongly depends on dataset quality, lighting conditions, object variety, and image diversity.
* For real-world deployment, the trained model should be tested and validated on images collected from the target environment.

---

## Summary

This project demonstrates how ROS2, YOLO-based object detection, LiDAR sensing, and Gazebo simulation can be combined to create a cooperative multi-robot system. The system can be used as a foundation for further research and development in autonomous robotics, collaborative navigation, and real-time perception.
