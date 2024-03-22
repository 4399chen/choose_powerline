# Enhancing Power Line Inspections with YOLOv8 Image Segmentation and Realsense D455 Depth Sensing for Precise 3D Localization

Welcome to the GitHub repository for "Enhancing Power Line Inspections with YOLOv8 Image Segmentation and Realsense D455 Depth Sensing for Precise 3D Localization," as presented at ICGNC 2024. This repository contains all necessary code and documentation to replicate our research findings and further advance automated power line inspection technologies.

## Overview

Our project combines YOLOv8's image segmentation capabilities with the depth-sensing technology of the Realsense D455 camera. This approach enables precise 3D localization of power lines, significantly improving inspection efficiency and safety.

## Related Repositories

- **Image Segmentation and Detection**: For detailed implementations of image segmentation and detection, refer to [Yolov8-seg-TensorRT-ROS-Jetson](https://github.com/4399chen/Yolov8-seg-TensorRT-ROS-Jetson).

- **ROS Message Definitions**: Utilized ROS message definitions are available at [vision_msgs](https://github.com/ros-perception/vision_msgs/tree/noetic-devel) under the `noetic-devel` branch.

## Installation

### Prerequisites

- **ROS Noetic**: Install ROS Noetic following the [official guide](http://wiki.ros.org/noetic/Installation).

- **Realsense_ros**: Install Realsense_ros on Jetson using instructions from [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md).

### Setup Procedure

1. **Create and Initialize Catkin Workspace**:
   ```bash
   mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/
   catkin_make
   source devel/setup.bash
   ```

2. **Clone Necessary Repositories**:
   ```bash
   cd src
   git clone https://github.com/4399chen/Yolov8-seg-TensorRT-ROS-Jetson.git
   git clone -b noetic-devel https://github.com/ros-perception/vision_msgs.git
   git clone https://github.com/4399chen/choosepowerline.git
   ```
   Ensure to modify `rs_camera.launch` to enable `align_depth`.

3. **Compile the Project**:
   ```bash
   cd ~/catkin_ws
   catkin_make -j1
   ```

4. **Environment Setup**:
   Add the workspace source command to your `.bashrc` file for convenience:
   ```bash
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   ```

## Usage

To launch the system:

1. **Realsense D455 Camera**:
   ```roslaunch realsense2_camera rs_camera.launch```
2. **Image Segmentation**:
   ```roslaunch yolov8_seg yolov8_seg.launch```
3. **Depth Extraction Node**:
   ```roslaunch choose_powerline min_rects_processor.launch```
4. **Mouse Selection Node**:
   ```roslaunch choose_powerline mouse.launch```

**One-click Launch Scripts**:
- Without Visualization: `roslaunch choose_powerline powerline.launch`
- With Rviz Visualization: `roslaunch choose_powerline powerline_with_rviz.launch`

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Realsense D455 fails to start | Reconnect the Realsense D455 USB cable. |
| Abnormal depth image noise | Gently touch the Realsense D455 or limit continuous use. |
| Low RGB image resolution | Restart the Realsense camera's ROS node. |

## Citation

Please cite our work using the following Bibtex entry:

```bibtex
@inproceedings{chen2024enhancing,
  title={Enhancing Power Line Inspections with YOLOv8 Image Segmentation and Realsense D455 Depth Sensing for Precise 3D Localization},
  author={Chen, Dong and Li, Zhan and Liu, Jiayu},
  booktitle={ICGNC 2024},
  year={2024}
}
```

## License

This project is under the MIT license. See the LICENSE file for more details.

## Contact

For inquiries, please email us at zhanli@hit.edu.cn.
