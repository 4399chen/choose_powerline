# Enhancing Power Line Inspections with YOLOv8 and Realsense D455

Welcome to the official GitHub repository for the paper "Enhancing Power Line Inspections with YOLOv8 Image Segmentation and Realsense D455 Depth Sensing for Precise 3D Localization", presented at ICGNC 2024. This repository contains the code and documentation necessary to replicate our research findings and to facilitate further development in the field of automated power line inspections.

## Overview

Our work leverages the cutting-edge capabilities of YOLOv8 for image segmentation combined with the depth sensing prowess of the Realsense D455 camera. This innovative approach allows for the precise 3D localization of power lines, significantly enhancing the efficiency and safety of inspection processes.

## Related Repositories

- **YOLOv8 Segmentation and Detection**: For the implementation of image segmentation and detection as described in our study, please visit [Yolov8-seg-TensorRT-ROS-Jetson](https://github.com/4399chen/Yolov8-seg-TensorRT-ROS-Jetson).

- **Message Definitions**: The ROS message definitions utilized in our research can be found at the [vision_msgs repository](https://github.com/ros-perception/vision_msgs/tree/noetic-devel) under the `noetic-devel` branch.

## Installation

Provide detailed instructions on how to install and configure all necessary software and libraries to run the code. Include steps for cloning the repository, setting up a virtual environment, and installing dependencies.

```bash
git clone <repository-url>
cd <repository-directory>
pip install -r requirements.txt
```

## Usage

Explain how users can run your code, including any necessary commands and parameters. If your project includes scripts for training models, running simulations, or visualizing results, provide examples of how to execute these scripts effectively.

```bash
python main.py --options
```

## Citation

If you use our work in your research, please cite our paper using the following Bibtex entry:

```bibtex
@inproceedings{author2024enhancing,
  title={Enhancing Power Line Inspections with YOLOv8 Image Segmentation and Realsense D455 Depth Sensing for Precise 3D Localization},
  author={Chen Dong, Zhan Li, Jiayu Liu},
  booktitle={Proceedings of the International Conference on Geoscience, Navigational Computing and Cybernetics (ICGNC)},
  year={2024}
}
```

## License

Specify the license under which your code is released, and provide a link to the license text.

```markdown
This project is licensed under the terms of the MIT license.
```

## Contact

For any inquiries regarding this project, please contact us via email at `zhanli@hit.edu.cn`.

