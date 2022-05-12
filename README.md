# RoPose ROS-Package
This repository includes all ROS-related tools and packages of the RoPose Dataset Generator - A toolset generate Datasets for industrial robot-arm pose estimation systems.

This includes:
* The **RoPose DataGrabber** to collect new ROS independent RoPose Datasets with real or simulated industrial manipulators
* A **calibration tool** to calculate the extrinsic camera transformation based on manual labeling

### Prerequisites 
* Basic installation of ROS
* OpenCV (should come with ros)
* cv_bridge https://github.com/ros-perception/vision_opencv
* PCL (should come with ros)
* pcl_ros https://github.com/ros-perception/perception_pcl
* Eigen (should come with ros)
* custom_paramter https://github.com/guthom/custom_parameters
Optional:
* QT (for the manual labeling for the calibration tool)

## Installing
* Clone the package and all the dependecies in your ROS-Workspace.  
* Enter package directory and init submodules: 
  * git submodule init
  * git submodule update

## Data-Grabber Configuration
To start the Dataset generator you can use simulated and real robot systems.

## Launching

### Real Cameras

### Simulated Cameras

## Open Source Acknowledgments
This work uses parts from:
* the **Robot Operation System** (ROS) https://www.ros.org/
* **Boost** https://www.boost.org/
* **OpenCV** https://opencv.org/
* **Eigen** http://eigen.tuxfamily.org/
* **PCL** http://pointclouds.org/
* **QT** https://www.qt.io
* **

**Thanks to ALL the people who contributed to the projects!**

## Authors

* **Thomas Gulde** - Main Author
* **Johann Andrejtschik** - https://github.com/jeyberg, Part of the calibration system

Cognitive Systems Research Group, Reutlingen-University:
https://cogsys.reutlingen-university.de/

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Citation
Please cite the following papers if this code is helpful in your research. 

```bash
@inproceedings{gulde2019roposeReal,
  title={RoPose-Real: Real World Dataset Acquisition for Data-Driven Industrial Robot Arm Pose Estimation},
  author={Gulde, Thomas and Ludl, Dennis and Andrejtschik, Johann and Thalji, Salma and Curio, Crist{\'o}bal},
  booktitle={2019 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2019},
  organization={IEEE}
}

@inproceedings{gulde2018ropose,
title={RoPose: CNN-based 2D Pose Estimation of industrial Robots},
author={Gulde, Thomas and Ludl, Dennis and Curio, Crist{\'o}bal},
booktitle={2018 IEEE 14th International Conference on Automation Science and Engineering (CASE)},
pages={463--470},
year={2018},
organization={IEEE}
}
```

