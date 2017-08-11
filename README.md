# Fallen Person Detector
With this system we present a new , non-invasive approach for fallen people detection. Our approach uses only
stereo camera data for passively sensing the environment. The key novelty is a human fall detector which uses a CNN based human pose estimator in combination with stereo data to reconstruct the human pose in 3D and estimate the ground plane in 3D. We have tested our approach in different scenarios covering most activities elderly people might encounter living at home. Based on our extensive evaluations, our systems shows high accuracy and almost no miss-classification.

![GitHub Logo](/misc/steps.jpg)



[Click here for the associated paper.](https://arxiv.org/pdf/1707.07608.pdf)

## Requirements
- ROS Kinetic
- [openpose-ros](https://github.com/solbach/openpose-ros/blob/master/README.md) and all its requirements


## Installation
- ```cd``` to your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- ```cd src/```
- ```github clone https://github.com/solbach/fallen-person-detector```
- ```cd ../```
- ``` catkin_make```
- ```source devel/setup.bash```
  - _needs to be done with every new terminal running the fallen person detector_


## Run
1. ```roscore```
1. ```rosrun openpose-ros openpose-ros-node```
1. ``` rosrun fallen_person_detector fallen_person_detector_node ```

## Tested on
* Ubuntu 16.04
* ROS Kinetic
* CUDA 8.0
* cuDNN 6.0
* __OpenCV 3.2__

## Citation
Please cite this paper if you used it in your research:

```
Solbach, Markus D., and John K. Tsotsos.
"Vision-Based Fallen Person Detection for the Elderly."
arXiv preprint arXiv:1707.07608 (2017).
```


Accepted at [ACVR 2017](http://iplab.dmi.unict.it/acvr2017/index.php): Updated citation will follow soon.
