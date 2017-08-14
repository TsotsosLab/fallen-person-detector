# Fallen Person Detector
With this system we present a new , non-invasive approach for fallen people detection. Our approach uses only
stereo camera data for passively sensing the environment. The key novelty is a human fall detector which uses a CNN based human pose estimator in combination with stereo data to reconstruct the human pose in 3D and estimate the ground plane in 3D. We have tested our approach in different scenarios covering most activities elderly people might encounter living at home. Based on our extensive evaluations, our systems shows high accuracy and almost no miss-classification.

![System steps](/misc/steps.jpg)



[Click here for the associated paper.](https://arxiv.org/pdf/1707.07608.pdf)

[Original home of this project](https://github.com/solbach/fallen-person-detector)

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

## System Overview

![System steps](/misc/overview.png)


## COPYRIGHT NOTICE

THIS SOFTWARE IS THE PROPERTY OF ITS AUTHOR MARKUS D. SOLBACH. THE AUTHOR GRANTS PERMISSION TO DISTRIBUTE THE SOFTWARE IN ITS ORIGINAL FORM WITHOUT MODIFICATION. UNDER SUCH CIRCUMSTANCES THE COPYRIGHT NOTICE AND FUNCTIONALITY OF THE SOFTWARE MUST REMAIN ENTIRELY INTACT INCLUDING ALL SYNTACTIC AND SEMANTIC ELEMENTS OF ITS FUNCTION. THE AUTHOR GRANTS A NON-EXLUSIVE LICENSE TO INDIVIDUALS WISHING TO USE THE SOFTWARE FOR NON-PROFIT RESEARCH PURPOSES. THOSE WISHING TO EMPLOY THE SOFTWARE FOR COMMERCIAL OR PROFIT SEEKING ENDEAVORS SHOULD CONTACT THE AUTHOR TO DISCUSS LICENSING.

DERIVATIVE WORKS ARE ALLOWED BY INDIVIDUALS FOR NON-PROFIT RESEARCH PURPOSES. SUCH DERIVATIVE WORKS MAY NOT BE DISTRIBUTED WITHOUT WRITTEN CONSENT OF THE AUTHOR. ANY WORKS THAT USE THE ABOVE SOFTWARE IN WHOLE OR PART FOR A COMMERCIAL APPLICATION MUST OBTAIN A VALID LICENSE FROM THE AUTHOR.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
