# Pop Up SLAM #
This code contains a basic implementation for pop-up plane SLAM. Given RGB and ground segmentation image/video, the algorithm pop up walls from each frame then formulate a plane SLAM to optimize both camera pose and plane positions. ```pop_planar_slam``` is main package. Other code is based on and modified from single image [pop-up](https://github.com/shichaoy/pop_up_image).

**Authors:** [Shichao Yang](http://www.frc.ri.cmu.edu/~syang/)

**Related Paper:**

* **Pop-up SLAM: Semantic Monocular Plane SLAM for Low-texture Environments**, IROS 2016, S. Yang, Y. Song, M. Kaess, S. Scherer [**PDF**](http://www.frc.ri.cmu.edu/~syang/Publications/iros_2016_popslam.pdf)
* **Real-time 3D Scene Layout from a Single Image Using Convolutional Neural Networks**, ICRA 2016, S. Yang, D. Maturana, S. Scherer  [**PDF**](http://www.frc.ri.cmu.edu/~syang/Publications/icra_2016_sinpop.pdf)

If you use the code in your research work, please cite the above paper. Feel free to contact the authors if you have any further questions.



# Installation

### Prerequisites
This code contains several ros packages. We test it in **ROS indigo/kinetic, Ubuntu 14.04/16.04, Opencv 2/3**. Create or use existing a ros workspace.
```bash
mkdir -p ~/popup_ws/src
cd ~/popup_ws/src
catkin_init_workspace
git clone git@github.com:shichaoy/pop_up_slam.git
cd pop_up_slam
```

### Install dependency packages of python
```bash
sh install_dependenices.sh
```

### Download Data
```bash
sh download_data.sh
```
if the download link breaks, please download [here](https://drive.google.com/open?id=192wwtxHryaTYMNsdYEu_YI9iQNy1BCbd) and follow sh file to process it. Will fix it later.

If ```wget``` not installed, ```sudo apt-get install wget ```


### Compile
```bash
cd ~/popup_ws
catkin_make -j4
catkin_make --only-pkg-with-deps pop_up_python -j4     #just double check
catkin_make --only-pkg-with-deps pop_planar_slam -j4   #just double check
```


# Running #
```bash
source devel/setup.bash
roslaunch pop_planar_slam pop_planar_tum_far_example.launch
```
You will see results in Rviz.

### Notes

1. If it shows "NameError: 'pop_up fun...params' is not defined". That is due to python dependency modules are not installed properly. Make sure "from skimage.measure import find_contours,approximate_polygon" can work alone in python. Also 'souce setup.bash' when python changes.  There is some pop-up image python function I cannot find C++ replacement therefore we have to use python... All SLAM parts are in C++.
 
2. The bag file dataset contains the pre-processed dataset with synced rgb and label image. The code just gives an illustration of the core algorithm. No other odometry is used now, which needs to be integrated for large dataset.
