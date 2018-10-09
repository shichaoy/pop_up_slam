#!/bin/bash

# compile bag of words
cd pop_planar_slam/Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

# install isam dependency
sudo apt-get install cmake libsuitesparse-dev libeigen3-dev libsdl1.2-dev doxygen graphviz

# install python library
sudo easy_install pip
cd ../../../..
sudo pip install -r py_requirements.txt