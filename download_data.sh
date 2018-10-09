#!/bin/bash

# download the dataset
mkdir pop_planar_slam/dataset
cd pop_planar_slam/dataset
wget http://www.frc.ri.cmu.edu/~syang/dataset/iros_pop_slam_16/tum_far_preproc_tf.bag

cd ../params
wget http://www.frc.ri.cmu.edu/~syang/dataset/iros_pop_slam_16/ORBvoc.bin
