/*
 * Copyright Shichao Yang,2016, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu 
 */

#pragma once

#include "isam_plane3d.h"

#include <iostream>
#include <ros/ros.h>
#include "Eigen/Dense"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>



typedef pcl::PointCloud<pcl::PointXYZRGB> pccloudrgb;

class tracking_frame;

// plane landmark information. information may be updated as new observation of the nodes comes
class Map_plane {
public:
   
    Map_plane();
    isam::Plane3d temp_value; // tempoary storing.
    isam::Plane3d_Node* plane_vertex;  // slam vertex of plane landmark unique pointer
    
    int landmark_index;   // index in all the slam landmark node, I later don't change it even merge, always fixed, and increase
    bool deteted_by_merge=false;  // whether this node is being deleted after merging. if merged, will be removed from isam graph    
    
    std::vector<tracking_frame*> observer_frames; // frames observing this plane
        
    // NOTE the following plane properties are always being updated by latest frame.
    // HACK so whenever add new variables here, note to change the copy_plane()
    // TODO could use a plane property class pointer. then we don't need to manually copy.    
    
    pccloudrgb::Ptr plane_cloud;
    
    ros::Time t_stamp;   // latest observer image time stamp   .toSec()
    int frame_seq_id=0;    // latest observer frame sequence id
    int frame_plane_indice = 0;  // plane index in each frame's all planes  ground is 0, wall is 1,2,3... might not be continuous
    int good_plane_indice = 0;   // plane index in all good plane 0,1,2,...N   continuous number
    int being_tracked_times = 0;   // how many times being observed in different frames. length of observer_frames
        
    // a bounding polygons 2*n . each column of matrixXd is 2D image coor. used for plane association by reprojection. 
    Eigen::MatrixXf plane_bound_close_2D_polys;
    
    // a bounding polygons 3*n. each column of matrixXd is 3D world coor. used for plane association by reprojection.  
    Eigen::MatrixXf plane_bound_close_3D_polys;
    
    // bounded plane distance to camera. relate to covariance/uncertainty
    double plane_dist_cam;
    
    // n*4 all ground line segments. including the connected ones. one image has one.  may updated later
    Eigen::MatrixXf ground_seg2d_lines;    
};


// selectively copy some(most) elements copy second term into first term
void copy_plane(Map_plane* descriptor_to,const Map_plane* descriptor_from);