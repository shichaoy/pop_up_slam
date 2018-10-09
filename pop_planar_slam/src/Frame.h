#pragma once

#include "ros/ros.h"

#include "isam_plane3d.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "isam/isam.h"

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Thirdparty/ORB_SLAM/ORBextractor.h"
#include "Thirdparty/ORB_SLAM/ORBVocabulary.h"
#include "Thirdparty/ORB_SLAM/Converter.h"


#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"


using namespace Eigen;
typedef pcl::PointCloud<pcl::PointXYZRGB> pclRGB;

class Map_plane;

class tracking_frame{
public:

    int frame_seq_id;    // image topic sequence id, fixed
    cv::Mat frame_img;
    
    isam::Pose3d temp_pose; // tempoary storing. for first frame, it is pose,  for latter frames, it is odometry
    isam::Pose3d_Node* pose_vertex;  // one frame has one pose, pose never be deleted
      
    std::vector<isam::Pose3d_Plane3d_Factor*> pose_plane_facs;  // the same length as plane number.
    
    // NOTE only has the good planes, no manually connected edges
    std::vector<Map_plane*> observed_planes;  // all the planes nodes seen by this frame. might change due to merging

    ros::Time t_stamp;   // image time stamp   .toSec()  fixed

    
    MatrixX4f ground_seg2d_lines;  // n*4 all ground line segments. including the connected ones
    VectorXi good_plane_indices; // good plane indices in all the planes including manually connected
    std::vector<MatrixXf> all_3d_bound_polygons_good_world;  // all the plane's 3D bounding box in current frame, reprojected onto new plane  good planes
    std::vector<MatrixXf> all_3d_bound_polygons_world_raw;  // all the plane's 3D bounding box in current frame  raw
    std::vector<pclRGB::Ptr> partplane_clouds;  // reprojected  good planes, no manually connected
    std::vector<pclRGB::Ptr> partplane_cloud_raws; //raw

    std::vector<double> good_plane_dist_cam; // plane distance to camera. all good planes,  representing the plane uncertaintly
    
               
    void compute_frame_BoW(const ORB_SLAM::ORBVocabulary* mpORBvocabulary, ORB_SLAM::ORBextractor* mpORBextractor);
    
    float bow_score_to_origin;
    
    // Exctracted ORB points
    std::vector<cv::KeyPoint> mvKeys;

    // Bag of Words Vector structures
    DBoW2::BowVector mBowVec;  // vector of  (word_index, word_occurency), size is different of different images
    DBoW2::FeatureVector mFeatVec;    
};
