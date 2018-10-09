/*
 * Copyright Shichao Yang,2016, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu 
 */
// std c
#pragma once

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <ctime>
#include <assert.h>     /* assert */
#include <tuple>
// opencv pcl
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>
#include <pcl_conversions/pcl_conversions.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/LU>

// boost python to call python pop up
#include <boost/unordered_map.hpp>
#include <boost/python.hpp>
#include <python2.7/Python.h>


// ours
#include "pop_up_wall/matrix_utils.h"
#include "tictoc_profiler/profiler.hpp"
#include "line_lbd/line_lbd_allclass.h"
// #include "lineseg_detect/lsd.h"  // old wayof edge detection

using namespace std;
using namespace cv;
using namespace Eigen;
namespace bp = boost::python;


typedef pcl::PointCloud<pcl::PointXYZRGB> pclRGB;


class popup_plane
{
protected:  
  int width;
  int height;
  Matrix3f Kalib;
  Matrix3f invK;
 
  bp::object* py_module;
  bp::object py_dictionary;
  bool inited_python;

public:
  //some popup parameters  
  float ceiling_threshold = 2.5;     // ceiling height threshold
  float walllength_threshold = -1;   // wall length threshold, effective if >0
  bool downsample_contour=false;
  bool use_fast_edges_detection=false;
  // 3d plane distance threshold   camera distance to wall plane segments. cam dist to cloest wall endpoint
  double plane_cam_dist_thre=10;  

public:
  ros::NodeHandle nn;  // to parse the parameters

  popup_plane();
  ~popup_plane();

  cv::Mat edge_label_img;  // ground region formed by edges    ground is 255, wall is 0, void is 127  

  void Init_python();	//using python function to get ground polygons, generatly take ~5 ms more 

  line_lbd_detect* line_lbd_ptr;
  
//==============================================   fixed 2d information after getting ground edges.  ===========================  
  int total_plane_number; // include the ground either 0, or 2,3,... at least a ground a a wall
  MatrixX4f ground_seg2d_lines_connect;  // n*4 close polygons edges  contain some manually connected edges 
  MatrixXf ground_seg2d_lines_actual; // open (actual) polygons edges, not the manually connect ones
  VectorXi actual_plane_indices_inall;   //  ground_seg2d_lines_actual  in ground_seg2d_lines_connect
  
//==============================================   3d plane information related to pose. (see get_plane_equation())  ==============================================    
  MatrixX4f all_planes_world;   //n*4 store all the plane parameters include ground in world frame, each row is a plane  ( may inclue manually connected part)
  MatrixX4f all_planes_sensor;  //n*4 store all the plane parameters include ground in sensor frame, each row is a plane ( may inclue manually connected part)
  Vector4f ceiling_plane_world;
  Vector4f ceiling_plane_sensor;

  int good_plane_number; //// include the ground  either 0, or 2,3 ...
  VectorXi good_plane_indices;  // ground and good wall (not manually connected) indices in all the planes(all_planes_world,all_planes_sensor)
  MatrixXf ground_seg3d_lines_world;  // n*6 3D world coordinates of ground polygons. each row is two points x1 y1 z1 x2 y2 z2
  MatrixXf ground_seg3d_lines_sensor; // n*6 sensor coordinate of ground polygons  . each row is two points x1 y1 z1 x2 y2 z2  
  
  bool simple_polygon_mode; // in simple mode. wall is simple rectange, no complicated polygon, ground is empty poly. see find_2d_3d_closed_polygon
  vector<Matrix2Xf> all_closed_2d_bound_polygons; //each 2*n   pose will affect this. has ground (void in simpleMode)
  vector<Matrix3Xf> all_3d_bound_polygons_world;  //each 3*n world. has ground (void in simpleMode) considering the ceiling height
  vector<Matrix2Xf> all_closed_2d_cutted_polygons;  //each 2*n   cutted projected 2d polygon. has ground (void in simpleMode)
  VectorXf all_plane_dist_to_cam;  // camera distance to wall plane segments (pt to cloest line segment).   include the ground, and manually edges
  pclRGB::Ptr pcl_cloud_world;
  std::vector<pclRGB::Ptr> partplane_clouds_open_all;  
  
  
 //==============================================    interface functions  ======================================================
  void set_calibration(Matrix3f calib);
    
  // //given the rgb (or gray) image, label image(0-255), get the 2D ground edge, no need of pose,  ground is labeled 255 (not 1), wall label is 0. must be uint8.  
  void get_ground_edges(const cv::Mat &raw_img, const cv::Mat &label_map);

  // from the ground edges and current pose, compute the plane equation, also call find_2d_3d_closed_polygon to compute polygons  
  // can affect good plane indice and others.   ground_seg2d_lines might include manually connected part (depending on use)
  void get_plane_equation(const MatrixX4f& ground_seg2d_lines, const Matrix4f& transToWolrd, bool find_close_polygon =true);

  // give ground 2d n*4 segments,and camera pose, compute the plane equation including ground. don't check distance....
  // static function, doesn't depend on class variables!!
  static void update_plane_equation_from_seg(const MatrixX4f& ground_seg2d_lines, const Matrix3f& invK, Matrix4f& transToWorld, MatrixX4f& all_planes_sensor_out);
  // faster version of above, don't compute global stuff.
  static void update_plane_equation_from_seg_fast(const MatrixX4f& ground_seg2d_lines, const Matrix3f& invK, Matrix4f& transToWorld, MatrixX4f& all_planes_sensor_out);
  
  //create 3d point cloud in world frame 3*n (pcl_cloud ptr) wall plane indices (from 0, not including ground).  old name(python name)  pop_up_3dcloud_closed_polygons_transforms; 
  void generate_cloud(const cv::Mat &bgr_img,const Matrix4f& transToWolrd, bool downsample_poly=false, bool separate_cloud=false,
                      bool mixed_cloud=true,bool get_label_img=false,float depth_thre=10.0);
    
  // using stored results to get depth, by doing this, we don't use stored information from class object.
  void get_depth_map_good(cv::Mat& out_depth,const Matrix4f& transToWolrd, bool downsample_poly, VectorXi& good_plane_indexs,vector<Matrix2Xf> & all_closed_2d_bound_polys, \
			    MatrixX4f& all_planes_sensors, Vector4f& ceilin_plane_sensor) const;  
  
// ============================================== utility function   ============================================== 
  void matrixToCloud(const vector<Matrix3Xf> &pixels_homo, const vector<MatrixXf> &local_pts_parts, const vector<Matrix3Xf> &global_pts_parts, 
		     const cv::Mat &bgr_img=cv::Mat(), bool separate_cloud=false,bool mixed_cloud=true,float depth_thre=10.0);
  //given the ground 2d polygon segments n*2, find and update all the wall plane's close polygons. edge are connected
  void find_2d_3d_closed_polygon(const Matrix4f& transToWolrd, const MatrixX4f& ground_seg2d_lines);
  // in simple mode. wall is simple rectange, no complicated polygon    no relationship between each other edge. ground is empty polygon.
  void find_2d_3d_closed_polygon_simplemode(const Matrix4f& transToWolrd, const MatrixX4f& ground_seg2d_lines);

  // input is closed polygons, n*2, each row is a points, output is 3*m homo coord, (don't check ifinside image here) 
  // static function, doesn't depend on class variables!!
  static void closed_polygons_homo_pts(const MatrixX2f &closed_2d_polygon,Matrix3Xf &inside_pts_homo_out, bool downsample, bool show_debug_info=false);

  // c++ compute contours, then call python to find edges
  tuple<MatrixXf, MatrixXf, VectorXf> find_edges_contours(const cv::Mat &raw_img, const cv::Mat &label_map);

  // given raw edges, contour from c++, call python to do edge pre-process, interval tree, post processing
  tuple<MatrixXf, MatrixXf, VectorXf> edge_contour_python_get_polygons(const cv::Mat &lsd_edges,const cv::Mat &gnd_contour);  // c++ get contour
  
  // given raw edges and label map, call python to find contours, do edge pre-process, interval tree, post processing
  tuple<MatrixXf, MatrixXf, VectorXf> edge_python_get_polygons(const cv::Mat &lsd_edges,const cv::Mat &label_map); // python get contour
  
  // given raw edges, then only call python to find contours, interval tree, then most other parts are written in C++
  tuple<MatrixXf, MatrixXf, VectorXf> edge_get_polygons(cv::Mat &lsd_edges,const cv::Mat &label_map); // python get contour

  // used in orbslam2, to extrally project cloud onto plane....
  Vector4f cloud_projected_plane;
  
  
  bool show_debug_info =  false;
  
// image process parameters
  int erosion_distance;
  int dilation_distance;
  
// python pop up parameters
  double pre_vertical_thre=15;
  double pre_minium_len=15;
  double pre_contour_close_thre=50;
  double interval_overlap_thre=20;
  double post_short_thre=30;
  double post_bind_dist_thre=10;
  double post_merge_dist_thre=20;
  double post_merge_angle_thre=10;
  double post_extend_thre=15;
  
  double pre_boundary_thre=5;
  double pre_merge_angle_thre=10;
  double pre_merge_dist_thre=10; 
  double pre_proj_angle_thre=20; 
  double pre_proj_cover_thre=0.6;
  double pre_proj_cover_large_thre=0.8;
  double pre_proj_dist_thre=100;	    

  int cloud_downsample_rate=20;  // only used in final fetching. actually matter less

  
  
};