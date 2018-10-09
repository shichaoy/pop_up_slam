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

// opencv pcl
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

// Eigen
#include <Eigen/Dense>
#include <Eigen/LU>


// using namespace std;
// using namespace cv;
using namespace Eigen;

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> MatrixX8uu;

//output n*2    opencv point vector to eigen matrix
void points_to_matrix(const std::vector<cv::Point> &cv_pts_in,MatrixXf &eigen_pts_out);
void points_to_matrix(const std::vector<cv::Point2f> &cv_pts_in,Matrix2Xf &eigen_pts_out);
//output n*3    opencv point vector to eigen matrix homogeneous coordinates
void points_to_homo_matrix(const std::vector<cv::Point> &cv_pts_in,MatrixXf &eigen_homo_out);
//input  n*2    eigen matrix to opencv point vector
void matrix_to_points(const MatrixX2f &eigen_pts_int,std::vector<cv::Point> &cv_pts_out);
void matrix_to_points(const MatrixX2f &eigen_pts_int,std::vector<cv::Point2f> &cv_pts_out);

void cvMat_to_eigenMat(const cv::Mat& cvMat_in, MatrixXf& eigenMat);
MatrixXf cvMat_to_eigenMat(const cv::Mat& cvMat_in);
void cvMat_to_eigen44Mat(const cv::Mat& cvMat_in, Matrix4f& eigenMat);
Matrix4f cvMat_to_eigen44Mat(const cv::Mat& cvMat_in);
void eigenMat_to_cvMat(const MatrixXf& eigenMat_in, cv::Mat& cvMat);
cv::Mat eigenMat_to_cvMat(const MatrixXf& eigenMat_in);

//vertically stack a vertical vector to a matrix
void vert_stack_v(const MatrixXf &a_in, const VectorXf &b_in, MatrixXf &combined_out);
//vertically stack a vertical vector to a matrix itself, not creating new
void vert_stack_v_self(MatrixXf &a_in, const VectorXf &b_in);
//vertically stack a matrix to a matrix, increase rows, keep columns
void vert_stack_m(const MatrixXf &a_in, const MatrixXf &b_in, MatrixXf &combined_out);
//vertically stack a matrix to a matrix itself, not creating new
void vert_stack_m_self(MatrixXf &a_in, const MatrixXf &b_in);
//horizontal stack a matrix to a matrix itself, not creating new
void hori_stack_m_self(MatrixXf &a_in, const MatrixXf &b_in);
//horizontal stack a column vector to a matrix itself, not creating new
void hori_stack_v_self(MatrixXf &a_in, const VectorXf &b_in);
//horizontal stack a series of matrix
void hori_stack_vec_m(const std::vector<MatrixXf> &mat_vec_in, MatrixXf &combined_out);
void hori_stack_vec_m(const std::vector<MatrixX8uu> &mat_vec_in, MatrixX8uu &combined_out);

//rays is 3*n, each column is a ray starting from origin
//plane is (4，1） parameters, compute intersection    output is 3*n    plane doesn't need to be normalized
void ray_plane_interact(const MatrixXf &rays,const Eigen::Vector4f &plane,MatrixXf &intersections);
void ray_plane_interact_homo(const MatrixXf &rays,const Eigen::Vector4f &plane,MatrixXf &intersections);
Vector3f pixel_hit_plane(const Vector2f& pixel, const Vector4f& plane_in_sensor, const Matrix3f& invK);
Matrix3Xf pixel_hit_plane(const Matrix2Xf& pixels, const Vector4f& plane_in_world, const Matrix4f& transToWolrd, const Matrix3f& invK);

// give a point pt in the image with (0,0) origin, with a direction in the image (delta_u,delta_v). 
// find the hippting point of top or left/right boundary.
Vector2f direction_hit_boundary(Vector2f pt,Vector2f direc, int img_width, int img_height, bool nobottom = false);

// find the intersection of two point line segments with a z plane of height
bool points_intersect_height(const Vector3f &pt_a,const Vector3f &pt_b, float height,Vector3f &intersect);

Vector3f pointdir_intersect_plane(const Vector3f &pt_a,const Vector3f &dir, const Vector4f& plane_equation);

// comptue point cloest distance to a line segments
float point_dist_lineseg(const Vector2f &begin_pt, const Vector2f &end_pt, const Vector2f &query_pt);

// compute projection fraction onto a line segment. [0,1] in between. <0. before begin,  >1. after end
float point_proj_lineseg(const Vector2f &begin_pt, const Vector2f &end_pt, const Vector2f &query_pt);

// comptue point cloest distance to a infinite line
float point_dist_line(const Vector2f &begin_pt, const Vector2f &end_pt, const Vector2f &query_pt);


// query point distance and projection to a infinite line defined by two points
void point_distproj_to_line(const Vector2f &begin_pt, const Vector2f &end_pt, const Vector2f &query_pt, float& dist, float& proj_percent);

// input is 3*n (or 2*n)  output is 4*n (or 3*n)
void real_to_homo_coord(const MatrixXf &pts_in,MatrixXf &pts_homo_out);

// input is 3*n (or 2*n)  output is 4*n (or 3*n)
MatrixXf real_to_homo_coord(const MatrixXf &pts_in);
VectorXf real_to_homo_coord_vec(const VectorXf &pts_in);

void delete_columns(Matrix2Xf &pts_in, int delete_col_ind);

// input is 3*n (or 4*n)  output is 2*n(or 3*n)
void homo_to_real_coord(const MatrixXf &pts_homo_in, MatrixXf &pts_out);

// input is 3*n (or 4*n)  output is 2*n(or 3*n)
MatrixXf homo_to_real_coord(const MatrixXf &pts_homo_in);
VectorXf homo_to_real_coord_vec(const VectorXf &pt_homo_in);

void fit_ransac_3d_plane(const Eigen::MatrixX3f& points);
void fit_ransac_3d_plane_mat(const cv::Mat& points_homo,cv::Mat& fitted_plane,std::vector<bool>& point_inliers,cv::Mat &singular_values);

// ground in the label map is 1 or 255 (not 0)
void blend_segmentation(cv::Mat& raw_color_img, cv::Mat& label_map, cv::Mat& blend_img);

bool check_element_in_vector(const float& element, const VectorXf &vec_check);
bool check_element_in_vector(const int& element, const VectorXi &vec_check);

//change angle from [-180,180] to [-90,90]
float normalize_to_pi(float angle);

// the input might be closed poly with one duplicate point.  the output has no duplicate point!
void polygon_overlap_func(const std::vector<cv::Point2f>& closed_polys1,const std::vector<cv::Point2f>& closed_polys2, std::vector<cv::Point2f>& open_polysout);
void polygon_overlap_func(const std::vector<cv::Point2f>& closed_polys1,const std::vector<cv::Point2f>& closed_polys2, 
			  const std::vector<cv::Point2f>& closed_polys3, std::vector<cv::Point2f>& open_polysout);
void polygon_overlap_func(const Eigen::Matrix2Xf& closed_polys1,const Eigen::Matrix2Xf& closed_polys2, Eigen::Matrix2Xf& open_polysout);
void polygon_overlap_func(const Eigen::Matrix2Xf& closed_polys1,const Eigen::Matrix2Xf& closed_polys2, 
			  const Eigen::Matrix2Xf& closed_polys3, Eigen::Matrix2Xf& open_polysout);

// cut a wall closed polygons, in world upright frame.  only first two points on ground. if length_thres>0, cut plane from closed camera point.
Eigen::Matrix3Xf cut_wall_polygons(const Eigen::Matrix3Xf& plane_poly_3d_world, float ceiling_threshold, float walllength_threshold=-1, 
				   const Vector3f& camera_center = Vector3f(0,0,0));
// let the bounding box lie in two ground points  cut_range1, cut_range2 (should lie on plane)
Eigen::Matrix3Xf cut_wall_polygons(const Eigen::Matrix3Xf& plane_poly_3d_world, const Vector3f& cut_range1, const Vector3f& cut_range2,
				   bool range1_active=true,bool range2_active=true);
