/*
 * Copyright Shichao Yang,2016, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu
 *
 */

#include "pop_up_wall/matrix_utils.h"
#include <time.h>       /* time */


using namespace cv;
using namespace std;

//output n*2
void points_to_matrix(const vector<Point> &cv_pts_in,MatrixXf &eigen_pts_out)
{
   eigen_pts_out.resize(cv_pts_in.size(),2);
   for (int ind=0;ind<cv_pts_in.size();ind++)
   {
     eigen_pts_out(ind,0)=cv_pts_in[ind].x;
     eigen_pts_out(ind,1)=cv_pts_in[ind].y;
   }
}

void points_to_matrix(const vector<Point2f> &cv_pts_in,Matrix2Xf &eigen_pts_out)
{
   eigen_pts_out.resize(2,cv_pts_in.size());
   for (int ind=0;ind<cv_pts_in.size();ind++)
   {
     eigen_pts_out(0,ind)=cv_pts_in[ind].x;
     eigen_pts_out(1,ind)=cv_pts_in[ind].y;
   }
}

// n*3
void points_to_homo_matrix(const vector<Point> &cv_pts_in,MatrixXf &eigen_homo_out)
{
   eigen_homo_out.resize(cv_pts_in.size(),3);
   for (int ind=0;ind<cv_pts_in.size();ind++)
   {
     eigen_homo_out(ind,0)=cv_pts_in[ind].x;
     eigen_homo_out(ind,1)=cv_pts_in[ind].y;
     eigen_homo_out(ind,2)=1;
   }   
}

//input  n*2
void matrix_to_points(const MatrixX2f &eigen_pts_int,vector<Point> &cv_pts_out)
{
   cv_pts_out.resize(eigen_pts_int.rows());   
   for (int ind=0;ind<eigen_pts_int.rows();ind++)
     cv_pts_out[ind]=Point(eigen_pts_int(ind,0),eigen_pts_int(ind,1));    
}
void matrix_to_points(const MatrixX2f &eigen_pts_int,vector<Point2f> &cv_pts_out)
{
   cv_pts_out.resize(eigen_pts_int.rows());   
   for (int ind=0;ind<eigen_pts_int.rows();ind++)
     cv_pts_out[ind]=Point2f(eigen_pts_int(ind,0),eigen_pts_int(ind,1));    
}


void cvMat_to_eigenMat(const cv::Mat& cvMat_in, MatrixXf& eigenMat)
{
    eigenMat.resize(cvMat_in.rows,cvMat_in.cols);    
    for (int ii=0;ii<cvMat_in.cols;ii++)
      for (int jj=0;jj<cvMat_in.rows;jj++)
	eigenMat(ii,jj) = cvMat_in.at<float>(ii,jj);
}

MatrixXf cvMat_to_eigenMat(const cv::Mat& cvMat_in)
{
    MatrixXf eigenMat(cvMat_in.rows,cvMat_in.cols);    
    for (int ii=0;ii<cvMat_in.cols;ii++)
      for (int jj=0;jj<cvMat_in.rows;jj++)
	eigenMat(ii,jj) = cvMat_in.at<float>(ii,jj);  
    return eigenMat;
}

void cvMat_to_eigen44Mat(const cv::Mat& cvMat_in, Matrix4f& eigenMat)
{
    for (int ii=0;ii<4;ii++)
      for (int jj=0;jj<4;jj++)
	eigenMat(ii,jj) = cvMat_in.at<float>(ii,jj);
}

Matrix4f cvMat_to_eigen44Mat(const cv::Mat& cvMat_in)
{
    Matrix4f eigenMat;
    for (int ii=0;ii<4;ii++)
      for (int jj=0;jj<4;jj++)
	eigenMat(ii,jj) = cvMat_in.at<float>(ii,jj);
    return eigenMat;
}

void eigenMat_to_cvMat(const MatrixXf& eigenMat_in, cv::Mat& cvMat)
{
    cvMat = cv::Mat::zeros(eigenMat_in.rows(),eigenMat_in.cols(),CV_32F);
    for (int ii=0;ii<eigenMat_in.rows();ii++)
      for (int jj=0;jj<eigenMat_in.cols();jj++)
	cvMat.at<float>(ii,jj) = eigenMat_in(ii,jj);
}

cv::Mat eigenMat_to_cvMat(const MatrixXf& eigenMat_in)
{
    cv::Mat cvMat = cv::Mat::zeros(eigenMat_in.rows(),eigenMat_in.cols(),CV_32F);
    for (int ii=0;ii<eigenMat_in.rows();ii++)
      for (int jj=0;jj<eigenMat_in.cols();jj++)
	cvMat.at<float>(ii,jj) = eigenMat_in(ii,jj);
    return cvMat;
}

//vertical vector transpose it.
void vert_stack_v(const MatrixXf &a_in, const VectorXf &b_in, MatrixXf &combined_out)
{
   assert (a_in.cols() == b_in.rows());
   combined_out.resize(a_in.rows()+1,a_in.cols());
   combined_out<<a_in,
	         b_in.transpose();   
}

void vert_stack_v_self(MatrixXf &a_in, const VectorXf &b_in)
{
   assert (a_in.cols() == b_in.rows());
   MatrixXf combined_out(a_in.rows()+1,a_in.cols());
   combined_out<<a_in,
		 b_in.transpose();
   a_in=combined_out;
}

void vert_stack_m(const MatrixXf &a_in, const MatrixXf &b_in, MatrixXf &combined_out)
{
   assert (a_in.cols() == b_in.cols());
   combined_out.resize(a_in.rows()+b_in.rows(),a_in.cols());
   combined_out<<a_in,
		 b_in;
}
void vert_stack_m_self(MatrixXf &a_in, const MatrixXf &b_in)
{
   assert (a_in.cols() == b_in.cols());
   MatrixXf combined_out(a_in.rows()+b_in.rows(),a_in.cols());
   combined_out<<a_in,
		 b_in;
    a_in=combined_out;
}
void hori_stack_m_self(MatrixXf &a_in, const MatrixXf &b_in)
{
   assert (a_in.rows() == b_in.rows());
   MatrixXf combined_out(a_in.rows(),a_in.cols()+b_in.cols());
   combined_out<<a_in, b_in;
   a_in=combined_out;
}
void hori_stack_v_self(MatrixXf &a_in, const VectorXf &b_in)
{
   assert (a_in.rows() == b_in.rows());
   MatrixXf combined_out(a_in.rows(),a_in.cols()+1);
   combined_out<<a_in, b_in;
   a_in=combined_out;
}

void hori_stack_vec_m(const vector<MatrixXf> &mat_vec_in, MatrixXf &combined_out)
{
  //  assert (a_in.rows() == b_in.rows());
    int total_column=0;
    for (int i=0;i<mat_vec_in.size();i++)
      total_column+=mat_vec_in[i].cols();
    
    combined_out.resize(mat_vec_in[0].rows(),total_column);  // TODO hack here, I store only 10 walls   this is faster than push back one by one
    combined_out<<mat_vec_in[0], mat_vec_in[1],mat_vec_in[2],mat_vec_in[3],mat_vec_in[4],mat_vec_in[5], \
		  mat_vec_in[6],mat_vec_in[7],mat_vec_in[8],mat_vec_in[9],mat_vec_in[10],mat_vec_in[11],
		  mat_vec_in[12],mat_vec_in[13],mat_vec_in[14];
}


void hori_stack_vec_m(const vector<MatrixX8uu> &mat_vec_in, MatrixX8uu &combined_out)
{
  //  assert (a_in.rows() == b_in.rows());
    int total_column=0;
    for (int i=0;i<mat_vec_in.size();i++)
      total_column+=mat_vec_in[i].cols();
    
    combined_out.resize(mat_vec_in[0].rows(),total_column);  // TODO hack here, I store only 10 walls   this is faster than push back one by one
    combined_out<<mat_vec_in[0], mat_vec_in[1],mat_vec_in[2],mat_vec_in[3],mat_vec_in[4],mat_vec_in[5], \
		  mat_vec_in[6],mat_vec_in[7],mat_vec_in[8],mat_vec_in[9],mat_vec_in[10],mat_vec_in[11],
		  mat_vec_in[12],mat_vec_in[13],mat_vec_in[14];
}


//rays is 3*n, each column is a ray staring from origin  plane is (4，1） parameters, compute intersection  output is 3*n 
void ray_plane_interact(const MatrixXf &rays,const Eigen::Vector4f &plane,MatrixXf &intersections)
{  
    VectorXf frac = -plane[3]/(plane.head(3).transpose()*rays).array();   //n*1     
    intersections =  frac.transpose().replicate<3,1>().array() * rays.array();
}


//rays is 3*n, each column is a ray staring from origin  plane is (4，1） parameters, compute intersection  output is 3*n 
void ray_plane_interact_homo(const MatrixXf &rays,const Eigen::Vector4f &plane,MatrixXf &intersections)
{ 
    MatrixXf H=Matrix<float, 4, 3>::Identity(); // through homography
    H.row(3)=-plane.head(3)/plane(3);
    homo_to_real_coord(H*rays,intersections);
}


Vector3f pixel_hit_plane(const Vector2f& pixel, const Vector4f& plane_in_sensor, const Matrix3f& invK)
{
    Vector3f pixel_ray = invK*Vector3f(pixel(0),pixel(1),1);
    float frac = -plane_in_sensor[3]/(plane_in_sensor.head(3).dot(pixel_ray)); // simplified version of ray_plane_interact, with only one point
    return frac * pixel_ray;
}

Matrix3Xf pixel_hit_plane(const Matrix2Xf& pixels, const Vector4f& plane_in_world, const Matrix4f& transToWolrd, const Matrix3f& invK)
{
    Vector4f plane_sensor = transToWolrd.transpose()*plane_in_world;
    MatrixXf pixel_ray = invK*real_to_homo_coord(pixels);    //each column is a 3D world coordinate  3*n
    MatrixXf pts_sensor;
    ray_plane_interact(pixel_ray,plane_sensor,pts_sensor);
    MatrixXf pts_sensor_homo;
    real_to_homo_coord(pts_sensor,pts_sensor_homo);

    MatrixXf pts_world_homo=transToWolrd*pts_sensor_homo;//   # compute world ground polygons
    MatrixXf pts_world;
    homo_to_real_coord(pts_world_homo,pts_world); // #3*n
    
    return pts_world.topRows(3);
}


Vector2f direction_hit_boundary(Vector2f pt,Vector2f direc, int img_width, int img_height, bool nobottom)
{
    // line equation is (p_u,p_v)+lambda*(delta_u,delta_v)  parameterized by lambda
    
    float lambd;
    if (direc(1)<0){   // hit top edge          
	lambd=(0.0-pt(1))/direc(1);
	if (lambd>=0){  // up direction
	    Vector2f hit_pt=pt+lambd*direc;
	    if ((0<= (int)hit_pt(0)) && ((int)hit_pt(0)<= img_width-1))
		return hit_pt;
	}
    }
    
    if (!nobottom)
      if (direc(1)>0){  // hit bottom edge      
	  lambd=(img_height-1.0-pt(1))/direc(1);
	  if (lambd>=0){ // down direction	
	      Vector2f hit_pt=pt+lambd*direc;
	      if ((0<= (int)hit_pt(0)) && ((int)hit_pt(0)<= img_width-1))
		  return hit_pt;
	  }
      }    
    if (direc(0)>0){  // hit right edge
	lambd=(img_width-1.0-pt(0))/direc(0);
	if (lambd>=0){  // right direction
	    Vector2f hit_pt=pt+lambd*direc;
	    if ((0<= (int)hit_pt(1)) && ((int)hit_pt(1)<= img_height-1))
		return hit_pt;
	}
    }    
    if (direc(0)<0){  // hit left edge
	lambd=(0.0-pt(0))/direc(0);
	if (lambd>=0){  // left direction
	    Vector2f hit_pt=pt+lambd*direc;
	    if ((0<= (int)hit_pt(1)) && ((int)hit_pt(1)<= img_height-1))
		return hit_pt;
	}
    }
    return Vector2f(-1,-1);
    
}

// find the intersection of two point line segments with a z plane of height
bool points_intersect_height(const Vector3f &pt_a,const Vector3f &pt_b, float height,Vector3f &intersect)
{
  float lambda=(height-pt_a(2))/(pt_b(2)-pt_a(2));
  if (lambda>=0 && lambda<=1)
  {
    intersect=pt_a+lambda*(pt_b-pt_a);
    return true;
  }
  return false;
}

Vector3f pointdir_intersect_plane(const Vector3f &pt_a,const Vector3f &dir, const Vector4f& plane_equation)
{
  Vector3f normal = plane_equation.head<3>();
  return pt_a - (normal.dot(pt_a)+plane_equation(3))/(normal.dot(dir))*dir;
}

float point_dist_lineseg(const Vector2f &begin_pt, const Vector2f &end_pt, const Vector2f &query_pt)
{  // v w p
    // Return minimum distance between line segment vw and point p
  float length = (end_pt-begin_pt).norm();
  if (length < 0.001) return (query_pt-begin_pt).norm();   // v == w case
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line. 
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  float t = ((query_pt-begin_pt).dot(end_pt-begin_pt))/length/length;
  if (t < 0.0) return (query_pt-begin_pt).norm();       // Beyond the 'v' end of the segment
  else if (t > 1.0) return (query_pt-end_pt).norm();  // Beyond the 'w' end of the segment
  const Vector2f projection = begin_pt + t * (end_pt - begin_pt);  // Projection falls on the segment
  return (query_pt-projection).norm();
}

float point_proj_lineseg(const Vector2f &begin_pt, const Vector2f &end_pt, const Vector2f &query_pt)
{  // v w p
      // Return minimum distance between line segment vw and point p
    float length = (end_pt-begin_pt).norm();
    if (length < 0.001) return (query_pt-begin_pt).norm();   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line. 
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    float t = ((query_pt-begin_pt).dot(end_pt-begin_pt))/length/length;
    return t;
}


float point_dist_line(const Vector2f &begin_pt, const Vector2f &end_pt, const Vector2f &query_pt)
{  // v w p
    // Return minimum distance between line segment vw and point p
  float length = (end_pt-begin_pt).norm();
  if (length < 0.001) return (query_pt-begin_pt).norm();   // v == w case
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line. 
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  float t = ((query_pt-begin_pt).dot(end_pt-begin_pt))/length/length;
  const Vector2f projection = begin_pt + t * (end_pt - begin_pt);  // Projection falls on the segment
  return (query_pt-projection).norm();
}

void point_distproj_to_line(const Vector2f &begin_pt, const Vector2f &end_pt, const Vector2f &query_pt, float& dist, float& proj_percent)
{
      // Return minimum distance between line segment vw and point p
    float length = (end_pt-begin_pt).norm();
    if (length < 0.001){  // very short lines segments
	dist=(query_pt-begin_pt).norm();   // v == w case
	proj_percent=-1;
	return;
    }
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line. 
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    float t = ((query_pt-begin_pt).dot(end_pt-begin_pt))/length/length;
    const Vector2f projection = begin_pt + t * (end_pt - begin_pt);  // Projection falls on the segment
    dist=(query_pt-projection).norm();
    if (t>1)  // cut into [0 1]
      t=1;
    if (t<0)
      t=0;    
    proj_percent = t;
    return;
}


// input is 3*n (2*n)  output is 4*n(3*n)
void real_to_homo_coord(const MatrixXf &pts_in,MatrixXf &pts_homo_out)
{
  int raw_rows=pts_in.rows();
  int raw_cols=pts_in.cols();
  
  pts_homo_out.resize(raw_rows+1,raw_cols);
  pts_homo_out<<pts_in,
	        RowVectorXf::Ones(raw_cols);
}

// input is 3*n (2*n)  output is 4*n(3*n)
MatrixXf real_to_homo_coord(const MatrixXf &pts_in)
{
  MatrixXf pts_homo_out;
  int raw_rows=pts_in.rows();
  int raw_cols=pts_in.cols();
  
  pts_homo_out.resize(raw_rows+1,raw_cols);
  pts_homo_out<<pts_in,
	        RowVectorXf::Ones(raw_cols);
  return pts_homo_out;
}

VectorXf real_to_homo_coord_vec(const VectorXf &pt_in)
{
  VectorXf pt_homo_out;
  int raw_rows=pt_in.rows();  
  
  pt_homo_out.resize(raw_rows+1);
  pt_homo_out<<pt_in,
	       1;
  return pt_homo_out;
}


// input is 3*n(4*n)  output is 2*n(3*n)
void homo_to_real_coord(const MatrixXf &pts_homo_in, MatrixXf &pts_out)
{  
  pts_out.resize(pts_homo_in.rows()-1,pts_homo_in.cols());
  for (int i=0;i<pts_homo_in.rows()-1;i++)
      pts_out.row(i) = pts_homo_in.row(i).array()/pts_homo_in.bottomRows(1).array();   //replicate needs actual number, cannot be M or N
}
MatrixXf homo_to_real_coord(const MatrixXf &pts_homo_in)
{  
  MatrixXf pts_out(pts_homo_in.rows()-1,pts_homo_in.cols());
  for (int i=0;i<pts_homo_in.rows()-1;i++)
      pts_out.row(i) = pts_homo_in.row(i).array()/pts_homo_in.bottomRows(1).array();   //replicate needs actual number, cannot be M or N
  
  return pts_out;
}

VectorXf homo_to_real_coord_vec(const VectorXf &pt_homo_in)
{
  VectorXf pt_out;
  if (pt_homo_in.rows()==4)
    pt_out=pt_homo_in.head(3)/pt_homo_in(3);
  else if (pt_homo_in.rows()==3)
    pt_out=pt_homo_in.head(2)/pt_homo_in(2);

  return pt_out;
}

void delete_columns(Matrix2Xf &pts_in, int delete_col_ind)
{
    assert (delete_col_ind < pts_in.cols());
    if (delete_col_ind >= pts_in.cols())
    {
      cout<<"Error in delete_columns, out of range"<<endl;
      return;
    }
    pts_in.block(0,delete_col_ind,pts_in.rows(),pts_in.cols()-delete_col_ind-1) = pts_in.block(0,delete_col_ind+1,pts_in.rows(),pts_in.cols()-delete_col_ind-1);
    pts_in.conservativeResize(Eigen::NoChange,pts_in.cols()-1);
}



// works for few points, if too much, use map or set. or shuffle [0, range]
std::vector<int> rand_non_duplicate(int want_num, int sample_range)
{
//     srand (time(NULL));  // will generate same if use here.
    std::vector<int> selected_inds;
    while (selected_inds.size()<want_num)
    {
	int random_ind = rand() % sample_range;
	int found_duplicate = false;
	if (selected_inds.size()>0)
	  for (int k=0;k<selected_inds.size();k++)
	    if (random_ind == selected_inds[k])  // should not duplicated  
	    {
		found_duplicate = true;
		break;
	    }
	if (!found_duplicate)
	    selected_inds.push_back(random_ind);
    }
    return selected_inds;
}


// ground in the label map is 1 or 255 (not 0)
void blend_segmentation(Mat& raw_color_img, Mat& label_map, Mat& blend_img)
{
  blend_img=raw_color_img.clone();  
  cv::Vec3b bgr_value;
  
  vector<Point2i> ground_pts;
  findNonZero(label_map, ground_pts);    
  for (int ind=0;ind<ground_pts.size();ind++)
  {
      bgr_value=blend_img.at<cv::Vec3b>(ground_pts[ind].y,ground_pts[ind].x);  //at row (y) and col(x)      
      bgr_value=bgr_value*0.8+cv::Vec3b(0,255,0)*0.2;  // uint to float???
      for (int i=0;i<3;i++)
      {
	if (bgr_value(i)>255)
	  bgr_value(i)=255;
      }	
      blend_img.at<cv::Vec3b>(ground_pts[ind].y,ground_pts[ind].x)=bgr_value;
  }
}


bool check_element_in_vector(const float& element, const VectorXf &vec_check)
{
  for (int i=0;i<vec_check.rows();i++)  
    if (element==vec_check(i))    
	return true;
  return false;
}

bool check_element_in_vector(const int& element, const VectorXi &vec_check)
{
  for (int i=0;i<vec_check.rows();i++)
    if (element==vec_check(i))
	return true;
  return false;
}


//change angle from [-180,180] to [-90,90]
float normalize_to_pi(float angle)
{
    if (angle > 90)
        return angle-180; // # change to -90 ~90
    else if (angle<-90)
        return angle+180;
    else
        return angle;
}


cv::Point2f lineSegmentIntersect_f2(const cv::Point2f& pt1_start, const cv::Point2f& pt1_end, const cv::Point2f& pt2_start, const cv::Point2f& pt2_end, 
			      float& extcond_1, float& extcond_2, bool infinite_line)
{
    // treat as [x1 y1 x2 y2]    [x3 y3 x4 y4]
      float X2_X1 = pt1_end.x-pt1_start.x;
      float Y2_Y1 = pt1_end.y-pt1_start.y;
      float X4_X3 = pt2_end.x-pt2_start.x;
      float Y4_Y3 = pt2_end.y-pt2_start.y;
      float X1_X3 = pt1_start.x-pt2_start.x;
      float Y1_Y3 = pt1_start.y-pt2_start.y;
      float u_a = (X4_X3*Y1_Y3-Y4_Y3*X1_X3)/ (Y4_Y3*X2_X1-X4_X3*Y2_Y1);
      float u_b = (X2_X1*Y1_Y3-Y2_Y1*X1_X3)/ (Y4_Y3*X2_X1-X4_X3*Y2_Y1);      
      float INT_X = pt1_start.x+X2_X1*u_a;
      float INT_Y = pt1_start.y+Y2_Y1*u_a;
      float INT_B = float((u_a >= 0) && (u_a <= 1) && (u_b >= 0) && (u_b <= 1));
      if (infinite_line)
	  INT_B=1;
      
      extcond_1 = u_a; extcond_2 = u_b;
      return cv::Point2f(INT_X*INT_B, INT_Y*INT_B);

}

void polygon_overlap_func(const vector<Point2f>& closed_polys1,const vector<Point2f>& closed_polys2, vector<Point2f>& open_polysout)
{    
    vector<Point2f> all_intersect_pts;
    //intersect every edge of image plane, and projected polygons edges
    for (size_t edge_id1=0;edge_id1<closed_polys1.size()-1;edge_id1++)
   	for (size_t edge_id2=0;edge_id2<closed_polys2.size()-1;edge_id2++)
	{
	    float extcond_1, extcond_2; // remeber the whether start or end is extended to intersect. refer to matlab extend_occlusion_lines_v2.m
	    Point2f sectpt = lineSegmentIntersect_f2(closed_polys1[edge_id1],closed_polys1[edge_id1+1],
			      closed_polys2[edge_id2],closed_polys2[edge_id2+1],extcond_1,extcond_2,false);
	    if ((extcond_1 >= 0) && (extcond_1 <= 1) && (extcond_2 >= 0) && (extcond_2 <= 1))
	    {
		all_intersect_pts.push_back(sectpt);
// 		cout<<"Find new intersection:   "<<sectpt.x<<"  "<<sectpt.y<<endl;
	    }
	}
    
    vector<Point2f> all_corners;
    all_corners.insert(all_corners.end(),closed_polys1.begin(),closed_polys1.end()-1);  // don't use last duplicated point
    all_corners.insert(all_corners.end(),closed_polys2.begin(),closed_polys2.end()-1);
    
    // find corners which lie in both polygons    
    vector<Point2f> inside_corners;
    for (size_t ii=0;ii<all_corners.size();ii++)
      if (cv::pointPolygonTest(closed_polys1,all_corners[ii],false) >= 0)
	if (cv::pointPolygonTest(closed_polys2,all_corners[ii],false) >= 0)
	  inside_corners.push_back(all_corners[ii]);
    
//     cout<<"marker 4   all inside corners  "<<inside_corners.size()<<endl;
    
    // intersection definitely lie inside both   don't perform Pointpolytest, it might fail due to noise or floating-integer.
    inside_corners.insert(inside_corners.end(),all_intersect_pts.begin(),all_intersect_pts.end());
    
    open_polysout.clear();
    // find convex hull for these corners
    if (inside_corners.size()==0)
	return;
    else
	convexHull(inside_corners,open_polysout);
}

void polygon_overlap_func(const vector<Point2f>& closed_polys1,const vector<Point2f>& closed_polys2, const vector<Point2f>& closed_polys3,
			  vector<Point2f>& open_polysout)
{
    vector<Point2f> final_polygon_temp;
    polygon_overlap_func(closed_polys1,closed_polys2,final_polygon_temp);
    polygon_overlap_func(final_polygon_temp,closed_polys3,open_polysout);  
}

void polygon_overlap_func(const Eigen::Matrix2Xf& closed_polys1,const Eigen::Matrix2Xf& closed_polys2, Eigen::Matrix2Xf& open_polysout)
{
    vector<Point2f> closed_polys1_pts;matrix_to_points(closed_polys1.transpose(),closed_polys1_pts);
    vector<Point2f> closed_polys2_pts;matrix_to_points(closed_polys2.transpose(),closed_polys2_pts);
    vector<Point2f> closed_polysout_pts;
    polygon_overlap_func(closed_polys1_pts,closed_polys2_pts,closed_polysout_pts);
    points_to_matrix(closed_polysout_pts,open_polysout);
}

void polygon_overlap_func(const Eigen::Matrix2Xf& closed_polys1,const Eigen::Matrix2Xf& closed_polys2, 
			  const Eigen::Matrix2Xf& closed_polys3, Eigen::Matrix2Xf& open_polysout)
{
    Eigen::Matrix2Xf final_polygon_temp;
    polygon_overlap_func(closed_polys1,closed_polys2,final_polygon_temp);
    polygon_overlap_func(final_polygon_temp,closed_polys3,open_polysout);
}


vector<int> find_correct_ground_seq(const Matrix2Xf& open_2d_cutted_polys, bool originkept)
{
    int raw_first_ind = -1; int raw_second_ind = -1;
    for (int pt_id=0;pt_id<open_2d_cutted_polys.cols();pt_id++)
      if (fabs(open_2d_cutted_polys(1,pt_id))<1e-3) // y=0
      {
	if (originkept)
	{
	    if (fabs(open_2d_cutted_polys(0,pt_id))<1e-3) // x=0
		raw_first_ind = pt_id;
	    else
		raw_second_ind = pt_id; //x!=0
	}
	else
	{
	    if (raw_first_ind==-1) //first touched is ground. doesn't matter (counter)clock-wise
	      raw_first_ind = pt_id;
	    else
	      raw_second_ind = pt_id;
	}
      }
      
//     cout<<"open_2d_cutted_polys\n"<<open_2d_cutted_polys<<endl;
//     cout<<raw_first_ind<<"  "<<raw_second_ind<<endl;
    vector<int> correct_sequence_ind; // including last duplicated point
    if ((abs(raw_first_ind-raw_second_ind)==1)||(abs(raw_first_ind-raw_second_ind)==open_2d_cutted_polys.cols()-1))
    {
	int increment = raw_second_ind-raw_first_ind;
	if ((raw_first_ind==0)&&(raw_second_ind==open_2d_cutted_polys.cols()-1))
	  increment = -1;
	if ((raw_first_ind==open_2d_cutted_polys.cols()-1)&&(raw_second_ind==0))
	  increment = 1;
	for (int jj=0;jj<open_2d_cutted_polys.cols()+1;jj++)
	{
	    int curr = raw_first_ind + jj*increment;
	    if (curr>open_2d_cutted_polys.cols()-1)
	      curr = curr-open_2d_cutted_polys.cols();
	    if (curr<0)
	      curr = curr+open_2d_cutted_polys.cols();
	    correct_sequence_ind.push_back(curr);
	}
    }
    return correct_sequence_ind;
}


Eigen::Matrix3Xf cut_wall_polygons(const Eigen::Matrix3Xf& plane_poly_3d_world, float ceiling_threshold, float walllength_threshold, const Vector3f& camera_center)
{
    // project 3d polygons onto the plane. change to 2d problem.
    Matrix2Xf plane_poly_2d_projected(2,plane_poly_3d_world.cols());
    Vector3f plane_ground_line_dir = (plane_poly_3d_world.col(1)-plane_poly_3d_world.col(0)).normalized(); // plane2d x coor
    float wall_length = (plane_poly_3d_world.col(1)-plane_poly_3d_world.col(0)).norm();
    Vector3f plane_vertical_dir(0,0,1); // plane2d的y
    for (int pt_id=0;pt_id<plane_poly_3d_world.cols();pt_id++)
    {
	plane_poly_2d_projected(0,pt_id) = (plane_poly_3d_world.col(pt_id)-plane_poly_3d_world.col(0)).dot(plane_ground_line_dir);    //x coor 
	plane_poly_2d_projected(1,pt_id) =  plane_poly_3d_world(2,pt_id);   // y coor
    }
    Matrix2Xf open_2d_cutted_polys; // in the plane coordinate   NOTE no duplicat point.  the previous 'close_poly' has duplicat point
    
    Matrix2Xf cut_required_bound_polys(2,5);
    bool only_control_height = (walllength_threshold<0);    
    if (only_control_height)
    {
	// only cut ceiling height. same algorithm as before
	cut_required_bound_polys<<-1,wall_length+1,wall_length+1,-1,-1,
			    -1,-1,ceiling_threshold,ceiling_threshold,-1;
    }else
    {
	// also cut wall length
	float ground_start_to_camera = (camera_center - plane_poly_3d_world.col(0)).norm();
	float ground_end_to_camera = (camera_center - plane_poly_3d_world.col(1)).norm();
	// control both height and length   choose closer point as start.
	if (ground_start_to_camera<ground_end_to_camera)
	    cut_required_bound_polys<< -1,walllength_threshold,walllength_threshold,-1,-1,
					-1,-1,ceiling_threshold,ceiling_threshold,-1;
	else
	    cut_required_bound_polys<< plane_poly_2d_projected(0,1)-walllength_threshold,plane_poly_2d_projected(0,1)+1,plane_poly_2d_projected(0,1)+1,plane_poly_2d_projected(0,1)-walllength_threshold,plane_poly_2d_projected(0,1)-walllength_threshold,
					-1,-1,ceiling_threshold,ceiling_threshold,-1;
    }
    polygon_overlap_func(plane_poly_2d_projected,cut_required_bound_polys, open_2d_cutted_polys);


    // change back to 3d world
    Matrix3Xf cutted_plane_poly_3d_world(3,open_2d_cutted_polys.cols());  //no duplicate point
    for (int pt_id=0;pt_id<open_2d_cutted_polys.cols();pt_id++)
	cutted_plane_poly_3d_world.col(pt_id) = plane_poly_3d_world.col(0)+open_2d_cutted_polys(0,pt_id)*plane_ground_line_dir+
						open_2d_cutted_polys(1,pt_id)*plane_vertical_dir;
// 	cout<<"---------------------------"<<endl;
// 	change order, so that ground two points still at the begining
    if (1)
    {

	vector<int> correct_sequence_ind = find_correct_ground_seq(open_2d_cutted_polys, only_control_height);
	if (correct_sequence_ind.size()>0)
	{
	    Matrix3Xf cutted_correct_plane_poly_3d_world(3,correct_sequence_ind.size());
	    for (int pt_id=0;pt_id<correct_sequence_ind.size();pt_id++)
		cutted_correct_plane_poly_3d_world.col(pt_id) = cutted_plane_poly_3d_world.col(correct_sequence_ind[pt_id]);

// 		cout<<"curr polys:\n"<<cutted_correct_plane_poly_3d_world<<endl;
	    return cutted_correct_plane_poly_3d_world;
	}
	else
	{
	    cout<<"BAD cut_wall_polygons1, cannot find correct seq!!!!!!!!!!!!!!!!!!!!!!!!!!! \n "<<open_2d_cutted_polys<<endl;
	    return plane_poly_3d_world;
// 	    return Eigen::Matrix3Xf(3,0);
	}
	
    }
    else
    {
	cutted_plane_poly_3d_world.conservativeResize(NoChange,cutted_plane_poly_3d_world.cols()+1);
	cutted_plane_poly_3d_world.col(cutted_plane_poly_3d_world.cols()-1) = cutted_plane_poly_3d_world.col(0); // make it closed
	return cutted_plane_poly_3d_world;
    }
}


Eigen::Matrix3Xf cut_wall_polygons(const Eigen::Matrix3Xf& plane_poly_3d_world, const Vector3f& cut_range1, const Vector3f& cut_range2, bool range1_active,
				   bool range2_active)
{  
    // project 3d polygons onto the plane. change to 2d problem.
    Matrix2Xf plane_poly_2d_projected(2,plane_poly_3d_world.cols());
    Vector3f plane_ground_line_dir = (plane_poly_3d_world.col(1)-plane_poly_3d_world.col(0)).normalized(); // plane2d x coor
    float wall_length = (plane_poly_3d_world.col(1)-plane_poly_3d_world.col(0)).norm();
    Vector3f plane_vertical_dir(0,0,1); // plane2d y coordinate
    for (int pt_id=0;pt_id<plane_poly_3d_world.cols();pt_id++)
    {
	plane_poly_2d_projected(0,pt_id) = (plane_poly_3d_world.col(pt_id)-plane_poly_3d_world.col(0)).dot(plane_ground_line_dir);    //x coor 
	plane_poly_2d_projected(1,pt_id) =  plane_poly_3d_world(2,pt_id);   // y coor
    }
    float range_2d_1 = (cut_range1-plane_poly_3d_world.col(0)).dot(plane_ground_line_dir);
    float range_2d_2 = (cut_range2-plane_poly_3d_world.col(0)).dot(plane_ground_line_dir);
    float range_start,range_end;
    bool start_active,end_active;
    if (range_2d_1<range_2d_2)
    {
	range_start = range_2d_1; range_end = range_2d_2;
	start_active = range1_active; end_active = range2_active;
    }
    else
    {
	range_start = range_2d_2; range_end = range_2d_1;
	start_active = range2_active; end_active = range1_active;
    }
        
    float raw_poly_start = plane_poly_2d_projected.row(0).head<2>().minCoeff(); // should be 0
    float raw_poly_end = plane_poly_2d_projected.row(0).head<2>().maxCoeff();
    
    // if range is active, use it, otherwise don't use it
    if (!start_active)
      range_start = raw_poly_start-1; //1 is just for buffer
    if (!end_active)
      range_end = raw_poly_end+1;
    
        
    float current_max_height = plane_poly_3d_world.row(2).maxCoeff()+1; // 1 for buffer
    
    Matrix2Xf open_2d_cutted_polys; // in the plane coordinate   NOTE no duplicat point.  the previous 'close_poly' has duplicat point

    Matrix2Xf cut_required_bound_polys(2,5);
    cut_required_bound_polys<< range_start,range_end,range_end,range_start,range_start,
				-1,-1,current_max_height,current_max_height,-1;
    
    polygon_overlap_func(plane_poly_2d_projected,cut_required_bound_polys, open_2d_cutted_polys);


    if (open_2d_cutted_polys.cols()>0)
    {
	// change back to 3d world
	Matrix3Xf cutted_plane_poly_3d_world(3,open_2d_cutted_polys.cols());  //no duplicate point
	for (int pt_id=0;pt_id<open_2d_cutted_polys.cols();pt_id++)
	    cutted_plane_poly_3d_world.col(pt_id) = plane_poly_3d_world.col(0)+open_2d_cutted_polys(0,pt_id)*plane_ground_line_dir+
						    open_2d_cutted_polys(1,pt_id)*plane_vertical_dir;  
      
	vector<int> correct_sequence_ind = find_correct_ground_seq(open_2d_cutted_polys, false);
	if (correct_sequence_ind.size()>0)
	{
	    Matrix3Xf cutted_correct_plane_poly_3d_world(3,correct_sequence_ind.size());
	    for (int pt_id=0;pt_id<correct_sequence_ind.size();pt_id++)
		cutted_correct_plane_poly_3d_world.col(pt_id) = cutted_plane_poly_3d_world.col(correct_sequence_ind[pt_id]);

    // 		cout<<"curr polys:\n"<<cutted_correct_plane_poly_3d_world<<endl;
	    return cutted_correct_plane_poly_3d_world;
	}
	else
	{
	    cout<<"BAD cut_wall_polygons2, cannot find correct seq!!!!!!!!!!!!!!!!!!!!!!!!!!! \n "<<open_2d_cutted_polys<<endl;
// 	    cout<<"raw plane proj 2d \n "<<plane_poly_2d_projected<<endl;
// 	    cout<<"raw plane_poly_3d_world  \n"<<plane_poly_3d_world<<endl;
// 	    cout<<"cut range  "<<range_start<<"  "<<range_end<<endl;
    // 		return plane_poly_3d_world;
	    return Eigen::Matrix3Xf(3,0);
	}
    }
    else
    {
	cout<<"Didn't find closed 2d polygon, range may be inappropriate   "<<cut_range1.transpose()<<"    "<<cut_range2.transpose()<<endl;
// 	cout<<"raw plane_poly_3d_world  \n"<<plane_poly_3d_world<<endl;
// 	cout<<"cut range  "<<range_start<<"  "<<range_end<<endl;
	return Eigen::Matrix3Xf(3,0);
    }
						
}