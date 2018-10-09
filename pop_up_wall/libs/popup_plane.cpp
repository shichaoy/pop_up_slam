/*
 * Copyright Shichao Yang,2016, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu
 *
 */

#include "pop_up_wall/popup_plane.h"
#include "pop_up_wall/matrix_utils.h"

using namespace std;
using namespace cv;
using namespace Eigen;

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXf_row;

popup_plane::popup_plane():nn( "~" )
{ 
    width=640;
    height=480;
  //   Kalib << 570, 0, 320,  // kinect
  // 	   0, 570, 240, 
  // 	   0,   0,   1;
    Kalib << 371.975264, 0, 315.632343,   //ueye
	    0, 372.163582, 250.592551, 
	    0,   0,   1;
    invK=Kalib.inverse();
    
    
    all_planes_world.resize(1,4);
    all_planes_sensor.resize(1,4);
      
    erosion_distance=11;
    dilation_distance=11;

    pcl_cloud_world.reset(new pclRGB);
    
    inited_python=false;

    line_lbd_ptr = new line_lbd_detect();
    line_lbd_ptr->use_LSD=true; //lsd or edline
    
    simple_polygon_mode = false;
    
    cloud_projected_plane = Vector4f(0,0,0,0);
    
  // receive the paramters for python pop up  
    nn.param ("/pre_vertical_thre", pre_vertical_thre, pre_vertical_thre);
    nn.param ("/pre_minium_len", pre_minium_len, pre_minium_len);
    nn.param ("/pre_contour_close_thre", pre_contour_close_thre, pre_contour_close_thre);
    nn.param ("/interval_overlap_thre", interval_overlap_thre, interval_overlap_thre);
    nn.param ("/post_short_thre", post_short_thre, post_short_thre);
    nn.param ("/post_bind_dist_thre", post_bind_dist_thre, post_bind_dist_thre);
    nn.param ("/post_merge_dist_thre", post_merge_dist_thre, post_merge_dist_thre);
    nn.param ("/post_merge_angle_thre", post_merge_angle_thre, post_merge_angle_thre);
    nn.param ("/post_extend_thre", post_extend_thre, post_extend_thre);
        
    nn.param ("/downsample_contour", downsample_contour, downsample_contour);
    nn.param ("/use_fast_edges_detection", use_fast_edges_detection, use_fast_edges_detection);
    nn.param ("/cloud_downsample_rate", cloud_downsample_rate, cloud_downsample_rate);
    nn.param ("/plane_cam_dist_thre", plane_cam_dist_thre, plane_cam_dist_thre);  
}

popup_plane::~popup_plane()
{
    if (inited_python)
    {
      delete py_module;
      Py_Finalize();
    }
}

void popup_plane::set_calibration(Matrix3f calib)
{
    Kalib=calib;
    invK=Kalib.inverse();  
}
  



void popup_plane::closed_polygons_homo_pts(const MatrixX2f &closed_2d_polygon,Matrix3Xf &inside_pts_homo_out, bool downsample_poly, bool show_debug_info)
{
    if (closed_2d_polygon.cols()<1)
	return;
    MatrixXf new_polys_close = closed_2d_polygon;
    if (downsample_poly)
	new_polys_close=new_polys_close/2;
    
    vector<Point> polygon;
    matrix_to_points(new_polys_close,polygon);  
    Rect bounding_box=boundingRect(polygon);  // only need to consider small image area to save time.
    MatrixXf polys_close_shift=new_polys_close;  
    polys_close_shift.rowwise() -= Vector2f(bounding_box.x,bounding_box.y).transpose();
    cv::Mat raw_img_cp8(Size(bounding_box.width, bounding_box.height),CV_8UC1,Scalar(0));  
    vector<Point> polygon_shift;
    matrix_to_points(polys_close_shift,polygon_shift);
    fillConvexPoly(raw_img_cp8,&polygon_shift[0], new_polys_close.rows(), Scalar(255));    //faster than point in polygon test.
    vector<Point2i> locations_shift;   // output, locations of non-zero pixels 
    findNonZero(raw_img_cp8, locations_shift);  

    inside_pts_homo_out.resize(3,locations_shift.size());    
    for (int ind=0;ind<locations_shift.size();ind++)
    {
	if (downsample_poly){
	    inside_pts_homo_out(0,ind)=(locations_shift[ind].x+bounding_box.x)*2;
	    inside_pts_homo_out(1,ind)=(locations_shift[ind].y+bounding_box.y)*2;
	    inside_pts_homo_out(2,ind)=1;
	}
	else{
	    inside_pts_homo_out(0,ind)=locations_shift[ind].x+bounding_box.x;
	    inside_pts_homo_out(1,ind)=locations_shift[ind].y+bounding_box.y;
	    inside_pts_homo_out(2,ind)=1;  
	}
    }
  //   inside_pts_homo_out.colwise() += Vector3f(bounding_box.x,bounding_box.y,0); 
}


// for old ground model: edge are connected. area below edge is a closed ground polygon
void popup_plane::find_2d_3d_closed_polygon(const Matrix4f& transToWolrd, const MatrixX4f& ground_seg2d_lines)
{
   Matrix4f invT = transToWolrd.inverse();
   ground_seg2d_lines_connect = ground_seg2d_lines;
  //   Vector<Vector4f> direc(ground_seg2d_lines_connect.rows());    //draw a vector in world, and project to camera
    MatrixXf up_direc(ground_seg2d_lines_connect.rows(),4);
    MatrixXf down_direc(ground_seg2d_lines_connect.rows(),4);
    for (int line_ind=0;line_ind<ground_seg2d_lines_connect.rows();line_ind++)  
    {
	for (int pt_ind=0;pt_ind<2;pt_ind++)
	{
	    MatrixXf vertical_3d_world(3,2); // each column is point
	    Vector3f temp_vertex=ground_seg3d_lines_world.row(line_ind).segment<3>(pt_ind*3);
	    vertical_3d_world.col(0)=temp_vertex;     //.row(line_ind).segment<3>(pt_ind*3);
	    vertical_3d_world.col(1)=temp_vertex+Vector3f(0,0,2);
	    MatrixXf vertical_3d_sensor=homo_to_real_coord(invT*real_to_homo_coord(vertical_3d_world));
	    MatrixXf vertical_2d_image=homo_to_real_coord(Kalib*vertical_3d_sensor);
	    Vector2f temp_dir=vertical_2d_image.col(1)-vertical_2d_image.col(0);
	    if (temp_dir(1)>0)
	      temp_dir=-temp_dir;
	    up_direc.row(line_ind).segment<2>(pt_ind*2)=temp_dir;
	    down_direc.row(line_ind).segment<2>(pt_ind*2)=-temp_dir;
	}
    }
    
	// record: for each ground edge start/end pt, upward/downward to hit boundary.  downward for ground regions.
    MatrixXf boudaryies_hits=MatrixXf::Zero(ground_seg2d_lines_connect.rows(),ground_seg2d_lines_connect.cols());  
    MatrixXf boudaryies_hits_down=MatrixXf::Zero(ground_seg2d_lines_connect.rows(),ground_seg2d_lines_connect.cols());
    for (int line_ind=0;line_ind<ground_seg2d_lines_connect.rows();line_ind++)    
	for (int pt_ind=0;pt_ind<2;pt_ind++)      
	{
	    boudaryies_hits.row(line_ind).segment<2>(pt_ind*2)=direction_hit_boundary(ground_seg2d_lines_connect.row(line_ind).segment<2>(pt_ind*2),
								  up_direc.row(line_ind).segment<2>(pt_ind*2),width,height);
	    boudaryies_hits_down.row(line_ind).segment<2>(pt_ind*2)=direction_hit_boundary(ground_seg2d_lines_connect.row(line_ind).segment<2>(pt_ind*2),
								      down_direc.row(line_ind).segment<2>(pt_ind*2),width,height);
	}

	
    // find ground closed polygons...  this holds if there is no object....
    int num_wall_seg=ground_seg2d_lines_connect.rows();
    vector<MatrixX2f> all_2d_closed_polygons_out(num_wall_seg+1);  // each element is a polygon n*2. each row of matrixxd is a vertex
    MatrixXf ground_seg2d_closed=ground_seg2d_lines_connect.leftCols(2);
    vert_stack_v_self(ground_seg2d_closed,ground_seg2d_lines_connect.row(num_wall_seg-1).tail(2));  
    
    int seg=num_wall_seg-1;  //considering the last segment
    Vector2f begin_hit_bound=boudaryies_hits_down.row(seg).head(2);
    Vector2f end_hit_bound=boudaryies_hits_down.row(seg).tail(2);
    if (ground_seg2d_lines_connect(seg,3)==height-1 && ground_seg2d_lines_connect(seg,2)<width-1)  {}  
    else if (ground_seg2d_lines_connect(seg,2)==width-1 && ground_seg2d_lines_connect(seg,3)<=height-1)  
	  vert_stack_v_self(ground_seg2d_closed,Vector2f(width-1,height-1));  
    else{
	  if (end_hit_bound(0)==width-1){
		vert_stack_v_self(ground_seg2d_closed,end_hit_bound);
		vert_stack_v_self(ground_seg2d_closed,Vector2f(width-1,height-1));
	  }
	  if (0<=end_hit_bound(0) & end_hit_bound(0)<width-1)
	      vert_stack_v_self(ground_seg2d_closed,end_hit_bound);
    }
      
    seg=0;  //considering the first segment wall
    begin_hit_bound=boudaryies_hits_down.row(seg).head(2);
    end_hit_bound=boudaryies_hits_down.row(seg).tail(2);
    if (ground_seg2d_lines_connect(seg,1)==height-1 && ground_seg2d_lines_connect(seg,0)>0)  {}  
    else if (ground_seg2d_lines_connect(seg,0)==0 && ground_seg2d_lines_connect(seg,1)<=height-1)  
	  vert_stack_v_self(ground_seg2d_closed,Vector2f(0,height-1));  
    else{
	if (begin_hit_bound(0)==width-1){
	    vert_stack_v_self(ground_seg2d_closed,Vector2f(0,height-1));  
	    vert_stack_v_self(ground_seg2d_closed,end_hit_bound);	 
	}
	if (0<=begin_hit_bound(0) & begin_hit_bound(0)<width-1)
	    vert_stack_v_self(ground_seg2d_closed,end_hit_bound);
    }

    vert_stack_v_self(ground_seg2d_closed,ground_seg2d_closed.row(0));
    all_2d_closed_polygons_out[0]=ground_seg2d_closed;  
        
    
    // find wall closed polygons
    for(seg=0;seg<num_wall_seg;seg++)
    {
      MatrixXf_row line_vertexs= ground_seg2d_lines_connect.row(seg);
      line_vertexs.resize(2,2);
      MatrixXf part_wall_close_polys=line_vertexs;
      
      begin_hit_bound=boudaryies_hits.row(seg).head(2);
      end_hit_bound=boudaryies_hits.row(seg).tail(2);
      if (seg==num_wall_seg-1)   // last segment  rightmost wall
      {
	  if (ground_seg2d_lines_connect(seg,3)==height-1 && ground_seg2d_lines_connect(seg,2)<width-1){
	      vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, height-1)); 
	      
	      if (begin_hit_bound(0)==width-1)
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      if (0<begin_hit_bound(0) && begin_hit_bound(0)<width-1) {
		  vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (begin_hit_bound(0)==0) {
		  vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
		  vert_stack_v_self(part_wall_close_polys,Vector2f(0, 0));
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	  }
	  else if (ground_seg2d_lines_connect(seg,2)==width-1 && ground_seg2d_lines_connect(seg,3)<=height-1){            
	      if (begin_hit_bound(0)==width-1)
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      if (0<begin_hit_bound(0) && begin_hit_bound(0)<width-1) {
		  vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (begin_hit_bound(0)==0) {
		  vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
		  vert_stack_v_self(part_wall_close_polys,Vector2f(0, 0));
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	  }
	  else{
	      if (begin_hit_bound(0)==width-1 && end_hit_bound(0)==width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (0<begin_hit_bound(0) && begin_hit_bound(0)<width-1 && end_hit_bound[0]==width-1){
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (0<begin_hit_bound(0) && begin_hit_bound(0)<width-1 && 0<end_hit_bound(0) && end_hit_bound(0)<width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (begin_hit_bound(0)==0 && 0<end_hit_bound(0) && end_hit_bound(0)<width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,Vector2f(0,0));
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (begin_hit_bound(0)==0  && end_hit_bound(0)==0) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	  }
      }
      else if (0<seg && seg<num_wall_seg-1)  // middle segments
      {	    
	      if (begin_hit_bound(0)==width-1 && end_hit_bound(0)==width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (0<begin_hit_bound(0) && begin_hit_bound(0)<width-1 && end_hit_bound[0]==width-1){
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (0<begin_hit_bound(0) && begin_hit_bound(0)<width-1 && 0<end_hit_bound(0) && end_hit_bound(0)<width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (begin_hit_bound(0)==0 && 0<end_hit_bound(0) && end_hit_bound(0)<width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,Vector2f(0,0));
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (begin_hit_bound(0)==0  && end_hit_bound(0)==0) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
      }
      else  // leftmost wall
      {        
	  if (ground_seg2d_lines_connect(seg,1)==height-1 && ground_seg2d_lines_connect(seg,0)>0){
	      if (end_hit_bound(0)==width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
		  vert_stack_v_self(part_wall_close_polys,Vector2f(0, 0));	    
	      }
	      if (0<end_hit_bound(0) && end_hit_bound(0)<width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,Vector2f(0, 0));
	      }
	      if (end_hit_bound(0)==0)
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
	      
	      vert_stack_v_self(part_wall_close_polys,Vector2f(0, height-1));
	  }
	  else if (ground_seg2d_lines_connect(seg,0)==0 && ground_seg2d_lines_connect(seg,1)<=height-1){
	      if (end_hit_bound(0)==width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
		  vert_stack_v_self(part_wall_close_polys,Vector2f(0, 0));	    
	      }
	      if (0<end_hit_bound(0) && end_hit_bound(0)<width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,Vector2f(0, 0));
	      }
	      if (end_hit_bound(0)==0)
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
	  }
	  else{
	      if (begin_hit_bound(0)==width-1 && end_hit_bound(0)==width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (0<begin_hit_bound(0) && begin_hit_bound(0)<width-1 && end_hit_bound[0]==width-1){
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (0<begin_hit_bound(0) && begin_hit_bound(0)<width-1 && 0<end_hit_bound(0) && end_hit_bound(0)<width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (begin_hit_bound(0)==0 && 0<end_hit_bound(0) && end_hit_bound(0)<width-1) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,Vector2f(0,0));
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	      if (begin_hit_bound(0)==0  && end_hit_bound(0)==0) {
		  vert_stack_v_self(part_wall_close_polys,end_hit_bound);
		  vert_stack_v_self(part_wall_close_polys,begin_hit_bound);
	      }
	  }
      }
      vert_stack_v_self(part_wall_close_polys,ground_seg2d_lines_connect.row(seg).head(2).transpose());
      all_2d_closed_polygons_out[seg+1]=part_wall_close_polys;    
    }

    // compute the bounded 3D polygons (considering the ceiling.)
    all_closed_2d_bound_polygons.resize(all_2d_closed_polygons_out.size());
    all_3d_bound_polygons_world.resize(all_2d_closed_polygons_out.size());
    all_closed_2d_cutted_polygons.resize(all_2d_closed_polygons_out.size());
    for (int ii=0;ii<num_wall_seg+1;ii++)  
    {      
  //       cout<<"polygons "<<all_2d_closed_polygons_out[ii]<<endl;
	all_closed_2d_bound_polygons[ii]=all_2d_closed_polygons_out[ii].transpose();  // project closed 2d polygons onto the wall plane
	all_closed_2d_cutted_polygons[ii]=all_closed_2d_bound_polygons[ii]; 
	MatrixXf plane_seg_ray=invK*real_to_homo_coord(all_closed_2d_bound_polygons[ii]);
	MatrixXf plane_poly_3d_sensor;
	ray_plane_interact(plane_seg_ray, all_planes_sensor.row(ii),plane_poly_3d_sensor);
	
	if (ii>0){  // if wall
	  
	    Matrix3Xf plane_poly_3d_world = homo_to_real_coord(transToWolrd*real_to_homo_coord(plane_poly_3d_sensor));  //3*n
	    plane_poly_3d_world(2,0)=0;plane_poly_3d_world(2,1)=0;
	    
	    if (0)
	    {
		//cut a fixed height  should be fine...
		MatrixXf cut_plane_poly_3d_world(3,2); 
		cut_plane_poly_3d_world.col(0)=plane_poly_3d_world.col(0); //first two points is on the ground line
		cut_plane_poly_3d_world.col(1)=plane_poly_3d_world.col(1);
		for (int pt_ind=2;pt_ind<plane_poly_3d_world.cols();pt_ind++) //previous pt_ind=0 bug... already pushed first two ground points
		{
		    if (plane_poly_3d_world(2,pt_ind)<=ceiling_threshold)
			  hori_stack_v_self(cut_plane_poly_3d_world,plane_poly_3d_world.col(pt_ind));
		    else{  // need to find intersection of a line segments with threshold height
			  if (plane_poly_3d_world(2,pt_ind-1)<ceiling_threshold) {
			      Vector3f intersec;
			      if (points_intersect_height(plane_poly_3d_world.col(pt_ind-1),plane_poly_3d_world.col(pt_ind),ceiling_threshold,intersec))
				hori_stack_v_self(cut_plane_poly_3d_world,intersec);
			  }
			  if (plane_poly_3d_world(2,pt_ind+1)<ceiling_threshold) {
			      Vector3f intersec;
			      if (points_intersect_height(plane_poly_3d_world.col(pt_ind),plane_poly_3d_world.col(pt_ind+1),ceiling_threshold,intersec))
				hori_stack_v_self(cut_plane_poly_3d_world,intersec);
			  }
		    }
		}
		all_3d_bound_polygons_world[ii]=cut_plane_poly_3d_world; // if don't need cutting ceiling, just use  plane_poly_3d_world
	    }
	    
	    // new method to cut polygon    only depend on   plane_poly_3d_world  and  camera center
	    if (1)
	    {
		Matrix3Xf cutted_plane_poly_3d_world = cut_wall_polygons(plane_poly_3d_world, ceiling_threshold, walllength_threshold, transToWolrd.col(3).head<3>());
		all_3d_bound_polygons_world[ii] = cutted_plane_poly_3d_world;
		Matrix2Xf cutted_proj_pts = homo_to_real_coord(Kalib*homo_to_real_coord(invT*(real_to_homo_coord(all_3d_bound_polygons_world[ii]))));
		all_closed_2d_cutted_polygons[ii] = cutted_proj_pts;
	    }
	    
       }
       
       if (ii==0){  // if ground plane
	    //TODO could use good plane indice, to select edges otherwise might contain some bad points, front or back
       }
      
    }
}


void popup_plane::find_2d_3d_closed_polygon_simplemode(const Matrix4f& transToWolrd, const MatrixX4f& ground_seg2d_lines)
{
    Matrix4f invT = transToWolrd.inverse();
  //   Vector<Vector4f> direc(ground_seg2d_lines.rows());    //draw a vector in world, and project to camera
    
    MatrixXf up_direc(ground_seg2d_lines.rows(),4);  // for each vertex, store a direction
    for (int line_ind=0;line_ind<ground_seg2d_lines.rows();line_ind++)  
    {
	for (int pt_ind=0;pt_ind<2;pt_ind++)
	{
	    MatrixXf vertical_3d_world(3,2); // each column is point
	    Vector3f temp_vertex=ground_seg3d_lines_world.row(line_ind).segment<3>(pt_ind*3);
	    vertical_3d_world.col(0)=temp_vertex;     //.row(line_ind).segment<3>(pt_ind*3);
	    vertical_3d_world.col(1)=temp_vertex+Vector3f(0,0,2);
	    MatrixXf vertical_3d_sensor=homo_to_real_coord(invT*real_to_homo_coord(vertical_3d_world));
	    MatrixXf vertical_2d_image=homo_to_real_coord(Kalib*vertical_3d_sensor);
	    Vector2f temp_dir=vertical_2d_image.col(1)-vertical_2d_image.col(0);
	    if (temp_dir(1)>0)
	      temp_dir=-temp_dir;
	    up_direc.row(line_ind).segment<2>(pt_ind*2)=temp_dir;
	}
    }
    
    MatrixXf boudaryies_hits=MatrixXf::Zero(ground_seg2d_lines.rows(),4);
    for (int line_ind=0;line_ind<ground_seg2d_lines.rows();line_ind++)    
	for (int pt_ind=0;pt_ind<2;pt_ind++)      
	{
	    boudaryies_hits.row(line_ind).segment<2>(pt_ind*2)=direction_hit_boundary(ground_seg2d_lines.row(line_ind).segment<2>(pt_ind*2),
									up_direc.row(line_ind).segment<2>(pt_ind*2),width,height);
	}

  // find ground closed polygons...  this holds if there is no object....
    int num_wall_seg = ground_seg2d_lines.rows();  // no ground, just wall
    vector<MatrixX2f> all_2d_closed_polygons_out(num_wall_seg);  // each element is a polygon n*2. each row of matrixxd is a vertex  no ground!

  // find wall closed 2d polygons  NOTE assumes edge is already sorted. start from left. camera mostly upward, thus upward line from start is also on left.
	// simpler than previous method, as doesn't require ground polys. also don't consider left/right most edge intersection with image bottom.
    for(int seg=0;seg<num_wall_seg;seg++)
    {
	  MatrixXf_row line_vertexs= ground_seg2d_lines.row(seg);
	  line_vertexs.resize(2,2);
	  MatrixXf part_wall_close_polys=line_vertexs;
      
	  Vector2f begin_hit_bound=boudaryies_hits.row(seg).head(2);
	  Vector2f end_hit_bound=boudaryies_hits.row(seg).tail(2);

// 	  if ( (fabs(end_hit_bound(0)-line_vertexs(1,0))<1e-4) && fabs((end_hit_bound(1)-line_vertexs(1,1))<1e-4))  // hit point is not same as line endpoint (line not on boundary)
	  if ( (end_hit_bound(0)!=line_vertexs(1,0)) || (end_hit_bound(1)!=line_vertexs(1,1))) // always add end_hit_bound as long as different from end_pt
	      vert_stack_v_self(part_wall_close_polys,end_hit_bound);	  
	  if (0<begin_hit_bound(0) && begin_hit_bound(0)<width-1 && end_hit_bound[0]==width-1){  // choose to add top right corner
	      vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
	  }
	  if (begin_hit_bound(0)==0 && end_hit_bound[0]==width-1){
	      vert_stack_v_self(part_wall_close_polys,Vector2f(width-1, 0));
	      vert_stack_v_self(part_wall_close_polys,Vector2f(0,0)); 
	  }
	  if (begin_hit_bound(0)==0 && 0<end_hit_bound(0) && end_hit_bound(0)<width-1) {
	      vert_stack_v_self(part_wall_close_polys,Vector2f(0,0));
	  }
	  if ( (begin_hit_bound(0)!=line_vertexs(0,0)) || (begin_hit_bound(1)!=line_vertexs(0,1))) // always add start_hit_bound as long as different from start_pt
	      vert_stack_v_self(part_wall_close_polys,begin_hit_bound);

	  vert_stack_v_self(part_wall_close_polys,ground_seg2d_lines.row(seg).head(2).transpose());// for final close
	  all_2d_closed_polygons_out[seg]=part_wall_close_polys;
	  
	  if ((begin_hit_bound(0)==-1)||(end_hit_bound(0)==-1)) // if not found hit bound... should not happen
	    all_2d_closed_polygons_out[seg]=MatrixXf(0,0);
    }

    // compute the bounded polygons, actually just rectange in simple mode.
    all_closed_2d_bound_polygons.resize(num_wall_seg+1);
    all_closed_2d_bound_polygons[0] = MatrixXf(2,0);  //TODO void for ground. if there is object, it is not good to add a polygon....
    all_3d_bound_polygons_world.resize(num_wall_seg+1);
    all_3d_bound_polygons_world[0] = MatrixXf(2,0);
    all_closed_2d_cutted_polygons.resize(num_wall_seg+1);
    all_closed_2d_cutted_polygons[0] = MatrixXf(2,0);
//     cout<<"pop up polygon:   "<<-1<<"  "<<all_closed_2d_bound_polygons[0]<<all_closed_2d_bound_polygons[0].rows()<<"  "<<
// 	      all_closed_2d_bound_polygons[0].cols()<<endl;
    
    // NOTE just use a rectangle to represent the wall. don't need complicated polygon.
    for (int ii=0;ii<num_wall_seg;ii++)
    {
	all_closed_2d_bound_polygons[ii+1] = all_2d_closed_polygons_out[ii].transpose();
// 	cout<<"pop up polygon:   "<<ii<<"  "<<all_closed_2d_bound_polygons[ii+1]<<endl;
	all_3d_bound_polygons_world[ii+1] =  MatrixXf(3,5);
	all_3d_bound_polygons_world[ii+1].col(0) = ground_seg3d_lines_world.row(ii).head(3);
	all_3d_bound_polygons_world[ii+1].col(1) = ground_seg3d_lines_world.row(ii).tail(3);
	all_3d_bound_polygons_world[ii+1].col(2) = Vector3f(ground_seg3d_lines_world.row(ii).tail(3))+Vector3f(0,0,ceiling_threshold);
	all_3d_bound_polygons_world[ii+1].col(3) = Vector3f(ground_seg3d_lines_world.row(ii).head(3))+Vector3f(0,0,ceiling_threshold);
	all_3d_bound_polygons_world[ii+1].col(4) = ground_seg3d_lines_world.row(ii).head(3);
    }
    

    if (walllength_threshold>0)  // curr polygon based on ceiling and wall length, and project onto camera
    {
	for (int ii=0;ii<num_wall_seg;ii++)  
	{
	    // project closed 2d polygons onto the wall plane
	    
	    Matrix2Xf plane_2d_polys = all_2d_closed_polygons_out[ii].transpose();
	    if (plane_2d_polys.cols()==0)
	       continue;
	    // if edge adjacent to ground edge is too short, remove it. otherwise my cut_wall_polygons may not reorder properly...
	    if ((plane_2d_polys.col(2)-plane_2d_polys.col(1)).norm()<1) 
	       delete_columns(plane_2d_polys,2);
	    if ((plane_2d_polys.col(plane_2d_polys.cols()-2)-plane_2d_polys.col(plane_2d_polys.cols()-1)).norm()<1)
	       delete_columns(plane_2d_polys,plane_2d_polys.cols()-2);
	  
	    MatrixXf plane_seg_ray=invK*real_to_homo_coord(plane_2d_polys);
	    MatrixXf plane_poly_3d_sensor;
	    ray_plane_interact(plane_seg_ray, all_planes_sensor.row(ii+1),plane_poly_3d_sensor);
	    
	    Matrix3Xf plane_poly_3d_world = homo_to_real_coord(transToWolrd*real_to_homo_coord(plane_poly_3d_sensor));  //3*n
	    plane_poly_3d_world(2,0)=0;plane_poly_3d_world(2,1)=0;plane_poly_3d_world(2,plane_poly_3d_world.cols()-1)=0;

	    // new method to cut polygon    only depend on plane_poly_3d_world and camera center
	    Matrix3Xf cutted_plane_poly_3d_world = cut_wall_polygons(plane_poly_3d_world, ceiling_threshold, walllength_threshold, transToWolrd.col(3).head<3>());	    
	    Matrix2Xf cutted_proj_pts = homo_to_real_coord(Kalib*homo_to_real_coord(invT*(real_to_homo_coord(cutted_plane_poly_3d_world))));
	    for (int jj=0;jj<cutted_proj_pts.cols();jj++) // some close zero negative number, change to 0
	    {
	      if ( (cutted_proj_pts(0,jj)<0.5) && (cutted_proj_pts(0,jj)>-1) )
		cutted_proj_pts(0,jj) = 0;
	      if ( (cutted_proj_pts(1,jj)<0.5) && (cutted_proj_pts(1,jj)>-1) )
		cutted_proj_pts(1,jj) = 0;
	      if ( (cutted_proj_pts(0,jj)<width) && (cutted_proj_pts(0,jj)>width-1.5) )
		cutted_proj_pts(0,jj) = width-1;
	      if ( (cutted_proj_pts(1,jj)<height) && (cutted_proj_pts(1,jj)>height-1.5) )
		cutted_proj_pts(1,jj) = height-1;
	    }
	    
	    all_3d_bound_polygons_world[ii+1] = cutted_plane_poly_3d_world;
	    all_closed_2d_cutted_polygons[ii+1] = cutted_proj_pts;

    // 	cout<<"raw poly[ii]  \n"<<all_closed_2d_bound_polygons[ii+1]<<endl;
    // 	cout<<"cutted_polygons[ii]  \n"<<all_closed_2d_cutted_polygons[ii+1]<<endl;
	}
    }
    
}	



void popup_plane::get_plane_equation( const MatrixX4f& ground_seg2d_lines, const Matrix4f& transToWolrd, bool find_close_polygon)
{
    if (ground_seg2d_lines.rows()>0)
    {         
        Vector4f ground_plane_world(0,0,-1,0);  //plane parameters for ground in world(footprint), treated as column vector. normal is away from camera
	Vector4f ground_plane_sensor=transToWolrd.transpose()*ground_plane_world;
	MatrixXf ground_seg2d_homo(ground_seg2d_lines.rows()*2,3); // n*4
	MatrixXf_row temp=ground_seg2d_lines;  // row order for resize
	temp.resize(ground_seg2d_lines.rows()*2,2);  //2n*2	
	ground_seg2d_homo<<temp.array(),VectorXf::Ones(ground_seg2d_lines.rows()*2); // 2n*3
	MatrixXf ground_seg_ray=invK*ground_seg2d_homo.transpose();    //each column is a 3D world coordinate  3*n
	MatrixXf ground_seg3d_sensor;
	ray_plane_interact(ground_seg_ray,ground_plane_sensor,ground_seg3d_sensor);
	MatrixXf ground_seg3d_homo_sensor;
	real_to_homo_coord(ground_seg3d_sensor,ground_seg3d_homo_sensor);  //3*n
	temp=ground_seg3d_sensor.transpose();
	temp.resize(ground_seg3d_sensor.cols()/2,6); // n/2 * 6
	ground_seg3d_lines_sensor=temp;

	MatrixXf ground_seg3d_homo_world=transToWolrd*ground_seg3d_homo_sensor;//   # compute world ground polygons  
	homo_to_real_coord(ground_seg3d_homo_world,ground_seg3d_lines_world); // #3*n 
	
	temp=ground_seg3d_lines_world.transpose();
	temp.resize(ground_seg3d_lines_world.cols()/2,6); // n/2 * 6
	ground_seg3d_lines_world = temp;
	for (int ii=0;ii<ground_seg3d_lines_world.rows();ii++)
	{
	    ground_seg3d_lines_world(ii,2)=0;ground_seg3d_lines_world(ii,5)=0; // make it exact zero
	}

	//compute wall plane parameters in world frame  
	int num_seg=ground_seg2d_lines.rows();
	all_planes_world.resize(num_seg+1,4);   // store all the plane parameters in world frame
	all_planes_sensor.resize(num_seg+1,4);  // store all the plane parameters in sensor frame
	all_planes_world.row(0)=ground_plane_world;  // first plane is ground
	all_planes_sensor.row(0)=ground_plane_sensor;
	
	for (int seg=0;seg<num_seg;seg++)
	{
	    Vector3f partwall_line3d_world_bg=ground_seg3d_lines_world.row(seg).head(3);
	    Vector3f partwall_line3d_world_en=ground_seg3d_lines_world.row(seg).tail(3);
	    Vector3f temp1=partwall_line3d_world_en-partwall_line3d_world_bg;
	    Vector3f temp2=ground_plane_world.head(3);
	    Vector3f partwall_normal_world=temp1.cross(temp2);
	    float dist=-partwall_normal_world.transpose()*partwall_line3d_world_bg;
	    Vector4f partwall_plane_world;   partwall_plane_world<<partwall_normal_world,dist;
	    Vector4f partwall_plane_sensor=transToWolrd.transpose()*partwall_plane_world;// wall plane in sensor frame    
	    
	    all_planes_world.row(seg+1)=partwall_plane_world;
	    all_planes_sensor.row(seg+1)=partwall_plane_sensor;
	}
	ceiling_plane_world<<0,0,-1,ceiling_threshold;
	ceiling_plane_sensor=transToWolrd.transpose()*ceiling_plane_world;
	if (find_close_polygon)
	{
	    if (!simple_polygon_mode)
		find_2d_3d_closed_polygon(transToWolrd, ground_seg2d_lines);  //also find polygon
	    else
		find_2d_3d_closed_polygon_simplemode(transToWolrd, ground_seg2d_lines);  //also find polygon
	}

// 	cout<<"ground_seg3d_lines_sensor "<<ground_seg3d_lines_sensor<<endl;
	// find good, not far edges
	all_plane_dist_to_cam.resize(ground_seg3d_lines_world.rows()+1);
	all_plane_dist_to_cam(0) = transToWolrd(2,3);  // height for ground plane.
	Vector2f camera_pose=transToWolrd.col(3).head(2);
	vector<int> close_front_plane_inds(1);
	close_front_plane_inds[0]=0;  // always push ground plane
	for (int seg=0;seg<ground_seg3d_lines_world.rows();seg++)
	{	    
	    Vector2f begin_pt=ground_seg3d_lines_world.row(seg).segment<2>(0);
	    Vector2f end_pt=ground_seg3d_lines_world.row(seg).segment<2>(3);
	    all_plane_dist_to_cam(seg+1)=point_dist_lineseg(begin_pt,end_pt,camera_pose);
	    if ( (ground_seg3d_lines_sensor(seg,2)>0) && (ground_seg3d_lines_sensor(seg,5)>0))  // in front of the camera
// 	      if ( (ground_seg3d_lines_sensor(seg,2)<20) && (ground_seg3d_lines_sensor(seg,5)<20))  // not to far in z
// 		if  ((Vector2f(ground_seg3d_lines_sensor(seg,3),ground_seg3d_lines_sensor(seg,5))).norm()<25)
		if (all_plane_dist_to_cam(seg+1)<plane_cam_dist_thre)  // distance threshold
		{
		  if (actual_plane_indices_inall.rows()>0)  // whether want to exclude manually connect lines
		  {
		      if (check_element_in_vector(seg+1,actual_plane_indices_inall))  // if it is not manually connected edges
			close_front_plane_inds.push_back(seg+1);
		  }
		  else
		    close_front_plane_inds.push_back(seg+1);
		}
	}
	good_plane_number = close_front_plane_inds.size();
	good_plane_indices.resize(good_plane_number);
	for (int i=0;i<good_plane_number;i++)
	    good_plane_indices(i)=close_front_plane_inds[i];
	
	total_plane_number = ground_seg2d_lines.rows()+1; // will include ground
	ground_seg2d_lines_connect = ground_seg2d_lines;
    }
    else    
    {
	total_plane_number=0;
	good_plane_number=0;
	cout<<"pop_up_wall: cannot compute plane 3D polygons!!!"<<endl;    
    }
}

void popup_plane::update_plane_equation_from_seg(const MatrixX4f& ground_seg2d_lines, const Matrix3f& invK, Matrix4f& transToWorld, 
						 MatrixX4f& all_planes_sensor_out)
{
    if (ground_seg2d_lines.rows()>0)
    {
	Vector4f ground_plane_world(0,0,-1,0);  //plane parameters for ground in world(footprint), treated as column vector
	Vector4f ground_plane_sensor=transToWorld.transpose()*ground_plane_world;  // using a new pose to project
      
	MatrixXf ground_seg2d_homo(ground_seg2d_lines.rows()*2,3);
	MatrixXf_row temp=ground_seg2d_lines;  // row order for resize
	temp.resize(ground_seg2d_lines.rows()*2,2);  //2n*2
	ground_seg2d_homo<<temp.array(),VectorXf::Ones(ground_seg2d_lines.rows()*2); // 2n*3	    
	MatrixXf ground_seg_ray=invK*ground_seg2d_homo.transpose();    //each column is a 3D world coordinate  3*n
	MatrixXf ground_seg3d_sensor;
	ray_plane_interact(ground_seg_ray,ground_plane_sensor,ground_seg3d_sensor);
	MatrixXf ground_seg3d_homo_sensor;
	real_to_homo_coord(ground_seg3d_sensor,ground_seg3d_homo_sensor);  //4*n
	
	MatrixXf ground_seg3d_lines_world_cp;
	MatrixXf ground_seg3d_homo_world=transToWorld*ground_seg3d_homo_sensor;//   # compute world ground polygons    
	homo_to_real_coord(ground_seg3d_homo_world,ground_seg3d_lines_world_cp); // #3*n 
	temp=ground_seg3d_lines_world_cp.transpose();
	temp.resize(ground_seg3d_lines_world_cp.cols()/2,6); // n/2 * 6
	ground_seg3d_lines_world_cp=temp;
	
	int num_seg = ground_seg2d_lines.rows();
	all_planes_sensor_out.resize(num_seg+1,4);
	all_planes_sensor_out.row(0)=ground_plane_sensor;
	MatrixXf all_planes_world_cp(num_seg+1,4);  // store all the plane parameters in world frame  
	all_planes_world_cp.row(0)=ground_plane_world;	
	
	for (int seg=0;seg<num_seg;seg++)
	{
	    Vector3f partwall_line3d_world_bg=ground_seg3d_lines_world_cp.row(seg).head(3);
	    Vector3f partwall_line3d_world_en=ground_seg3d_lines_world_cp.row(seg).tail(3);
	    Vector3f temp1=partwall_line3d_world_en-partwall_line3d_world_bg;
	    Vector3f temp2=ground_plane_world.head(3);
	    Vector3f partwall_normal_world=temp1.cross(temp2);
	    float dist=-partwall_normal_world.transpose()*partwall_line3d_world_bg;
	    Vector4f partwall_plane_world;   partwall_plane_world<<partwall_normal_world,dist;
	    Vector4f partwall_plane_sensor=transToWorld.transpose()*partwall_plane_world;// wall plane in sensor frame    
	    
	    all_planes_world_cp.row(seg+1)=partwall_plane_world;
	    all_planes_sensor_out.row(seg+1)=partwall_plane_sensor;
	}
// 	cout<<"MatrixXf all_planes_world_cp   "<<all_planes_world_cp<<endl;
    }
    else
    {
	all_planes_sensor_out.resize(0,4);
    }
}


void popup_plane::update_plane_equation_from_seg_fast(const MatrixX4f& ground_seg2d_lines, const Matrix3f& invK, Matrix4f& transToWorld, 
						      MatrixX4f& all_planes_sensor_out)
{
  // don't compute world position. just locallly
    if (ground_seg2d_lines.rows()>0)
    {
	Vector4f ground_plane_world(0,0,-1,0);  //plane parameters for ground in world(footprint), treated as column vector
	Vector4f ground_plane_sensor=transToWorld.transpose()*ground_plane_world;  // using a new pose to project
      
	MatrixXf ground_seg2d_homo(ground_seg2d_lines.rows()*2,3);
	MatrixXf_row temp=ground_seg2d_lines;  // row order for resize
	temp.resize(ground_seg2d_lines.rows()*2,2);  //2n*2
	ground_seg2d_homo<<temp.array(),VectorXf::Ones(ground_seg2d_lines.rows()*2); // 2n*3	    
	MatrixXf ground_seg_ray=invK*ground_seg2d_homo.transpose();    //each column is a 3D world coordinate  3*2n
	MatrixXf ground_seg3d_sensor; // 3*2n
	ray_plane_interact(ground_seg_ray,ground_plane_sensor,ground_seg3d_sensor);
	temp = ground_seg3d_sensor.transpose();
	temp.resize(ground_seg3d_sensor.cols()/2,6); // n * 6
	ground_seg3d_sensor=temp;
	
	int num_seg = ground_seg2d_lines.rows();
	all_planes_sensor_out.resize(num_seg+1,4);
	all_planes_sensor_out.row(0)=ground_plane_sensor;

	for (int seg=0;seg<num_seg;seg++)
	{
	    Vector3f partwall_line3d_sensor_bg=ground_seg3d_sensor.row(seg).head(3);
	    Vector3f partwall_line3d_sensor_en=ground_seg3d_sensor.row(seg).tail(3);
	    Vector3f temp1=partwall_line3d_sensor_en-partwall_line3d_sensor_bg;
	    Vector3f temp2=ground_plane_sensor.head(3);
	    Vector3f partwall_normal_sensor=temp1.cross(temp2);
	    float dist=-partwall_normal_sensor.transpose()*partwall_line3d_sensor_bg;
	    Vector4f partwall_plane_sensor;   partwall_plane_sensor<<partwall_normal_sensor,dist;

	    all_planes_sensor_out.row(seg+1)=partwall_plane_sensor;
	}
    }
    else
    {
	all_planes_sensor_out.resize(0,4);
    }
}



void popup_plane::get_ground_edges(const cv::Mat &raw_img, const cv::Mat &label_map)
{
    width=label_map.cols;
    height=label_map.rows;    

    cv::Mat gray_img;
    if ( raw_img.channels()==3 )
	cv::cvtColor(raw_img, gray_img, CV_BGR2GRAY);
    else
	gray_img=raw_img;        
    
    cv::Mat contrast_img=gray_img;
    cv::Mat lsd_edge_mat;
    cv::Scalar mean_intensity=cv::mean(gray_img);
    if (mean_intensity[0]<90)  // if very dark
	cv::equalizeHist(gray_img, contrast_img);
    
    ca::Profiler::tictoc("get edge sum");
    ca::Profiler::tictoc("detect edge");
    std::clock_t begin2 = clock();    
    // whole_lsd(contrast_img,lsd_edge_mat);   // half_lsd 3 ms 
    line_lbd_ptr->detect_raw_lines(contrast_img,lsd_edge_mat,use_fast_edges_detection);
    ca::Profiler::tictoc("detect edge");
//     std::cout<<"detect edges "<< double(clock() - begin2) / CLOCKS_PER_SEC<<std::endl;
    
    ca::Profiler::tictoc("select edge");
    std::clock_t begin3 = clock();
    VectorXf actual_walls_in_closepoly_ind;  // actual existing walls in all planes,  not the manually connected wall
    if (inited_python)
	std::tie(ground_seg2d_lines_actual, ground_seg2d_lines_connect, actual_walls_in_closepoly_ind)\
	     =edge_get_polygons(lsd_edge_mat,label_map);  // using python function to select the 2d segments

//     cout<<"python return edges "<<ground_seg2d_lines_actual<<endl;
//     std::cout<<"select edges "<< double(clock() - begin3) / CLOCKS_PER_SEC<<std::endl;
    ca::Profiler::tictoc("select edge");
    ca::Profiler::tictoc("get edge sum");
    
    if (ground_seg2d_lines_connect.size()>0)   // if there is any gound edges
    {
	VectorXi actual_plane_indices_temp(actual_walls_in_closepoly_ind.rows()+1);
	actual_plane_indices_temp<<0, actual_walls_in_closepoly_ind.cast<int>().array()+1;
	actual_plane_indices_inall = actual_plane_indices_temp;
	total_plane_number = ground_seg2d_lines_connect.rows()+1;
    }
    else   // if there is no ground edges, then ground plane has no meaning as there is no boundary for the ground plane
    {
	actual_plane_indices_inall.resize(0);
	total_plane_number = 0;
	cout<<"cannot find ground edges!!!"<<endl;
    }
}


// if separate_cloud and mixed_cloud both true, will publish both.
void popup_plane::generate_cloud(const cv::Mat &bgr_img, const Matrix4f& transToWolrd, bool downsample_poly, bool separate_cloud,
				 bool mixed_cloud,bool get_label_img,float depth_thre)
{
   if (total_plane_number>0)
   {
      vector<MatrixXf> partplane_pts_sensor_all(good_plane_number);//3*n// due to ray_plane_interact and real to homo, cannot change to 3Xf
      vector<Matrix3Xf> partplane_pts_world_all(good_plane_number);//3*n
      vector<Matrix3Xf> partplane_pts_2d_homo(good_plane_number); // 3*m homo point      
      if (get_label_img)
	  edge_label_img=cv::Mat(Size(width,height),CV_8UC1,Scalar(127)); // initialized as void  could also use plane mask image multification
      for (int i=0;i<good_plane_indices.rows();i++)  //0 is ground
      {	  
	  int total_plane_id=good_plane_indices(i);
	  
	  Matrix2Xf plane_close_polys = all_closed_2d_bound_polygons[total_plane_id];
	  if (walllength_threshold>0)
	     plane_close_polys = all_closed_2d_cutted_polygons[total_plane_id];
	  if (plane_close_polys.cols()==0)
	      continue;
	  popup_plane::closed_polygons_homo_pts(plane_close_polys.transpose(),partplane_pts_2d_homo[i],downsample_poly,show_debug_info);   //3*m  ~3ms
	  Matrix3Xf partwall_pts_ray = invK*partplane_pts_2d_homo[i];
	  Vector4f partwall_plane_sensor = all_planes_sensor.row(total_plane_id);  // plane parameters in sensor frame
	  ray_plane_interact(partwall_pts_ray,partwall_plane_sensor,partplane_pts_sensor_all[i]);  //3*m  3-4ms
	  partplane_pts_world_all[i] = transToWolrd.topLeftCorner<3,3>()*partplane_pts_sensor_all[i]; // can save sometime for large points
	  partplane_pts_world_all[i].colwise() += transToWolrd.col(3).head(3);
	  
	  // project onto plane
	  if (cloud_projected_plane.norm()>0)
	  {
	      cloud_projected_plane /= cloud_projected_plane.head<3>().norm();
	      MatrixXf aa=(cloud_projected_plane.head<3>().transpose()*partplane_pts_world_all[i]).array()+cloud_projected_plane(3);
	      partplane_pts_world_all[i] = partplane_pts_world_all[i] - cloud_projected_plane.head<3>()*(aa);
	  }
	  if (get_label_img){   // generate ground label image
	    for (int pt=0;pt<partplane_pts_2d_homo[i].cols();pt++){
	      if (total_plane_id==0)  // ground
		edge_label_img.at<uchar>(partplane_pts_2d_homo[i](1,pt),partplane_pts_2d_homo[i](0,pt))=255; 
	      else    // wall
		edge_label_img.at<uchar>(partplane_pts_2d_homo[i](1,pt),partplane_pts_2d_homo[i](0,pt))=0; 
	    }
	  }
      }
      if (separate_cloud | mixed_cloud)
	  matrixToCloud(partplane_pts_2d_homo,partplane_pts_sensor_all,partplane_pts_world_all, bgr_img, separate_cloud,mixed_cloud,depth_thre);
      
      if (get_label_img & downsample_poly){ // if downsample polygon, there mayby many void pixels
	  cv::resize(edge_label_img,edge_label_img,cv::Size(),0.5,0.5,INTER_NEAREST);  //downsample raw image	    
	  cv::resize(edge_label_img,edge_label_img,cv::Size(),2,2,INTER_NEAREST);      //upsample raw image	
      }
   }
   else
   {
     pcl_cloud_world.reset(new pclRGB);
     partplane_clouds_open_all.resize(0);
     cout<<"cannot generate cloud!!!"<<endl;
   }
}


void popup_plane::get_depth_map_good(cv::Mat& out_depth,const Matrix4f& transToWolrd, bool downsample_poly,VectorXi& good_plane_indexs,\
			      vector<Matrix2Xf> & all_closed_2d_bound_polys, MatrixX4f& all_planes_sensors, Vector4f& ceilin_plane_sensor) const
{
    out_depth=cv::Mat(Size(width,height),CV_32FC1,Scalar(0));
    int valid_plane_numbers=good_plane_indexs.rows();
    if (valid_plane_numbers>0)
    {
	vector<MatrixXf> partplane_pts_sensor_all(valid_plane_numbers);  // depth init should use all the planes, the more the better
	vector<Matrix3Xf> partplane_pts_world_all(valid_plane_numbers);
	vector<Matrix3Xf> partplane_pts_2d_homo(valid_plane_numbers);

	for (int i=0;i<valid_plane_numbers;i++)  // 0 is ground
	{   // if in the original open polygons. (no manually connected edges)  // now I only pop up original lines.
	      int total_plane_id=good_plane_indexs(i);
	      if (all_closed_2d_bound_polygons[total_plane_id].cols()==0)
		  continue;
	      popup_plane::closed_polygons_homo_pts(all_closed_2d_bound_polys[total_plane_id].transpose(),partplane_pts_2d_homo[i],downsample_poly);   //3*m  ~3ms
	      Matrix3Xf partwall_pts_ray=invK*partplane_pts_2d_homo[i];
	      Vector4f partwall_plane_sensor=all_planes_sensors.row(total_plane_id);  // plane parameters in sensor frame
	      ray_plane_interact(partwall_pts_ray,partwall_plane_sensor,partplane_pts_sensor_all[i]);  //3*m  3-4ms
	      partplane_pts_world_all[i] = transToWolrd.topLeftCorner<3,3>()*partplane_pts_sensor_all[i]; // can save sometime for large points
	      partplane_pts_world_all[i].colwise() += transToWolrd.col(3).head(3);	      
	}

    // 	ca::Profiler::tictoc("init depth");
	for (int i=0;i<valid_plane_numbers;i++){
	    for (int pt=0;pt<partplane_pts_2d_homo[i].cols();pt++){
		int x=partplane_pts_2d_homo[i](0,pt); // col
		int y=partplane_pts_2d_homo[i](1,pt); // row
		float depth_value=partplane_pts_sensor_all[i](2,pt);
		float height=partplane_pts_world_all[i](2,pt);
		if (height<ceiling_threshold){
		    if (depth_value<0)  //don't show back points
			continue;
		    out_depth.at<float>(y,x)=depth_value;
		}else{
		    MatrixXf ceiling_pts_ray=invK*Vector3f(x,y,1);
		    MatrixXf ceiling_pts_sensor;
		    ray_plane_interact(ceiling_pts_ray,ceilin_plane_sensor,ceiling_pts_sensor);  //3*m  3-4ms
		    if (ceiling_pts_sensor(2,0)<0)  //don't show back points
			continue;
		    out_depth.at<float>(y,x)=ceiling_pts_sensor(2,0);
		}
	  }
	}
	
      if (downsample_poly){
	    cv::resize(out_depth,out_depth,cv::Size(),0.5,0.5);  //downsample raw image
	    out_depth.convertTo(out_depth,-1,4);
	    cv::resize(out_depth,out_depth,cv::Size(),2,2);      //upsample raw image
	}
    }
    else{
      cout<<"cannot comptue depth"<<endl;
    }
}


/// // xyz rgb are all 3*m, will remove receiling point  local_pts_parts is just to check point validity
void popup_plane::matrixToCloud(const vector<Matrix3Xf> &pixels_homo, const vector<MatrixXf> &local_pts_parts, const vector<Matrix3Xf> &global_pts_parts, 
				const cv::Mat &bgr_img,bool separate_cloud,bool mixed_cloud,float depth_thre)
{
    if(separate_cloud)
        partplane_clouds_open_all.resize(good_plane_number);  // note that vector resize will copy previous elements, not clear them.  eigen resize will clear all      
    if(mixed_cloud)
    {
	pcl_cloud_world.reset(new pclRGB);    
	int total_pt_num=0;
	for (int plane_id=0;plane_id<good_plane_number;plane_id++)
	    total_pt_num=total_pt_num+global_pts_parts[plane_id].cols();
	pcl_cloud_world->reserve(total_pt_num); // it is ok to resize a large number.
    }
    
    pcl::PointXYZRGB pt;
    int total_pt_counter=-1;    
    for (int plane_id=0;plane_id<good_plane_number;plane_id++)
    {
	if (separate_cloud){
	    partplane_clouds_open_all[plane_id].reset(new pclRGB);
	    partplane_clouds_open_all[plane_id]->clear();
	    partplane_clouds_open_all[plane_id]->reserve(global_pts_parts[plane_id].cols());
	}
	for (int ind=0;ind<global_pts_parts[plane_id].cols();ind++)
	{
	    total_pt_counter++;
	    if (local_pts_parts[plane_id](2,ind)<0)
	       continue;
	    if (local_pts_parts[plane_id](2,ind)>depth_thre)
	       continue;
// 	    if (Vector2f(local_pts_parts[plane_id](0,ind),local_pts_parts[plane_id](2,ind)).norm()>5)  # compute distance to camera, instead of z
// 	       continue;
	    if (global_pts_parts[plane_id](2,ind)<-0.2) // don't show underground pts
	      continue;
	    pt.x = global_pts_parts[plane_id](0,ind);
	    pt.y = global_pts_parts[plane_id](1,ind);
	    pt.z = global_pts_parts[plane_id](2,ind)<ceiling_threshold ? global_pts_parts[plane_id](2,ind):ceiling_threshold;
	    
	    if (bgr_img.rows>0)
	    {
		cv::Vec3b bgr_value = bgr_img.at<cv::Vec3b>(pixels_homo[plane_id](1,ind),pixels_homo[plane_id](0,ind));  //at row (y) and col(x)
		pt.r = bgr_value[2];   //r
		pt.g = bgr_value[1];   //g
		pt.b = bgr_value[0];   //b
	    }

	    if (bgr_img.cols==960)
	      if (pixels_homo[plane_id](0,ind)<4||pixels_homo[plane_id](1,ind)<4) // for kinect2 image, left and top boundary is likely to have black pixels....
  // 	      if ((int(pt.r)==0)&&(int(pt.g)==0)&&(int(pt.b)==0)) // if black, sometimes near black. skip it
		{
  // 		ROS_ERROR_STREAM("Popup matrix to cloud black pt!!!  "<<pt.x<<" "<<pt.y<<" "<<pt.z);
		  continue;
		}

	    if (separate_cloud)
	       partplane_clouds_open_all[plane_id]->points.push_back(pt); // ind
	    if (mixed_cloud)
	       pcl_cloud_world->points.push_back(pt); // total_plane_id
	}
    }
}