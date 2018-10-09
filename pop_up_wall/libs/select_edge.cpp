/*
 * Copyright Shichao Yang,2016, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu
 *
 */

// ours
#include "pop_up_wall/popup_plane.h"
#include "pop_up_wall/matrix_utils.h"

using namespace std;
using namespace cv;
using namespace Eigen;

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXf_row;

#define PI 3.14159265

void removeRow(MatrixXf_row& matrix, int rowToRemove)
{
    int numRows = matrix.rows()-1;
    int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

void addRow(MatrixXf_row& matrix, int rowToAdd, VectorXf& newRowVec) // new row will be in the rowToAdd place.
{    
    assert(matrix.cols()==newRowVec.rows());        
    int new_numRows = matrix.rows()+1;
    int numCols = matrix.cols();
    matrix.conservativeResize(new_numRows,numCols);
    if (rowToAdd<new_numRows-1)
	matrix.block(rowToAdd+1,0,new_numRows-rowToAdd-1,numCols) = matrix.block(rowToAdd,0,new_numRows-rowToAdd-1,numCols);
    matrix.row(rowToAdd)=newRowVec;
}

void popup_plane::Init_python()
{    
    try {
	Py_Initialize();
	PyRun_SimpleString("from pop_up_python import pop_up_fun");
	py_module = new bp::object(bp::handle<>(bp::borrowed(PyImport_AddModule("__main__"))));
	py_dictionary = py_module->attr("__dict__");
	PyRun_SimpleString("params=pop_up_fun.load_params()");
	// changing python pop up parameters //TODO python can also read yaml file?
	py_dictionary["pre_vertical_thre"] = pre_vertical_thre; PyRun_SimpleString("params['pre_vertical_thre']=pre_vertical_thre");
	py_dictionary["pre_minium_len"] = pre_minium_len; PyRun_SimpleString("params['pre_minium_len']=pre_minium_len");
	py_dictionary["pre_contour_close_thre"] = pre_contour_close_thre; PyRun_SimpleString("params['pre_contour_close_thre']=pre_contour_close_thre");
	py_dictionary["interval_overlap_thre"] = interval_overlap_thre; PyRun_SimpleString("params['interval_overlap_thre']=interval_overlap_thre");
	py_dictionary["post_short_thre"] = post_short_thre; PyRun_SimpleString("params['post_short_thre']=post_short_thre");
	py_dictionary["post_bind_dist_thre"] = post_bind_dist_thre; PyRun_SimpleString("params['post_bind_dist_thre']=post_bind_dist_thre");
	py_dictionary["post_merge_dist_thre"] = post_merge_dist_thre; PyRun_SimpleString("params['post_merge_dist_thre']=post_merge_dist_thre");
	py_dictionary["post_merge_angle_thre"] = post_merge_angle_thre; PyRun_SimpleString("params['post_merge_angle_thre']=post_merge_angle_thre");
	py_dictionary["post_extend_thre"] = post_extend_thre; PyRun_SimpleString("params['post_extend_thre']=post_extend_thre");
	inited_python=true;
    }
    catch (bp::error_already_set) {
	      PyErr_Print();}
}


tuple<MatrixXf, MatrixXf, VectorXf> popup_plane::edge_get_polygons(cv::Mat &lsd_edges,const cv::Mat &label_map)
{
    bool print_details=false;
    cv::Mat label_map_preproc; //1ms  
    if (downsample_contour){
	cv::resize(label_map,label_map_preproc,cv::Size(),0.5,0.5, INTER_NEAREST);  //downsample to speed up.
	dilate(label_map_preproc,label_map_preproc,Mat::ones(8, 8, CV_8U));
	erode(label_map_preproc,label_map_preproc,Mat::ones(8, 8, CV_8U));
    }else{
	dilate(label_map,label_map_preproc,Mat::ones(dilation_distance, dilation_distance, CV_8U));
	erode(label_map_preproc,label_map_preproc,Mat::ones(erosion_distance, erosion_distance, CV_8U));
    }
    label_map_preproc.convertTo(label_map_preproc,-1,-1,255);
    try {
	py_dictionary["label_map"] = label_map_preproc; // fast <<1ms
	// I couldn't find c++ opencv replacement functions for this python function...
	PyRun_SimpleString("ground_contour = pop_up_fun.python_get_contours(label_map)");  // downsampled CNN contours  ~3ms
	cv::Mat ground_contour_mat = bp::extract<cv::Mat_<float>>(py_dictionary["ground_contour"]);
	Eigen::Map<MatrixXf_row> ground_contour(ground_contour_mat.ptr<float>(),ground_contour_mat.rows,ground_contour_mat.cols);
	if (downsample_contour)
	      ground_contour*=2;
	
	lsd_edges.convertTo(lsd_edges,CV_32FC1); //change from int32 to float
	Eigen::Map<MatrixXf_row> raw_lsd_lines_eig(lsd_edges.ptr<float>(),lsd_edges.rows,lsd_edges.cols); 	

	// Step 1: remove very short lines then remove nearly vertical lines	
	std::vector<int> not_vertical_ind;
	{
	    MatrixXf_row line_vector=raw_lsd_lines_eig.rightCols(2)-raw_lsd_lines_eig.leftCols(2);
	    VectorXf line_length=line_vector.rowwise().norm();
	    for (int i=0;i<raw_lsd_lines_eig.rows();i++){
	      if (line_length(i)<pre_minium_len)
		continue;
	      if ( ((raw_lsd_lines_eig(i,0)<pre_boundary_thre) && (raw_lsd_lines_eig(i,2)<pre_boundary_thre)) || \
		  ((raw_lsd_lines_eig(i,0)>width-pre_boundary_thre) && (raw_lsd_lines_eig(i,2)>width-pre_boundary_thre)) || \
		  ((raw_lsd_lines_eig(i,1)<pre_boundary_thre) && (raw_lsd_lines_eig(i,3)<pre_boundary_thre)) || \
		  ((raw_lsd_lines_eig(i,1)>height-pre_boundary_thre) && (raw_lsd_lines_eig(i,3)>height-pre_boundary_thre)) )
		continue;
	      float line_angle=normalize_to_pi(atan2(line_vector(i,1),line_vector(i,0))/PI*180);
	      if ((std::abs(std::abs(line_angle)-90)>pre_vertical_thre))
		  not_vertical_ind.push_back(i);
	    }
	}
	if (print_details){
	    cout<<"not_vertical_ind size "<<not_vertical_ind.size()<<endl;
	}

	// Step 2: remove lines that are far from CNN boudary
	std::vector<int> close_line_ind;
	for (int& line_id: not_vertical_ind){
	    VectorXf boundary_dist_oneline(10);
	    Vector2f sample_pt;
	    for (int sample_ind=0;sample_ind<10;sample_ind++){
	      sample_pt=raw_lsd_lines_eig.row(line_id).head(2)+sample_ind/10.0*(raw_lsd_lines_eig.row(line_id).tail(2)-raw_lsd_lines_eig.row(line_id).head(2));
	      boundary_dist_oneline[sample_ind]= ( ground_contour.rowwise()-sample_pt.transpose()).rowwise().norm().minCoeff();
	    }
	    if (boundary_dist_oneline.maxCoeff()<pre_contour_close_thre)
	      close_line_ind.push_back(line_id);
	}
// 	cout<<"close_line_ind size "<<close_line_ind.size()<<endl;
	MatrixXf_row good_edges(close_line_ind.size(),4);
	for (int j=0;j<close_line_ind.size();j++){
	    if (raw_lsd_lines_eig(close_line_ind[j],0)>raw_lsd_lines_eig(close_line_ind[j],2)){
		  good_edges.row(j).head(2)=raw_lsd_lines_eig.row(close_line_ind[j]).tail(2);
		  good_edges.row(j).tail(2)=raw_lsd_lines_eig.row(close_line_ind[j]).head(2);
	    }
	    else
		good_edges.row(j)=raw_lsd_lines_eig.row(close_line_ind[j]);
	}
	if (print_details){
	    cout<<"close_nonvertical_lines  "<<good_edges<<endl;
	}

	// Step3: merge two short edges if they are very parallel, and very close (basically from the same line segments)  
	// example:   ----- ----  and 
	// --------    this have problems for  -------------
	// -------			          -----
	{
	    bool can_force_merge=true;
	    int counter=0;
	    while ((can_force_merge) && (counter<100)){
		    counter++;
		    can_force_merge=false;
		    MatrixXf_row line_vectors=good_edges.rightCols(2)-good_edges.leftCols(2);
		    int line_number=good_edges.rows();
		    VectorXf all_angles(line_number);
		    for (int i=0;i<line_number;i++)
			    all_angles(i)=normalize_to_pi(atan2(line_vectors(i,1),line_vectors(i,0))/PI*180);
		    for (int seg1=0;seg1<line_number;seg1++) {
			for (int seg2=seg1+1;seg2<line_number;seg2++){
			      float diff=std::abs(all_angles(seg1)-all_angles(seg2));
			      float angle_diff=min(diff,180-diff);
			      if (angle_diff<pre_merge_angle_thre){
				  float dist_1ed_to_2=(good_edges.row(seg1).tail(2)-good_edges.row(seg2).head(2)).norm();
				  float dist_2ed_to_1=(good_edges.row(seg2).tail(2)-good_edges.row(seg1).head(2)).norm();
				  if (dist_1ed_to_2<pre_merge_dist_thre){
					good_edges.row(seg1).tail(2)=good_edges.row(seg2).tail(2);
					removeRow(good_edges,seg2);
					can_force_merge=true;
					break;
				  }
				  if (dist_2ed_to_1<pre_merge_dist_thre){
					good_edges.row(seg1).head(2)=good_edges.row(seg2).head(2);
					removeRow(good_edges,seg2);
					can_force_merge=true;
					break;
				  }
			      }
			}
			if (can_force_merge)
			      break;			
		    }
	    }
	}
	if (print_details){
	    std::cout<<"close_nonvertical_long_lines "<<good_edges<<endl;
	}
	
	//Step 4: for every two edges, if they are similiar in angle and closer, if their projections onto each other overlap much, delete it.  
	// examples  ------
	//             ------
	{
	    bool can_force_delete=true;
	    int counter=0;
	    while ((can_force_delete) && (counter<100)){
		    counter++;
		    can_force_delete=false;
		    MatrixXf_row line_vector=good_edges.rightCols(2)-good_edges.leftCols(2);
		    int line_number=good_edges.rows();
		    VectorXf all_angles(line_number);
		    float dist_1_bg_to_2,proj_1_bg_to_2,dist_1_ed_to_2,proj_1_ed_to_2,dist_2_bg_to_1,proj_2_bg_to_1,dist_2_ed_to_1,proj_2_ed_to_1;
		    for (int i=0;i<line_number;i++)
			    all_angles(i)=normalize_to_pi(atan2(line_vector(i,1),line_vector(i,0))/PI*180);
		    for (int seg1=0;seg1<line_number;seg1++) {
			for (int seg2=seg1+1;seg2<line_number;seg2++){
			    float diff=std::abs(all_angles(seg1)-all_angles(seg2));
			    float angle_diff=min(diff,180-diff);
			    if (angle_diff<pre_proj_angle_thre){   // if nearly parallel
				point_distproj_to_line(good_edges.row(seg2).head(2),good_edges.row(seg2).tail(2),\
							good_edges.row(seg1).head(2),  dist_1_bg_to_2, proj_1_bg_to_2);
				point_distproj_to_line(good_edges.row(seg2).head(2),good_edges.row(seg2).tail(2),\
							good_edges.row(seg1).tail(2),dist_1_ed_to_2, proj_1_ed_to_2);
				point_distproj_to_line(good_edges.row(seg1).head(2),good_edges.row(seg1).tail(2),\
							good_edges.row(seg2).head(2),dist_2_bg_to_1, proj_2_bg_to_1);
				point_distproj_to_line(good_edges.row(seg1).head(2),good_edges.row(seg1).tail(2),\
							good_edges.row(seg2).tail(2),dist_2_ed_to_1, proj_2_ed_to_1);
				if ( (dist_1_bg_to_2<pre_proj_dist_thre) && (dist_1_ed_to_2<pre_proj_dist_thre) && \
					(dist_2_bg_to_1<pre_proj_dist_thre) && (dist_2_ed_to_1<pre_proj_dist_thre)) {
					float covering_1_to_2=std::abs(proj_1_bg_to_2-proj_1_ed_to_2);
					float covering_2_to_1=std::abs(proj_2_bg_to_1-proj_2_ed_to_1);
					if ((covering_1_to_2>pre_proj_cover_thre) || (covering_2_to_1>pre_proj_cover_thre)){  // delete one if has larger overlapping
						int to_delete_ind=-1;
						if (min(covering_1_to_2,covering_2_to_1)<pre_proj_cover_large_thre){
						    if (covering_1_to_2>covering_2_to_1)
							to_delete_ind=seg2;
						    else
							to_delete_ind=seg1;
						}else{  // if there are very similiar, nearly the same, use the one more close to cnn
							Vector2f boundary_dist,sample_pt;
							int check_line_id=0;
							VectorXf boundary_dist_oneline(10);
							for (int line_ind=0;line_ind<2;line_ind++){
							      for (int sample_ind=0;sample_ind<10;sample_ind++){
								  if (line_ind==0)  check_line_id=seg1;
								  if (line_ind==1)  check_line_id=seg2;
								  sample_pt=good_edges.row(check_line_id).head(2)+sample_ind/10.0*\
										  (good_edges.row(check_line_id).tail(2)-good_edges.row(check_line_id).head(2));
								  boundary_dist_oneline(sample_ind)= ( ground_contour.rowwise()-sample_pt.transpose()).rowwise().norm().minCoeff();
							      }
							      boundary_dist(line_ind)=boundary_dist_oneline.maxCoeff();
							}
							if (boundary_dist(0)>boundary_dist(1))
							      to_delete_ind=seg1;
							else
							      to_delete_ind=seg2;
						}
						removeRow(good_edges,to_delete_ind);
						can_force_delete=1;
						break;
					}
				}
			    }
			}
			if (can_force_delete)
			      break;
		    }
	    }
	}
	if (print_details){
	    std::cout<<"close_nonvertical_long_delete "<<good_edges<<endl;
	}
// 	std::cout<<"close_nonvertical_long_delete "<<good_edges.rows()<<endl;
	if (good_edges.rows()==0){
	    VectorXf final_lines_in_extend_ind;
	    final_lines_in_extend_ind.resize(0);
	    return make_tuple(good_edges,good_edges,final_lines_in_extend_ind);  // return all void matrix
	}
	
	//Step 5: interval tree optimization
	Mat good_edges_mat(good_edges.rows(), good_edges.cols(), CV_32FC1, good_edges.data());  
	py_dictionary["close_nonvertical_lines"] = good_edges_mat;
	PyRun_SimpleString("optimized_line_segs = pop_up_fun.interval_tree_optimization(close_nonvertical_lines,params)");
	cv::Mat optimized_line_segs_mat = bp::extract<cv::Mat_<float>>(py_dictionary["optimized_line_segs"]);
	Eigen::Map<MatrixXf_row> optimized_line_segs(optimized_line_segs_mat.ptr<float>(),optimized_line_segs_mat.rows,optimized_line_segs_mat.cols);
	if (print_details){
	    cout<<"optimized_line_segs "<<optimized_line_segs<<endl;
	}
// 	cout<<"optimized_line_segs_mat row "<<optimized_line_segs_mat.rows<<endl;
	
	//Step 6: post process
	MatrixXf_row final_long_segs;
	{
	    MatrixXf_row line_vector=optimized_line_segs.rightCols(2)-optimized_line_segs.leftCols(2);
	    VectorXf line_length=line_vector.rowwise().norm();
	    std::vector<int> long_seg_ind;
	    for (int i=0;i<line_length.rows();i++)
		  if (line_length(i)>post_short_thre)
		      long_seg_ind.push_back(i);	      
	    final_long_segs.resize(long_seg_ind.size(),4);
	    for (int j=0;j<long_seg_ind.size();j++)
		  final_long_segs.row(j)=optimized_line_segs.row(long_seg_ind[j]);
	    bool can_force_bind=true;
	    int counter=0;
	    while ((can_force_bind) && (counter<100)){
		  counter++;
		  can_force_bind=false;
		  for (int seg=0;seg<final_long_segs.rows()-1;seg++){
		      if ( (final_long_segs(seg,2)!= final_long_segs(seg+1,0)) || ((final_long_segs(seg,3)!= final_long_segs(seg+1,1))) )
			  if ( (final_long_segs.row(seg).tail(2)-final_long_segs.row(seg+1).head(2)).norm() < post_bind_dist_thre ){
				Vector2i mean_pose=((final_long_segs.row(seg).tail(2)+final_long_segs.row(seg+1).head(2))/2).cast<int>();
				final_long_segs(seg,2)=mean_pose(0);final_long_segs(seg,3)=mean_pose(1);
				final_long_segs(seg+1,0)=mean_pose(0);final_long_segs(seg+1,1)=mean_pose(1);
				can_force_bind=true;
			  }
		  }
	    }
	    if (print_details)
		  cout<<"final_bind_segs "<<final_long_segs<<endl;
	}
	// if two successive edges are nearly parallel and close（boundary distance, because all the edges 
	// don't overlap in x direction）, merge them as one.  note!!! this might reduce the number of edges. I actually doubt this
	//  ------
	//           ----
	{
	    bool can_force_merge=true;
	    int counter=0;
	    while ((can_force_merge) && (counter<100)){
		    counter++;
		    can_force_merge=false;
		    MatrixXf_row line_vector=final_long_segs.rightCols(2)-final_long_segs.leftCols(2);
		    int line_number=final_long_segs.rows();
		    VectorXf all_angles(line_number);
		    for (int i=0;i<line_number;i++)
			    all_angles(i)=normalize_to_pi(atan2(line_vector(i,1),line_vector(i,0))/PI*180);
		    for (int seg=0;seg<line_number-1;seg++) {
			    float diff=std::abs(all_angles(seg)-all_angles(seg+1));
			    float angle_diff=min(diff,180-diff);
			    if (angle_diff<post_merge_angle_thre){   // if nearly parallel
				  float dist_1=point_dist_line(final_long_segs.row(seg+1).head(2),final_long_segs.row(seg+1).tail(2),final_long_segs.row(seg).head(2));
				  float dist_2=point_dist_line(final_long_segs.row(seg+1).head(2),final_long_segs.row(seg+1).tail(2),final_long_segs.row(seg).tail(2));
				  float dist_3=point_dist_line(final_long_segs.row(seg).head(2),final_long_segs.row(seg).tail(2),final_long_segs.row(seg+1).head(2));
				  float dist_4=point_dist_line(final_long_segs.row(seg).head(2),final_long_segs.row(seg).tail(2),final_long_segs.row(seg+1).tail(2));
				  if ( ((dist_1<post_merge_dist_thre)&(dist_2<post_merge_dist_thre)) | ((dist_3<post_merge_dist_thre)&(dist_4<post_merge_dist_thre))){
					final_long_segs.row(seg).tail(2)=final_long_segs.row(seg+1).tail(2);
					removeRow(final_long_segs,seg+1);
					can_force_merge=true;
					break;
				  }
			    }
		    }
	    }
	    if (print_details)
		  cout<<"final_merge_segs "<<final_long_segs<<endl;
	}
        // extend first and last seg to hit the boundary.
	{
	    Vector2f old_start_pt=final_long_segs.row(0).head(2);
	    Vector2f old_end_pt=final_long_segs.bottomRows<1>().tail(2);
	    Vector2i new_start_pt=(direction_hit_boundary(final_long_segs.row(0).tail(2),final_long_segs.row(0).head(2)-final_long_segs.row(0).tail(2),
							      width,height)).cast<int>();
	    Vector2i new_end_pt=(direction_hit_boundary(final_long_segs.bottomRows<1>().head(2),final_long_segs.bottomRows<1>().tail(2)-final_long_segs.bottomRows<1>().head(2),
							      width,height)).cast<int>();
	    final_long_segs(0,0)=new_start_pt(0);final_long_segs(0,1)=new_start_pt(1);
	    final_long_segs.bottomRows<1>()(2)=new_end_pt(0);final_long_segs.bottomRows<1>()(3)=new_end_pt(1);
	    Vector2f boundary_dist,sample_pt;
	    VectorXf boundary_dist_oneline(10);
	    for (int line_ind=0;line_ind<2;line_ind++){
		  int check_line_id=0;
		  for (int sample_ind=0;sample_ind<10;sample_ind++){
		      if (line_ind==0)  check_line_id=0;
		      if (line_ind==1)  check_line_id=final_long_segs.rows()-1;
		      sample_pt=final_long_segs.row(check_line_id).head(2)+sample_ind/10.0*\
				(final_long_segs.row(check_line_id).tail(2)-final_long_segs.row(check_line_id).head(2));
		      boundary_dist_oneline(sample_ind)= ( ground_contour.rowwise()-sample_pt.transpose()).rowwise().norm().minCoeff();
		  }
		  boundary_dist(line_ind)=boundary_dist_oneline.maxCoeff();
	    }
	    if (boundary_dist(0)>post_extend_thre)
		final_long_segs.row(0).head(2)=old_start_pt;
	    if (boundary_dist(1)>post_extend_thre)
	       final_long_segs.bottomRows<1>().tail(2)=old_end_pt;
	    if (print_details)
		cout<<"final_extend_segs "<<final_long_segs<<endl;
	}

      // add some edges to connect all the existing lines.  only used for depth initialization, otherwise,
      // there is a lot of big holes in the depth image	
	MatrixXf_row final_open_segs=final_long_segs; // output
	VectorXf final_lines_in_extend_ind(final_long_segs.rows());
	for (int i=0;i<final_lines_in_extend_ind.rows();i++)
	    final_lines_in_extend_ind(i)=i;
	{
	    vector<int> extend_line_ind;
	    for (int seg=0;seg<final_long_segs.rows()-1;seg++){
		if ((final_long_segs(seg,2)!= final_long_segs(seg+1,0)) || (final_long_segs(seg,3)!= final_long_segs(seg+1,1))){
		  extend_line_ind.push_back(seg+1);
		}
	    }
	    if (extend_line_ind.size()==0)
		return make_tuple(final_open_segs, final_long_segs, final_lines_in_extend_ind);
	    
	    final_long_segs.conservativeResize(final_long_segs.rows()+extend_line_ind.size(),4);
	    int block_end;
	    for (int i=0;i<extend_line_ind.size();i++){
		if ( i!= (extend_line_ind.size()-1) )
		      block_end=extend_line_ind[i+1];
		else
		      block_end=final_open_segs.rows();
		for (int j=extend_line_ind[i];j<block_end;j++){
		    final_long_segs.row(j+i+1)=final_open_segs.row(j);
		    final_lines_in_extend_ind(j)=j+i+1;
		}
		final_long_segs.row(extend_line_ind[i]+i).head(4)=Vector4f(final_open_segs(extend_line_ind[i]-1,2),final_open_segs(extend_line_ind[i]-1,3),\
									    final_open_segs(extend_line_ind[i],0),final_open_segs(extend_line_ind[i],1) );
	    }
	    if (print_details)
		cout<<"final_connect_segs "<<final_long_segs<<endl;
// 	    cout<<"final_lines_in_extend_ind "<<final_lines_in_extend_ind<<endl;
	}
	return make_tuple(final_open_segs, final_long_segs, final_lines_in_extend_ind);
    }
    catch (bp::error_already_set) {
	      PyErr_Print();}
}
