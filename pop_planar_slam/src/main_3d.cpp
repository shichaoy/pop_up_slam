#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/common/time.h>
#include <unistd.h>
#include <math.h>


#include "Mapping.h"
#include "Map_plane.h"
#include "Frame.h"
#include "tictoc_profiler/profiler.hpp"
#include "pop_up_wall/popup_plane.h"


#include <ros/ros.h>
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry> 


// for recording data analysis
#include <fstream>
#include <iostream>
#include <string> 
#include <sstream>

using namespace std;
using namespace Eigen;
using namespace cv;


const double PI=3.14159265;
const int recordEveryNth=10;
int counter=-1;
int marker_int=0;


template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
	return ((x.array() == x.array())).all();
}

geometry_msgs::Pose posenode_to_geomsgs(const isam::Pose3d &pose)
{	
	geometry_msgs::Pose pose_msg;
	Eigen::Matrix4d wTo = pose.wTo();
	
	pose_msg.position.x=wTo(0,3);
	pose_msg.position.y=wTo(1,3);
	pose_msg.position.z=wTo(2,3);
	
	Matrix3d pose_rot=wTo.block(0,0,3,3);
	Eigen::Quaterniond pose_quat(pose_rot);
	pose_msg.orientation.x = pose_quat.x();  
	pose_msg.orientation.y = pose_quat.y();
	pose_msg.orientation.z = pose_quat.z();
	pose_msg.orientation.w = pose_quat.w();
	return pose_msg;
}


class plane_slam
{
public:
	plane_slam(ros::NodeHandle &nn);
	~plane_slam();
	
	cv::Mat _color;
	ros::Time _timestamp;
	std_msgs::Header img_header;	
	string world_frame_id,sensor_frame_id;
	
	Eigen::Matrix3f Kalib;
	Eigen::Matrix4d InitToWolrd;  //odometry initial frame
	int width;
	int height;
	ros::NodeHandle n;
	
	shared_ptr<Mapper_mono> mapper;

	// Plane pop up object	
	popup_plane *pop_up_obj;
	
	ros::Publisher pub_rect_raw_img,pub_rect_raw_sync_img, pub_cnn_img, pub_crf_img, pup_polygon_img, pup_good_polygon_img, pub_cnn_blend,pub_cnn_label, pub_crf_blend;
	ros::Publisher pub_pose_sync,pub_pose_odom_sync,pub_modified_odom_pose;
	ros::Publisher pub_slam_pose,pub_slam_bound_polygon,pub_slam_bound_polygon_hist,pub_curr_bound_polygon;	
	ros::Publisher pub_curr_cloud,pub_all_cloud;
	
	// purely plane slam
	void img_callback_cnn_crf(const sensor_msgs::Image::ConstPtr& msg_image,const sensor_msgs::Image::ConstPtr& label_image);


	// for the current frame, draw the bounding polygon for each plane, each bounding polygon is 3*N
	void pub_curr_bounding_plane(const vector<MatrixXf>& all_3d_bound_polygons_world);
	int max_slam_bounding_plane_marker_num;
	
	// draw all the landmark planes in SLAM, directly use the plane's latest bounding vertex (updated by projection in after each slam update)
	void pub_all_plane_landmark_poly(const vector<Map_plane*> &all_plane_landmarks);

	// draw all the previouly planes in each frames.
	void pub_all_hist_frame_planes_poly(const std::vector< tracking_frame* >& all_frames);
	
protected:
  // camera  calibration parameters, default is ueye camera
	double calib_fx=371.975264;
	double calib_fy=315.632343;
	double calib_cx=372.163582;
	double calib_cy=250.592551;
	
	bool final_reproject_label_img=false;
	bool final_reproject_depth_img=false;
	bool save_edge_labelmap=false;
	bool use_loop_close=false;
	int last_frame_id = -1; // finally show all the reprojected point cloud
	cv_bridge::CvImageConstPtr cv_ptr_rgb_img,cv_ptr_label;
		
	bool if_record_pose=false;
	ofstream costsLogfile;
	
	std::string package_path;
};


plane_slam::plane_slam(ros::NodeHandle &nn)
{
	n=nn;
// 	_timestamp=0;

	n.param ("/planar_test/package_path", package_path, package_path); // no '/' at last
	
	n.param ("/planar_test/final_reproject_label_img", final_reproject_label_img, final_reproject_label_img);
	n.param ("/planar_test/final_reproject_depth_img", final_reproject_depth_img, final_reproject_depth_img);
	n.param ("/planar_test/save_edge_labelmap", save_edge_labelmap, save_edge_labelmap);
	n.param ("/planar_test/use_loop_close", use_loop_close, use_loop_close);
	n.param ("/planar_test/last_frame_id", last_frame_id, last_frame_id);
	n.param ("/planar_test/if_record_pose", if_record_pose, if_record_pose);
	
	n.param ("/planar_test/calib_fx", calib_fx, calib_fx);
	n.param ("/planar_test/calib_fy", calib_fy, calib_fy);
	n.param ("/planar_test/calib_cx", calib_cx, calib_cx);
	n.param ("/planar_test/calib_cy", calib_cy, calib_cy);
	
	Kalib << calib_fx, 0, calib_fy,
		0, calib_cx, calib_cy, 
		0,   0,   1;

	world_frame_id="/world";
	sensor_frame_id="/camera_rgb_optical_frame";
	
	pop_up_obj = new popup_plane();
	pop_up_obj->set_calibration(Kalib);

	pop_up_obj->Init_python();  //whether using python funtion
	
	pop_up_obj->walllength_threshold = 3.0;

// 	pop_up_obj->simple_polygon_mode = true;
// 	pop_up_obj->plane_cam_dist_thre = 50;
// 	pop_up_obj->ceiling_threshold = 2.8;
	
        InitToWolrd<<1, 0, 0, 0,
		     0, 0, 1, 0,
		     0,-1, 0, 1,
		     0, 0, 0, 1;
	  
	mapper = make_shared<Mapper_mono>();
	mapper->calib=Kalib;
	mapper->inv_calib=Kalib.inverse();
	mapper->initialize_orb_matching();
	mapper->pop_up_obj=pop_up_obj;
	
	pub_rect_raw_img = n.advertise<sensor_msgs::Image>("/rect_color_raw", 10);
	pub_rect_raw_sync_img = n.advertise<sensor_msgs::Image>("/rect_color_raw_sync", 10);
	pub_cnn_img = n.advertise<sensor_msgs::Image>("/cnn_pred", 10);
	pub_cnn_label = n.advertise<sensor_msgs::Image>("/cnn_label", 10);
	pub_crf_img = n.advertise<sensor_msgs::Image>("/crf_opti", 10);
	pub_cnn_blend = n.advertise<sensor_msgs::Image>("/cnn_pred_blend", 10);
	pub_crf_blend = n.advertise<sensor_msgs::Image>("/crf_opti_blend", 10);
	pup_polygon_img = n.advertise<sensor_msgs::Image>("/pop_polygon", 10);
	pup_good_polygon_img = n.advertise<sensor_msgs::Image>("/pop_good_polygon", 10);
			
	pub_slam_bound_polygon = n.advertise<visualization_msgs::MarkerArray>( "/slam_plane_3d_polygons", 10 );
	pub_slam_bound_polygon_hist = n.advertise<visualization_msgs::MarkerArray>( "/slam_plane_3d_polygons_hist", 10 );
	pub_curr_bound_polygon = n.advertise<visualization_msgs::MarkerArray>( "/curr_plane_3d_polygons", 10 );	

	pub_slam_pose = n.advertise<geometry_msgs::PoseArray>( "/slam_pose", 10 );
	pub_pose_sync=n.advertise<geometry_msgs::PoseStamped>( "/lsd_slam/pose_sync", 10 );
	pub_pose_odom_sync=n.advertise<nav_msgs::Odometry>( "/lsd_slam/pose_odom_sync", 10 );
	pub_modified_odom_pose=n.advertise<nav_msgs::Odometry>( "/lsd_slam/corrput_odom_pose", 10 );
	
	pub_curr_cloud = n.advertise<pclRGB> ("pop_cloud", 50);
	pub_all_cloud = n.advertise<pclRGB> ("pop_cloud_all", 50);
	
	max_slam_bounding_plane_marker_num=0;
	
	if (if_record_pose){
	    string cost_record_name=package_path+"/temp/slam_pose_array.txt";
	    costsLogfile.open(cost_record_name.c_str());
	}	
}

plane_slam::~plane_slam()
{
	if (if_record_pose) costsLogfile.close();
	delete pop_up_obj;
}

void plane_slam::pub_all_plane_landmark_poly(const vector<Map_plane*> &all_plane_landmarks)
{
	visualization_msgs::MarkerArray plane_markers;
	visualization_msgs::Marker marker;
	marker.header=img_header;
	marker.header.frame_id="/world";
	marker.id = 0; //0
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
	marker.scale.x = 0.05;
	  
	for (int c = 0; c < all_plane_landmarks.size(); c++){
	    if (all_plane_landmarks[c]->frame_seq_id<=counter-15)
	      if (all_plane_landmarks[c]->being_tracked_times<10)  // for older frames, only visualize the plane that has been tracked over enough times.
		continue;
	    if (all_plane_landmarks[c]->frame_seq_id<=counter-8)
	      if (all_plane_landmarks[c]->being_tracked_times<5)
		continue;
	      if (all_plane_landmarks[c]->deteted_by_merge)  // if it is a dead node, being merged node, skip this
		  continue;
  // 	   cout<<"show plane frame id "<<all_plane_landmarks[c].frame_seq_id<<"  tracked time  "<<all_plane_landmarks[c].being_tracked_times<<endl;	     
	      marker.id++;
	      marker.points.resize(all_plane_landmarks[c]->plane_bound_close_3D_polys.cols());
	      for (int i=0;i<all_plane_landmarks[c]->plane_bound_close_3D_polys.cols();i++)
	      {
		marker.points[i].x=all_plane_landmarks[c]->plane_bound_close_3D_polys(0,i);
		marker.points[i].y=all_plane_landmarks[c]->plane_bound_close_3D_polys(1,i);
		marker.points[i].z=all_plane_landmarks[c]->plane_bound_close_3D_polys(2,i);
	      }
	      plane_markers.markers.push_back(marker);
	}
	for (int i=0;i<plane_markers.markers.size();i++) {
	    plane_markers.markers[i].color.r= (double)i/plane_markers.markers.size();
	    plane_markers.markers[i].color.g= 1-(double)i/plane_markers.markers.size();
	}
	pub_slam_bound_polygon.publish(plane_markers);         
}


void plane_slam::pub_all_hist_frame_planes_poly(const std::vector< tracking_frame* >& all_frames)
{
	visualization_msgs::MarkerArray plane_markers;
	visualization_msgs::Marker marker;
	marker.header=img_header;
	marker.header.frame_id="/world";
	marker.id = 0; //0
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
    //     marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1;  // paper color
	marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1;    
	marker.scale.x = 0.05;
	
	for (int frame_ind=0;frame_ind<all_frames.size();frame_ind=frame_ind+2){  // every two frames    
	    for (int plane_ind=0;plane_ind<all_frames[frame_ind]->all_3d_bound_polygons_good_world.size();plane_ind++){
		if (all_frames[frame_ind]->good_plane_indices(plane_ind)==0)  // don't show ground
		    continue;
		int tracked_times=all_frames[frame_ind]->observed_planes[plane_ind]->being_tracked_times;
		int frame_sequ_id=all_frames[frame_ind]->frame_seq_id;
		if ( frame_sequ_id<=counter-15 )  // for older frames, only visualize the plane that has been tracked over enough times.
		  if (tracked_times<10)
		    continue;
		if ( frame_sequ_id<=counter-8 )
		  if (tracked_times<5)
		    continue;
		marker.id++;
		marker.points.resize(all_frames[frame_ind]->all_3d_bound_polygons_good_world[plane_ind].cols());
		for (int i=0;i<all_frames[frame_ind]->all_3d_bound_polygons_good_world[plane_ind].cols();i++){
		  marker.points[i].x=all_frames[frame_ind]->all_3d_bound_polygons_good_world[plane_ind](0,i);
		  marker.points[i].y=all_frames[frame_ind]->all_3d_bound_polygons_good_world[plane_ind](1,i);
		  marker.points[i].z=all_frames[frame_ind]->all_3d_bound_polygons_good_world[plane_ind](2,i);
		}
		plane_markers.markers.push_back(marker);
	    }
	}
	for (int i=0;i<plane_markers.markers.size();i++){  // paper color not using this
	    plane_markers.markers[i].color.r= (double)i/plane_markers.markers.size();
	    plane_markers.markers[i].color.g= 1-(double)i/plane_markers.markers.size();
	}
	
	pub_slam_bound_polygon_hist.publish(plane_markers);     
}


void plane_slam::pub_curr_bounding_plane(const vector<MatrixXf>& all_3d_bound_polygons_world)
{
	visualization_msgs::MarkerArray plane_markers;    
	visualization_msgs::Marker marker;
	marker.header=img_header;
	marker.header.frame_id="/world";      
	marker.id = 0; //0
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 1.0;
	marker.scale.x = 0.05;
	
	for (int c = 0; c < all_3d_bound_polygons_world.size(); c++){
	      marker.id++;
	      marker.points.resize(all_3d_bound_polygons_world[c].cols());
	      for (int i=0;i<all_3d_bound_polygons_world[c].cols();i++)
	      {
		marker.points[i].x=all_3d_bound_polygons_world[c](0,i);
		marker.points[i].y=all_3d_bound_polygons_world[c](1,i);
		marker.points[i].z=all_3d_bound_polygons_world[c](2,i);
	      }
	      plane_markers.markers.push_back(marker);
	}    
	if (max_slam_bounding_plane_marker_num<marker.id)  // if last frame has more planes, some planes will still show if we don't push the void landmarks.
	    max_slam_bounding_plane_marker_num=marker.id;
	else{
	  while (plane_markers.markers.size()!=max_slam_bounding_plane_marker_num){
	    marker.id++;
	    visualization_msgs::Marker marker_new;
	    marker_new.scale.x=0.05;
	    marker_new.color.a=0;	
	    marker_new.header.frame_id="/world";  
	    marker_new.id=marker.id;
	    plane_markers.markers.push_back(marker_new);
	  }
	}    
	pub_curr_bound_polygon.publish(plane_markers);
}


void plane_slam::img_callback_cnn_crf(const sensor_msgs::Image::ConstPtr& color_msg_img,const sensor_msgs::Image::ConstPtr& label_msg_img)
{
	counter++;
	cout<<"------------------------------------------------------------"<<endl;
	cout<<"processing kf "<<counter<<"   "<<color_msg_img->header.stamp<<endl;
	ca::Profiler::tictoc("process one frame");
        img_header=color_msg_img->header;
	width=color_msg_img->width;
	height=color_msg_img->height;
		
	isam::Pose3d modified_slam_odom_pose;   // lastest slam plus latest modifieded odometry
	isam::Pose3d modified_odometry;
	if (counter==0){	    
	    Eigen::Matrix4d curr_transToWolrd;   // a camera space point multiplied by this, goes to world frame.
	    curr_transToWolrd.setIdentity();
	    curr_transToWolrd<<-0.4563,  0.2444, -0.8556,-1.2432,  // tum fr3 no texture far initial pose by ground truth
				0.8898,  0.1249, -0.4388,1.9219,
				-0.0004, -0.9616, -0.2745,1.0547;
				 0,   0,   0,   1;
	    modified_odometry=isam::Pose3d(curr_transToWolrd);
	    modified_slam_odom_pose=modified_odometry;
	}else if (counter>0){
	    if (counter==1)
		modified_odometry=isam::Pose3d();   // zero odometry
	    else{
		//NOTE a simple constant motion model is used. can be replaced by more accurate odometry
	        modified_odometry=mapper->all_frames.back()->pose_vertex->value().ominus(mapper->all_frames[mapper->all_frames.size()-2]->pose_vertex->value());   // const motion assumption
	    }
	    modified_slam_odom_pose=mapper->all_frames.back()->pose_vertex->value().oplus(modified_odometry);
	}
	
	cv::Mat bgr_raw_img, label_img;
	cv_ptr_rgb_img = cv_bridge::toCvShare(color_msg_img,sensor_msgs::image_encodings::TYPE_8UC3);	//BGR format
	cv_ptr_rgb_img->image.copyTo(bgr_raw_img);
	ROS_ASSERT(bgr_raw_img.channels()==3);  // color image
	
	cv::Mat gray_img;
	cv::cvtColor(bgr_raw_img, gray_img, CV_BGR2GRAY);
	
	
	cv_ptr_label = cv_bridge::toCvShare(label_msg_img,label_msg_img->encoding);
	cv_ptr_label->image.copyTo(label_img);

	cv_bridge::CvImage out_image;
	out_image.header=img_header;

	if (label_msg_img->encoding==sensor_msgs::image_encodings::TYPE_32FC1){  // if it is cnn unary image,    crf image type is CV_8UC1	
	    cv::resize(label_img,label_img,bgr_raw_img.size(),0,0, CV_INTER_LINEAR);  // resize to normal size
	    threshold(label_img,label_img,0.5,255,cv::THRESH_BINARY);  // threshold to be 0,255
	    label_img.convertTo(label_img,CV_8UC1);	    
	    out_image.image=label_img;
	    out_image.encoding=sensor_msgs::image_encodings::TYPE_8UC1;
	    pub_cnn_label.publish(out_image);
	}

	Mat blend_img;
	blend_segmentation(bgr_raw_img,label_img,blend_img);
	out_image.image=blend_img;
	out_image.encoding=sensor_msgs::image_encodings::TYPE_8UC3; 
	pub_crf_blend.publish(out_image.toImageMsg());

	pop_up_obj->walllength_threshold = 3.0;
	pop_up_obj->simple_polygon_mode = false;
	pop_up_obj->ceiling_threshold = 5.0;
	pop_up_obj->line_lbd_ptr->use_LSD = false;

	// Plane pop up polygons
	ca::Profiler::tictoc("get edges");
	pop_up_obj->get_ground_edges(bgr_raw_img,label_img);
	ca::Profiler::tictoc("get edges");
	Matrix4d pop_up_pose=modified_slam_odom_pose.wTo();
	
	MatrixX4f utilized_ground_seg2d_lines  = pop_up_obj->ground_seg2d_lines_actual;     // whether include connected segments
	
	ca::Profiler::tictoc("get equation");
	pop_up_obj->get_plane_equation(utilized_ground_seg2d_lines,pop_up_pose.cast<float>());	// using latest plane slam pose plus odometry	
	ca::Profiler::tictoc("get equation");
	
	if (!is_finite<MatrixX4f>(pop_up_obj->all_planes_sensor))  //check nan
	    ROS_ERROR_STREAM("detect nan elements in plane parameters");
		
	// publish good ground 2d polygons
	cv::Mat raw_img_cp=bgr_raw_img.clone();
	for (int i=0;i<utilized_ground_seg2d_lines.rows();i++)      // plot the lines
	{
	    if (check_element_in_vector(i,pop_up_obj->good_plane_indices))  // good lines in red, bad lines in blue
		line(raw_img_cp, Point(utilized_ground_seg2d_lines(i,0),utilized_ground_seg2d_lines(i,1)), \
	             Point(utilized_ground_seg2d_lines(i,2),utilized_ground_seg2d_lines(i,3)), Scalar(0,0,255),4 , 8, 0);  
	    else
		line(raw_img_cp, Point(utilized_ground_seg2d_lines(i,0),utilized_ground_seg2d_lines(i,1)), \
	             Point(utilized_ground_seg2d_lines(i,2),utilized_ground_seg2d_lines(i,3)), Scalar(255,0,0),4 , 8, 0);
	}
	out_image.image=raw_img_cp;
	out_image.encoding=sensor_msgs::image_encodings::TYPE_8UC3;
	pup_good_polygon_img.publish(out_image.toImageMsg());
	
	
	// Plane pop up point cloud  this is only for visulization
	pop_up_obj->generate_cloud(bgr_raw_img, pop_up_pose.cast<float>(),true,true,true,save_edge_labelmap);
// 	if (counter % 10 == 0){
	    pop_up_obj->pcl_cloud_world->header.frame_id="/world";
	    pop_up_obj->pcl_cloud_world->header.seq=counter;
	    pop_up_obj->pcl_cloud_world->header.stamp=img_header.stamp.toNSec() / 1000ull;
	    pub_curr_cloud.publish(*(pop_up_obj->pcl_cloud_world));
// 	}
	    
	
	cout<<"Detect plane number "<<pop_up_obj->good_plane_number<<endl;	
	if (pop_up_obj->good_plane_number>0){  // use the good planes to do slam, not including manually connected edges
	      vector<Map_plane> new_planes(pop_up_obj->good_plane_number);
	      tracking_frame*  newframe( new tracking_frame);
	      newframe->good_plane_dist_cam.resize(pop_up_obj->good_plane_number);
	      vector<MatrixXf> all_3d_bound_polygons_good_world(pop_up_obj->good_plane_number);  //3d
	      for (int i=0;i< pop_up_obj->good_plane_number;i++)
	      {
		  int good_plane_ind=pop_up_obj->good_plane_indices[i]; // plane index in all the planes, including ground and manually connected plane
		  new_planes[i].temp_value= isam::Plane3d(pop_up_obj->all_planes_sensor.row(good_plane_ind).cast<double>());
		  new_planes[i].t_stamp=img_header.stamp;
		  new_planes[i].frame_seq_id=counter;
		  new_planes[i].plane_cloud=pop_up_obj->partplane_clouds_open_all[i];   // plane's point cloud
		  new_planes[i].plane_bound_close_2D_polys=pop_up_obj->all_closed_2d_bound_polygons[good_plane_ind];
		  new_planes[i].plane_bound_close_3D_polys=pop_up_obj->all_3d_bound_polygons_world[good_plane_ind];
		  new_planes[i].ground_seg2d_lines=utilized_ground_seg2d_lines;  // all the planes havs the same ground2d segmentation image
		  new_planes[i].frame_plane_indice=good_plane_ind;
		  new_planes[i].good_plane_indice=i;
		  new_planes[i].plane_dist_cam=pop_up_obj->all_plane_dist_to_cam[good_plane_ind];
		  new_planes[i].observer_frames.push_back(newframe);
		  new_planes[i].being_tracked_times = 1;
		  all_3d_bound_polygons_good_world[i]=pop_up_obj->all_3d_bound_polygons_world[good_plane_ind];
		  newframe->good_plane_dist_cam[i]=pop_up_obj->all_plane_dist_to_cam[good_plane_ind];
	      }
	      newframe->t_stamp=img_header.stamp;
	      newframe->frame_seq_id=counter;
	      newframe->frame_img=bgr_raw_img;
	      newframe->ground_seg2d_lines=new_planes[0].ground_seg2d_lines;
	      newframe->all_3d_bound_polygons_good_world=all_3d_bound_polygons_good_world;
	      newframe->all_3d_bound_polygons_world_raw=all_3d_bound_polygons_good_world;
	      newframe->partplane_clouds=pop_up_obj->partplane_clouds_open_all;
	      newframe->good_plane_indices=pop_up_obj->good_plane_indices;
	      newframe->temp_pose=modified_odometry;
	      
	      pub_curr_bounding_plane(all_3d_bound_polygons_good_world);  // current 3d bounding box.
	      if (counter==last_frame_id)
		 mapper->last_frame_process=true;
	      
	      ca::Profiler::tictoc("slam process");
	      mapper->processFrame(new_planes, newframe);
	      ca::Profiler::tictoc("slam process");
	      mapper->update_plane_measurement();
	}
	
	isam::Pose3d pose_node;
	geometry_msgs::PoseArray all_poses;
	for (int i = 0; i < mapper->all_frames.size(); i++){ 
	    pose_node = mapper->all_frames[i]->pose_vertex->value();
	    all_poses.poses.push_back(posenode_to_geomsgs(pose_node));
	}
	all_poses.header=img_header;
	all_poses.header.frame_id=world_frame_id;
	pub_slam_pose.publish(all_poses);
	
	if (counter==last_frame_id){
	      if (if_record_pose){	    
		  costsLogfile << "# timestamp tx ty tz qx qy qz qw"<<"\n";
		  for (int i=0;i<all_poses.poses.size();i++)
		  {
		    costsLogfile<<mapper->all_frames[i]->t_stamp<<" "<<all_poses.poses[i].position.x<<" "<<all_poses.poses[i].position.y<<" "<<all_poses.poses[i].position.z<<" ";
		    costsLogfile<<all_poses.poses[i].orientation.x<<" "<<all_poses.poses[i].orientation.y<<" "<<all_poses.poses[i].orientation.z<<" "<<all_poses.poses[i].orientation.w<<endl;
		  }
	      }
	}

// 	cout<<"mapper->all_landmarks   "<<mapper->all_landmarks.size()<<endl;
	
        pub_all_plane_landmark_poly(mapper->all_landmarks);
	pub_all_hist_frame_planes_poly(mapper->all_frames);

	ca::Profiler::tictoc("process one frame");
	
	if (counter==last_frame_id){
	    ROS_ERROR_STREAM("publish all cloud");
	    int cloud_seq_id=0;
	    for (int frame_ind=0;frame_ind<mapper->all_frames.size();frame_ind++) {
		Matrix4d latest_pose = mapper->all_frames[frame_ind]->pose_vertex->value().wTo();  // reproject to get plane measurements
		for (int plane_ind=0;plane_ind<mapper->all_frames[frame_ind]->partplane_clouds.size();plane_ind++){
		    int tracked_times=mapper->all_frames[frame_ind]->observed_planes[plane_ind]->being_tracked_times;
		    int frame_sequ_id=mapper->all_frames[frame_ind]->frame_seq_id;
    // 		    cout<<tracked_times<<endl;
		    if (!final_reproject_label_img){
		      if ( frame_sequ_id<=counter-10 ){
			  if (frame_ind %3 !=0)  // show every N frame's depth
			    continue;}
		      else{
			if (frame_ind %2 !=0)  // show every N frame's depth
			    continue;
		      }
		    }
		    
		    if ( frame_sequ_id<=counter-15 )  // for older frames, only visualize the plane that has been tracked over enough times.
		      if (tracked_times<10)
			continue;
		    if ( frame_sequ_id<=counter-8 )
		      if (tracked_times<5)
			continue;
    		    if ( frame_sequ_id<=counter-4 )
    		      if (tracked_times<2)
    			continue;
		    pclRGB::Ptr plane_cloud=mapper->all_frames[frame_ind]->partplane_clouds[plane_ind];
		    for (int i=0;i<plane_cloud->points.size();i++){
			Vector3d raw_pt(plane_cloud->points[i].x,plane_cloud->points[i].y,plane_cloud->points[i].z);
			isam::Plane3d curr_plane=mapper->all_frames[frame_ind]->observed_planes[plane_ind]->plane_vertex->value();

			Vector3f updated_pt=curr_plane.project_to_plane(raw_pt).cast<float>();

    // 			double cam_pt_dist=(updated_pt.head(2)-latest_pose.col(3).head(2)).norm();
    // 			if (cam_pt_dist<5)  // if point too far from camera, don't show
    // 			    continue;
			
			plane_cloud->points[i].x=updated_pt(0);
			plane_cloud->points[i].y=updated_pt(1);
			plane_cloud->points[i].z=updated_pt(2);
		    }
		    plane_cloud->header.frame_id="/world";
		    plane_cloud->header.seq=cloud_seq_id;
		    pop_up_obj->pcl_cloud_world->header.stamp=(ros::Time::now().toNSec() / 1000ull)+cloud_seq_id*1000;
		    cloud_seq_id++;
		    pub_all_cloud.publish(*plane_cloud);
		    usleep(25000);  //sleep for 25ms
		}
	    }
	    ROS_ERROR_STREAM("finish pub cloud num "<<cloud_seq_id);	
	    return;
	}

}



int main(int argc, char** argv)
{    
	ca::Profiler::enable();
	
	ros::init(argc, argv, "pop_slam");
	ros::NodeHandle nh;
	ros::NodeHandle n;
	plane_slam planar(n);
	
	// whole plane slam without using pose odometry
	static message_filters::Subscriber<sensor_msgs::Image>  rgb_img_sub(nh, "/rect_color_raw" , 10);  //  /rect_color_raw   /mono/image_rect_color  rect_color_raw_sync
	static message_filters::Subscriber<sensor_msgs::Image>  crf_img_sub(nh , "/cnn_label", 10);  // crf_opti  // cnn_pred
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;
	static message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_img_sub, crf_img_sub);
	sync.registerCallback(boost::bind(&plane_slam::img_callback_cnn_crf,&planar, _1, _2));
	

	ros::spin();
	ca::Profiler::print_aggregated(std::cout);

	return 0;      
}
