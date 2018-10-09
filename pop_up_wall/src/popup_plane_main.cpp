/*
 * Copyright Shichao Yang,2016, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu 
 */


#include "pop_up_wall/popup_plane.h"

#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>

void pub_curr_bounding_plane(const vector<MatrixXf> all_3d_bound_polygons_world,ros::Publisher& pub_polygon)
{
    visualization_msgs::MarkerArray plane_markers;    
    visualization_msgs::Marker marker;    
    marker.header.frame_id="/world";  
    marker.header.stamp=ros::Time::now();
    marker.id = 0; //0
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
    marker.scale.x = 0.05;
    
    for (int c = 0; c < all_3d_bound_polygons_world.size(); c++)
    {
	  if (c==0)  // ground polygons is not fully right now
	    continue;
	  marker.id++;
	  marker.points.resize(all_3d_bound_polygons_world[c].cols());
	  for (int i=0;i<all_3d_bound_polygons_world[c].cols();i++)
	  {
	    marker.points[i].x=all_3d_bound_polygons_world[c](0,i);
	    marker.points[i].y=all_3d_bound_polygons_world[c](1,i);
	    marker.points[i].z=all_3d_bound_polygons_world[c](2,i);
	  }
// 	  marker.color.r= (double)c/all_3d_bound_polygons_world.size();
// 	  marker.color.g= 1-(double)c/all_3d_bound_polygons_world.size();
	  plane_markers.markers.push_back(marker);
    }        
    pub_polygon.publish(plane_markers);
}

int main(int argc, char** argv)
{  
  ros::init (argc, argv, "pub_pop_cloud");  
  if (argc<2){ 
	    cout<<"error: please provide image number"<<endl;
	    return 1;
  }  

  ros::NodeHandle nh;
    
  std::string package_path = ros::package::getPath("pop_up_wall");
  
  ros::Publisher pub_cloud = nh.advertise<pclRGB> ("pop_cloud", 1);
  ros::Publisher pub_curr_bound_polygon = nh.advertise<visualization_msgs::MarkerArray>( "/curr_plane_3d_polygons", 10 );
  ros::Publisher pup_good_polygon_img = nh.advertise<sensor_msgs::Image>("/pop_good_polygon", 10);
  
  int frame_number=atoi(argv[1]);
  char frame_index_c[256];  sprintf(frame_index_c,"%04d",frame_number);  // format into 4 digit
  std::string raw_img_name = package_path+"/data/rgb_"+std::string(frame_index_c)+".png";
  std::string label_img_name = package_path+"/data/label_"+std::string(frame_index_c)+".png";
  
  cv::Mat raw_img = cv::imread(raw_img_name, 1);    //  bgr data
  cv::Mat label_map = cv::imread(label_img_name, 0);   //  change to grapyscale   0 is wall. 255 is ground  
  if(!raw_img.data)            // Check for invalid input
  {
     cout <<  "Could not open or find the image  "<<raw_img_name << std::endl ;
     return 1;
  }
  if(!label_map.data)
  {
     cout <<  "Could not open or find the image  "<<label_img_name << std::endl ;
     return 1;    
  }
   

  Eigen::Matrix3f Kalib;  
  if (frame_number==50)
      Kalib << 371.975264, 0, 315.632343,   //ueye camera
	      0, 372.163582, 250.592551, 
	      0,   0,   1;
  else
      Kalib << 570.3422, 0, 319.5,  // kinect
	      0, 570.3422, 239.5, 
	      0,   0,   1;
  
  // camera pose is important for accurate pop up, you can assume it, or estimate from other sources
  // world: x right  y forward  z upward     camera: x right y downward, z forward
  Eigen::Matrix4f transToWolrd;   // a camera space point multiplied by this, goes to world frame.
  transToWolrd<<1, 0, 0, 0,
      0, 0, 1, 0,
      0,-1, 0, 1,
      0, 0, 0, 1;
  popup_plane pop_up_obj;
  pop_up_obj.ceiling_threshold = 2.8;
  pop_up_obj.walllength_threshold = 5; // if want to cut wall length
  pop_up_obj.cloud_downsample_rate = 2;
  pop_up_obj.set_calibration(Kalib);
  pop_up_obj.Init_python();

  pop_up_obj.simple_polygon_mode = true; //false  true
//   pop_up_obj.plane_cam_dist_thre = 10;

  pop_up_obj.get_ground_edges(raw_img, label_map);
  
  pop_up_obj.get_plane_equation(pop_up_obj.ground_seg2d_lines_connect,transToWolrd);   //can choose _actual or _connect
  
  // publish good ground 2d polygons
  cv::Mat raw_img_cp2=raw_img.clone();
  for (int i=1;i<pop_up_obj.good_plane_number;i++){      // plot the lines
	line(raw_img_cp2, Point(pop_up_obj.ground_seg2d_lines_connect(pop_up_obj.good_plane_indices(i)-1,0),pop_up_obj.ground_seg2d_lines_connect(pop_up_obj.good_plane_indices(i)-1,1)), \
		Point(pop_up_obj.ground_seg2d_lines_connect(pop_up_obj.good_plane_indices(i)-1,2),pop_up_obj.ground_seg2d_lines_connect(pop_up_obj.good_plane_indices(i)-1,3)), Scalar(255,0,0), 2, 8, 0);  
  }
  cv_bridge::CvImage out_image;
  out_image.header.stamp=ros::Time::now();
  out_image.image=raw_img_cp2;
  out_image.encoding=sensor_msgs::image_encodings::TYPE_8UC3;  
  

//   cout<<"class ground_seg2d_lines_connect "<<pop_up_obj.ground_seg2d_lines_connect<<endl;
//   cout<<"ground_seg3d_lines_world  "<<pop_up_obj.ground_seg3d_lines_world<<endl;
//   cout<<"good_plane_indices  "<<pop_up_obj.good_plane_indices.transpose()<<endl;
  
//   cout<<"pop_up_obj.good_plane_number "<<pop_up_obj.good_plane_number<<endl;
  vector<MatrixXf> all_3d_bound_polygons_good_world(pop_up_obj.good_plane_number);  //3d
  for (int i=0;i< pop_up_obj.good_plane_number;i++)
  {
      int good_plane_ind=pop_up_obj.good_plane_indices[i]; // plane index in all the planes, including ground and manually connected plane
      all_3d_bound_polygons_good_world[i]=pop_up_obj.all_3d_bound_polygons_world[good_plane_ind];
  }  
    
  pop_up_obj.generate_cloud(raw_img,transToWolrd,true, true);

  pclRGB::Ptr topub_cloud = pop_up_obj.pcl_cloud_world;
  topub_cloud->header.frame_id="/world";
  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    topub_cloud->header.stamp=ros::Time::now().toNSec() / 1000ull;
    pub_cloud.publish(topub_cloud);
    pub_curr_bounding_plane(all_3d_bound_polygons_good_world,pub_curr_bound_polygon);
    pup_good_polygon_img.publish(out_image.toImageMsg());
    
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  return 0;      
}
  