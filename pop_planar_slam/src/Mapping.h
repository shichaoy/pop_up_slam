/*
 * Copyright Shichao Yang,2016, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu 
 */

#pragma once

#include "ros/ros.h"
#include "isam/isam.h"
#include "isam/slam3d.h"

#include <Eigen/Dense>
#include <Eigen/Geometry> 

// for slam loop closure
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Thirdparty/ORB_SLAM/ORBextractor.h"
#include "Thirdparty/ORB_SLAM/ORBVocabulary.h"
#include "Thirdparty/ORB_SLAM/Converter.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;


// forward declaration
class tracking_frame;
class Map_plane;
class popup_plane;

class Mapper_mono {
public:    
    Mapper_mono();
    ~Mapper_mono();
    
    ros::NodeHandle nn;  // to parse the parameters
    
    isam::Slam _slam;	// isam graph
    
    std::vector<tracking_frame*> all_frames;    
    std::vector<Map_plane*>  all_landmarks; // plane property is being updated all the time, it also contains planenodes. always add, no delete.   
    
    //plane association  returns the <id, matcherror> of the best matching plane, or -1 if no match found  
    std::tuple<int, double> findClosestPlane(const isam::Pose3d& pose,const Map_plane* check_plane);
    std::tuple<int, double> findLoopPlane(const isam::Pose3d& pose, const Map_plane* check_plane, int old_loop_frame_ind=0);
        
    void processFrame(std::vector<Map_plane> &frame_planes, tracking_frame* newframe);
    
    bool loopclose_merge(int to_merge_frame_ind); // merge existing plane landmarks into this frame's landmark

    void reproj_to_newplane();        // project 3D bouding polygon or point cloud onto new plane landmark position
    void update_plane_measurement();  // using updated pose to reproject planes
    bool last_frame_process=false;
    
    // ORB SLAM's loop closing
    void initialize_orb_matching();
    std::vector<tracking_frame*> DetectLoopCandidates(tracking_frame* pKF);    
    ORB_SLAM::ORBextractor* mpORBextractor;
    ORB_SLAM::ORBVocabulary* mpORBVocabulary;    

    Eigen::Matrix3f calib;
    Eigen::Matrix3f inv_calib;    
    
    popup_plane *pop_up_obj;// for updating the plane measurement
    
private:  
    bool use_loop_close=false; // whether loop close detection
    std::string package_path;
    
    // for plane matching (allows for camera motion)
    double maxDistance = 0.10; //0.1; //0.12; //0.1; //0.05; //0.12;
    double maxAngle = /*8.  3.  5. 8. 20.*/   20. / 180. * M_PI;
    double edge_asso_2ddist=50;
    double edge_asso_planedist=4;
    double edge_asso_proj=0.5;
    double edge_asso_angle=60.0;
    int assoc_near_frames=5;    
    bool print_assoc_detail=true;
    
    // skip frames
    int renderEveryNth = 5;  
    
    // isam graph parameters.
    double pose_sigma_x=0.001;
    double pose_sigma_y=0.001;
    double pose_sigma_z=0.001;
    double pose_sigma_yaw=0.01;
    double pose_sigma_pitch=0.01;
    double pose_sigma_roll=0.01;
      
    double plane_sigma_x=0.001;
    double plane_sigma_y=0.001;
    double plane_sigma_z=0.001;  
    
    double plane_sigma_dist_mul=3;
      
    Vector6d poseSigmas;
    Eigen::Vector3d planeSigmas;
    boost::shared_ptr<isam::Covariance> poseCov;
    boost::shared_ptr<isam::Covariance> planeCov;      
};