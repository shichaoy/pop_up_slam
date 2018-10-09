/*
 * Copyright Shichao Yang,2016, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu 
 */

#include "Mapping.h"
#include "Frame.h"
#include "Map_plane.h"
#include "isam_plane3d.h"

#include "pop_up_wall/matrix_utils.h"
#include "pop_up_wall/popup_plane.h"

#include <Eigen/Dense>
#include "ros/ros.h"

using namespace std;
using namespace isam;
using namespace Eigen;

const bool useRelative = false;

double tictoc_t0;
void _tic() {
  tictoc_t0 = pcl::getTime();
}
void _toc(const string& s) {
  cout << "Time for " << s << ": " << pcl::getTime() - tictoc_t0 << endl;
}

Mapper_mono::Mapper_mono():nn( "~" ) {
    Properties prop = _slam.properties();
     prop.method = LEVENBERG_MARQUARDT;  //defalut is gauss-newton
  //  prop.method = DOG_LEG;
    prop.mod_batch = 1;
    prop.quiet=true;  // print optimization
    prop.epsilon2*=0.1;
    prop.epsilon_abs*=0.1;  // more iterations
    prop.epsilon_rel*=0.1;
    /** Termination criterion:  stop whenever the /difference/ in absolute
    * sum-of-squares errors between two estimates falls below this fraction
    * of the /total/ absolute sum-of-squares errors. */        
    _slam.set_properties(prop);  
    
    nn.param ("/planar_test/package_path", package_path, package_path);
    nn.param ("/planar_test/edge_asso_2ddist", edge_asso_2ddist, edge_asso_2ddist);
    nn.param ("/planar_test/edge_asso_proj", edge_asso_proj, edge_asso_proj);
    nn.param ("/planar_test/edge_asso_angle", edge_asso_angle, edge_asso_angle);
    nn.param ("/planar_test/edge_asso_planedist", edge_asso_planedist, edge_asso_planedist);        
    nn.param ("/planar_test/assoc_near_frames", assoc_near_frames, assoc_near_frames);
    nn.param ("/planar_test/print_assoc_detail", print_assoc_detail, print_assoc_detail);
    
    nn.param ("/planar_test/pose_sigma_x", pose_sigma_x, pose_sigma_x);
    nn.param ("/planar_test/pose_sigma_y", pose_sigma_y, pose_sigma_y);
    nn.param ("/planar_test/pose_sigma_z", pose_sigma_z, pose_sigma_z);
    nn.param ("/planar_test/pose_sigma_yaw", pose_sigma_yaw, pose_sigma_yaw);
    nn.param ("/planar_test/pose_sigma_pitch", pose_sigma_pitch, pose_sigma_pitch);
    nn.param ("/planar_test/pose_sigma_roll", pose_sigma_roll, pose_sigma_roll);
    nn.param ("/planar_test/plane_sigma_x", plane_sigma_x, plane_sigma_x);
    nn.param ("/planar_test/plane_sigma_y", plane_sigma_y, plane_sigma_y);
    nn.param ("/planar_test/plane_sigma_z", plane_sigma_z, plane_sigma_z);
    nn.param ("/planar_test/plane_sigma_dist_mul", plane_sigma_dist_mul, plane_sigma_dist_mul);           
    
    poseSigmas<<pose_sigma_x,pose_sigma_y,pose_sigma_z,pose_sigma_yaw,pose_sigma_pitch,pose_sigma_roll;
    planeSigmas<<plane_sigma_x,plane_sigma_y,plane_sigma_z;
    poseCov = boost::shared_ptr<Covariance> ( new Covariance(poseSigmas.cwiseProduct(poseSigmas).asDiagonal()));
    planeCov = boost::shared_ptr<Covariance> ( new Covariance(planeSigmas.cwiseProduct(planeSigmas).asDiagonal()));
    
    nn.param ("/planar_test/use_loop_close", use_loop_close, use_loop_close);
}

Mapper_mono::~Mapper_mono()
{
    if (mpORBextractor!=nullptr)
      delete mpORBextractor;
    if (mpORBVocabulary!=nullptr)
      delete mpORBVocabulary;
}


void Mapper_mono::initialize_orb_matching()
{
    if (use_loop_close){
	string strVocFile=package_path+"/params/ORBvoc.bin";
	cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;    
	mpORBVocabulary=new ORB_SLAM::ORBVocabulary();

	bool bVocLoad = mpORBVocabulary->loadFromBinaryFile(strVocFile);
	
	if (bVocLoad)
	    cout << endl << "Finish Load ORB Vocabulary." << endl;
	else
	    ROS_ERROR("Cannod Load ORB Vocabulary.");
	assert(bVocLoad);
	string strSettingPath=package_path+"/params/ORB_Settings.yaml";     
	cv::FileStorage fSettings(strSettingPath.c_str(), cv::FileStorage::READ);
	if(!fSettings.isOpened())    
	    ROS_ERROR("Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory.");
	
	assert(fSettings.isOpened());
	int nFeatures = fSettings["ORBextractor.nFeatures"];
	float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
	int nLevels = fSettings["ORBextractor.nLevels"];
	int fastTh = fSettings["ORBextractor.fastTh"];    
	int Score = fSettings["ORBextractor.nScoreType"];
	assert(Score==1 || Score==0);
	mpORBextractor = new ORB_SLAM::ORBextractor(nFeatures,fScaleFactor,nLevels,Score,fastTh);
    }
}


float point_proj_to_lineseg(const Vector2f &begin_pt, const Vector2f &end_pt, const Vector2f &query_pt)
{  // v w p
      // Return minimum distance between line segment vw and point p
    float length = (end_pt-begin_pt).norm();
    if (length < 0.001) return (query_pt-begin_pt).norm();   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line. 
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    float t = ((query_pt-begin_pt).dot(end_pt-begin_pt))/length/length;
    if (t>1.0)
      return 1.0;
    if (t<0.0)
      return 0.0;
    return t;
}


vector<tracking_frame*> Mapper_mono::DetectLoopCandidates(tracking_frame* pKF)
{      
      list<pair<float,tracking_frame*> > lScoreAndMatch;
      int nscores=0;
      // Compute similarity score. Retain the matches whose score is higher than minScore
      float minScore=0.01;  // initial ORB slam determines this number based on covisity graph?
      float bestAccScore=minScore;
      for (int i=0;i<50;i++)  // HACK. only search begin 50 frames?? refer to orb slam for better loop closure
      {	
	  tracking_frame* pKFi=all_frames[i];
	  float si = mpORBVocabulary->score(pKF->mBowVec,pKFi->mBowVec);
	  if(si>=minScore){
	       lScoreAndMatch.push_back(make_pair(si,pKFi));
	       if(si>bestAccScore)
                   bestAccScore=si;  // update the best score
	  }
      }
      if(lScoreAndMatch.empty())
	  return vector<tracking_frame*>();

      list<pair<float,tracking_frame*> > lAccScoreAndMatch;      
      // Return all those keyframes with a score higher than 0.75*bestScore
      float minScoreToRetain = 0.75f*bestAccScore;

      set<tracking_frame*> spAlreadyAddedKF;
      vector<tracking_frame*> vpLoopCandidates;
      vpLoopCandidates.reserve(lAccScoreAndMatch.size());

      for(list<pair<float,tracking_frame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
      {
	  if(it->first>minScoreToRetain)
	  {
	      tracking_frame* pKFi = it->second;
	      if(!spAlreadyAddedKF.count(pKFi))
	      {
		  vpLoopCandidates.push_back(pKFi);
		  spAlreadyAddedKF.insert(pKFi);
	      }
	  }
      }
      return vpLoopCandidates;
}


// find the loop planes from the loop frame 
std::tuple<int, double> Mapper_mono::findLoopPlane(const Pose3d& curr_esti_pose, const Map_plane* check_plane, int old_loop_frame_ind) {
      int numMatches = 0;
      int bestMatch = -1;      
      double bestReprojError = -1;

      const Plane3d curr_plane_local=check_plane->plane_vertex->value();      
      Plane3d curr_plane_world = curr_plane_local.transform_from(curr_esti_pose.oTw());  //  transform plane to global coordinate      
      Eigen::Matrix4d curr_plane_pose_mat= curr_esti_pose.wTo();
      Eigen::MatrixXf curr_bounding_2d_poly=check_plane->plane_bound_close_2D_polys;      
      
      int start_frame_id=std::max(old_loop_frame_ind-0,  0);   // search neighbour frames' planes       
      int end_frame_id=std::min(old_loop_frame_ind+1,  (int)all_frames.size());   // better method is covisity graph to find connected frames
      bool plane_found=false;
      for (int frame_id = start_frame_id; frame_id < end_frame_id; frame_id++) {     // could speed up, only use recent frames
	   tracking_frame* potential_frame=all_frames[frame_id];
	   for (int plane_id=0;plane_id<potential_frame->observed_planes.size();plane_id++){
		int loop_cand_global_id=potential_frame->observed_planes[plane_id]->landmark_index;
		if (all_landmarks[loop_cand_global_id]->deteted_by_merge)  // if it is a dead node, being merged node, skip this
		    continue;
		if ( check_plane->frame_plane_indice==0 && all_landmarks[loop_cand_global_id]->frame_plane_indice==0 ){  // if both are ground, associate them immediately.
		      numMatches++;
		      bestMatch = loop_cand_global_id;
		      plane_found=true;
		      break;
		} 
		if ( ( check_plane->frame_plane_indice==0 && all_landmarks[loop_cand_global_id]->frame_plane_indice>=1 )  ||    // if one is ground and the other is wall, must not be associated
		      ( check_plane->frame_plane_indice>=1 && all_landmarks[loop_cand_global_id]->frame_plane_indice==0 ) )
		      continue;   // skip this plane, check further plane
		
		if (all_landmarks[loop_cand_global_id]->being_tracked_times<10)
		     continue;
		
// 		Plane3d old_plane_world = _planeNodes[loop_cand_global_id]->value();
		cout<<"to test global id "<<all_landmarks[loop_cand_global_id]->landmark_index<<"  at frame "<<all_landmarks[loop_cand_global_id]->frame_seq_id<<" indice "<<all_landmarks[loop_cand_global_id]->good_plane_indice<<endl;
		Vector4f old_plane_2d_edge=potential_frame->ground_seg2d_lines.row( potential_frame->good_plane_indices(plane_id)-1);
		float poly_2d_dist=0;   //for loop closing, I think we can only use image based difference, bow or pixel difference
		for (int i=0;i<2;i++){
		      Vector2d distance_all;
		      for (int j=0;j<2;j++)  
			  distance_all(j)=(curr_bounding_2d_poly.col(i)-old_plane_2d_edge.segment<2>(j*2)).norm();
		      poly_2d_dist+=distance_all.minCoeff();
		}
		poly_2d_dist=poly_2d_dist/2;
		float poly_2d_dist_cross=0;
		for (int i=0;i<2;i++){
		      Vector2f distance_all;
		      for (int j=0;j<2;j++) 
			  distance_all(j)=(curr_bounding_2d_poly.col(j)-old_plane_2d_edge.segment<2>(i*2)).norm(); 
		      poly_2d_dist_cross+=distance_all.minCoeff();
		}
		poly_2d_dist_cross=poly_2d_dist_cross/2;
		float plane_2d_img_dis=poly_2d_dist+poly_2d_dist_cross;
		
		numMatches++;
		float total_error=plane_2d_img_dis;
//      	if (print_assoc_detail)
		    cout<<"total error "<<total_error<<endl;
		if (numMatches==1 || total_error<bestReprojError ){ // the smaller, the better  //different frame might say the same plane node, it doesn't matter.
		    bestMatch = loop_cand_global_id;
		    bestReprojError = total_error;   // weight combination of angle and 3d projection overlapping
		}
		continue;
	   }
	   if (plane_found)
	     break;
      }            
//       if (numMatches > 1) {
// 	  cout << "Warning: multiple potential matches for one plane." << endl;
//       }
      if (0)
      {
	  if (bestMatch<0){
	      ROS_WARN_STREAM("Not finding matched planes, add as new landmark.");    
	  }else{
	      ROS_WARN_STREAM("Final matches global at  "<<bestMatch<<"  in frame  "<< all_landmarks[bestMatch]->frame_seq_id <<"  at indice  "<<all_landmarks[bestMatch]->good_plane_indice);    
	  }
      }
      return make_tuple(bestMatch, bestReprojError);
}


// returns the <id, matcherror> of the best matching plane, or -1 if no match found.  naive association based on geometric distance.  
std::tuple<int, double> Mapper_mono::findClosestPlane(const Pose3d& curr_esti_pose, const Map_plane* check_plane) {
      int numMatches = 0;
      int bestMatch = -1;
      double bestDist = -1;
      double bestReprojError = -1;

      const Plane3d curr_plane_local=check_plane->temp_value;
      
      Plane3d curr_plane_world = curr_plane_local.transform_from(curr_esti_pose.oTw());  //  transform plane to global coordinate      
      Eigen::Matrix4d curr_plane_pose_mat= curr_esti_pose.wTo();
      Eigen::MatrixXf curr_bounding_2d_poly=check_plane->plane_bound_close_2D_polys;
      Eigen::MatrixXf curr_bounding_3d_poly=check_plane->plane_bound_close_3D_polys;
//       cout<<"curr 1 "<<curr_bounding_3d_poly.col(0).head(2).transpose()<<endl;
//       cout<<"curr 2 "<<curr_bounding_3d_poly.col(1).head(2).transpose()<<endl;
//       cout<<"curr normal "<<curr_plane_world.normal().transpose()<<endl;
      int start_plane_id = 0; //std::max((int)_planeNodes.size() - 20,0);
      int end_plane_id = all_landmarks.size();
            
      for (int c = start_plane_id; c < end_plane_id; c++) {     // could speed up, only use recent frames
	    if (all_landmarks[c]->deteted_by_merge)  // if it is a dead node, being merged node, skip this
		continue;
	    if ( check_plane->frame_plane_indice==0 && all_landmarks[c]->frame_plane_indice==0 ){  // if both are ground, associate them immediately.
		  numMatches++;
		  bestMatch = c;
		  break;
	    }
	    if ( ( check_plane->frame_plane_indice==0 && all_landmarks[c]->frame_plane_indice>=1 )  ||    // if one is ground and the other is wall, must not be associated
		  ( check_plane->frame_plane_indice>=1 && all_landmarks[c]->frame_plane_indice==0 ) )
		  continue;   // skip this plane, check further plane
	    
	    Plane3d old_plane_world = all_landmarks[c]->plane_vertex->value();
	    Plane3d old_plane_local;
	    if (useRelative) {
		old_plane_world = old_plane_world.transform_from(all_landmarks[c]->plane_vertex->base()->value().oTw());
	    }
	    old_plane_local = old_plane_world.transform_to(curr_esti_pose.wTo());

	    Eigen::MatrixXf old_bounding_3d_poly=all_landmarks[c]->plane_bound_close_3D_polys;
	    
	    if (check_plane->frame_seq_id-all_landmarks[c]->frame_seq_id>assoc_near_frames)  // only recent several frames
		continue;
	    // first check their plane angle distance, NOTE  that this actually different left wall and right wall, and front wall 
	    double angle_diff_normal=acos(curr_plane_world.normal().dot(old_plane_world.normal()))*180.0/PI; // 0~pi
	    if (print_assoc_detail){
		cout<<"To compare frame  "<< all_landmarks[c]->frame_seq_id <<"  plane indice  "<<all_landmarks[c]->good_plane_indice<<endl;
		cout<<"angle_diff_normal "<<angle_diff_normal<<endl;
	    }
	    
	    if (angle_diff_normal>edge_asso_angle)  // angle too big difference, don't associate
		continue;

	    double actual_2d_error_thre=edge_asso_2ddist;
	    double actual_covering_thre=edge_asso_proj;	      
	    if (angle_diff_normal<25.0){   // if angle very similiar, reduce the threshold of 2d image distance		  
		actual_covering_thre=edge_asso_proj/3;
		actual_2d_error_thre=edge_asso_2ddist*1.5;
		if (angle_diff_normal<=10.0){
		  actual_covering_thre=edge_asso_proj/2;
		}
	    }
	    
	    // calculate distance to plane; this assumes that normals are similar
	    double plane_dist = curr_plane_local.distance(old_plane_local.point0());
	    if (print_assoc_detail){
// 		  cout<<"curr_plane_world pram "<<curr_plane_world<<" old_world_plane_pram "<<old_plane_world<<endl;
// 		  cout<<"curr_plane_local pram "<<curr_plane_local<<" old_local_plane_pram "<<old_plane_local<<endl;
		cout<<"plane dist "<<plane_dist<<endl;
	    }
	    if (plane_dist > edge_asso_planedist)
		  continue;
	    if (plane_dist < 1.5)
		actual_covering_thre=actual_covering_thre/2;
	    
	    // check edges 2d coordinates difference TODO this only works in recent frames as plane_bound_close_2D_polys is always the latest
	    float poly_2d_dist=0;
	    for (int i=0;i<2;i++){  //(double)curr_bounding_2d_poly.cols()  // only need to evaluate ground edges instead of 2d polygons
		  Vector2f distance_all;
		  for (int j=0;j<2;j++)  		// all_landmarks[c]->.plane_bound_close_2D_polys.cols()  could be replaced by reproj_bounding_2d_poly	    
		      distance_all(j)=(curr_bounding_2d_poly.col(i)- all_landmarks[c]->plane_bound_close_2D_polys.col(j)).norm();  //if we use _plane_boundaries directly, plane matching only works for short period	      	    
		  poly_2d_dist+=distance_all.minCoeff();
	    }
	    poly_2d_dist=poly_2d_dist/2;  //(double)curr_bounding_2d_poly.cols()	
	    float poly_2d_dist_cross=0;
	    for (int i=0;i<2;i++){
		  Vector2f distance_all;		  
		  for (int j=0;j<2;j++)  		// could be replaced by reproj_bounding_2d_poly	    
		      distance_all(j)=(curr_bounding_2d_poly.col(j)- all_landmarks[c]->plane_bound_close_2D_polys.col(i)).norm();  //if we use _plane_boundaries directly, plane matching only works for short period
		  poly_2d_dist_cross+=distance_all.minCoeff();
	    }
	    poly_2d_dist_cross=poly_2d_dist_cross/2; // (double)curr_bounding_2d_poly.cols()	
	    
	    if (print_assoc_detail)
		  cout<<"mean 2d error  "<<poly_2d_dist<<"  "<<poly_2d_dist_cross<<"   thre "<<actual_2d_error_thre<<endl;
	    if (poly_2d_dist>actual_2d_error_thre || poly_2d_dist_cross> actual_2d_error_thre)
		  continue;
		    
// 	      projection overlap I only need to compute line projection overlap namely ground segments  
// 	      TODO this also works for recent frames as the the bounding poly  could use history...namely all previous polys
// 	      is always the latest  // more complicated method is to compute polygon overlapping: http://doc.cgal.org/latest/Boolean_set_operations_2/index.html#Chapter_2D_Regularized_Boolean_Set-Operations  	    	      
	    float old_bg_proj_new=point_proj_to_lineseg(curr_bounding_3d_poly.col(0).head(2),curr_bounding_3d_poly.col(1).head(2),old_bounding_3d_poly.col(0).head(2));
	    float old_ed_proj_new=point_proj_to_lineseg(curr_bounding_3d_poly.col(0).head(2),curr_bounding_3d_poly.col(1).head(2),old_bounding_3d_poly.col(1).head(2));
	    float covering_old_to_new=fabs(old_bg_proj_new-old_ed_proj_new);
	    float new_bg_proj_old=point_proj_to_lineseg(old_bounding_3d_poly.col(0).head(2),old_bounding_3d_poly.col(1).head(2),curr_bounding_3d_poly.col(0).head(2));
	    float new_ed_proj_old=point_proj_to_lineseg(old_bounding_3d_poly.col(0).head(2),old_bounding_3d_poly.col(1).head(2),curr_bounding_3d_poly.col(1).head(2));
	    float covering_new_to_old=fabs(new_bg_proj_old-new_ed_proj_old);
	    
	    if (print_assoc_detail)
		cout<<"covering_old_to_new "<<covering_old_to_new<<"  "<<covering_new_to_old<<"  thre "<<actual_covering_thre<<endl;
	    if (covering_old_to_new<actual_covering_thre || covering_new_to_old<actual_covering_thre)  // if very small overlapping, don't associate
		continue;

	    numMatches++;
	    double total_error=angle_diff_normal/edge_asso_angle*3+(1-covering_old_to_new)+(1-covering_new_to_old);
	    total_error+=std::max(poly_2d_dist,poly_2d_dist_cross)/actual_2d_error_thre+plane_dist/4;
// 	    double total_error=std::max(poly_2d_dist,poly_2d_dist_cross)/actual_2d_error_thre;
	    
	    if (print_assoc_detail)
		  cout<<"total score "<<total_error<<endl;
	    if (numMatches==1 || total_error<bestReprojError ){  // the smaller, the better	      
		bestMatch = c;
		bestReprojError = total_error;   // weight combination of angle and 3d projection overlapping
	    }
	    if (print_assoc_detail)	      
		cout<<"add as potential match"<<endl;
        }
      
// 	if (numMatches > 1) {
// 	    cout << "Warning: multiple potential matches for one plane." << endl;
// // 	    cout<<"Warning: multiple potential matches for one plane."<<endl;
// 	}
	
	if (0)
	{
	    if (bestMatch<0){
		ROS_WARN_STREAM("Not finding matched planes, add as new landmark.");    
	    }else{
		ROS_WARN_STREAM("Final matches global at  "<<bestMatch<<"  in frame  "<< all_landmarks[bestMatch]->frame_seq_id <<"  at indice  "<<all_landmarks[bestMatch]->good_plane_indice);    
	    }
	}
	return make_tuple(bestMatch, bestReprojError);
//         return bestMatch;
}


// only one thread, add frame, associate, change graph, then optimize
void Mapper_mono::processFrame(vector<Map_plane> &new_planes, tracking_frame* newframe)
{
      vector<Map_plane> new_planes_mod;
      for (int jj=0;jj<new_planes.size();jj++)  // start from 1, if want to skip ground
	  new_planes_mod.push_back(new_planes[jj]);
      //NOTE In current code, if don't use ground, frame still stores ground plane polygons, but won't have plane node. so attention when using them...
      //eg: in  pub_all_hist_frame_planes_poly()  update_plane_measurement()  reproj_to_newplane() ...  
      //for (int plane_id=1;....)   plane_id-1;
	      
//       cout<<"process frame plane size   "<<new_planes_mod.size()<<endl;
      _tic();
      // data association
      Pose3d estimate_pose;  //get the latest isam estimates of pose, assumed to be the place where planes measurements come from. if isam is fast, assumed right.
      if (all_frames.size() > 0) {
	  estimate_pose = (all_frames.back()->pose_vertex->value()).oplus(newframe->temp_pose);
      }
      int numOldPlanes = all_landmarks.size();
      int numNewPlanes = 0;
      vector<int> assoc(new_planes_mod.size());   // find the association id.
      vector<double> assoc_error(new_planes_mod.size());  
      int id=-1;
      double error=-1;      
      for (int i = 0; i < new_planes_mod.size(); i++) {
// 	  cout<<"To match plane---------------------------  "<<i<<"  at  frame "<<new_planes_mod[i].frame_seq_id<<endl;
	  std::tie(id, error) = findClosestPlane(estimate_pose, &new_planes_mod[i]);
	  
	  if (id > -1) {  //if found match, check some conditions
	      bool conflict=false;  // force one-to-one match, not allow two-to-one match
	      for (int j=0;j<i;j++){
		  if (assoc[j]==id){
		      double raw_length=(new_planes_mod[j].plane_bound_close_2D_polys.col(0)-new_planes_mod[j].plane_bound_close_2D_polys.col(1)).norm();
		      double new_length=(new_planes_mod[i].plane_bound_close_2D_polys.col(0)-new_planes_mod[i].plane_bound_close_2D_polys.col(1)).norm();
		      if (new_length>raw_length){  // new is better, replace old, set as new plane    error<assoc_error[j]
			  assoc[j]=numOldPlanes + numNewPlanes;
			  numNewPlanes++;
			  assoc_error[j]=-1;
			  assoc[i]=id;
			  assoc_error[i]=error;
			  ROS_WARN_STREAM("Dual match, add new landmark  "<<j<<"  at frame  "<<new_planes_mod[j].frame_seq_id);
		      }
		      else{   // Old is better, add new plane as new landmark
			  assoc[i] = numOldPlanes + numNewPlanes;
			  numNewPlanes++;
			  assoc_error[i] = 0.0;
			  ROS_WARN_STREAM("Dual match, add new landmark  "<<i<<"  at frame  "<<new_planes_mod[i].frame_seq_id);
		      }
		      conflict=true;
		      break;
		    }
	      }
	      if (conflict == false) {  // single-match
		  assoc[i] = id;
		  assoc_error[i] = error;
	      }
	  }
	  else {
	    assoc[i] = numOldPlanes + numNewPlanes; // no match, later create new plane, add to the map
	    numNewPlanes++;
	  }
      }
      _toc("data association");
      
      // add new pose node
      Pose3d_Node* poseNode = new Pose3d_Node();   
      _slam.add_node(poseNode);
      newframe->pose_vertex=poseNode;
      all_frames.push_back(newframe);
      
      // add pose constraints
      if (all_frames.size() == 1) {       // if this is the first node, set as origin      
	    Pose3d origin=newframe->temp_pose;   // prior on first pose only   first odometry (I set in main.cpp) is initial pose
	    Pose3d_Factor* pose_prior = new Pose3d_Factor(poseNode, origin, *poseCov);
	    _slam.add_factor(pose_prior);
      } else {   // else, add odometry relative measurements
	    poseNode->init(estimate_pose);   // initialize pose, not as important as odometry constraints
	    // could use some hacks to change the posecov, see previous nsh2 commits
	    Pose3d_Pose3d_Factor* constraint = new Pose3d_Pose3d_Factor(all_frames[all_frames.size()-2]->pose_vertex, poseNode, newframe->temp_pose,*poseCov);   //pose1, pose2, measure (odometry), noise   
	    _slam.add_factor(constraint);
      }      
      
      // add new planes node
      for (int i = 0; i < numNewPlanes; i++) {
	    Plane3d_Node* planeNode = new Plane3d_Node();
	    _slam.add_node(planeNode);
	    
	    Map_plane* new_landmark= new Map_plane();
	    new_landmark->plane_vertex=planeNode;
	    all_landmarks.push_back(new_landmark);
	    new_landmark->landmark_index=all_landmarks.size()-1;    
      }
      
      // add plane-pose constraints
      for (int i = 0; i < assoc.size(); i++){  // for all the planes observation in this frame
	  isam::Plane3d_Node* planeNode = all_landmarks[assoc[i]]->plane_vertex;
	  isam::Plane3d measure = new_planes_mod[i].temp_value;    // plane measurements come from current frame's segmentation.	  
	  if (assoc[i]>numOldPlanes-1){  // if it is added as a new plane, initialize the plane's pose	  
		Plane3d p = measure.transform_from(estimate_pose.oTw());  // transform to global frame
  // 	      p.plane_type=new_planes_mod[i].frame_plane_indice; // want to reduce plane dof, should not use it. will make the information matrix invertible, not positive definite.
		planeNode->init(p);   //initialize plane position
		if (new_planes_mod[i].frame_plane_indice==0){  //if ground change to anchor node, not optimized
		    Plane3d origin(Vector4d(0,0,-1,0));    // gronud plane is ideally (0,0,-1,0)
		    Plane3d_Factor *ground_prior=new Plane3d_Factor(planeNode, origin, *planeCov); // add a prior for ground plane		    
		    _slam.add_factor(ground_prior);
		}
	  }

	  double plane_dist=new_planes_mod[i].plane_dist_cam; //TODO variance is related to depth, could also be related to wide. the wider, the longer edges, the less covariance
	  plane_dist=plane_dist<3?3:plane_dist;
	  plane_dist=plane_dist>8?8:plane_dist;  // cut into [1,6]m
	  planeSigmas<<(plane_dist-1)*plane_sigma_dist_mul+5,(plane_dist-1)*plane_sigma_dist_mul+5,(plane_dist-1)*plane_sigma_dist_mul+5;  // nsh 3, tum 2

	  Covariance planeCov_new(planeSigmas.cwiseProduct(planeSigmas).asDiagonal());
	  Pose3d_Plane3d_Factor* fac = new Pose3d_Plane3d_Factor(poseNode,planeNode, measure, planeCov_new, useRelative);
	  
	  // NOTE if want to pop up plane from lines in each iteration!!!  to use it: comment some parts in isam_plane3d.h and don't use ground planes
	  // but it is sensitive if there is bad object association.... not good on this dataset.
	  // results: it is not good on this dataset.
// 	  Eigen::MatrixXf plane_ground_2d_edge(1,4);
// 	  plane_ground_2d_edge.row(0).head<2>() =  new_planes_mod[i].plane_bound_close_2D_polys.col(0).head<2>();
// 	  plane_ground_2d_edge.row(0).tail<2>() =  new_planes_mod[i].plane_bound_close_2D_polys.col(1).head<2>();
// 	  fac->precompute_edge_ray(inv_calib,plane_ground_2d_edge);
	  
	  _slam.add_factor(fac);
	  
	  // update using the latest plane property, not landmark_index
	  copy_plane(all_landmarks[assoc[i]],&(new_planes_mod[i]));	  
	  
	  newframe->observed_planes.push_back(all_landmarks[assoc[i]]);
	  newframe->pose_plane_facs.push_back(fac);
      }
      
       // detect loop closing
      bool loop_success=false;
      if (use_loop_close){
	  newframe->compute_frame_BoW(mpORBVocabulary, mpORBextractor); // compute Bag of words
	  newframe->bow_score_to_origin = mpORBVocabulary->score(newframe->mBowVec,all_frames[0]->mBowVec);  // the higher, the more similiar,   
	  //TODO now I only detect loop with first frame. actually we should compare with all previous frames. see orbslam for more details.
	  // successive KF image might by 0.1~0.15  quite different image is around 0.001.  a little similar image is 0.01-0.09
	  cout<<"orb similarity score---------------- "<<newframe->bow_score_to_origin<<endl;
	  float loop_closing_thre=0.01;  // here I detect three successive frames, more clever method is to follow orbslam, to use covisity graph.      
	  if (newframe->frame_seq_id>100 && newframe->bow_score_to_origin>0.01 && all_frames[all_frames.size()-2]->bow_score_to_origin>0.01 &&all_frames[all_frames.size()-3]->bow_score_to_origin>0.01 ){
	      ROS_ERROR("begin loop closing !!!!!!!!!!!");
	      loop_success=loopclose_merge(all_frames.size()-1);  // if detect close loop, then re-research the global candidate, merge them
	  }
      }
      
      cout<<"BA num of Poses/planes/nodes/factors:   "<<all_frames.size()<<"  "<<all_landmarks.size()<<"   "<<_slam.get_nodes().size()<<"   "
	  <<"  "<<_slam.get_factors().size()<<endl;

      _tic();
      if ( (new_planes_mod[0].frame_seq_id % 5==0) || loop_success)
	  int iterations=_slam.batch_optimization();
      else
	  _slam.update();  //incremental update
      
            
      // draw information matrix pattern
//       if (all_frames.size()==300)
//       {
// 	    ROS_ERROR("save eps");
// 		    
// 	    SparseSystem  fat_mat= _slam.get_R();
// 	    string pattern_img=package_path+"/pattern_with_g.eps";
// 	    fat_mat.save_pattern_eps(pattern_img);
// 	    fat_mat.print_stats();
// 	
// 	    _slam.remove_node(all_landmarks[0]->plane_vertex);
// 	    cout << "New Number of nodes: " << _slam.get_nodes().size() << endl;
// 	    cout << "New Number of factors: " << _slam.get_factors().size() << endl;
// 	    
// 	    int iterations=_slam.batch_optimization();
// 	    
// 	    SparseSystem  fat_mat2= _slam.get_R();
// 	    string pattern_img2=package_path+"/pattern_no_g.eps";
// 	    fat_mat2.save_pattern_eps(pattern_img2);
// 	    fat_mat2.print_stats();
//       }


	_toc("SLAM");

//       if (last_frame_process){
	reproj_to_newplane();
      
	cout<<"mapping: finish processing"<<endl;
}

//   TODO if want to optimize pose (especially orientation), need to reproject planes. pose x,y  doesn't change plane measurements
//      update measurements using the latest pose. to reproject again, computation is low, because I didn't change cloud here. otherwise it might by heavy
void Mapper_mono::update_plane_measurement()
{      
//      return;

// 	ca::Profiler::tictoc("update plane measurements");
	MatrixX4f all_planes_sensor_new;
	isam::Plane3d plane_measurement_new;
	for (const tracking_frame* one_frame: all_frames){
	    Matrix4d latest_pose = one_frame->pose_vertex->value().wTo();  // reproject to get plane measurements
	    Matrix4f pose=latest_pose.cast<float>();
	    popup_plane::update_plane_equation_from_seg(one_frame->ground_seg2d_lines,inv_calib,pose, all_planes_sensor_new);
	    for (int plane_id=0; plane_id<one_frame->good_plane_indices.rows();plane_id++) {
		plane_measurement_new=isam::Plane3d(all_planes_sensor_new.row(one_frame->good_plane_indices(plane_id)).cast<double>());
		one_frame->pose_plane_facs[plane_id]->set_measurement(plane_measurement_new);  // update measurements in slam factor
	    }
	}
// 	ca::Profiler::tictoc("update plane measurements");	
}

void Mapper_mono::reproj_to_newplane()   //TODO now I reproject everything, later I could reproject only recent frames to save time
{
      for (int i=0;i<all_landmarks.size();i++){
	  // TODO reproject all the plane vertex, now I forget previous slam projected bouding polygon, only use latest descriptors, 
	  // project a better way is to merge the polygons
	  if (all_landmarks[i]->deteted_by_merge)  // if it is a dead node, being merged node, skip this
	      continue;
	  for (int pt=0;pt<all_landmarks[i]->plane_bound_close_3D_polys.cols();pt++){
	      Vector3d updated_polyvertex=all_landmarks[i]->plane_vertex->value().project_to_plane(all_landmarks[i]->plane_bound_close_3D_polys.col(pt).cast<double>());
	      all_landmarks[i]->plane_bound_close_3D_polys.col(pt)=updated_polyvertex.cast<float>();
	  }
      }
      // could skip this step to speed up, just for visualization
      for (int frame_ind=0;frame_ind<all_frames.size();frame_ind++) {
	  //?skip the first groud plane, not used in slam anymore
	  for (int plane_ind=0;plane_ind<all_frames[frame_ind]->all_3d_bound_polygons_good_world.size();plane_ind++){
	       Plane3d curr_plane=all_frames[frame_ind]->observed_planes[plane_ind]->plane_vertex->value();
	      for (int pt=0;pt<all_frames[frame_ind]->all_3d_bound_polygons_good_world[plane_ind].cols();pt++){		    
		    Vector3d updated_polyvertex=curr_plane.project_to_plane(all_frames[frame_ind]->all_3d_bound_polygons_world_raw[plane_ind].col(pt).cast<double>());
		    all_frames[frame_ind]->all_3d_bound_polygons_good_world[plane_ind].col(pt)=updated_polyvertex.cast<float>();
	      }
	  }
      }
}


// loop close for one frame, merge all the planes in that frame, NOTE!!! I didn't delete the node, I just set the descriptor's state as deleted
// so this doesn't change the landmark index
bool Mapper_mono::loopclose_merge(int to_merge_frame_ind) 
{  
      bool really_merged=false;
      tracking_frame* to_merge_frame=all_frames[to_merge_frame_ind];
      int global_asso_id = -1;
      double error = -1.0;
      vector<int> removed_idxs;
      vector<int> assoc_closing_idxs;
      vector<Pose3d_Plane3d_Factor*> to_delete_facs;
      vector<Pose3d_Plane3d_Factor*> to_add_facs;
      
      // containing  frame_id  curr_plane_ind, to_plane_id, error;  initialize all -1
//       MatrixXd loop_cond_matrix=-1* (MatrixXd::Ones(to_merge_frame->observed_planes.size(),4));
      
      for (int i = 0; i < to_merge_frame->observed_planes.size(); ++i) {
	  int raw_plane_index_all=to_merge_frame->observed_planes[i]->landmark_index;  // this is current association id, I need to shift to actual loop closing ones.
	  cout<<"Might search loop close for plane----------------  "<<i<<"  at  frame "<<to_merge_frame->frame_seq_id<<endl;
	  if (raw_plane_index_all<30)  // HACK in nsh2 if it is already associated to beginning loop node, don't need to merge
	      continue;
	  
	  cout<<"Really to find global matches"<<endl;	  
	  // HACK  now always associate to first frame... later, we could search the similar frame id using DetectLoopCandidates, see orb slam for more details
	  std::tie(global_asso_id, error) = findLoopPlane(to_merge_frame->pose_vertex->value(), all_landmarks[raw_plane_index_all], 0); 
	  
	  // loop close with first frame
	  if (global_asso_id >= 0 && global_asso_id!=raw_plane_index_all) {   // if not the same plane
	      
	      list<Factor*> move_facs = all_landmarks[raw_plane_index_all]->plane_vertex->factors();
	      for (auto it : move_facs) {
		    Pose3d_Plane3d_Factor* old_fac = dynamic_cast<Pose3d_Plane3d_Factor*>(it);
		    Pose3d_Node* pose_node=dynamic_cast<Pose3d_Node*>(old_fac->nodes()[0]);  // cast the base class pointer to the inherited class
		    Pose3d_Plane3d_Factor* new_fac = new Pose3d_Plane3d_Factor(pose_node, all_landmarks[global_asso_id]->plane_vertex, old_fac->measurement(), old_fac->noise(), useRelative);
		    to_delete_facs.push_back(old_fac);
		    to_add_facs.push_back(new_fac);
		    
		    _slam.add_factor(new_fac);
		    _slam.remove_factor(old_fac);  // the factor is still there, not deallocated
	      }
	      removed_idxs.push_back(raw_plane_index_all);
	      assoc_closing_idxs.push_back(global_asso_id);
	      really_merged=true;
	      ROS_WARN_STREAM("loop close merge " << raw_plane_index_all << " into " << global_asso_id);
	  }
      }

      if (removed_idxs.size()>0)  // change all_frames's factors information, and frame index, need this to update factors (update measurements)
	  for (int i=0;i<removed_idxs.size();i++)
	      for (tracking_frame* & one_frame: all_landmarks[removed_idxs[i]]->observer_frames)
		  for (int plane_id=0;plane_id<one_frame->observed_planes.size();plane_id++)
		  {
		      for (int fac_id=0;fac_id<to_delete_facs.size();fac_id++)
			  if (one_frame->pose_plane_facs[plane_id]==to_delete_facs[fac_id])
			      one_frame->pose_plane_facs[plane_id]==to_add_facs[fac_id];

		      if (one_frame->observed_planes[plane_id]->landmark_index==removed_idxs[i])
			  one_frame->observed_planes[plane_id]=all_landmarks[assoc_closing_idxs[i]];
		  }
      
      if (removed_idxs.size()>0)   // Replace old descriptor with the new descriptor, delete node
	  for (int i=0;i<removed_idxs.size();i++){
		copy_plane(all_landmarks[assoc_closing_idxs[i]],all_landmarks[removed_idxs[i]]);  // will replace timestamps..
		all_landmarks[removed_idxs[i]]->deteted_by_merge=true;
		_slam.remove_node(all_landmarks[removed_idxs[i]]->plane_vertex);
		delete all_landmarks[removed_idxs[i]]->plane_vertex;     // note this is deallocation, not delete vector element, didn't change vector size
	  }      
      
      return really_merged;
}