/*
 * Plane3d.h
 *
 *  Created on: Feb 12, 2014   //most important cpp, define 3D plane vertex, and pose-plane edge (namely landmark measurements)
 *      Author: kaess
 */

#pragma once

#include <Eigen/Dense>

#include <boost/math/special_functions/sinc.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <valarray>

#include <isam/isam.h>

//#define WRONG  // use 4-vector for optimization
//#define WRONG2 // use 4-vector for error evaluation (also change sigmas 3d/4d)
//#define prior_geometry // 

#include <iostream>

namespace isam {


class Plane3d {
private:
      friend std::ostream& operator<<(std::ostream& out, const Plane3d& p) {
	p.write(out);
	return out;
      }

      Eigen::Vector4d _abcd;	//plane itself is a 4D unit vector  head is normal, last is distance   // _abcd is the homogeneous plane parameters directly. namely pi*x=0
      // normalize the homogeneous vector
      void _normalize() {
	_abcd.normalize();
      }

public:
      int plane_type;        // by me :  to use prior geometry   0 is ground, >=1 is wall  -1 is arbitrary plane  ground is always (0,0,1,0), wall is always (a,b,0,d)
    #ifdef WRONG
      static const int dim = 4; // wrong DoF
    #else
      static const int dim = 3; // 3 DoF
    #endif
      static const int size = 4; // 4 params
      static const char* name() {
	return "Plane3d";
      }
      Plane3d() : _abcd(1.,0.,0.,0.) {plane_type=-1;}
      Plane3d(const Eigen::Vector3d& normal, double dist) {
	Eigen::Vector3d nn = normal.normalized();
	_abcd.head(3) = nn;
	_abcd(3) = -dist;
	_normalize();
	plane_type=-1;
      }
      Plane3d(const Eigen::Vector4d& vec) {
    //     if (vec(3)>0)
    //        _abcd=-vec;
    //     else
	  _abcd=vec;  // HACK in my plane pop up, I make vec(1-3) be the actual normal direction in world., so the last element is always negative
	
	_normalize();
	plane_type=-1;}

      Eigen::VectorXb is_angle() const {
	Eigen::VectorXb isang (dim);
	isang << false, false, false, false;
	return isang;
      }
      
      Eigen::Quaterniond toQuaternion() const {		//add by me.
	return Eigen::Quaterniond(_abcd[3],_abcd[0],_abcd[1],_abcd[2]);  // quaterniond initialize as (qw,qx,qy,qz)
      }
      
      static Eigen::Quaterniond delta3_to_quat(const Eigen::Vector3d& delta) {
	double theta = delta.norm();
    //     if (theta > M_PI) {std::cout << "t=" << theta << " delta3_to_quat error\n"; exit(1);} // original: has this line TODO ?????
    #if 0
	double S;
	if (theta < 0.0001) { // sqrt4(machine precession)
	  S = 0.5 + theta*theta/48.;
	} else {
	  S = sin(0.5*theta)/theta;
	}
    #else
	double S = 0.5 * boost::math::sinc_pi(0.5*theta); // 0.5 from sinc!
    #endif
	double C = cos(0.5*theta);
	return Eigen::Quaterniond(C, S*delta(0), S*delta(1), S*delta(2)); //quaternion is initialized as (qw,qx,qy,qz)
      }

      /**  most important
      * Updates the homogeneous plane representation (4 parameters) using
      * a minimal parametrization (3DoF).
      * @param delta Vector containing update.
      * @return Plane updated by increment delta.
      */
      Plane3d exmap_3dof(const Eigen::Vector3d& delta) const {
    #if 1
      #if 0
	  // using isam::Rot3d.exmap
	  Rot3d r(Eigen::Quaterniond(_abcd));
	  Eigen::Quaterniond q = r.exmap(delta).quaternion();
      #else
	  Eigen::Quaterniond q = delta3_to_quat(delta) *  this->toQuaternion(); //original: Eigen::Quaterniond(_abcd); // TODO is this order right?
      #endif
    #else
	double theta = delta.norm();
	Eigen::Quaterniond dquat = (theta<1e-10)?Eigen::Quaterniond():Eigen::Quaterniond(Eigen::AngleAxisd(theta, delta.normalized()));
	Eigen::Quaterniond q = dquat * Eigen::Quaterniond(_abcd);
    #endif

      // TODO this is actually wrong, reduce the dimension, the information matrix wil not be positive definite.   The only method is to redefine a plane. composed of two degrees of free or fixed.
      if (plane_type==0)
	q=Eigen::Quaterniond(0,0,0,1);              // set the ground
      else if (plane_type>=1)
	q=Eigen::Quaterniond(q.w(),q.x(),q.y(),0);  // set the plane    

    //    std::cout<<"updated quate  "<<q.coeffs().transpose()<<std::endl;
	
	Plane3d newPlane=Plane3d(q.coeffs());   //q.coeffs() gives out x,y,z,w     set quaternion should be w,x,y,z     our plane is parameterized as x,y,z,w
	newPlane.plane_type=this->plane_type;
	return newPlane;   //quaternion.coeffs is ordered as (qx,qy,qz,qw)    
      }

    #ifdef WRONG
      Plane3d exmap(const Eigen::Vector4d& delta) const {
	return Plane3d(vector()+delta);
      }
    #else
      Plane3d exmap(const Eigen::Vector3d& delta) const {   //actually a virtual function
	return exmap_3dof(delta);
      }
    #endif

      Eigen::Vector4d vector() const {
	return Eigen::Vector4d(_abcd);
      }

      void set(const Eigen::Vector4d& v) {
	_abcd = v;
	_normalize();
      }

      // normal of n*x=d parameterization in my pop up, I make the normal always point outwards from the robot
      Eigen::Vector3d normal() const {
	return _abcd.head(3).normalized();
      }

      // d of n*x=d parameterization
      double d() const {
	return - _abcd(3) / _abcd.head(3).norm();
      }

      // distance from origin   always positive.
      double distance() const {
	return fabs(d());
      }

      // point on the plane that is closest to the origin
      Eigen::Vector3d point0() const {
	return d() * normal();
      }
      
      double distance(Eigen::Vector3d point) const {
	return fabs(normal().dot(point-point0()));
      }
	
      Eigen::Vector3d project_to_plane(Eigen::Vector3d point) const
      {
	  Eigen::Vector3d plane_normal=normal();    
    //       return point-plane_normal*(plane_normal.dot(point-plane_normal*d()));
	  return point-plane_normal*(plane_normal.dot(point)-d());
      }
      
      //me: transform a global plane here, to the local pose, wTo: cam to world, wTo * local point = global point    wTo^(T) * global plane = local plane  
      Plane3d transform_to(Eigen::Matrix4d wTo_pose) const {	
	return Plane3d(wTo_pose.transpose() * vector()); // wTo_pose.transpose() * vector() might not be unit length
      }
      
      //me: transform a local plane here, to the global, oTw_pose: world to cam,  oTw*global_point=local_point, so it becomes wTo^(-T) *local plane=global plane 
      // namely   oTw^(T) * local plane = global plane
      Plane3d transform_from(Eigen::Matrix4d oTw_pose) const {
	return Plane3d(oTw_pose.transpose() * vector());
      }

      void write(std::ostream &out) const {
	out << "(" << _abcd(0) << ", " << _abcd(1) << ", " << _abcd(2) << "; " << _abcd(3) << ")";
      }
};


// iSAM node for plane   a vertex for plane3d
class Plane3d_Node : public NodeT<Plane3d> {
private:
  Pose3d_Node* _base;

public:  
  
  Plane3d_Node() : _base(NULL) {}
  Plane3d_Node(const char* name) : NodeT<Plane3d>(name), _base(NULL) {}

  ~Plane3d_Node() {}

  void set_base(Pose3d_Node* base) { _base = base; }
  Pose3d_Node* base() { return _base; }
};


#ifdef WRONG2
 const int MEASURE_SIZE = 4;
#else
 const int MEASURE_SIZE = 3;
#endif


// The actual measurement of a plane from a camera/pose,  an edge between pose3d to plane3d
class Pose3d_Plane3d_Factor : public FactorT<Plane3d> {
private:
  Pose3d_Node* _pose;  // both _pose and _plane are the global coordinates
  Plane3d_Node* _plane;
  bool _relative;
  Pose3d_Node* _base;
//   Pose3d_Node* _;

public:
  Pose3d_Plane3d_Factor(Pose3d_Node* pose, Plane3d_Node* plane,
                        const Plane3d& measure, const Noise& noise,
                        bool relative = false)
    : FactorT<Plane3d>("Pose3d_Plane3d_Factor", MEASURE_SIZE, noise, measure), _pose(pose), _plane(plane), _relative(relative), _base(NULL) {
    _nodes.resize(2);
    _nodes[0] = pose;
    _nodes[1] = plane;
    // for relative parameterization recover base pose
    if (_relative) {
      if (!plane->factors().empty()) {
        _nodes.resize(3);	// add a new factor node.
        _nodes[2] = plane->factors().front()->nodes()[0];  //a factor's first node is pose
        // TODO: first factor might refer to a prior or other type of node...
        _base = dynamic_cast<Pose3d_Node*>(_nodes[2]);
      } else {
        // first factor: base is this pose, also init base for plane
        _base = _pose;
      }
      plane->set_base(_base);
    }
  }

  void initialize() {
    require(_pose->initialized(), "Plane3d: Pose3d_Plane3d_Factor requires pose to be initialized");
    if (!_plane->initialized()) {
      const isam::Pose3d& p = _pose->value();
      Plane3d predict;
      if (_relative) {
        predict = _measure;
      } else {
        predict = _measure.transform_from(p.oTw());
      }
      _plane->init(predict);
    }
  }

  /**  most important: log map. change plane measurements to error vector.
   * Error between the predicted and measured planes (orientation and distance)
   * @param s Determines if error based on estimate or linearization point.
   * @return Error vector of size 3.
   */
  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    // calculate error in tangent space
    // TODO: transform actual measure covariance into tangent space
    Pose3d pose = _pose->value(s);
    const Plane3d& global_plane = _plane->value(s);
    if (_base) {
      // pose of camera relative to base camera (which might be this one!) TODO
      pose = pose.ominus(_base->value(s));
    }
    Plane3d local_plane = global_plane.transform_to(pose.wTo());  //first transform the global plane to local plane

#ifdef WRONG2
    Eigen::Vector4d error = local_plane.vector() - _measure.vector();   // wrong way, use four dimensional error vector.
#else
    // logmap
    Eigen::Quaterniond q(local_plane.vector());
    Eigen::Quaterniond q_measured(_measure.vector());  //_measure is set by each frame's plane measurements.
    Eigen::Quaterniond dq = q * q_measured.conjugate();	//conjugate() is the inverse(), if the quaternion is a unit quaternion, in our case it is true.
#if 1
    Eigen::AngleAxisd aa(dq);
    if (aa.angle() > M_PI) aa.angle() -= 2.*M_PI;  // >-pi
    if (aa.angle() < -M_PI) aa.angle() += 2.*M_PI; // < pi
    if (aa.angle() > M_PI || aa.angle() < -M_PI) std::cout << "error in basic_error" << std::endl;
    Eigen::Vector3d error = aa.axis() * aa.angle();  // TODO, why exactly? change error angle to (-pi,pi), because original axis/angle range is (0,2*pi)
#else
    // own logmap
    Eigen::Vector3d qv(dq.x(), dq.y(), dq.z());
    Eigen::Vector3d error = (qv.norm()<1e-8) ? Eigen::Vector3d() : 2.*acos(dq.w())*qv/qv.norm();
    if (dq.w() < -M_PI || dq.w() > M_PI) std::cout << "logmap issue " << error << std::endl;	//TODO what's the errror, and how to deal with it?
#endif
#endif

    return error;
  }
  
  //if want to manually derive the jacobian
  // Jacobian jacobian()
};


void get_wall_plane_equation(const Eigen::MatrixXd& ground_seg_ray, const Eigen::Matrix4d& transToWorld, Eigen::MatrixXd& all_planes_sensor_out);  

// The actual measurement of a plane from a camera/pose,  an edge between pose3d to plane3d
class Pose3d_Plane3d_Factor2 : public FactorT<Plane3d> {
private:
  Pose3d_Node* _pose;  // both _pose and _plane are the global coordinates
  Plane3d_Node* _plane;
  Eigen::MatrixXd ground_edge_ray; // actually 3*2
  bool _relative;
  Pose3d_Node* _base;
//   Pose3d_Node* _;

public:
  Pose3d_Plane3d_Factor2(Pose3d_Node* pose, Plane3d_Node* plane,
                        const Plane3d& measure, const Noise& noise,
                        bool relative = false)
    : FactorT<Plane3d>("Pose3d_Plane3d_Factor", MEASURE_SIZE, noise, measure), _pose(pose), _plane(plane), _relative(relative), _base(NULL) {
    _nodes.resize(2);
    _nodes[0] = pose;
    _nodes[1] = plane;
    // for relative parameterization recover base pose
    if (_relative) {
      if (!plane->factors().empty()) {
        _nodes.resize(3);	// add a new factor node.
        _nodes[2] = plane->factors().front()->nodes()[0];  //a factor's first node is pose
        // TODO: first factor might refer to a prior or other type of node...
        _base = dynamic_cast<Pose3d_Node*>(_nodes[2]);
      } else {
        // first factor: base is this pose, also init base for plane
        _base = _pose;
      }
      plane->set_base(_base);
    }
  }

  void initialize() {
    require(_pose->initialized(), "Plane3d: Pose3d_Plane3d_Factor requires pose to be initialized");
    if (!_plane->initialized()) {
      const isam::Pose3d& p = _pose->value();
      Plane3d predict;
      if (_relative) {
        predict = _measure;
      } else {
        predict = _measure.transform_from(p.oTw());
      }
      _plane->init(predict);
    }
  }
  
  // call function before using this factor!!! not for ground plane
  // ground_seg2d_lines n*4, here should be 1*4    must be called before optimization, doesn't depend on pose.
  void precompute_edge_ray(const Eigen::Matrix3f& invK, const Eigen::MatrixXf& ground_seg2d_lines)
  {
      Eigen::MatrixXf ground_seg2d_homo(ground_seg2d_lines.rows()*2,3);
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp=ground_seg2d_lines;  // row order for resize
      temp.resize(ground_seg2d_lines.rows()*2,2);  //2n*2
      ground_seg2d_homo<<temp.array(),Eigen::VectorXf::Ones(ground_seg2d_lines.rows()*2); // 2n*3	    
      ground_edge_ray = (invK*ground_seg2d_homo.transpose()).cast<double>();    //each column is a 3D world coordinate  3*2n
//       std::cout<<"ground_edge_ray    "<<ground_edge_ray<<std::endl;
      
      Pose3d pose = _pose->value(ESTIMATE);
//       std::cout<<"factor pop height!!!   "<<pose.wTo()(2,3)<<std::endl;
  }  
    
  
  /**  most important: log map. change plane measurements to error vector.
   * Error between the predicted and measured planes (orientation and distance)
   * @param s Determines if error based on estimate or linearization point.
   * @return Error vector of size 3.
   */
  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    // calculate error in tangent space
    // TODO: transform actual measure covariance into tangent space
    Pose3d pose = _pose->value(s);
    const Plane3d& global_plane = _plane->value(s);
    if (_base) {
      // pose of camera relative to base camera (which might be this one!) TODO
      pose = pose.ominus(_base->value(s));
    }
    Plane3d local_plane_landmark = global_plane.transform_to(pose.wTo());  //first transform the global plane to local plane
    
    Eigen::MatrixXd updated_local_plane_meas;
    get_wall_plane_equation(ground_edge_ray,pose.wTo(),updated_local_plane_meas);    
    Eigen::Vector4d localplane_param = updated_local_plane_meas.row(0).head<4>();
    localplane_param.normalize();
    Eigen::Quaterniond q_measured(localplane_param);  // NOTE use new method
//     Eigen::Quaterniond q_measured(_measure.vector());  // TEST using old method

#ifdef WRONG2
    Eigen::Vector4d error = local_plane_landmark.vector() - _measure.vector();   // wrong way, use four dimensional error vector.
#else
    // logmap
    Eigen::Quaterniond q(local_plane_landmark.vector());    
    Eigen::Quaterniond dq = q * q_measured.conjugate();	//conjugate() is the inverse(), if the quaternion is a unit quaternion, in our case it is true.
#if 1
    Eigen::AngleAxisd aa(dq);
    if (aa.angle() > M_PI) aa.angle() -= 2.*M_PI;  // >-pi
    if (aa.angle() < -M_PI) aa.angle() += 2.*M_PI; // < pi
    if (aa.angle() > M_PI || aa.angle() < -M_PI) std::cout << "error in basic_error" << std::endl;
    Eigen::Vector3d error = aa.axis() * aa.angle();  // TODO, why exactly? change error angle to (-pi,pi), because original axis/angle range is (0,2*pi)
#else
    // own logmap
    Eigen::Vector3d qv(dq.x(), dq.y(), dq.z());
    Eigen::Vector3d error = (qv.norm()<1e-8) ? Eigen::Vector3d() : 2.*acos(dq.w())*qv/qv.norm();
    if (dq.w() < -M_PI || dq.w() > M_PI) std::cout << "logmap issue " << error << std::endl;	//TODO what's the errror, and how to deal with it?
#endif
#endif

    return error;
  }
  
  //if want to manually derive the jacobian
  // Jacobian jacobian()
};


// a prior pose for plane3d
class Plane3d_Factor : public FactorT<Plane3d> {
  Plane3d_Node* _plane;
public:
  /**
   * Constructor.
   * @param pose The plane node the prior acts on.
   * @param prior The actual prior measurement.
   * @param noise The 4x4 square root information matrix (upper triangular).
   */
  Plane3d_Factor(Plane3d_Node* plane, const Plane3d& prior, const Noise& noise)
    : FactorT<Plane3d>("Pose3d_Factor", MEASURE_SIZE, noise, prior), _plane(plane) {
    _nodes.resize(1);
    _nodes[0] = plane;
  }

  void initialize() {
    if (!_plane->initialized()) {
      Plane3d predict = _measure;
      _plane->init(predict);
    }
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    const Plane3d& pl = _plane->value(s);
    #ifdef WRONG2
	Eigen::Vector4d error = pl.vector() - _measure.vector();   // wrong way, use four dimensional error vector.
    #else
	// logmap
	Eigen::Quaterniond q(pl.vector());
	Eigen::Quaterniond q_measured(_measure.vector());  //_measure is set by each frame's plane measurements.
	Eigen::Quaterniond dq = q * q_measured.conjugate();	//conjugate() is the inverse(), if the quaternion is a unit quaternion, in our case it is true.
    #if 1
	Eigen::AngleAxisd aa(dq);
	if (aa.angle() > M_PI) aa.angle() -= 2.*M_PI;  // >-pi
	if (aa.angle() < -M_PI) aa.angle() += 2.*M_PI; // < pi
	if (aa.angle() > M_PI || aa.angle() < -M_PI) std::cout << "error in basic_error" << std::endl;
	Eigen::Vector3d error = aa.axis() * aa.angle();  // TODO, why exactly? change error angle to (-pi,pi), because original axis/angle range is (0,2*pi)
    #else
	// own logmap
	Eigen::Vector3d qv(dq.x(), dq.y(), dq.z());
	Eigen::Vector3d error = (qv.norm()<1e-8) ? Eigen::Vector3d() : 2.*acos(dq.w())*qv/qv.norm();
	if (dq.w() < -M_PI || dq.w() > M_PI) std::cout << "logmap issue " << error << std::endl;	//TODO what's the errror, and how to deal with it?
    #endif
    #endif
    return error;
  }
};



#if 1 // not used (and probably not useful either because a plane graph is not fully constrained
// direct constraint between two planes (e.g. from marginalization of poses)
class Plane3d_Plane3d_Factor : public FactorT<Plane3d> {
private:
  Plane3d_Node* _plane1;
  Plane3d_Node* _plane2;

public:
  Plane3d_Plane3d_Factor(Plane3d_Node* plane1, Plane3d_Node* plane2,
                        const Plane3d& measure, const Noise& noise)  // measure of two planes means what??? vector difference?
    : FactorT<Plane3d>("Plane3d_Plane3d_Factor", MEASURE_SIZE, noise, measure), _plane1(plane1), _plane2(plane2) {
    _nodes.resize(2);
    _nodes[0] = plane1;
    _nodes[1] = plane2;
  }

  void initialize() {
    require(_plane1->initialized(), "Plane3d: Plane3d_Plane3d_Factor requires first plane to be initialized");
    if (!_plane2->initialized()) {
      Plane3d predict(_plane1->value().vector()+_measure.vector());      
      _plane2->init(predict);
    }
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {      
      Plane3d plane_1_val=_plane1->value(s);
      Plane3d plane_2_pred=Plane3d(plane_1_val.vector()+_measure.vector());
      
      Plane3d plane_2_val=_plane2->value(s);
      Eigen::Quaterniond q_plane2_pred(plane_2_pred.vector());
      Eigen::Quaterniond q_plane2_measured(plane_2_val.vector()); 
//       std::cout<<"plane_2_pred "<<plane_2_pred.vector().transpose()<<std::endl;
//       std::cout<<"plane_2_val "<<plane_2_val.vector().transpose()<<std::endl;
      Eigen::Quaterniond dq = q_plane2_pred * q_plane2_measured.conjugate(); 
      
      Eigen::AngleAxisd aa(dq);
      if (aa.angle() > M_PI) aa.angle() -= 2.*M_PI;  // >-pi
      if (aa.angle() < -M_PI) aa.angle() += 2.*M_PI; // < pi
      if (aa.angle() > M_PI || aa.angle() < -M_PI) std::cout << "error in basic_error" << std::endl;
      Eigen::Vector3d error = aa.axis() * aa.angle();  // TODO, why exactly? change error angle to (-pi,pi), because original axis/angle range is (0,2*pi)    
      std::cout<<"error "<<error.transpose()<<std::endl;
      return error;
      
      
//       Eigen::Vector3d qv(dq.x(), dq.y(), dq.z());
//       Eigen::Vector3d error = (qv.norm()<1e-8) ? Eigen::Vector3d() : 2.*acos(dq.w())*qv/qv.norm();
//       if (dq.w() < -M_PI || dq.w() > M_PI) std::cout << "logmap issue " << error << std::endl;	//TODO what's the errror, and how to deal with it?
//       
  }
};
#endif


}
