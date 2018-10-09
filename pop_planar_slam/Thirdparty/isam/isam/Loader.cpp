/**
 * @file Loader.cpp
 * @brief Loading files with constraints/factors.
 * @author Michael Kaess
 * @version $Id: Loader.cpp 6902 2012-06-26 02:43:17Z kaess $
 *
 * Copyright (C) 2009-2013 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson, David Rosen,
 * Nicholas Carlevaris-Bianco and John. J. Leonard
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <iostream>
#include <vector>
#include <map>
#include <list>

#include <isam/slam2d.h>

#include "Loader.h"

// set to true for external data files with other convention
const bool Y_FORWARD = false;

using namespace std;
using namespace isam;
using namespace Eigen;

/**
 * Create first node at origin: we add a prior to keep the
 * first pose at the origin, which is an arbitrary choice.
 */
void Loader::add_prior() {
  _nodes.resize(1);
  _factors.resize(1);
  _pose_mapper.add(0);
  // add 2D or 3D prior
  if (_is_3d) {
    // create first node
    Pose3d pose0;
    Noise noise = SqrtInformation(100. * eye(6));
    Pose3d_Node* new_pose_node = new Pose3d_Node();
    _nodes[_pose_mapper[0]].push_back(new_pose_node);
    if (_verbose) cout << *new_pose_node << endl;
    _pose_nodes.push_back(new_pose_node);
    // create prior measurement
    Pose3d_Factor* prior = new Pose3d_Factor(new_pose_node, pose0, noise);
    _factors[0].push_back(prior);
    if (_verbose) cout << *prior << endl;
  } else {
    Pose2d pose0;
    Noise noise = SqrtInformation(100. * eye(3));
    Pose2d_Node* new_pose_node = new Pose2d_Node();
    _nodes[_pose_mapper[0]].push_back(new_pose_node);
    if (_verbose) cout << *new_pose_node << endl;
    _pose_nodes.push_back(new_pose_node);
    Pose2d_Factor* prior = new Pose2d_Factor(new_pose_node, pose0, noise);
    _factors[0].push_back(prior);
    if (_verbose) cout << *prior << endl;
  }
}

bool Loader::advance(unsigned int idx_x1, unsigned int next_point_id) {
  // advance to next time step if needed
  bool added = _pose_mapper.add(idx_x1);
  if (added ) {
    _step++;
    _nodes.resize(_step+1);
    _factors.resize(_step+1);
    _num_points.resize(_step+1);
    _num_points[_step] = next_point_id;
    _num_constraints.resize(_step+1);
    _num_constraints[_step] = _constraints.size();
    _num_measurements.resize(_step+1);
    _num_measurements[_step] = _measurements.size();
  }
  return added;
}

void Loader::add_odometry(unsigned int idx_x0, unsigned int idx_x1, const Pose2d& measurement, const Noise& noise) {
  if (advance(idx_x1, _point_nodes.size())) {
    Pose2d_Node* new_pose_node = new Pose2d_Node();
    _nodes[_step].push_back(new_pose_node);
    _pose_nodes.push_back(new_pose_node);
    if (_verbose) cout << idx_x1 << " " << *new_pose_node << endl;
  }
  unsigned int i_x0 = _pose_mapper[idx_x0];
  unsigned int i_x1 = _pose_mapper[idx_x1];
  Pose2d_Pose2d_Factor* factor = new Pose2d_Pose2d_Factor(
      dynamic_cast<Pose2d_Node*>(_pose_nodes[i_x0]),
      dynamic_cast<Pose2d_Node*>(_pose_nodes[i_x1]),
      measurement, noise);
  _factors[i_x1].push_back(factor);
  _constraints.push_back(make_pair(i_x0, i_x1));
  _num_constraints[i_x1] = _constraints.size();
  if (_verbose) cout << i_x1 << " " << *factor << endl;
}

void Loader::add_odometry3(unsigned int idx_x0, unsigned int idx_x1, const Pose3d& measurement, const Noise& noise) {
  if (advance(idx_x1, _point_nodes.size())) {
    Pose3d_Node* new_pose_node = new Pose3d_Node();
    _nodes[_step].push_back(new_pose_node);
    _pose_nodes.push_back(new_pose_node);
    if (_verbose) cout << idx_x1 << " " << *new_pose_node << endl;
  }
  unsigned int i_x0 = _pose_mapper[idx_x0];
  unsigned int i_x1 = _pose_mapper[idx_x1];
  Pose3d_Pose3d_Factor* factor = new Pose3d_Pose3d_Factor(
      dynamic_cast<Pose3d_Node*>(_pose_nodes[i_x0]),
      dynamic_cast<Pose3d_Node*>(_pose_nodes[i_x1]),
      measurement, noise);
  _factors[i_x1].push_back(factor);
  _constraints.push_back(make_pair(i_x0, i_x1));
  _num_constraints[i_x1] = _constraints.size();
  if (_verbose) cout << i_x1 << " " << *factor << endl;
}

void Loader::add_measurement(unsigned int idx_x, unsigned int idx_l, const Point2d& measurement, const Noise& noise) {
  if (_point_mapper.add(idx_l)) {
    // new point has to be added
    Point2d_Node* new_point_node = new Point2d_Node();
    _nodes[_step].push_back(new_point_node);
    _point_nodes.push_back(new_point_node);
    _num_points[_step] = _point_nodes.size();
    if (_verbose) cout << idx_x << " " << *new_point_node << endl;
  }
  unsigned int i_x = _pose_mapper[idx_x];
  unsigned int i_l = _point_mapper[idx_l];
  Pose2d_Point2d_Factor* factor = new Pose2d_Point2d_Factor(
      dynamic_cast<Pose2d_Node*>(_pose_nodes[i_x]),
      dynamic_cast<Point2d_Node*>(_point_nodes[i_l]),
      measurement, noise);
  _factors[i_x].push_back(factor);
  _measurements.push_back(make_pair(i_x, i_l));
  _num_measurements[i_x] = _measurements.size();
  if (_verbose) cout << i_x << " " << *factor << endl;
}

void Loader::add_point3(unsigned int idx_x, unsigned int idx_p, const Point3d& m, const Noise& noise) {
  // We require that the pose node already exists
  if (_point_mapper.add(idx_p)) {
    Point3d_Node* new_point_node = new Point3d_Node();
    _nodes[_step].push_back(new_point_node);
    _point_nodes.push_back(new_point_node);
    _num_points[_step] = _point_nodes.size();
    if (_verbose) cout << idx_x << " " << *new_point_node << endl;
  }
  unsigned int i_x = _pose_mapper[idx_x];
  unsigned int i_p = _point_mapper[idx_p];
  Pose3d_Point3d_Factor* factor =
    new Pose3d_Point3d_Factor(dynamic_cast<Pose3d_Node*>(_pose_nodes[i_x]),
                              dynamic_cast<Point3d_Node*>(_point_nodes[i_p]),
                              m, noise);
  _factors[i_x].push_back(factor);
  _measurements.push_back(make_pair(i_x, i_p));
  _num_measurements[i_x] = _measurements.size();
  if (_verbose) cout << i_x << " " << *factor << endl;
}

void Loader::add_stereo(isam::StereoCamera* camera,
                        unsigned int idx_x,
                        unsigned int idx_p,
                        const StereoMeasurement& m,
                        const Noise& noise)
{
  // We require that the pose node already exists
  if (_point_mapper.add(idx_p)) {
    Point3dh_Node* new_point_node = new Point3dh_Node();
//    Point3d_Node* new_point_node = new Point3d_Node();
    _nodes[_step].push_back(new_point_node);
    _point_nodes.push_back(new_point_node);
    _num_points[_step] = _point_nodes.size();
    if (_verbose) cout << idx_x << " " << *new_point_node << endl; 
  }
  unsigned int i_x = _pose_mapper[idx_x];
  unsigned int i_p = _point_mapper[idx_p];
  Stereo_Factor* factor = 
    new Stereo_Factor(dynamic_cast<Pose3d_Node*>(_pose_nodes[i_x]),
                      dynamic_cast<Point3dh_Node*>(_point_nodes[i_p]),
//                      dynamic_cast<Point3d_Node*>(_point_nodes[i_p]),
                      camera, m, noise);
  _factors[i_x].push_back(factor);
  _measurements.push_back(make_pair(i_x, i_p));
  _num_measurements[i_x] = _measurements.size();
  if (_verbose) cout << i_x << " " << *factor << endl;
}

bool Loader::parse_line(char* str) {
  bool solve = false;
  char keyword_c[2000];
  int key_length;
  sscanf(str, "%s%n", keyword_c, &key_length);
  const char* arguments = &str[key_length];
  string keyword(keyword_c);
  if (keyword == "ODOMETRY" || keyword == "EDGE2" || keyword == "Constraint2") {
    unsigned int idx_x0, idx_x1;
    double x, y, t, ixx, ixy, ixt, iyy, iyt, itt;
    int res = sscanf(arguments, "%i %i %lg %lg %lg %lg %lg %lg %lg %lg %lg", &idx_x0, &idx_x1, &x, &y, &t, &ixx, &ixy, &ixt, &iyy, &iyt, &itt);
    if (res!=11) {
      cout << "Error while parsing ODOMETRY entry" << endl;
      exit(1);
    }
#if 0 // generate ground truth
    x = round(x);
    y = round(y);
    t = round(t/(M_PI/2)) * (M_PI/2);
    printf("EDGE2 %i %i %g %g %g 50 0 0 50 0 100\n", idx_x0, idx_x1, x, y, t);
#endif
    Pose2d measurement(x, y, t);
    MatrixXd sqrtinf(3,3);
    sqrtinf <<
      ixx, ixy, ixt,
      0.,  iyy, iyt,
      0.,   0., itt;
    if (_step==0) {
      add_prior();
    }
    add_odometry(idx_x0, idx_x1, measurement, SqrtInformation(sqrtinf));
  } else if (keyword == "LANDMARK") {
    unsigned int idx_x, idx_l;
    double x, y, ixx, ixy, iyy;
    int res = sscanf(arguments, "%i %i %lg %lg %lg %lg %lg", &idx_x, &idx_l, &x, &y, &ixx, &ixy, &iyy);
    if (res!=7) {
      cout << "Error while parsing LANDMARK entry" << endl;
      exit(1);
    }
#if 0 // generate ground truth
    x = round(x+0.5)-0.5;
    y = round(y+0.5)-0.5;
    printf("LANDMARK %i %i %g %g 10 0 10\n", idx_x, idx_l, x, y);
#endif
    Point2d measurement(x, y);
    MatrixXd sqrtinf(2,2);
    sqrtinf <<
      ixx, ixy,
      0.,  iyy;
    add_measurement(idx_x, idx_l, measurement, SqrtInformation(sqrtinf));
  } else if (keyword == "POINT3") {
    unsigned int idx_x; // camera node
    unsigned int idx_p; // point node
    double x,y,z; // Measurement, Euclidean coordinates in camera frame
    double i11, i12, i13, i22, i23, i33;
    int res = sscanf(arguments, "%i %i %lg %lg %lg %lg %lg %lg %lg %lg %lg",&idx_x,&idx_p,&x,&y,&z,&i11,&i12,&i13,&i22,&i23,&i33);

    MatrixXd sqrtinf(3,3);
    if (res!=11 && res!=5) {
      cout << "Error while parsing POINT3 entry" << endl;
      exit(1);
    }
    if (res == 5) {
      sqrtinf = eye(3); // no information matrix: use identity
    } else {
      sqrtinf <<
       i11, i12, i13,
         0, i22, i23,
         0,   0, i33;
    }
    Point3d m(x,y,z);
    add_point3(idx_x, idx_p, m, SqrtInformation(sqrtinf));
  } else if (keyword == "CAMERA_STEREO") {
    unsigned int idx_camera; // camera id
    double f;
    double cx, cy;
    double b;

    int res = sscanf(arguments, "%i %lg %lg %lg %lg",&idx_camera,&f,&cx,&cy,&b);
    if (res != 5) {
      cout << "Error while parsing CAMERA_STEREO entry" << endl;
      exit(1);
    }

    _cameras[idx_camera] = new StereoCamera(f, Vector2d(cx,cy), b);

  } else if (keyword == "POINT_STEREO") {
    unsigned int idx_camera; // camera id
    unsigned int idx_x; // camera node
    unsigned int idx_p; // point node
    double u,v,u2; // Measurement, where w is disparity
    double i11, i12, i13, i22, i23, i33;
    int res = sscanf(arguments, "%i %i %i %lg %lg %lg %lg %lg %lg %lg %lg %lg",&idx_camera,&idx_p,&idx_x,&u,&v,&u2,&i11,&i12,&i13,&i22,&i23,&i33);

    MatrixXd sqrtinf(3,3);
    if (res!=12 && res!=6) {
      cout << "Error while parsing POINT_STEREO entry" << endl;
      exit(1);
    }
    if (res == 6) {
      sqrtinf = 4.0 * eye(3); // no information matrix: use identity
    } else {
      sqrtinf <<
       i11, i12, i13,
         0, i22, i23,
         0,   0, i33;
    }
    StereoMeasurement m(u,v,u2);
    if (_cameras.find(idx_camera) == _cameras.end()) {
      cout << "Error while parsing POINT_STEREO entry: Camera (" << idx_camera << ") calibration not found." << endl;
      exit(1);
    }
    add_stereo(_cameras[idx_camera], idx_x, idx_p, m, SqrtInformation(sqrtinf));
  } else if (keyword == "EDGE3") {
    unsigned int idx_x0, idx_x1;
    double x, y, z, yaw, pitch, roll, i11, i12, i13, i14, i15, i16;
    double i22, i23, i24, i25, i26, i33, i34, i35, i36, i44, i45, i46, i55, i56, i66;
    int res = sscanf(arguments, "%i %i %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg",
                     &idx_x0, &idx_x1, &x, &y, &z, &roll, &pitch, &yaw, // note reverse order of angles, also see covariance below
                     &i11, &i12, &i13, &i14, &i15, &i16, &i22, &i23, &i24, &i25, &i26,
                     &i33, &i34, &i35, &i36, &i44, &i45, &i46, &i55, &i56, &i66);
    if (res!=29 && res!=8) {
      cout << "Error while parsing EDGE3 entry" << endl;
      exit(1);
    }
    Pose3d delta;
    if (Y_FORWARD) {
      delta = Pose3d(y, -x, z, yaw, pitch, roll); // converting from external format with Y pointing forward
    } else {
      delta = Pose3d(x, y, z, yaw, pitch, roll); // standard convention (X points forward)
    }
    // square root information matrix
    MatrixXd sqrtinf(6,6);
    if (res==8) {
      sqrtinf = eye(6); // no information matrix: use identity
    } else {
      sqrtinf <<
        i11, i12, i13, i14, i15, i16,
        0., i22, i23, i24, i25, i26,
        0.,  0., i33, i34, i35, i36,
        //                0.,  0.,  0., i44, i45, i46,
        //                0.,  0.,  0.,  0., i55, i56,
        //                0.,  0.,  0.,  0.,  0., i66);
        // todo: this is wrong in the presence of off-diagonal entries because sqrtinf...
        0.,  0.,  0., i66, i56, i46, // note: reversed yaw, pitch, roll, also see sscanf above
        0.,  0.,  0.,  0., i55, i45,
        0.,  0.,  0.,  0.,  0., i44;
    }
    // reverse constraint if needed
    unsigned int i, j;
    if (idx_x0<idx_x1) {
      i = idx_x1;
      j = idx_x0;
    } else {
      delta = Pose3d(delta.oTw());
      i = idx_x0;
      j = idx_x1;
    }
    if (_step==0) {
      _is_3d = true;
      add_prior();
    }
    add_odometry3(j, i, delta, SqrtInformation(sqrtinf));
  } else if (keyword == "POSE3D_INIT") {
    // Initialize a POSE3D node
    unsigned int idx_x0;
    double x, y, z, yaw, pitch, roll;
    int res = sscanf(arguments, "%i %lg %lg %lg %lg %lg %lg",
                     &idx_x0, &x, &y, &z, &roll, &pitch, &yaw); // note reverse order of angles, also see covariance below
    if (res!=7) {
      cout << "Error while parsing POSE3D_INIT entry" << endl;
      exit(1);
    }

    if (advance(idx_x0, _point_nodes.size())) {
      Pose3d_Node* new_pose_node = new Pose3d_Node();
      _nodes[_step].push_back(new_pose_node);
      _pose_nodes.push_back(new_pose_node);
      if (_verbose) cout << idx_x0 << " " << *new_pose_node << endl;
      new_pose_node->init(Pose3d(x,y,z,yaw,pitch,roll));
    }

  } else if (keyword == "EDGE3_INIT") {
    // Provide an edge that is only used for initialization.
  } else if (keyword == "POSE3D_TRUE") {
    // Use to calculate a performance metric

  } else if (keyword == "EDGE3_TRUE") {
    // Use to calculate a performance metric

  } else if (keyword == "SOLVE") {
    solve = true;
  }
  return solve;
}

Loader::Loader(const char* fname, int num_lines, bool verbose) {
  _verbose = verbose;
  _step = 0;
  _is_3d = false;

  // parse and process data file
  _in = fopen(fname, "r");
  if (!_in) {
    printf("ERROR: Failed to open log file %s.\n", fname);
    exit(1);
  }
  int i = 0;
  while (!feof(_in) && (num_lines==0 || i<num_lines)) {
    char str[2000];
    if (fgets(str, 2000, _in)) {
      parse_line(str);
    }
    i++;
  }
  fclose(_in);
}

Loader::~Loader()
{
  for(std::map<int, isam::StereoCamera*>::iterator it = _cameras.begin(); it != _cameras.end(); ++it) delete it->second;
}

void Loader::print_stats() const {
  int n = num_steps();
  cout << "Number of poses: " << n << endl;
  if (_point_nodes.size()>0) {
    cout << "Number of landmarks: " << _num_points[n-1] << endl;
    cout << "Number of measurements: " << _num_measurements[n-1] << endl;
  }
  cout << "Number of constraints: " << _num_constraints[n-1] << endl;
}

bool Loader::more_data(unsigned int* step) {
  (*step)++;
  return (*step)<=num_steps();
}

const Loader::PoseList Loader::poses(unsigned int step) const {
  Loader::PoseList poses;
  poses.resize(step+1);
  for (unsigned int i=0; i<step+1; i++) {
    if (is_3d()) {
      poses[i] = dynamic_cast<Pose3d_Node*>(_pose_nodes[i])->value();
    } else {
      poses[i].of_pose2d(dynamic_cast<Pose2d_Node*>(_pose_nodes[i])->value());
    }
  }
  return poses;
}

const Loader::PoseList Loader::points(unsigned int step) const {
  Loader::PoseList points;
  points.resize(_num_points[step]);
  for (unsigned int i=0; i<points.size(); i++) {
    if (is_3d()) {
      //points[i].of_point3d(dynamic_cast<Point3d_Node*>(_point_nodes[i])->value());
      if (dynamic_cast<Point3d_Node*>(_point_nodes[i])) {
        points[i].of_point3d(dynamic_cast<Point3d_Node*>(_point_nodes[i])->value());
      } else if (dynamic_cast<Point3dh_Node*>(_point_nodes[i])) {
        Point3dh_Node* node = dynamic_cast<Point3dh_Node*>(_point_nodes[i]);
        points[i].of_point3d(node->value().to_point3d());
      }
    } else {
      points[i].of_point2d(dynamic_cast<Point2d_Node*>(_point_nodes[i])->value());
    }
  }
  return points;
}

const vector<pair<int,int> > Loader::constraints(unsigned int step) const {
  return vector<pair<int,int> >(_constraints.begin(), _constraints.begin()+_num_constraints[step]);
}

const vector<pair<int,int> > Loader::measurements(unsigned int step) const {
  return vector<pair<int,int> >(_measurements.begin(), _measurements.begin()+_num_measurements[step]);
}
