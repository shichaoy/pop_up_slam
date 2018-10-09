/**
 * @file Collections.cpp
 * @brief 3D visualization.
 * @author Michael Kaess
 * @version $Id: Collections.cpp 6335 2012-03-22 23:13:52Z kaess $
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

#include <vector>
#include <map>
#include <list>

#if defined(__APPLE__) && defined(__MACH__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "isam/Pose3d.h"
#include "isam/SparseMatrix.h"

#include "Collections.h"

using namespace std;
using namespace isam;
using namespace Eigen;

#define rad_to_degrees(rad) ((rad)*180/M_PI)

float colors[] = {
  1.0, 0.0, 0.0, // red
  0.0, 1.0, 0.0, // green
  0.0, 0.0, 1.0, // blue
  1.0, 1.0, 0.0,
  1.0, 0.0, 1.0,
  0.0, 1.0, 1.0,
  0.5, 1.0, 0.0,
  1.0, 0.5, 0.0,
  0.5, 0.0, 1.0,
  1.0, 0.0, 0.5,
  0.0, 0.5, 1.0,
  0.0, 1.0, 0.5,
  1.0, 0.5, 0.5, 
  0.5, 1.0, 0.5, 
  0.5, 0.5, 1.0, 
  0.5, 0.5, 1.0,
  0.5, 1.0, 0.5,
  0.5, 0.5, 1.0
};
const int num_colors = sizeof(colors)/(3*sizeof(float));

collections_t collections;

GLUquadricObj* quadric_internal = NULL;

GLUquadricObj* getQuadric() {
  if (quadric_internal==NULL) {
    quadric_internal = gluNewQuadric();
  }
  return quadric_internal;
}

void draw_point(const Point3d& point) {
  glPointSize(2.0);
  glBegin(GL_POINTS);
  glColor3f(1.0,1.0,1.0);
  glVertex3f(point.x(), point.y(), point.z());
  glEnd();
}

void draw_tree(const Point3d& point) {
  glPushAttrib(GL_CURRENT_BIT);
  glPushMatrix();
  glTranslatef(point.x(), point.y(), point.z());
  glColor3f(139./255., 69./255., 19./255.);
  glPushMatrix();
  glTranslatef(0,0,0.01);
  gluCylinder(getQuadric(), 0.2, 0.2, 2.6, 8, 8);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(0,0,4.5);
  glColor3f(0.1,0.7,0.1);
  gluSphere(getQuadric(), 2., 8, 8);
  glPopMatrix();
  glPopMatrix();
  glPopAttrib();
}

void draw_tetra(const Pose3d& pose, double size, bool mark) {
  glPushMatrix();
  glTranslatef(pose.x(), pose.y(), pose.z());
  glRotatef(rad_to_degrees(pose.yaw()),  0., 0., 1.);
  glRotatef(rad_to_degrees(pose.pitch()),0., 1., 0.);
  glRotatef(rad_to_degrees(pose.roll()), 1., 0., 0.);

  if (mark) {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    gluSphere(getQuadric(), size*1.5, 5, 5);
    glPopAttrib();
  }
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  // draw outline in black
  glPushAttrib(GL_CURRENT_BIT);
  glColor3f(0,0,0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glPopAttrib();
  glPopMatrix();
}

// Calculate eigenvalues for 3x3 symmetric, non-singular matrix.
void eigenvals(const MatrixXd& cov, double& eval1, double& eval2, double& eval3) {
  double m = cov.trace()/3.;
  MatrixXd b = cov - m * eye(3);
  double det_b = b(0,0)*b(1,1)*b(2,2) + b(0,1)*b(1,2)*b(2,0) + b(0,2)*b(1,0)*b(2,1)
    - b(0,0)*b(1,2)*b(2,1) - b(0,1)*b(1,0)*b(2,2) - b(0,2)*b(1,1)*b(2,0);
  double q = det_b/2;
  double p = 0;
  for (int i=0; i<3; i++) {
    for (int j=0; j<3; j++) {
      p = p + b(i,j)*b(i,j);
    }
  }
  p = p/6.;
  double phi = 0.;
  if (fabs(q) <= fabs(p*sqrt(p))) {
    phi = acos(q/(p*sqrt(p))) / 3.;
    if (phi<0) {
      phi=phi + M_PI / 3.;
    }
  }
  double cphi = cos(phi);
  double sphi3 = sqrt(3)*sin(phi);
  eval1 = m + 2.*sqrt(p)*cphi;
  eval2 = m - sqrt(p)*(cphi + sphi3);
  eval3 = m - sqrt(p)*(cphi - sphi3);
}

// Calculate eigenvector for a given eigenvalue.
VectorXd eigenvec(const MatrixXd& mat, double eval) {
  MatrixXd a = mat - eval * eye(3);
  // zeroing out entries below diagonal is easy
  SparseMatrix s = sparseMatrix_of_matrix(a);
  s.triangulate_with_givens();
  VectorXd res(3);
  // multiple cases because underconstrained
  if (s(0,0)==0.) {
    res(0) = 1.; // arbitrary
    res(1) = 0.;
    res(2) = 0.;
  } else {
    if (s(1,1)!=0.) {
      res(2) = 1.; // arbitrary
      // partial backsubstitution
      res(1) = -s(1,2)/s(1,1);
      res(0) = -(s(0,1)*res(1)+s(0,2)) / s(0,0);
    } else {
      res(1) = 1.; // arbitrary
      res(2) = 0.;
      res(0) = -s(0,1)/s(0,0);
    }
  }
  // normalize
  return (res/res.norm());
}

void draw_ellipsoid(const Point3d& center, const MatrixXd& covariance, bool is_3d) {
  MatrixXd cov = covariance;
  if (!is_3d && covariance.cols()==3) {
    // for 2D pose, remove orientation theta
    cov = cov.topLeftCorner(2, 2);
  }
  if (cov.cols()==2) {
    // for 2x2 matrix, extend to 3x3
    cov.resize(3,3);
    cov(0,2) = 0;
    cov(1,2) = 0;
    cov(2,0) = 0;
    cov(2,1) = 0;
    cov(2,2) = 0.001; // ellipsoid cannot have 0 extension, even if representing 2D ellipse
  }
  // cut Matrix to 3x3 (necessary for Pose3d)
  cov = cov.topLeftCorner(3,3);

  // eigenvectors and eigenvalues
  double eval1, eval2, eval3;
  eigenvals(cov, eval1, eval2, eval3);
  VectorXd evec1 = eigenvec(cov, eval1);
  VectorXd evec2 = eigenvec(cov, eval2);
  VectorXd evec3 = eigenvec(cov, eval3);

  // draw ellipsoid
  double k = 1.; // scale factor
  double radius1 = k * sqrt(eval1);
  double radius2 = k * sqrt(eval2);
  double radius3 = k * sqrt(eval3);
  const double max_radius = 6.;
  if (radius1<max_radius && radius2<max_radius && radius3<max_radius) {
    GLUquadricObj *quadric = getQuadric();
    glPushMatrix();
    glTranslated(center.x(), center.y(), center.z());
    GLdouble rotation[16] = {
      evec1(0), evec1(1), evec1(2), 0.,
      evec2(0), evec2(1), evec2(2), 0.,
      evec3(0), evec3(1), evec3(2), 0.,
      0., 0., 0., 1.
    };
    glMultMatrixd(rotation);
    glScaled(radius1, radius2, radius3);
    gluQuadricDrawStyle(quadric, GLU_FILL);
    gluQuadricNormals(quadric, GLU_SMOOTH);
    gluSphere(quadric, 0.3, 10, 10);
    glPopMatrix();
  }
}

ObjCollection::ObjCollection(int id,
     string name,
     int type,
     const vector<Pose3d, Eigen::aligned_allocator<isam::Pose3d> >& nodes) : Collection(id, name, type) {
  maxid = nodes.size()-1;
  int i = 0;
  for (vector<Pose3d, Eigen::aligned_allocator<isam::Pose3d> >::const_iterator it = nodes.begin(); it!=nodes.end(); it++, i++) {
    objs.insert(make_pair(i, *it));
  }
}

void ObjCollection::draw() {
  glPushAttrib(GL_CURRENT_BIT);
  glColor3fv(&colors[3*(id%num_colors)]);
  // preparations
  switch(type) {
  case VIEWER_OBJ_TREE:
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_RESCALE_NORMAL);
    glShadeModel(GL_SMOOTH);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    break;
  }
  // draw each object
  for (objs_t::const_iterator it = objs.begin(); it != objs.end(); it++) {
    int obj_id = it->first;
    const Pose3d& obj = it->second;
    // only draw if within range
    double size = 0.1; // size of plotted 3D poses
    bool is_last = (maxid == obj_id);
    switch(type) {
    case VIEWER_OBJ_POSE3D:
      draw_tetra(obj, size, is_last);
      break;
    case VIEWER_OBJ_TREE:
      draw_tree(obj.trans());
      break;
    case VIEWER_OBJ_POINT3D:
      draw_point(obj.trans());
      break;
    }
  }
  // cleanup
  switch(type) {
  case VIEWER_OBJ_TREE:
    glPopAttrib();
    break;
  }
  glPopAttrib();
}

LinkCollection::LinkCollection(int id, string name, const vector<pair<int,int> >& links, int col1, int col2)
  : Collection(id, name, type), elements(links), col1(col1), col2(col2) {}

void LinkCollection::draw() {
  glPushAttrib(GL_CURRENT_BIT);
  glColor3fv(&colors[3*(id%num_colors)]);
  int i=0;
  for (elements_t::iterator it = elements.begin(); it != elements.end(); it++, i++) {
    pair<int, int>& link = *it;
    collections_t::iterator collection_it1 = collections.find(col1);
    collections_t::iterator collection_it2 = collections.find(col2);
    if (collection_it1 != collections.end()
        && collection_it2 != collections.end()) {
      const ObjCollection::objs_t& objs1 = ((ObjCollection*)collection_it1->second)->objs;
      ObjCollection::objs_t::const_iterator it1 = objs1.find(link.first);
      const ObjCollection::objs_t& objs2 = ((ObjCollection*)collection_it2->second)->objs;
      ObjCollection::objs_t::const_iterator it2 = objs2.find(link.second);
      if (it1 != objs1.end() && it2 != objs2.end()) {
        const Pose3d& obj1 = it1->second;
        const Pose3d& obj2 = it2->second;
        glBegin(GL_LINES);
        glVertex3f(obj1.x(), obj1.y(), obj1.z());
        glVertex3f(obj2.x(), obj2.y(), obj2.z());
        glEnd();
      }
    }
  }
  glPopAttrib();
}

CovCollection::CovCollection(int id, string name, const list<MatrixXd>& covs, int collection, bool is_3d)
  : Collection(id, name, type), covs(covs), collection(collection), is_3d(is_3d) {}

void CovCollection::draw() {
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  //    glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  int i=0;
  for (covs_t::iterator it = covs.begin(); it != covs.end(); it++, i++) {
    MatrixXd& cov = *it;
    collections_t::iterator collection_it = collections.find(collection);
    if (collection_it != collections.end()) {
      const ObjCollection::objs_t& objs = ((ObjCollection*)collection_it->second)->objs;
      ObjCollection::objs_t::const_iterator it = objs.find(i);
      if (it != objs.end()) {
        const Pose3d& obj = it->second;
        // only draw if at least one end point is within the range
        float alpha = 0.4;
        float* rgb = &colors[3*(id%num_colors)];
        glColor4f(rgb[0],rgb[1],rgb[2],alpha);
        draw_ellipsoid(obj.trans(), cov, is_3d);
      }
    }
  }
  glDisable(GL_BLEND);
  //    glEnable(GL_DEPTH_TEST);
  glPopAttrib();
}
