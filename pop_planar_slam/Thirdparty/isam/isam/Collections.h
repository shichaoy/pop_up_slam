/**
 * @file Collections.h
 * @brief 3D visualization.
 * @author Michael Kaess
 * @version $Id: Collections.h 4232 2011-04-02 20:18:18Z hordurj $
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

#pragma once

#include <map>
#include <list>
#include <vector>
#include <string>

#include <isam/Pose3d.h>

enum ViewerObjects {
  VIEWER_OBJ_POSE3D,
  VIEWER_OBJ_TREE,
  VIEWER_OBJ_POINT3D
};

class Collection {
public:
  int id;
  std::string name;
  int type;

  Collection(int id, std::string name, int type) : id(id), name(name), type(type) {}

  virtual ~Collection() {}
  virtual void draw() = 0;
};

class ObjCollection : public Collection {
  typedef std::map<int,
                   isam::Pose3d,
                   std::less<int>,
                   Eigen::aligned_allocator<isam::Pose3d> > objs_t;
  int maxid;
  objs_t objs;

public:
  ObjCollection(int id,
                std::string name,
                int type,
                const std::vector<isam::Pose3d, Eigen::aligned_allocator<isam::Pose3d> >& nodes);
  virtual void draw();

  friend class LinkCollection;
  friend class CovCollection;
};

class LinkCollection : public Collection {
  typedef std::vector<std::pair<int,int> > elements_t;
  elements_t elements;
  int col1;
  int col2;

public:
  LinkCollection(int id, std::string name, const std::vector<std::pair<int,int> >& links, int col1, int col2);
  virtual void draw();
};

class CovCollection : public Collection {
  typedef std::list<Eigen::MatrixXd> covs_t;
  covs_t covs;
  int collection;
  bool is_3d;

public:
  CovCollection(int id, std::string name, const std::list<Eigen::MatrixXd>& covs, int collection, bool is_3d);
  virtual void draw();
};

typedef std::map<int, Collection*> collections_t;
extern collections_t collections;
