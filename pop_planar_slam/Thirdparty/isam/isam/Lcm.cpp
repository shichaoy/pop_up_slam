/**
 * @file lcm.cpp
 * @brief LCM interface
 * @author Michael Kaess
 * @version $Id: Lcm.cpp 4232 2011-04-02 20:18:18Z hordurj $
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
#include <list>

#include "lcmtypes/mrlcm_obj_collection_t.h"
#include "lcmtypes/mrlcm_obj_t.h"
#include "lcmtypes/mrlcm_link_collection_t.h"
#include "lcmtypes/mrlcm_link_t.h"
#include "lcmtypes/mrlcm_cov_collection_t.h"
#include "lcmtypes/mrlcm_cov_t.h"
#include "lcmtypes/mrlcm_reset_collections_t.h"

#include <lcm/lcm.h>

#include <isam/Slam.h>
#include <isam/slam2d.h>

#include "Lcm.h"

using namespace std;
using namespace isam;
using namespace Eigen;


Lcm::Lcm() {
  lcm = lcm_create(NULL);
}

Lcm::~Lcm() {
  if (lcm) {
    lcm_destroy (lcm);
  }
}

void Lcm::send_reset() const {
  if (lcm) {
    mrlcm_reset_collections_t msg;
    mrlcm_reset_collections_t_publish(lcm, "RESET_COLLECTIONS", &msg);
  }
}

void Lcm::send_nodes(const vector<Pose3d, Eigen::aligned_allocator<isam::Pose3d> >& poses,
                     int id,
                     char* name,
                     int type) const {
  unsigned int num = poses.size();
  if (lcm) {
    mrlcm_obj_collection_t objs;
    objs.id = id;
    objs.name = name;
    objs.type = type;
    objs.reset = true;
    objs.nobjs = num;
    mrlcm_obj_t ps[num];
    for (unsigned int i = 0; i < num; i++) {
      Pose3d pose = poses[i];
      ps[i].id = i;
      ps[i].x = pose.x();
      ps[i].y = pose.y();
      ps[i].z = pose.z();
      ps[i].yaw   = pose.yaw();
      ps[i].pitch = pose.pitch();
      ps[i].roll  = pose.roll();
    }
    objs.objs = ps;
    mrlcm_obj_collection_t_publish(lcm, "OBJ_COLLECTION", &objs);
  }
}

void Lcm::send_links(const vector<pair<int,int> >& links, int id, char* name, int collection1, int collection2) const {
  unsigned int num = links.size();
  if (lcm) {
    mrlcm_link_collection_t ls;
    ls.id = id;
    ls.name = name;
    ls.type = 0; // unused
    ls.reset = true;
    ls.nlinks = num;
    mrlcm_link_t constrs[num];
    for (unsigned int i=0; i<num; i++) {
      pair<int, int> v = links[i];
      constrs[i].id = i;
      constrs[i].collection1 = collection1;
      constrs[i].id1 = v.first;
      constrs[i].collection2 = collection2;
      constrs[i].id2 = v.second;
    }
    ls.links = constrs;
    mrlcm_link_collection_t_publish(lcm, "LINK_COLLECTION", &ls);
  }
}

void Lcm::send_covariances(const list<MatrixXd>& covariances, int id, char* name, int collection, bool is_3d) const {
  unsigned int num = covariances.size();
  mrlcm_cov_collection_t covs;
  covs.id = id;
  covs.name = name;
  covs.type = MRLCM_COV_COLLECTION_T_ELLIPSOID;
  covs.reset = true;
  covs.ncovs = num;
  mrlcm_cov_t cs[num];
  list<MatrixXd>::const_iterator it = covariances.begin();
  for (unsigned int i=0; i<num; i++, it++) {
    cs[i].id = i;
    cs[i].collection = collection;
    cs[i].element_id = i;
    cs[i].n = is_3d?6:3;
    cs[i].entries = new double[cs[i].n];
    double* entries = cs[i].entries;
    const MatrixXd& cov = *it;
    entries[0] = cov(0,0);
    entries[1] = cov(0,1);
    if (is_3d) {
      entries[2] = cov(0,2);
      entries[3] = cov(1,1);
      entries[4] = cov(1,2);
      entries[5] = cov(2,2);
    } else {
      entries[2] = cov(1,1);
    }
  }
  covs.covs = cs;
  mrlcm_cov_collection_t_publish(lcm, "COV_COLLECTION", &covs);
  for (unsigned int i=0; i<num; i++) {
    delete[] cs[i].entries;
  }
}
