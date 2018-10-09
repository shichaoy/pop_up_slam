/**
 * @file Viewer.h
 * @brief 3D visualization.
 * @author Michael Kaess
 * @version $Id: Viewer.h 4232 2011-04-02 20:18:18Z hordurj $
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

#include <vector>
#include <string>

#include "Collections.h"

class Viewer {

public:

  /**
   * GLUT initialization, open window.
   */
  void init(int (*process)(void*));

  /**
   * Adds or overwrites a collection of nodes (poses, landmarks...).
   * @param nodes Vector of nodes.
   * @param id Collection id (also determines color).
   * @param name Collection name.
   * @param type Type of object (pose, tree etc.)
   */
  void set_nodes(const std::vector<isam::Pose3d, Eigen::aligned_allocator<isam::Pose3d> >& nodes,
                 int id,
                 const std::string& name,
                 int type);

  /**
   * Adds or overwrites a collection of links
   * (measurements, odometry constraints...).
   * @param links Vector of links.
   * @param id Collection id (also determines color).
   * @param name Collection name.
   * @param collection1 Links start from elements of this collection id.
   * @param collection2 Links end at elements of this collection id.
   */
  void set_links(const std::vector<std::pair<int,int> >& links,
                 int id, const std::string& name, int collection1, int collection2);

  /**
   * Adds or overwrites a collection of covariances (2D or 3D)
   * @param covariances Covariances matrices in same order as elements they refer to.
   * @param id Collection id (also determines color).
   * @param name Collection name.
   * @param collection Collection number that the covariances refer to.
   * @param is_3d True if 3D covariances.
   */
  void set_covariances(const std::list<Eigen::MatrixXd>& covariances,
                       int id, char* name, int collection, bool is_3d) const;

  /**
   * Signals processing thread to quit upon user request through GUI.
   */
  bool exit_requested();

};
