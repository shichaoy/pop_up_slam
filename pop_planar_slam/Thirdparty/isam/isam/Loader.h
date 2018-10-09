/**
 * @file Loader.h
 * @brief Loading files with constraints/factors.
 * @author Michael Kaess
 * @version $Id: Loader.h 5691 2011-11-10 21:25:23Z kaess $
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

/** @class Loader
 * Loading multiple file formats containing constraints or factors.
 *
 * Note that Loader expects files to be sorted, ie. the data should
 * arrive in the correct time order.
 */

#pragma once

#include <vector>
#include <list>

#include <isam/Node.h>
#include <isam/Factor.h>
#include <isam/slam2d.h>
#include <isam/slam3d.h>
#include <isam/slam_stereo.h>

typedef std::list<isam::Node*> nodes_t;
typedef std::list<isam::Factor*> factors_t;

/**
 * Utility class to map non-continuous indices to continuous based
 * on time of appearance
*/
class IndexMapper {
  int next_i;
  std::map<int, int> _map_index;
  int create_if_needed(int i, bool& created) {
    return _map_index[i];
  }
public:
  IndexMapper() : next_i(0) {}

  /*
   * Provide translated index - entry must already exist.
   */
  int operator[](int i) {
    std::map<int, int>::iterator it = _map_index.find(i);
    require(it!=_map_index.end(), "Loader::IndexMapper::operator[]: object does not exist");
    return _map_index[i];
  }

  /*
   * Add new translation if it doesn't exist yet.
   * @return True if new entry was created.
   */
  bool add(int i) {
    bool added;
    std::map<int, int>::iterator it = _map_index.find(i);
    if (it==_map_index.end()) {
      // if entry does not exist, create one with next ID
      _map_index[i] = next_i;
      next_i++;
      added = true;
    } else {
      added = false;
    }
    return added;
  }
};

class Loader {

  IndexMapper _point_mapper;
  IndexMapper _pose_mapper;

  bool _verbose;
  unsigned int _step;
  bool _is_3d;
  FILE* _in;

  // for each time step, we have some new nodes and some new factors
  std::vector<nodes_t> _nodes;
  std::vector<factors_t> _factors;

  // for each time step, the corresponding pose
  std::vector<isam::Node*> _pose_nodes;

  // points, not by time step (needed for indexing below)
  std::vector<isam::Node*> _point_nodes;
  // for each time step, the number of points observed so far
  std::vector<int> _num_points;

  // indices into _pose_nodes vector
  std::vector< std::pair<int,int> > _constraints;
  // number of measurements up to each time step
  std::vector<int> _num_constraints;

  // indices into _pose_nodes and _point_nodes vectors
  std::vector< std::pair<int,int> > _measurements;
  // number of measurements up to each time step
  std::vector<int> _num_measurements;

  std::map<int, isam::StereoCamera*> _cameras;

  void add_prior();
  bool advance(unsigned int idx_x1, unsigned int next_point_id);
  void add_odometry(unsigned int idx_x0, unsigned int idx_x1, const isam::Pose2d& meausurement, const isam::Noise& noise);
  void add_odometry3(unsigned int idx_x0, unsigned int idx_x1, const isam::Pose3d& meausurement, const isam::Noise& noise);
  void add_point3(unsigned int idx_x, unsigned int idx_p, const isam::Point3d& m, const isam::Noise& noise);
  void add_measurement(unsigned int idx_x, unsigned int idx_l, const isam::Point2d& measurement, const isam::Noise& noise);
  void add_stereo(isam::StereoCamera* camera, unsigned int idx_x, unsigned int idx_p, const isam::StereoMeasurement& m, const isam::Noise& noise);

  bool parse_line(char* str);

public:
  typedef std::vector<isam::Pose3d, Eigen::aligned_allocator<isam::Pose3d> > PoseList;

  /**
   * Loads factors and organizes in suitable data structure together with nodes.
   * @param fname File name of log file.
   * @param num_lines Number of lines to process (0 means process complete file).
   * @param verbose Print constraints.
   */
  Loader(const char* fname, int num_lines, bool verbose);

  ~Loader();

  /**
   * Print statistics about loaded data.
   */
  void print_stats() const;

  /**
   * Returns true if step was not the last step.
   */
  bool more_data(unsigned int* step);

  /**
   * Number of time steps in data loaded.
   * @return Number of time steps.
   */
  unsigned int num_steps() const {return _nodes.size();}

  /**
   * @return true if loaded data is 3D.
   */
  bool is_3d() const {return _is_3d;}

  /**
   * Nodes for incremental SLAM.
   * @param i Time step.
   * @return Nodes created at step i.
   */
  const nodes_t& nodes(int i) const {return _nodes[i];}

  /**
   * Factors for incremental SLAM.
   * @param i Time step.
   * @return Factors created at step i.
   */
  const factors_t& factors(int i) const {return _factors[i];}

  /**
   * Returns vector of current 3D poses up to time step (converted from 2D as needed).
   * @param step Time step.
   * @return Vector of 3D poses up to step.
   */
  const PoseList poses(unsigned int step) const;

  /**
   * Returns vector of current 3D points up to time step (converted from 2D as needed).
   * @param step Time step.
   * @return Vector of 3D points up to step.
   */
  const PoseList points(unsigned int step) const;

  /**
   * Returns all pose nodes.
   * @return Vector of 2D or 3D pose nodes.
   */
  const std::vector<isam::Node*> pose_nodes() const {
    return _pose_nodes;
  }

  /**
   * Returns all point nodes.
   * @return Vector of 2D or 3D point nodes.
   */
  const std::vector<isam::Node*> point_nodes() const {
    return _point_nodes;
  }

  /**
   * Returns the number of points encountered up to the given step.
   * @param step Time step.
   * @return Number of points up to step.
   */
  unsigned int num_points(unsigned int step) const {
    return _num_points[step];
  }

  /**
   * Constraints for visualization.
   * @param step Time step.
   * @return List of constraints (pairs of indices into pose_nodes) up to step.
   */
  const std::vector< std::pair<int,int> > constraints(unsigned int step) const;

  /**
   * Measurements for visualization.
   * @param step Time step.
   * @return List of measurements (pairs of indices into pose_nodes and point_nodes) up to step.
   */
  const std::vector< std::pair<int,int> > measurements(unsigned int step) const;
};
