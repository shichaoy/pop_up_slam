/**
 * @file covariances.cpp
 * @brief How to recover covariances.
 * @author Michael Kaess
 * @version $Id: covariances.cpp 6377 2012-03-30 20:06:44Z kaess $
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
#include <stdio.h>

#include <Eigen/LU> 

#include <isam/isam.h>

using namespace std;
using namespace isam;
using namespace Eigen;

Slam slam;

int main(int argc, const char* argv[]) {
  Pose2d origin;
  Noise noise = SqrtInformation(10. * eye(3));
  Pose2d_Node* pose_node_1 = new Pose2d_Node(); // create node
  slam.add_node(pose_node_1); // add it to the Slam graph
  Pose2d_Factor* prior = new Pose2d_Factor(pose_node_1, origin, noise); // create prior measurement, an factor
  slam.add_factor(prior); // add it to the Slam graph

  Pose2d_Node* pose_node_2 = new Pose2d_Node(); // create node
  slam.add_node(pose_node_2); // add it to the Slam graph

  Pose2d delta(1., 0., 0.);
  Pose2d_Pose2d_Factor* odo = new Pose2d_Pose2d_Factor(pose_node_1, pose_node_2, delta, noise);
  slam.add_factor(odo);

  slam.batch_optimization();

#if 0
  const Covariances& covariances = slam.covariances();
#else
  // operate on a copy (just an example, cloning is useful for running
  // covariance recovery in a separate thread)
  const Covariances& covariances = slam.covariances().clone();
#endif

  // recovering the full covariance matrix
  cout << "Full covariance matrix:" << endl;
  MatrixXd cov_full = covariances.marginal(slam.get_nodes());
  cout << cov_full << endl << endl;

  // sanity checking by inverting the information matrix, not using R
  SparseSystem Js = slam.jacobian();
  MatrixXd J(Js.num_cols(), Js.num_cols());
  for (int r=0; r<Js.num_cols(); r++) {
    for (int c=0; c<Js.num_cols(); c++) {
      J(r,c) = Js(r,c);
    }
  }
  MatrixXd H = J.transpose() * J;
  MatrixXd cov_full2 = H.inverse();
  cout << cov_full2 << endl;

  // recovering the block-diagonals only of the full covariance matrix
  cout << "Block-diagonals only:" << endl;
  Covariances::node_lists_t node_lists;
  list<Node*> nodes;
  nodes.push_back(pose_node_1);
  node_lists.push_back(nodes);
  nodes.clear();
  nodes.push_back(pose_node_2);
  node_lists.push_back(nodes);
  list<MatrixXd> cov_blocks = covariances.marginal(node_lists);
  int i = 1;
  for (list<MatrixXd>::iterator it = cov_blocks.begin(); it!=cov_blocks.end(); it++, i++) {
    cout << "block " << i << endl;
    cout << *it << endl;
  }

  // recovering individual entries, here the right block column
  cout << "Right block column:" << endl;
  Covariances::node_pair_list_t node_pair_list;
  node_pair_list.push_back(make_pair(pose_node_1, pose_node_2));
  node_pair_list.push_back(make_pair(pose_node_2, pose_node_2));
  list<MatrixXd> cov_entries = covariances.access(node_pair_list);
  for (list<MatrixXd>::iterator it = cov_entries.begin(); it!=cov_entries.end(); it++) {
    cout << *it << endl;
  }
}
