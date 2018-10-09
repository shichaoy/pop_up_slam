/**
 * @file example.cpp
 * @brief Simple iSAM example with dynamic allocation.
 * @author Michael Kaess
 * @version $Id: example.cpp 6377 2012-03-30 20:06:44Z kaess $
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

/** @page Example Simple iSAM Example

The iSAM library represents general optimization problems as a factor graph, where nodes represent variables and factors represent measurements on sets of one or more nodes.

In this simple SLAM example, five poses are created and connected into a chain by odometry measurements. A landmark is added with a single measurement from one of the poses.

Note that the optimization problem described so far is not fully constrained as the complete graph can be shifted and rotated arbitrarily in the plane without changing the relative measurements described by odometry and landmark measurements. The graph needs to be constraints, and a common solution is to add a prior to the first pose to fix it at the origin.

The following code can be compiled and run with
@verbatim
make examples
bin/example
@endverbatim

To compile without the iSAM cmake system:
@verbatim
cd examples
c++ example.cpp ../lib/libisam.a -lcholmod -I../include -I/usr/include/eigen3
@endverbatim

examples/example.cpp :
@includelineno examples/example.cpp

*/

#include <isam/isam.h>

using namespace std;
using namespace isam;
using namespace Eigen;

int main() {
  // instance of the main class that manages and optimizes the pose graph
  Slam slam;

  // locally remember poses
  vector<Pose2d_Node*> pose_nodes;

  Noise noise3 = Information(100. * eye(3));
  Noise noise2 = Information(100. * eye(2));

  // create a first pose (a node)
  Pose2d_Node* new_pose_node = new Pose2d_Node();
  // add it to the graph
  slam.add_node(new_pose_node);
  // also remember it locally
  pose_nodes.push_back(new_pose_node);

  // create a prior measurement (a factor)
  Pose2d origin(0., 0., 0.);
  Pose2d_Factor* prior = new Pose2d_Factor(pose_nodes[0], origin, noise3);
  // add it to the graph
  slam.add_factor(prior);

  for (int i=1; i<4; i++) {
    // next pose
    Pose2d_Node* new_pose_node = new Pose2d_Node();
    slam.add_node(new_pose_node);
    pose_nodes.push_back(new_pose_node);

    // connect to previous with odometry measurement
    Pose2d odometry(1., 0., 0.); // x,y,theta
    Pose2d_Pose2d_Factor* constraint = new Pose2d_Pose2d_Factor(pose_nodes[i-1], pose_nodes[i], odometry, noise3);
    slam.add_factor(constraint);
  }

  // create a landmark
  Point2d_Node* new_point_node = new Point2d_Node();
  slam.add_node(new_point_node);

  // create a pose and the landmark by a measurement
  Point2d measure(5., 3.); // x,y
  Pose2d_Point2d_Factor* measurement =
    new Pose2d_Point2d_Factor(pose_nodes[1], new_point_node, measure, noise2);
  slam.add_factor(measurement);

  // optimize the graph
  slam.batch_optimization();

  // accessing the current estimate of a specific pose
  cout << "Pose 2: " << pose_nodes[2]->value() << endl;

  // printing the complete graph
  cout << endl << "Full graph:" << endl;
  slam.write(cout);

  return 0;
}
