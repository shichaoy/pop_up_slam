/**
 * @file anchorNodes.cpp
 * @brief Multiple relative pose graphs with relative constraints.
 * @author Michael Kaess
 * @version $Id: anchorNodes.cpp 6335 2012-03-22 23:13:52Z kaess $
 *
 * For details on the concept of anchor nodes see:
 * “Multiple Relative Pose Graphs for Robust Cooperative Mapping”
 * B. Kim, M. Kaess, L. Fletcher, J. Leonard, A. Bachrach, N. Roy, and S. Teller
 * IEEE Intl. Conf. on Robotics and Automation, ICRA, (Anchorage, Alaska), May 2010, pp. 3185-3192.
 * online available at http://www.cc.gatech.edu/~kaess/pub/Kim10icra.html
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

#include <isam/isam.h>
#include "isam/Anchor.h"

using namespace std;
using namespace isam;
using namespace Eigen;


int main() {
  // locally defined, as nodes and factors below are allocated on the
  // stack, and therefore will go out of scope after the end of this
  // function; note that you can (and typically have to ) dynamically
  // allocate, see demo.cpp, but I used local variables here to make
  // the example simpler.
  Slam slam;

  Noise noise = SqrtInformation(10. * eye(3));

  // first pose graph
  Pose2d prior_origin(0., 0., 0.);
  Pose2d_Node a0;
  slam.add_node(&a0);
  Pose2d_Factor p_a0(&a0, prior_origin, noise);
  slam.add_factor(&p_a0);
  Pose2d odo(1., 0., 0.);
  Pose2d_Node a1;
  slam.add_node(&a1);
  Pose2d_Pose2d_Factor o_a01(&a0, &a1, odo, noise);
  slam.add_factor(&o_a01);

  // second pose graph
  Pose2d_Node b0;
  slam.add_node(&b0);
  Pose2d_Factor p_b0(&b0, prior_origin, noise);
  slam.add_factor(&p_b0);
  Pose2d_Node b1;
  slam.add_node(&b1);
  Pose2d_Pose2d_Factor o_b01(&b0, &b1, odo, noise);
  slam.add_factor(&o_b01);

  cout << "two independent pose graphs:" << endl;
  cout << "batch optimization... (nothing changes)" << endl;
  slam.batch_optimization();
  cout << "...done" << endl;
  cout << "a0: " << a0.value() << endl;
  cout << "a1: " << a1.value() << endl;
  cout << "b0: " << b0.value() << endl;
  cout << "b1: " << b1.value() << endl;

  // constraint between two pose graphs:

  // first pose graph now also has a anchor node, requires an arbitrary
  // prior (here origin); also initializes the anchor node
  Anchor2d_Node anchor0(&slam);
  slam.add_node(&anchor0);
  //Pose2d_Factor p_anchor0(&anchor0, prior_origin, noise);
  //slam.add_factor(&p_anchor0);

  // anchor node for second trajectory (as before)
  Anchor2d_Node anchor1(&slam);
  slam.add_node(&anchor1);

  Pose2d measure0(0., 1., 0.);
  // now we need 2 optional parameters to refer to both anchor nodes -
  // the order determines which trajectory the measurement originates
  // from; also, the first one already needs to be initialized (done
  // using prior above), the second one gets initialized if needed
  Pose2d_Pose2d_Factor d_a1_b1(&a1, &b1, measure0, noise, &anchor0, &anchor1);
  slam.add_factor(&d_a1_b1);

  cout << "\nfirst constraint added:" << endl;

  cout << "batch optimization... (nothing changes)" << endl;
  slam.batch_optimization();
  cout << "...done" << endl;

  cout << "a0: " << a0.value() << endl;
  cout << "a1: " << a1.value() << endl;
  cout << "b0: " << b0.value() << endl;
  cout << "b1: " << b1.value() << endl;
  cout << "anchor0: " << anchor0.value() << endl;
  cout << "anchor1: " << anchor1.value() << endl;

  Pose2d measure1(0., 0.5, 0.); // conflicting measurement, want least squares
  Pose2d_Pose2d_Factor d_a1_b1_2(&a1, &b1, measure1, noise, &anchor0, &anchor1);
  slam.add_factor(&d_a1_b1_2);

  cout << "\nsecond conflicting constraint added (least squares is 0.75):" << endl;

  cout << "batch optimization... (least squares solution)" << endl;
  slam.batch_optimization();
  cout << "...done" << endl;

  cout.precision(2);
  cout << fixed;
  cout << "a0: " << a0.value() << endl;
  cout << "a1: " << a1.value() << endl;
  cout << "b0: " << b0.value() << endl;
  cout << "b1: " << b1.value() << endl;
  cout << "anchor0: " << anchor0.value() << endl;
  cout << "anchor1: " << anchor1.value() << endl;

  slam.save("anchorNodes.graph");

  return 0;
}
