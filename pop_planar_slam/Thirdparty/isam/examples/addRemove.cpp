/**
 * @file addRemove.cpp
 * @brief Editing of pose graphs.
 * @author Michael Kaess
 * @version $Id: addRemove.cpp 6335 2012-03-22 23:13:52Z kaess $
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

#include <isam/Slam.h>
#include <isam/slam2d.h>

using namespace std;
using namespace isam;
using namespace Eigen;

// note that as the nodes have no IDs to print, it's not easy to know
// which node is which; the user really has to keep track of that, or
// the user could extend the Pose2d_Node class to for example contain
// a string ID.
void print_all(Slam& slam) {
  list<Node*> ids = slam.get_nodes();
  for (list<Node*>::iterator iter = ids.begin(); iter != ids.end(); iter++) {
    Pose2d_Node* pose = dynamic_cast<Pose2d_Node*>(*iter);
    cout << pose->value() << endl;
  }
}

int main() {
  // locally defined, as nodes and factors below are allocated on the
  // stack, and therefore will go out of scope after the end of this
  // function; note that you can (and typically have to) dynamically
  // allocate, but I used local variables here to make the example
  // simpler - see example.cpp for dynamic allocation.
  Slam slam;

  Noise noise = Covariance(0.01 * eye(3));

  Pose2d prior(0., 0., 0.);
  Pose2d_Node x0;
  Pose2d_Node x1;
  Pose2d_Node x2;
  slam.add_node(&x0);
  Pose2d_Factor p_x0(&x0, prior, noise);
  slam.add_factor(&p_x0);
  Pose2d odo(1., 0., 0.);
  slam.add_node(&x1);
  Pose2d_Pose2d_Factor o_01(&x0, &x1, odo, noise);
  slam.add_factor(&o_01);
  slam.add_node(&x2);
  Pose2d_Pose2d_Factor o_12(&x1, &x2, odo, noise);
  slam.add_factor(&o_12);

  slam.batch_optimization();

  Pose2d_Node a1;
  slam.add_node(&a1); // new node for x1
  Pose2d_Pose2d_Factor o_01_new(&x0, &a1, odo, noise);
  slam.add_factor(&o_01_new);
  Pose2d_Pose2d_Factor o_12_new(&a1, &x2, odo, noise);
  slam.add_factor(&o_12_new);
  slam.remove_node(&x1);

  slam.batch_optimization();

  slam.print_stats();
  slam.print_graph();

  print_all(slam);

  return 0;
}
