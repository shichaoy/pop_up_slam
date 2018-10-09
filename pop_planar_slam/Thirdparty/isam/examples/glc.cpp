/**
 * @file glc.cpp
 * @brief 2D Generic Linear Constraints Example with poses and a landmark
 * @author Nicholas Carlevaris-Bianco
 * @author Michael Kaess
 * @version $Id: glc.cpp 8578 2013-07-01 00:28:49Z kaess $
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

#include <ctime>
#include <isam/Slam.h>
#include <isam/slam2d.h>
#include <isam/glc.h>

using namespace std;
using namespace isam;
using namespace Eigen;

MatrixXd get_info (Slam *slam) {
  const SparseSystem& R = slam->get_R();
  const int *a_to_r = R.a_to_r(); // column ordering
  MatrixXd Rsub(R.num_rows(), R.num_cols());
  // get colums of R for each of the inds
  for (int i=0; i<R.num_rows(); i++) {
    for (int j=0; j<R.num_cols(); j++) {
      Rsub(i,j) = R(a_to_r[i], a_to_r[j]);
    }
  }

  return Rsub.transpose()*Rsub;
}

void print_all(Slam& slam) {
  list<Node*> ids = slam.get_nodes();
  for (list<Node*>::iterator iter = ids.begin(); iter != ids.end(); iter++) {
    Pose2d_Node* pose = dynamic_cast<Pose2d_Node*>(*iter);
    cout << pose->value() << endl;
  }
}

double snrv (void) {
  srand(time(0));
  double U = rand() / double(RAND_MAX);
  double V = rand() / double(RAND_MAX);
  // Box-Muller method
  return sqrt (-2.0*log(U)) * cos(2.0*M_PI*V);
}

Pose2d add_noise(Pose2d in, double sigma) {
  Pose2d out (in.x()+sigma*snrv(),
              in.y()+sigma*snrv(),
              in.t()+sigma*snrv());
  return out;
}

Point2d add_noise(Point2d in, double sigma) {
  Point2d out (in.x()+sigma*snrv(),
               in.y()+sigma*snrv());
  return out;
}

int main() {
  // setup a simple graph
  // known poses
  Pose2d x0 (0,  0,   M_PI/9.0);
  Pose2d x1 (10, 10,  0.0     );
  Pose2d x2 (20, 20,  M_PI/6.0);
  Pose2d x3 (30, 30, -M_PI/4.0);
  Pose2d x4 (40, 40, -M_PI/8.0);
  // known landmarks
  Point2d la (100, 100);
  
  // mesurements
  Pose2d z0 = x0;
  Pose2d z01 = x1.ominus(x0);
  Pose2d z12 = x2.ominus(x1);
  Pose2d z23 = x3.ominus(x2);
  Pose2d z34 = x4.ominus(x3);
  Pose2d z13 = x3.ominus(x1);
  // landmark measurement
  Point2d z1a = x1.transform_to(la);
    
  // add noise to measurements
  double sigma = 0.01;
  MatrixXd Q = sigma*sigma*eye(3);
  Noise Qsqinf = Information(Q.inverse());
  MatrixXd Q2 = sigma*sigma*eye(2);
  Noise Q2sqinf = Information(Q2.inverse());
  z0  = add_noise(z0,  sigma);
  z01 = add_noise(z01, sigma);
  z12 = add_noise(z12, sigma);
  z23 = add_noise(z23, sigma);
  z34 = add_noise(z34, sigma);
  z13 = add_noise(z13, sigma);
  z1a = add_noise(z1a, sigma);
  
  // nodes
  Pose2d_Node*  n0 = new Pose2d_Node();
  Pose2d_Node*  n1 = new Pose2d_Node();
  Pose2d_Node*  n2 = new Pose2d_Node();
  Pose2d_Node*  n3 = new Pose2d_Node();
  Pose2d_Node*  n4 = new Pose2d_Node();
  Point2d_Node* na = new Point2d_Node();
  
  // make graph
  Slam slam;
  Properties prop;
  prop.verbose = true; prop.quiet = false; prop.method = GAUSS_NEWTON;
  slam.set_properties(prop);
  
  // add nodes to graph
  slam.add_node(n0);
  slam.add_node(n1);
  slam.add_node(n2),
  slam.add_node(n3);
  slam.add_node(n4);
  slam.add_node(na);
    
  // create factors and add them to the graph
  Pose2d_Factor* z0f = new Pose2d_Factor(n0, z0, Qsqinf);
  slam.add_factor(z0f);
  Pose2d_Pose2d_Factor* z01f = new Pose2d_Pose2d_Factor(n0, n1, z01, Qsqinf);
  slam.add_factor(z01f);
  Pose2d_Pose2d_Factor* z12f = new Pose2d_Pose2d_Factor(n1, n2, z12, Qsqinf);
  slam.add_factor(z12f);
  Pose2d_Pose2d_Factor* z23f = new Pose2d_Pose2d_Factor(n2, n3, z23, Qsqinf);
  slam.add_factor(z23f);
  Pose2d_Pose2d_Factor* z34f = new Pose2d_Pose2d_Factor(n3, n4, z34, Qsqinf);
  slam.add_factor(z34f);
  Pose2d_Pose2d_Factor* z13f = new Pose2d_Pose2d_Factor(n1, n3, z13, Qsqinf);
  slam.add_factor(z13f);
  Pose2d_Point2d_Factor* z1af = new Pose2d_Point2d_Factor(n1, na, z1a, Q2sqinf);
  slam.add_factor(z1af);
  
  slam.batch_optimization();
  slam.print_stats();
  ofstream f; f.open ("before.graph"); slam.write(f); f.close();
  //slam.print_graph();
  //print_all(slam);
  
  // get true marginal distribution over p(x0, x2, x3, x4)
  list<Node*> nodes_subset;
  nodes_subset.push_back(n0);
  nodes_subset.push_back(n2);
  nodes_subset.push_back(n3);
  nodes_subset.push_back(n4);
  MatrixXd P_true = slam.covariances().marginal(nodes_subset);
  //MatrixXd L_true = get_info (&slam);
  VectorXd mu_true(12);
  mu_true.segment<3>(0) = n0->value().vector();
  mu_true.segment<3>(3) = n2->value().vector();
  mu_true.segment<3>(6) = n3->value().vector();
  mu_true.segment<3>(9) = n4->value().vector();  
  
  // remove node 1 using GLC ========================================
  bool sparse = true;  // sparse approximate or dense exact
  vector<Factor*> felim = glc_elim_factors (n1); //usefull for local managment of factors
  //vector<Factor*> fnew = glc_remove_node (slam, n1, sparse); // not root shifted
  GLC_RootShift rs;
  vector<Factor*> fnew = glc_remove_node (slam, n1, sparse, &rs); // root shifted
  // ================================================================
  cout << felim.size() << " factor(s) removed." << endl;
  cout << fnew.size() << " new GLC factor(s) added." << endl;
  slam.batch_optimization();
  slam.print_stats();
  f.open ("after.graph"); slam.write(f); f.close();

  // get glc marginal distribution over p(x0, x2, x3, x4)
  MatrixXd P_glc = slam.covariances().marginal(nodes_subset);
  //MatrixXd L_glc = get_info (&slam);
  VectorXd mu_glc(12);
  mu_glc.segment<3>(0) = n0->value().vector();
  mu_glc.segment<3>(3) = n2->value().vector();
  mu_glc.segment<3>(6) = n3->value().vector();
  mu_glc.segment<3>(9) = n4->value().vector();  

  // calcualte the KLD
  int n = mu_glc.size();
  double A = (P_glc.inverse() * P_true).trace();
  VectorXd du = mu_glc - mu_true;
  // deal with difference in angles
  for(int i=0; i<du.size(); i++) {
    if(i % 3 == 2)
      du(i) = standardRad(du(i));
  }   
  double B = du.transpose() * P_glc.inverse() * du;
  double C = log(P_true.determinant()) - log(P_glc.determinant());
  double kld = 0.5*(A + B - C - n);
  cout << "KLD: " << kld << endl;
  
  return 0;
}
