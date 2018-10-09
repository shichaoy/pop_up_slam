/**
 * @file generateData.cpp
 * @brief Create 3D simulation data for ICRA 2012 paper.
 * @author Michael Kaess
 * @author David Rosen
 * @version $Id: generateSpheresICRA2012.cpp 6377 2012-03-30 20:06:44Z kaess $
 */

#include <stdio.h>
#include <string>
#include <fstream>
#include <sstream>

#include <isam/Pose3d.h>

using namespace std;
using namespace isam;
using namespace Eigen;

//Number of samples to generate
const int num_samples = 1000;

//Covariances
const double sigmas[6] = {0.1, 0.1, 0.1, 0.04, 0.04, 0.04};

string directory_name = "sphere_data/";
string base_filename = "sample_";
string file_extension = ".txt";

char* write_string = new char[500];

const bool ADD_NOISE = true;
const bool LOOP_CLOSING = true;
// vehicle on surface of sphere, instead of upright (pitch=roll=0)
const bool PERPENDICULAR = true;

// sample from a normal distribution
#include <boost/random/linear_congruential.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
static boost::minstd_rand generator(27u);
double sample_normal(double sigma = 1.0) 
{
  typedef boost::normal_distribution<double> Normal;
  Normal dist(0.0, sigma);
  boost::variate_generator<boost::minstd_rand&, Normal> norm(generator, dist);
  return(norm());
}

void write_constraint(ofstream& outfile, int id0, int id1, const Pose3d& delta_) {
  Pose3d delta = delta_;
  if (ADD_NOISE) {
    // corrupt measurement with Gaussian noise
    VectorXd v = delta.vector();
    for (int i=0; i<6; i++) {
      v(i) += sample_normal(sigmas[i]);
    }
    delta.set(v);
  }

  // X Y Z roll pitch yaw
  sprintf(write_string, "EDGE3 %i %i %g %g %g %g %g %g",
          id0, id1, delta.x(), delta.y(), delta.z(), delta.roll(), delta.pitch(), delta.yaw());
  outfile << write_string;
  for (int i=0; i<6; i++) {
    for (int j=i; j<6; j++) {
      double sqrtinf = 0.;
      // only diagonal entries populated
      if (i==j) {
        // roll,pitch,yaw order
        if (i<3) {
          sqrtinf = 1./sigmas[i];
        } else {
          sqrtinf = 1./sigmas[8-i];
        }
      }
      sprintf(write_string, " %g", sqrtinf);
      outfile << write_string;
    }
  }
  outfile << "\n";
}

void add_constraint(ofstream& outfile, Pose3d* poses, int id0, int id1) {
  Pose3d delta = poses[id1].ominus(poses[id0]);
  write_constraint(outfile, id0, id1, delta);
}

int main(int argc, const char* argv[])  {

  // Make sure that we have a 'test_data' directory.
  int ret = system(("mkdir " + directory_name).c_str());
  require(ret!=-1, "Failed to created directory.");

  for (int s = 1; s <= num_samples; s++) {
    // Form the filename that will be used to hold the output of this sample.

    stringstream current_filename;

    current_filename << directory_name;
    current_filename << base_filename;
    current_filename << s;
    current_filename << file_extension;

    // Use this filename to open up a new file for writing.
    ofstream outfile( (current_filename.str()).c_str());

    // generate poses on the surface of a sphere
    const int steps = 50; // number of poses for one turn around the sphere
    const int slices = 50; // number of rounds around the sphere
    const double radius0 = 2.; // starting radius
    const double radius = 50.; // sphere radius
    
    int n = steps*slices;
    Pose3d poses[n];
    int pose_id = 0;
    // current vertical angle for elevation/slice (starting angle and increment)
    double alpha = atan(radius0/radius);
    double d_alpha = (M_PI-2*alpha) / (double)(n);
    // angle in horizontal plane (starting angle and increment)
    double phi = 0.;
    double d_phi = 2*M_PI / (double)steps;
    for (int i=0; i<steps; i++) {
      for (int j=0; j<slices; j++) {
        // calculate position
        double r = radius * sin(alpha);
        double h = sqrt(radius*radius - r*r);
        // bottom half of sphere?
        if (i<(slices/2)) h = -h;
        // calculate pose
        double y = r * sin(phi);
        double x = r * cos(phi);
        double z = -(radius + h);
        double yaw = phi + M_PI/2.0;
        double roll = 0.;
        if (PERPENDICULAR) {
          roll = alpha;
        }
        // generate measurements
        poses[pose_id] = Pose3d(x,y,z, yaw,0.,roll);
        if (pose_id>0) {
          
          add_constraint(outfile, poses, pose_id-1, pose_id);
          if (LOOP_CLOSING) {
            if (pose_id >= steps) {
              add_constraint(outfile, poses, pose_id-steps, pose_id);
            }
          }
        }
        // update for next step
        alpha += d_alpha;
        phi += d_phi;
        pose_id++;
      }
    }
  }
}
