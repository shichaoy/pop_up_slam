#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>    // std::swap

#include "isam_plane3d.h"


namespace isam {
  
//copied from pop_up_wall  rays is 3*n, each column is a ray staring from origin  plane is (4，1） parameters, compute intersection  output is 3*n 
void ray_plane_interact(const Eigen::MatrixXd &rays,const Eigen::Vector4d &plane,Eigen::MatrixXd &intersections)
{  
    Eigen::VectorXd frac=-plane[3]/(plane.head(3).transpose()*rays).array();   //n*1 
    intersections= frac.transpose().replicate<3,1>().array() * rays.array();
}
// copied from pop_up_wall (update_plane_equation_from_seg_fast), don't want to depend on that. no ground. ground_seg_ray=invK*groundlines  3*2n, precomputed
// output is not quaternion normized
void get_wall_plane_equation(const Eigen::MatrixXd& ground_seg_ray, const Eigen::Matrix4d& transToWorld, Eigen::MatrixXd& all_planes_sensor_out)
{
    if (ground_seg_ray.cols()>0)
    {
	Eigen::Vector4d ground_plane_world(0,0,-1,0);  //plane parameters for ground in world(footprint), treated as column vector
	Eigen::Vector4d ground_plane_sensor = transToWorld.transpose()*ground_plane_world;  // using a new pose to project

	Eigen::MatrixXd ground_seg3d_sensor; // 3*2n
	ray_plane_interact(ground_seg_ray,ground_plane_sensor,ground_seg3d_sensor);

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp;
	temp = ground_seg3d_sensor.transpose();
	temp.resize(ground_seg3d_sensor.cols()/2,6); // n * 6
	ground_seg3d_sensor = temp;

	int num_seg = ground_seg3d_sensor.rows();
	all_planes_sensor_out.resize(num_seg,4);

	for (int seg=0;seg<num_seg;seg++)
	{
	    Eigen::Vector3d partwall_line3d_sensor_bg=ground_seg3d_sensor.row(seg).head(3);
	    Eigen::Vector3d partwall_line3d_sensor_en=ground_seg3d_sensor.row(seg).tail(3);
	    Eigen::Vector3d temp1=partwall_line3d_sensor_en-partwall_line3d_sensor_bg;
	    Eigen::Vector3d temp2=ground_plane_sensor.head(3);
	    Eigen::Vector3d partwall_normal_sensor=temp1.cross(temp2);
	    double dist=-partwall_normal_sensor.transpose()*partwall_line3d_sensor_bg;
	    Eigen::Vector4d partwall_plane_sensor;   partwall_plane_sensor<<partwall_normal_sensor,dist;

	    all_planes_sensor_out.row(seg)=partwall_plane_sensor;
	}
    }
    else
    {
	all_planes_sensor_out.resize(0,4);
    }
}


}