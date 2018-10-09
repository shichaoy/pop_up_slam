#include "Map_plane.h"
using namespace std;
using namespace Eigen;


Map_plane::Map_plane()
{
      plane_vertex=nullptr;
}

void copy_plane(Map_plane* plane_to,const Map_plane* plane_from)
{
      plane_to->t_stamp=plane_from->t_stamp;
      plane_to->frame_seq_id=plane_from->frame_seq_id;
      plane_to->frame_plane_indice=plane_from->frame_plane_indice;
      plane_to->good_plane_indice=plane_from->good_plane_indice;

      plane_to->plane_cloud=plane_from->plane_cloud;
      plane_to->plane_dist_cam=plane_from->plane_dist_cam;
      plane_to->plane_bound_close_2D_polys=plane_from->plane_bound_close_2D_polys;
      plane_to->plane_bound_close_3D_polys=plane_from->plane_bound_close_3D_polys;
      plane_to->ground_seg2d_lines=plane_from->ground_seg2d_lines;
      plane_to->plane_cloud=plane_from->plane_cloud;
              
      plane_to->being_tracked_times += plane_from->being_tracked_times;
      plane_to->observer_frames.insert(plane_to->observer_frames.end(),plane_from->observer_frames.begin(),plane_from->observer_frames.end());
      
      // not copyied term  plane_vertex deteted_by_merge
}
