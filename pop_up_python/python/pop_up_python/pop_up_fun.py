import numpy as np
import cv2
import time
from skimage.measure import find_contours,approximate_polygon
import math

from intervaltree import Interval, IntervalTree  # to select not overlapping edges

import numpy_opencv

def load_params():
    params={'Kalib': np.array([[570.34,0,319.5],[0, 570.34, 239.5], [0, 0, 1]]),
            'width': 640,
            'height': 512,
            'ceiling_height': 2.8,
            'ground_label_id':1,
            'wall_label_id':0,
            'polyline_tolerance': 15,
            'downsample_popup': 0,
            'erosion_distance':11,
            'dilation_distance':11,            
            'plot_ground_polys': 0,
            'pre_vertical_thre':15,
            'pre_minium_len': 15,
            'pre_boundary_thre':5,
            'pre_merge_angle_thre':10,
            'pre_merge_dist_thre':10,            
            'pre_proj_angle_thre':20,
            'pre_proj_cover_thre':0.6,
            'pre_proj_cover_large_thre':0.8,
            'pre_proj_dist_thre':100,            
            'pre_contour_close_thre':50,
            'interval_overlap_thre':20,
            'post_short_thre':30,
            'post_bind_dist_thre':10,
            'post_merge_dist_thre':20,
            'post_merge_angle_thre':10,
            'post_extend_thre':15,
            'dark_threshold':90
           }
    params['invK'] = np.linalg.inv(params['Kalib'])
    return params

def get_intersect_len_tree(query, interval_tree):
    '''get the total intersection length of a query segment with a interval tree storing all the existing segments
        query 1*2 array  [small, large], must be sorted. if not sorted, need to manually sort here 
    '''
    all_intersect_length=0
    all_intervals=sorted(interval_tree.search(query[0],query[1]))
    if (len(all_intervals)==0):   # if not found any intervals
        return all_intersect_length
    else:
        for i in xrange(len(all_intervals)):            
            overlap1=query[1]- all_intervals[i].begin;
            overlap2=all_intervals[i].end-query[0];
            intersect_length=min(overlap1, overlap2, abs(query[1]-query[0]), all_intervals[i].length())
            all_intersect_length = all_intersect_length+intersect_length    
    return all_intersect_length 


def get_approxi_polys(label_map,polyline_tolerance):
    '''ICRA 2016 function, C++ call python function to find ground polygon label_map: ground is 0, wall is 1. it is preprocessed label map (already erosion
    dilation... just want to call python approximate polygon)
    '''
    contours=find_contours(label_map, 0)  # row ind, col ind  y x
    if contours !=[]:
        # there might be several contours, find the longest one
        contour_lengths=np.zeros(len(contours))
        for contour_ind in xrange(len(contours)):
            bg_end_diff=contours[contour_ind][0,:]-contours[contour_ind][-1,:]
            length=np.sum(np.linalg.norm(bg_end_diff))
            contour_lengths[contour_ind]=length
        ground_contour_ind=np.argmax(contour_lengths)

        ground_contour=contours[ground_contour_ind].copy()  # whether swap x and y
        ground_contour[:,0]=contours[ground_contour_ind][:,1]
        ground_contour[:,1]=contours[ground_contour_ind][:,0]

        # piecewise poly line simplification  # each row is a vertex's coordinate x and y
        ground2d=approximate_polygon(ground_contour, polyline_tolerance)
        ground2d=ground2d[::-1,:]  # reverse the order 
        return ground2d


def python_get_contours(label_map):
    '''
    label_map: ground is 0, wall is 1
    '''
    contours=find_contours(label_map, 0)  # row ind, col ind  y x
    final_line_segs=np.array([]);final_extend_segs=np.array([]);final_lines_in_extend_ind=np.array([]);    
    if contours !=[]:
        # there might be several contours, find the longest one
        contour_lengths=np.zeros(len(contours))
        for contour_ind in xrange(len(contours)):
            bg_end_diff=contours[contour_ind][0,:]-contours[contour_ind][-1,:]
            length=np.sum(np.linalg.norm(bg_end_diff))
            contour_lengths[contour_ind]=length
        ground_contour_ind=np.argmax(contour_lengths)

        ground_contour=contours[ground_contour_ind].copy()  # whether swap x and y
        ground_contour[:,0]=contours[ground_contour_ind][:,1]
        ground_contour[:,1]=contours[ground_contour_ind][:,0]
        
    return ground_contour[0:-1:20]


def interval_tree_optimization(close_nonvertical_lines, params):
    ''' using optimization to select the best set of edges which maximize union in x direction while minimizing
        intersection in x direction. use a greedy selection method
        lines n*4 lines [x1 y1 x2 y2]   output is m*4 remaining lines. there won't be any overlapping between edges
    '''
    optimized_line_segs=[]  # each row is x1,y1,x2,y2       
    if (close_nonvertical_lines.shape[0]>0):
        closed_list=np.empty((0,1),int)  # already visited actual index
        open_list=np.arange(close_nonvertical_lines.shape[0])  # remaining to visit actual index
    
        # initially select the longest line    TODO. may be a mix of length, x_duration
        close_nonvertical_lengths=np.linalg.norm(close_nonvertical_lines[:,2:4]-close_nonvertical_lines[:,0:2],axis=1)    
        best_ind=np.argmax(close_nonvertical_lengths[open_list])
        current_node=open_list[best_ind]  # actual index of raw lines (close_nonvertical_lines)
        open_list=np.delete(open_list,best_ind)    
        closed_list=np.append(closed_list,current_node)
    
        x_cover_range=np.array(close_nonvertical_lines[current_node][0:3:2])  #all the line x range covered  n*2 array
        x_cover_range=x_cover_range.reshape(1,2)
        x_cover_tree=IntervalTree()
        x_cover_tree[close_nonvertical_lines[current_node][0]:close_nonvertical_lines[current_node][2]] = (close_nonvertical_lines[current_node,:]).tolist()
        # each row is a segment interval
        
        while (open_list.size!=0):
            # find all not overlapping or little overlapping lines. then select the longest
            potential_open_ind=np.empty((0,1),int)  # index of openlist
            for i in range(open_list.size):
                inter_length=get_intersect_len_tree(close_nonvertical_lines[open_list[i]][0:3:2],x_cover_tree)
                if (inter_length<params['interval_overlap_thre']):  # if very small overlapping
                    potential_open_ind=np.append(potential_open_ind,i)
            if potential_open_ind.size==0:
                break;  # cannot find potential list
            best_potentia_ind=np.argmax(close_nonvertical_lengths[open_list[potential_open_ind]])
            current_node=open_list[potential_open_ind[best_potentia_ind]]
            closed_list=np.append(closed_list, current_node)  # push into visited node
            open_list=np.delete(open_list,potential_open_ind[best_potentia_ind])    # delete from open list        
            x_cover_tree[close_nonvertical_lines[current_node][0]:close_nonvertical_lines[current_node][2]] = (close_nonvertical_lines[current_node,:]).tolist()    
        
        # handle all the overlapping areas. use the longer edge. or you can also manually choose the mean point
        # after this, there won't be any overlapping in x direction  note!!! this will increase the number of edges
        raw_seg=len(x_cover_tree)
        x_cover_tree.split_overlaps()  # break into many small edges if they overlap
    
        needs_post_processing=(len(x_cover_tree)!=raw_seg)
        if (needs_post_processing):  # if there exist overlap
            all_intervals=sorted(x_cover_tree)
            to_delete_intervals=[]  # to remove short intervals
            for i in range(len(all_intervals)):
                for j in range(i+1, len(all_intervals)):
                    if ( (all_intervals[i].begin==all_intervals[j].begin) & (all_intervals[i].end==all_intervals[j].end) ):            
                        if ( (all_intervals[i].data[2]-all_intervals[i].data[0]) < (all_intervals[j].data[2]-all_intervals[j].data[0]) ):
                            to_delete_intervals.append(all_intervals[i]) # TODO or remove the longer lines
                        else:
                            to_delete_intervals.append(all_intervals[j])
            to_delete_intervals=list(set(to_delete_intervals)) # remove duplicate terms, avoid being deleted twice
            for k in range(len(to_delete_intervals)):
                x_cover_tree.remove(to_delete_intervals[k])    
                
        # merge the intervals if there are originally in the same line (share the same interval data) and currently adjacent
        if (needs_post_processing):
            can_merge=1;
            counter=0;
            while ((can_merge==1)& (counter<100)):
                can_merge=0;
                counter=counter+1;
                all_intervals=sorted(x_cover_tree)
                for i in range(len(all_intervals)):
                    for j in range(i+1, len(all_intervals)):
                        # if there are ajacent, and original come from the same line, then, merge them
                        if ( ((all_intervals[i].begin==all_intervals[j].end) | (all_intervals[i].end==all_intervals[j].begin)) & (all_intervals[i].data==all_intervals[j].data) ):
                            merge_min=min(all_intervals[i].begin,all_intervals[j].begin);
                            merge_max=max(all_intervals[i].end,all_intervals[j].end);
                            merge_data=all_intervals[j].data
                            x_cover_tree.remove(all_intervals[i])
                            x_cover_tree.remove(all_intervals[j])
                            x_cover_tree.addi(merge_min,merge_max,merge_data)
                            can_merge=1;
                            break;
                    if (can_merge==1):
                        break;            
                        
        # refine the remained ones, so that all the intervals data (line boundaries) matches the begin/end point of interval tree
        # done of interval tree optimization part
        all_intervals=sorted(x_cover_tree)  # all_intervals is linked with the tree, share the same memory    
        if (needs_post_processing):    
            for i in range(len(all_intervals)):
                if ( (all_intervals[i].begin!=all_intervals[i].data[0]) | (all_intervals[i].end!=all_intervals[i].data[2])):
                    raw_line=all_intervals[i].data
                    frac1=float((all_intervals[i].begin-raw_line[0]))/(raw_line[2]-raw_line[0])
                    frac2=float((all_intervals[i].end-raw_line[0]))/(raw_line[2]-raw_line[0])
                    optimized_line_segs.append([all_intervals[i].begin, int(frac1*(raw_line[3]-raw_line[1])+raw_line[1]), all_intervals[i].end, int(frac2*(raw_line[3]-raw_line[1])+raw_line[1])])
                else:
                    optimized_line_segs.append(all_intervals[i].data)
        else:
            for i in range(len(all_intervals)):    
                optimized_line_segs.append(all_intervals[i].data)

    optimized_line_segs=np.array(optimized_line_segs)    
    return optimized_line_segs