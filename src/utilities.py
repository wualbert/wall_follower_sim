#!/usr/bin/env python2

import numpy as np

def hough_least_sq_line_detection(points, line_num = 1, r_res = 0.03,\
        theta_res = 0.03, return_polar = False):
    '''NOT WORKING'''
    #find max r
    r_max = 0
    for i,point in enumerate(points):
        r_max = max(np.hypot(point[0],point[1]), r_max)
    #create Hough bins
    hough_bins = np.zeros([2*int(np.ceil(r_max/r_res)),\
                            int(np.ceil(np.pi/theta_res))])
    #create Hough dictionaries
    hough_dict = {}

    #evaluate points
    for i,point in enumerate(points):
        for j in range(hough_bins.shape[1]):
            theta = j*theta_res
            r = point[0]*np.cos(theta)+point[1]*np.sin(theta)
            r_bin = int((r+r_max)/r_res)
            hough_bins[r_bin, j]+=1
            if (r_bin, j) not in hough_dict:
                new_set = set()
                new_set.add(tuple(point))
                hough_dict[(r_bin,j)]=new_set
            else:
                hough_dict[(r_bin,j)].add(tuple(point)) 
    #find max bins
    indices = np.argsort(hough_bins, axis=None)
    lines_hl = []
    line_num = min(line_num, len(indices))
    flattened_hough_bins = hough_bins.flatten()
    i=0
    lines_points = []
    max_index = len(flattened_hough_bins)
    while(1):
        if i >= max_index-1:
            break
        hough_counts = flattened_hough_bins[indices[-(1+i)]]
        if hough_counts<=7:
            break
        hough_bin_x = int(np.floor(indices[-(1+i)]/hough_bins.shape[1]))
        hough_bin_y = int(np.mod(indices[-(1+i)], hough_bins.shape[1]))
        line_r = r_res*(hough_bin_x-int(np.ceil(r_max/r_res)))
        line_theta = theta_res*hough_bin_y
        #filter out similar lines
        similar_line = None
        i+=1
        for sim_l_index, hl in enumerate(lines_hl):
            hl_theta = np.arctan(-1/hl[1])
            hl_r = hl[0]*np.sin(hl_theta)
            if (hl_r-line_r)**2+(hl_theta-line_theta)**2<=100000*(r_res**2+theta_res**2):
                print ('sim')
        points_on_line = hough_dict[(hough_bin_x,hough_bin_y)]
        if similar_line:
            lines_points[similar_line].update(points_on_line)
            points_on_line_array = np.asarray([list(m) for m in lines_points[similar_line]])
        else:
            lines_points.append(points_on_line)
            points_on_line_array = np.asarray([list(m) for m in lines_points[-1]])
        x = np.vstack([points_on_line_array[:,0],np.ones(points_on_line_array.shape[0])]).T
        m, b = np.linalg.lstsq(x,points_on_line_array[:,1])[0]
        #record line
        if similar_line:
            lines_hl[similar_line]=(m,b)
        else:
            lines_hl.append((m, b))
        if len(lines_hl)>=line_num:
            break
    print(lines_hl)
    if len(lines_hl)==0:
        #no line detected
        return None
    return np.asarray(lines_hl)


def hough_line_detection(points, line_num = np.inf, r_res = 0.05, theta_res = 0.05, \
        return_polar = False):
    #find max r
    r_max = 0
    for i,point in enumerate(points):
        r_max = max(np.hypot(point[0],point[1]), r_max)
    #create Hough bins
    hough_bins = np.zeros([2*int(np.ceil(r_max/r_res)),\
                            int(np.ceil(np.pi/theta_res))])
    #evaluate points
    for i,point in enumerate(points):
        for j in range(hough_bins.shape[1]):
            theta = j*theta_res
            r = point[0]*np.cos(theta)+point[1]*np.sin(theta)
            r_bin = int((r+r_max)/r_res)
            hough_bins[r_bin, j]+=1
    #find max bins
    indices = np.argsort(hough_bins, axis=None)
    lines_hl = []
    line_num = min(line_num, len(indices))
    flattened_hough_bins = hough_bins.flatten()
    i=0
    max_i = len(flattened_hough_bins)
    while(1):
        if i>=max_i:
            break
        hough_counts = flattened_hough_bins[indices[-(1+i)]]
        if hough_counts<=5:
            break
        hough_bin_x = int(np.floor(indices[-(1+i)]/hough_bins.shape[1]))
        hough_bin_y = int(np.mod(indices[-(1+i)], hough_bins.shape[1]))
        line_r = r_res*(hough_bin_x-int(np.ceil(r_max/r_res)))
        line_theta = theta_res*hough_bin_y
        #filter out similar lines
        similar_line = False
        i+=1
        for hl in lines_hl: 
            if abs(hl[0]-line_r)<=3*r_res or abs(hl[1]-line_theta)<=3*theta_res:
                similar_line = True
                break
        if similar_line:
            continue
        #convert r theta to m b
        line_theta = max(line_theta, 1e-3) #prevent divide by zero
        m = -1/np.tan(line_theta)
        b = line_r/np.sin(line_theta)
        #record line
        lines_hl.append((line_r, line_theta, m, b))
        if len(lines_hl)>=line_num:
            break
    lines_hl.sort(key = lambda x: abs(x[0]))
    if len(lines_hl)==0:
        #no line detected
        return None
    if return_polar:
        return np.asarray(lines_hl)
    else:
        return np.asarray(lines_hl)[:,2:]

def line_to_point_distance(line):
    assert(len(line)==2)
    magnitude = line[1]/np.sqrt(1+line[0]**2)
    return magnitude


