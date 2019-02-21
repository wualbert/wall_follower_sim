#!/usr/bin/env python2

import numpy as np

def hough_line_detection(points, line_num = np.inf, r_res = 0.1, theta_res = 0.1, \
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
    while(1):
        hough_counts = flattened_hough_bins[indices[-(1+i)]]
        if hough_counts<=5:
            break
        hough_bin_x = int(np.floor(indices[-(1+i)]/hough_bins.shape[1]))
        hough_bin_y = int(np.mod(indices[-(1+i)], hough_bins.shape[1]))
        line_r = r_res*(hough_bin_x-int(np.ceil(r_max/r_res)))
        line_theta = theta_res*hough_bin_y
        i+=1
        #filter out similar lines
        similar_line = False
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
        if i>=line_num:
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
