## Written and copyright by Mohit Agarwal
## Georgia Institute of Technology
## me.agmohit@gmail.com


#!/usr/bin/env python
# coding: utf-8

# In[1]:


# Import Libraries

import os
import numpy as np
from scipy.signal import *
import matplotlib
# get_ipython().magic(u'matplotlib notebook')
import matplotlib.pyplot as plt
import csv

from scipy.interpolate import interp1d


# In[2]:


# Algorithm Parameteres
# file_idx: S_N : S_N is subject ID

mode = True # mode True means EEG-IO, otherwise v/r (EEG-V) data
data_path = 'data/EEG-MB' if mode else 'data/EEG-V'

# data_path = 'EEG-V'
fs = 250.0

blink_command = 3
flag_soft = True # if True, consider soft blinks as ground truth

blink_len_max = 2.0 # in seconds
blink_len_min = 0.2 # in seconds
blink_len_diff = 100 # in seconds

blink_dist_max = 1.0 # in seconds (between two blinks)
blink_dist_min = 0.2 # in seconds (between two blinks)
blink_dist_tol = 0.0 # in seconds
blink_dist_tol_chan = 0.2 # in seconds
delta_init = 200 # in uvolts
slope_win = 20  # 10 means 10/250.0 s

influence = 0.05

corr_thresh = 0.6
corr_update_thresh = corr_thresh


# In[3]:


# function to bandpass filter
def lowpass(sig, fc, fs, butter_filt_order):
    B,A = butter(butter_filt_order, np.array(fc)/(fs/2), btype='low')
    return lfilter(B, A, sig, axis=0)

def pred_to_command(blinks):
    predicted_command = []
    for idx in range(blink_command-2,len(blinks)):
        flag_cont = True
        idx_b = 0
        while idx_b < blink_command-2:
            if abs(blinks[idx-idx_b,0]- blinks[idx-idx_b-1,1]) < 0.2:
                flag_cont = flag_cont
            else:
                flag_cont = flag_cont and abs(blinks[idx-idx_b,0]- blinks[idx-idx_b-1,1]) < blink_dist_max
                idx_b = idx_b + 1
            idx_b = idx_b + 1

        if flag_cont:
            predicted_command.append([blinks[idx-(blink_command-2),0], blinks[idx-(blink_command-2),1], blinks[idx,0], blinks[idx,1]])
    return predicted_command

def decode_stim(data_path, file_stim):
    interval_corrupt = []
    blinks = []
    n_corrupt = 0
    with open(os.path.join(data_path,file_stim)) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
    #         print row
            if row[0]=="corrupt":
                n_corrupt = int(row[1])
            elif n_corrupt > 0:
                if float(row[1]) == -1:
                    t_end = data_sig[-1,0]
                else:
                    t_end = float(row[1])
                interval_corrupt.append([float(row[0]), t_end])
                n_corrupt = n_corrupt - 1
            elif row[0]=="blinks":
                #check that n_corrupt is 0
                if not n_corrupt==0:
                    print "!Error in parsing"
            else:
                blinks.append([float(row[0]), int(row[1])])
    blinks = np.array(blinks)

    groundtruth_dblinks = []
    groundtruth_sblinks = [] # soft blinks groundtruth
    for idx in range(blink_command-1,len(blinks)):
        flag_dist, none_soft = True, True
        for idx_b in range(blink_command-1):
            blink_dist = blinks[idx-idx_b,0] - blinks[idx-idx_b-1,0]
            none_soft = none_soft and not ((blinks[idx-idx_b,1]== 2) or (blinks[idx-idx_b-1,1]== 2))
            flag_dist = flag_dist and (blink_dist < blink_dist_max) and (blink_dist > blink_dist_min)
        if flag_dist:
            if flag_soft or none_soft:
                groundtruth_dblinks.append([blinks[idx-(blink_command-1),0], blinks[idx-(blink_command-2),0], blinks[idx-1,0], blinks[idx,0]])
            else:
                groundtruth_sblinks.append([blinks[idx-(blink_command-1),0], blinks[idx-(blink_command-2),0], blinks[idx-1,0], blinks[idx,0]])

    # groundtruth_dblinks = np.array(groundtruth_dblinks)
#     print len(groundtruth_dblinks)
#     print len(groundtruth_sblinks)

    return interval_corrupt, groundtruth_dblinks, groundtruth_sblinks

# Function to find peaks

def args_init():
    args = {}
    args['mintab'], args['maxtab'] = [], []
    args['mn'], args['mx'] = float("inf"), -1*float("inf")
    args['mnpos'], args['mxpos'] = None, None
    args['min_left'], args['min_right'] = [], []
    args['lookformax'] = True
    args['delta'] = delta_init
    return args


def peakdet(time, value, args):
    foundMin = False
    if value > args['mx']:
        args['mx'] = value
        args['mxpos'] = time
    if value < args['mn']:
        args['mn'] = value
        args['mnpos'] = time
    if args['lookformax']:
        if value < args['mx'] - args['delta']:
            args['maxtab'].append([args['mxpos'], args['mx']])
            args['mn'] = value
            args['mnpos'] = time
            args['lookformax'] = False
    else:
        if value > args['mn'] + args['delta']:
            args['mintab'].append([args['mnpos'], args['mn']])
            args['min_left'].append([-1, -1])
            args['min_right'].append([-1, -1])
            args['mx'] = value
            args['mxpos'] = time
            args['lookformax'] = True
            foundMin = True
    return foundMin

def compute_slope(t_start, t_end, chan_id):
    return 1.0*(data_sig[t_end, chan_id] - data_sig[t_start, chan_id])/(t_end - t_start)

# Can simplify this - no need to trigger the increasing slope, just make sure delta follows
def find_expoints(time, chan_id, args):
    found_right, found_left = False, False
    t_start = int(time*fs)
    amp_min = data_sig[t_start, chan_id]

    # Find Right Point
    t_idx, t_right = t_start, t_start
    slope_trig = True

    while (data_sig[t_idx, chan_id] - amp_min <  args['delta']) and ( t_idx - t_start < 0.5*fs*blink_len_max) and (t_idx < len(data_sig) - slope_win):
        t_idx = t_idx + 1

    if ( t_idx - t_start >= 0.5*fs*blink_len_max) or (t_idx > len(data_sig) - slope_win):
        slope_trig = False
    else:
        slope = compute_slope(t_idx, t_idx + slope_win, chan_id)
        t_idx = t_idx + slope_win
    while slope_trig and (t_idx + slope_win) < (t_start + fs) and (t_idx + slope_win) < len(data_sig):
        prev_slope = slope
        slope = compute_slope(t_idx, t_idx + slope_win, chan_id)
        if  slope < prev_slope:
            slope_trig = False
            found_right = True
            t_right = t_idx + slope_win
        else:
            prev_slope = slope
            t_idx = t_idx + slope_win

    # Find Left Point
    t_idx, t_left = t_start, t_start
    slope_trig = True
    while data_sig[t_idx, chan_id] - amp_min <  args['delta'] and (t_start - t_idx < 0.5*fs*blink_len_max) and (t_idx > slope_win):
        t_idx = t_idx - 1
    if (t_start - t_idx >= 0.5*fs*blink_len_max) or (t_idx < slope_win):
        slope_trig = False
    else:
        slope = compute_slope(t_idx - slope_win, t_idx, chan_id)
        t_idx = t_idx - slope_win

    while slope_trig and (t_idx - slope_win) > (t_start - fs) and (t_idx - slope_win) > 0:
        prev_slope = slope
        slope = compute_slope(t_idx - slope_win, t_idx, chan_id)
        if  slope > prev_slope:
            slope_trig = False
            found_left = True
            t_left = t_idx - slope_win
        else:
            prev_slope = slope
            t_idx = t_idx - slope_win

    return t_left, t_right, found_right and found_left

def validate_extreme_points(args):
    flag_return = False
    if (not (args['min_right'][-2] == [-1, -1])) and (not (args['min_right'][-1] == [-1, -1])):
        blinkA_tsec_min, blinkB_tsec_min = args['mintab'][-2][0], args['mintab'][-1][0]
        blinkA_tsec_left, blinkA_tsec_right = args['min_left'][-2][0], args['min_right'][-2][0]
        blinkB_tsec_left, blinkB_tsec_right = args['min_left'][-1][0], args['min_right'][-1][0]
        if (blinkA_tsec_right - blinkA_tsec_min > 0.5*blink_len_min) and (blinkA_tsec_right - blinkA_tsec_min < 0.5*blink_len_max):
            if (blinkA_tsec_min - blinkA_tsec_left > 0.5*blink_len_min) and (blinkA_tsec_min - blinkA_tsec_left < 0.5*blink_len_max):
                if (blinkB_tsec_right - blinkB_tsec_min > 0.5*blink_len_min) and (blinkB_tsec_right - blinkB_tsec_min < 0.5*blink_len_max):
                    if (blinkB_tsec_min - blinkB_tsec_left > 0.5*blink_len_min) and (blinkB_tsec_min - blinkB_tsec_left < 0.5*blink_len_max):
                        if abs((blinkB_tsec_right - blinkB_tsec_left) - (blinkA_tsec_right - blinkA_tsec_left)) < blink_len_diff:
                            flag_return = True
    return flag_return

def compute_correlation(args, chan_id):
    blinkA_tsec_min, blinkB_tsec_min = int(fs*args['mintab'][-2][0]), int(fs*args['mintab'][-1][0])
    blinkA_tsec_left, blinkA_tsec_right = int(fs*args['min_left'][-2][0]), int(fs*args['min_right'][-2][0])
    blinkB_tsec_left, blinkB_tsec_right = int(fs*args['min_left'][-1][0]), int(fs*args['min_right'][-1][0])
    blinkA_left_mid, blinkA_mid_right = data_sig[blinkA_tsec_left:blinkA_tsec_min, chan_id], data_sig[blinkA_tsec_min:blinkA_tsec_right, chan_id]
    blinkB_left_mid, blinkB_mid_right = data_sig[blinkB_tsec_left:blinkB_tsec_min, chan_id], data_sig[blinkB_tsec_min:blinkB_tsec_right, chan_id]

    left_interp = interp1d(np.arange(blinkA_left_mid.size), blinkA_left_mid)
    compress_left = left_interp(np.linspace(0,blinkA_left_mid.size-1, blinkB_left_mid.size))
    right_interp = interp1d(np.arange(blinkA_mid_right.size), blinkA_mid_right)
    compress_right = right_interp(np.linspace(0,blinkA_mid_right.size-1, blinkB_mid_right.size))

    sigA = np.concatenate((compress_left, compress_right))
    sigB = np.concatenate((blinkB_left_mid, blinkB_mid_right))
    corr = np.corrcoef(sigA, sigB)[0,1]

    return corr

def check_overlap_blinks(double_blinks_chan1, double_blinks_chan2):
    result = False
    mid_chan1 = (double_blinks_chan1[-1][0] + double_blinks_chan1[-1][1])/2.0
    mid_chan2 = (double_blinks_chan2[-1][0] + double_blinks_chan2[-1][1])/2.0
    # Check if mid point of 1 lies in the other interval, and vice-a-versa
    if mid_chan1 < double_blinks_chan2[-1][1] and mid_chan1 > double_blinks_chan2[-1][0] or mid_chan2 < double_blinks_chan1[-1][1] and mid_chan2 > double_blinks_chan1[-1][0]:
            result = True

    return result


# In[4]:


# Evaluation Statistics
def evaluate(predicted_command, interval_corrupt, groundtruth_sblinks, groundtruth_dblinks, data_sig):

    total_len_sec = len(data_sig)/fs
    corrupt_time = 0
    for corrupt_interval in interval_corrupt:
        corrupt_time = corrupt_time + corrupt_interval[1] - corrupt_interval[0]

    total_len_sec = total_len_sec - corrupt_time

    pred = []
    for blink_intervals in predicted_command:

        intervalA_start, intervalA_end = blink_intervals[0], blink_intervals[1]
        intervalB_start, intervalB_end = blink_intervals[2], blink_intervals[3]

        blink_midA = (intervalA_start + intervalA_end)/2.0
        blink_midB = (intervalB_start + intervalB_end)/2.0
        flag = False
        for corrupt_interval in interval_corrupt:
            corrupt_start = corrupt_interval[0]
            corrupt_end = corrupt_interval[1]

            # See if there is any intersection
            if (corrupt_end > intervalA_start) and (intervalB_end > corrupt_start):
                flag = True

        if flag is False:
            for soft_blink_interval in groundtruth_sblinks:
    #             if intervalB_end > soft_blink_interval[0] and intervalA_start < soft_blink_interval[3]:
    #                 flag = True
                mid_pt_gtA = (soft_blink_interval[0] + soft_blink_interval[1])/2.0
                mid_pt_gtB = (soft_blink_interval[2] + soft_blink_interval[3])/2.0
                if blink_midA < soft_blink_interval[1] and blink_midA > soft_blink_interval[0]:
                    if blink_midB < soft_blink_interval[3] and blink_midB > soft_blink_interval[2]:
    #                     if mid_pt_gtA < intervalA_end and mid_pt_gtA > intervalA_start:
    #                         if mid_pt_gtB < intervalB_end and mid_pt_gtB > intervalB_start:
                        flag = True

        if flag is False:
            pred.append(blink_intervals)

    pred = np.array(pred)
    ground_truth = np.array(groundtruth_dblinks)

    cmat = np.array([[0,0],[0,0]])

    total_pred =  len(pred)
    total_gt = len(ground_truth)


    incorrect_interval = []
    for gt_sample in ground_truth:
        found = False
        for pred_sample in pred:
            mid_ptA = (pred_sample[0] + pred_sample[1])/2.0
            mid_ptB = (pred_sample[2] + pred_sample[3])/2.0
            if mid_ptA > gt_sample[0] and mid_ptA < gt_sample[1] and mid_ptB > gt_sample[2] and mid_ptB < gt_sample[3]:
                found = True
        if found:
            # Correct
            cmat[0,0] = cmat[0,0] + 1
        else:
            #Incorrect
            cmat[1,0] = cmat[1,0] + 1
            incorrect_interval.append(gt_sample)

    false_positive_interval = []
    for pred_sample in pred:
        mid_ptA = (pred_sample[0] + pred_sample[1])/2.0
        mid_ptB = (pred_sample[2] + pred_sample[3])/2.0
        found, falsep = False, True
        for ii, gt_sample in enumerate(ground_truth):
            if mid_ptA > gt_sample[0] and mid_ptA < gt_sample[1] and mid_ptB > gt_sample[2] and mid_ptB < gt_sample[3]:
                found, falsep = True, False
            elif pred_sample[0] <= gt_sample[3] and pred_sample[3] >= gt_sample[0]:
                falsep = False
            elif abs(gt_sample[0] - pred_sample[3]) < 0.1 or abs(gt_sample[3] - pred_sample[0]) < 0.1:
                falsep = False # making sure that same blink in ground truth doesn't affect the false+
            else:
                # check no interval overlap b/w this interval and previous detected intervals
                if len(false_positive_interval) > 0:
                    for fp_interval in false_positive_interval:
                        if pred_sample[0] <= fp_interval[3] and pred_sample[3] >= fp_interval[0]:
                            falsep = False
        if found:
            # Correct
            cmat[1,1] = cmat[1,1] + 1
        elif falsep:
            #False+
            cmat[0,1] = cmat[0,1] + 1
            false_positive_interval.append(pred_sample)

    precision = cmat[0,0]*1.0/(cmat[0,0] + cmat[0,1])
    recall = cmat[0,0]*1.0/(cmat[0,0] + cmat[1,0])
    f1 = 2*precision*recall/(precision+recall)
    fp = cmat[0,1]*3600.0/total_len_sec


    return cmat, recall, precision, f1, fp


# In[5]:


def identify_blinks(data_sig):
    args_chan1 = args_init()
    args_chan2 = args_init()

    double_blinks_chan1, double_blinks_chan2, double_blinks = [], [], []
    for idx in range(len(data_sig[:,0])):
        # Step1: Find minimas that crosses delta_init
        foundMin_chan1 = peakdet(data_sig[idx,0], data_sig[idx, 1], args_chan1)
        foundMin_chan2 = peakdet(data_sig[idx,0], data_sig[idx, 2], args_chan2)

        validate_extreme_points_chan1, validate_extreme_points_chan2 = False, False

        # Let's work only on channel 1 from now
        # Step 2: check that there are two adjacent minimas
        if foundMin_chan1 and len(args_chan1['mintab']) > 1:
            blinkA_chan1_tsec_min, blinkB_chan1_tsec_min = args_chan1['mintab'][-2][0], args_chan1['mintab'][-1][0]
            blinkAB_dist_chan1_tsec = blinkB_chan1_tsec_min - blinkA_chan1_tsec_min
            bool_time_criteria_chan1 = blinkAB_dist_chan1_tsec > blink_dist_min and blinkAB_dist_chan1_tsec < blink_dist_max + blink_dist_tol
            if bool_time_criteria_chan1:
                # Step 3: Find extreme points of blinks through slopes
                if args_chan1['min_right'][-2] == [-1, -1]:
                    blinkA_chan1_tidx_left, blinkA_chan1_tidx_right, foundA_chan1  = find_expoints(blinkA_chan1_tsec_min, 1, args_chan1)
                    if foundA_chan1:
                        args_chan1['min_right'][-2] = [data_sig[blinkA_chan1_tidx_right,0], data_sig[blinkA_chan1_tidx_right,1]]
                        args_chan1['min_left'][-2] = [data_sig[blinkA_chan1_tidx_left,0], data_sig[blinkA_chan1_tidx_left,1]]
                if args_chan1['min_right'][-1] == [-1, -1]:
                    blinkB_chan1_tidx_left, blinkB_chan1_tidx_right, foundB_chan1 = find_expoints(blinkB_chan1_tsec_min, 1, args_chan1)
                    if foundB_chan1:
                        args_chan1['min_right'][-1] = [data_sig[blinkB_chan1_tidx_right,0], data_sig[blinkB_chan1_tidx_right,1]]
                        args_chan1['min_left'][-1] = [data_sig[blinkB_chan1_tidx_left,0], data_sig[blinkB_chan1_tidx_left,1]]

                # Step4: Put restrictions on left and right extreme points in terms of blink length

                validate_extreme_points_chan1 = validate_extreme_points(args_chan1)

        if validate_extreme_points_chan1:
            # Blink1 in Chan1
            corr = compute_correlation(args_chan1, 1)

            # Blink Detected
            if corr > corr_thresh:
                blink1_amp = (args_chan1['min_left'][-2][1] + args_chan1['min_right'][-2][1])/2.0 - args_chan1['mintab'][-2][1]
                blink2_amp = (args_chan1['min_left'][-1][1] + args_chan1['min_right'][-1][1])/2.0 - args_chan1['mintab'][-1][1]
                double_blinks_chan1.append([args_chan1['mintab'][-2][0], args_chan1['mintab'][-1][0], corr, blink1_amp, blink2_amp])


                if len(double_blinks_chan2) > 0 :#and bool_blink_overlap:#(double_blinks_chan1[-1][1] - double_blinks_chan2[-1][1]) < blink_dist_tol_chan:
                    bool_blink_overlap = check_overlap_blinks(double_blinks_chan1, double_blinks_chan2)
                    if bool_blink_overlap and corr > corr_update_thresh:
                        args_chan1['delta'] = influence*0.5*0.33*(double_blinks_chan1[-1][3] + double_blinks_chan1[-1][4]) + (1-influence)*args_chan1['delta']
                        args_chan2['delta'] = influence*0.5*0.33*(double_blinks_chan2[-1][3] + double_blinks_chan2[-1][4]) + (1-influence)*args_chan2['delta']
                    else:
                        double_blinks.append(double_blinks_chan1[-1])
    #                 print args_chan1['delta'], args_chan2['delta']



        if foundMin_chan2 and len(args_chan2['mintab']) > 1:
            blinkA_chan2_tsec_min, blinkB_chan2_tsec_min = args_chan2['mintab'][-2][0], args_chan2['mintab'][-1][0]
            blinkAB_dist_chan2_tsec = blinkB_chan2_tsec_min - blinkA_chan2_tsec_min
            bool_time_criteria_chan2 = blinkAB_dist_chan2_tsec > blink_dist_min and blinkAB_dist_chan2_tsec < blink_dist_max + blink_dist_tol
            if bool_time_criteria_chan2:
                # Step 3: Find extreme points of blinks through slopes
                if args_chan2['min_right'][-2] == [-1, -1]:
                    blinkA_chan2_tidx_left, blinkA_chan2_tidx_right, foundA_chan2 = find_expoints(blinkA_chan2_tsec_min, 2, args_chan2)
                    if foundA_chan2:
                        args_chan2['min_right'][-2] = [data_sig[blinkA_chan2_tidx_right,0], data_sig[blinkA_chan2_tidx_right,2]]
                        args_chan2['min_left'][-2] = [data_sig[blinkA_chan2_tidx_left,0], data_sig[blinkA_chan2_tidx_left,2]]
                if args_chan2['min_right'][-1] == [-1, -1]:
                    blinkB_chan2_tidx_left, blinkB_chan2_tidx_right, foundB_chan2 = find_expoints(blinkB_chan2_tsec_min, 2, args_chan2)
                    if foundB_chan2:
                        args_chan2['min_right'][-1] = [data_sig[blinkB_chan2_tidx_right,0], data_sig[blinkB_chan2_tidx_right,2]]
                        args_chan2['min_left'][-1] = [data_sig[blinkB_chan2_tidx_left,0], data_sig[blinkB_chan2_tidx_left,2]]

                validate_extreme_points_chan2 = validate_extreme_points(args_chan2)

        if validate_extreme_points_chan2:
            # Blink1 in Chan2
            corr = compute_correlation(args_chan2, 2)
            if corr > corr_thresh:
                blink1_amp = (args_chan2['min_left'][-2][1] + args_chan2['min_right'][-2][1])/2.0 - args_chan2['mintab'][-2][1]
                blink2_amp = (args_chan2['min_left'][-1][1] + args_chan2['min_right'][-1][1])/2.0 - args_chan2['mintab'][-1][1]
                double_blinks_chan2.append([args_chan2['mintab'][-2][0],args_chan2['mintab'][-1][0], corr, blink1_amp, blink2_amp])

                if len(double_blinks_chan1) > 0:
                    bool_blink_overlap = check_overlap_blinks(double_blinks_chan1, double_blinks_chan2)
                    if bool_blink_overlap and corr > corr_update_thresh:
                        args_chan1['delta'] = influence*0.5*0.33*(double_blinks_chan1[-1][3] + double_blinks_chan1[-1][4]) + (1-influence)*args_chan1['delta']
                        args_chan2['delta'] = influence*0.5*0.33*(double_blinks_chan2[-1][3] + double_blinks_chan2[-1][4]) + (1-influence)*args_chan2['delta']
                    else:
                        double_blinks.append(double_blinks_chan2[-1])

    mintab1, maxtab1 = np.array(args_chan1['mintab']), np.array(args_chan1['maxtab'])
    mintab2, maxtab2 = np.array(args_chan2['mintab']), np.array(args_chan2['maxtab'])
    minright1, minleft1 = np.array(args_chan1['min_right']), np.array(args_chan1['min_left'])
    minright2, minleft2 = np.array(args_chan2['min_right']), np.array(args_chan2['min_left'])

    double_blinks_chan1, double_blinks_chan2 = np.array(double_blinks_chan1), np.array(double_blinks_chan2)

    double_blinks = np.array(double_blinks)
    # print double_blinks

    predicted_command = pred_to_command(double_blinks)
    return double_blinks, predicted_command


# In[6]:


# Reading data files
# replace w/ "V_data" or "R_data" for individual analysis
list_of_files = [f for f in os.listdir(data_path) if os.path.isfile(os.path.join(data_path, f)) and '_data' in f]
recall_total, precision_total, f1_total, fp_total = 0, 0, 0, 0
for file_sig in list_of_files:
    # Signal and Stimulation File name
    file_stim = file_sig.replace('_data','_labels')

    # Loading Data
    if mode:
        data_sig = np.loadtxt(open(os.path.join(data_path,file_sig), "rb"), delimiter=";", skiprows=1, usecols=(0,1,2))
    else:
        data_sig = np.loadtxt(open(os.path.join(data_path,file_sig), "rb"), delimiter=",", skiprows=5, usecols=(0,1,2))
        data_sig = data_sig[0:(int(200*fs)+1),:]
        data_sig = data_sig[:,0:3]
        data_sig[:,0] = np.array(range(0,len(data_sig)))/fs

    # Step1: Low Pass Filter
    data_sig[:,1] = lowpass(data_sig[:,1], 10, fs, 4)
    data_sig[:,2] = lowpass(data_sig[:,2], 10, fs, 4)

    interval_corrupt, groundtruth_dblinks, groundtruth_sblinks = decode_stim(data_path, file_stim)
    double_blinks, predicted_command = identify_blinks(data_sig)
    cmat, recall, precision, f1, fp = evaluate(predicted_command, interval_corrupt, groundtruth_sblinks, groundtruth_dblinks, data_sig)


    recall_total = recall_total + recall
    precision_total = precision_total + precision
    f1_total = f1_total + f1
    fp_total = fp_total + fp

    print file_sig, recall, fp, precision, f1

print "Total", recall_total/len(list_of_files), fp_total/len(list_of_files), precision_total/len(list_of_files), f1_total/len(list_of_files)
