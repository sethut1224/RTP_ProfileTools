import rospy
import rosbag
import sys
import os
import matplotlib.pyplot as plt
import numpy as np
import math

def get_files_in_dir(dir_path):
    if not os.path.isdir(dir_path):
        print('is not directory!')
        return list()

    return sorted(os.listdir(dir_path))

def calc_time_diff_milli(start, end):
    return (end - start).to_nsec() * 1.0e-6

def to_milli(time):
    return time.to_nsec() * 1.0e-6

def find_cut_index(data):
    for x in range(len(data)-1, -1, -1):
        if data[x] != 0.0:
            return x+1

def make_pdf(data):
    denom = float(sum(data))
    pdf = [x / denom for x in data]

    return pdf

def get_data_description(data):
    np_data = np.array(data)

    return np.min(np_data), np.mean(np_data), np.max(np_data)

def print_description(string, min, mean, max):
    print('%s : %f / %f / %f'%(string, min, mean, max))
    
def plot(data, label=None, title=None, xlabel=None, ylabel=None, xlim=None, ylim=None):
    plt.plot(data, label=label)
    plt.title(title)
    plt.legend()
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.xlim(xlim)
    plt.ylim(ylim)

def show_plot():
    plt.show()
