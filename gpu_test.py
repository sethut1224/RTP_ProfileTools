import rospy
import rosbag
import sys
import os
import matplotlib.pyplot as plt
import numpy as np
import math

sys.path.append('./profile_tools/')
import libprofile as pl

base_path = '/home/autoware/shared_dir/tc_result/'

dir_list = pl.get_files_in_dir(base_path)

for dir in dir_list:
    print('---------------%s--------------'%(dir))
    data_path = base_path+dir
    bag_list = pl.get_files_in_dir(data_path)
    hist_pdf_dict = dict()
    result_dict = dict()

    for bag in bag_list:
        result = list()
        hist = np.zeros(100,dtype=int)
        bag_path = data_path+'/'+bag

        for topic, msg, t in rosbag.Bag(bag_path).read_messages(topics=['/gpu_duration']):
            response_time = int(round(pl.to_milli(msg.elapse)))
            hist[response_time] = hist[response_time] + 1
            result.append(response_time)

        result_dict[bag] = result
        pdf = pl.make_pdf(hist)
        hist_pdf_dict[bag] = pdf

    sorted_hist_pdf = sorted(hist_pdf_dict)

    for name in sorted_hist_pdf:
        key = name
        value = hist_pdf_dict[name]
        cut_idx = pl.find_cut_index(value)
        data = np.cumsum(value)
        min, mean, max = pl.get_data_description(result_dict[key])
        pl.print_description(key, min, mean, max)
        pl.plot(data[:cut_idx], 'VDD'+key[-5:-4], 'GPU Response Time', 'Time(ms)', 'Cumulative Probability', [0,50])

    pl.show_plot()

    



    
    
    


# while True:
#     path = base_path+str(dir_count)+'/'
#     print(path)
#     is_dir = os.path.isdir(path)
#     if is_dir == False:
#         break
#     file_list = os.listdir(path)
#     file_list.sort()
#     file_count = 1
#     result_lists = []
#     all_duration_list = []
#     name_list = []
#     for file in file_list:
#         duration_list = []
#         file_count = file_count + 1
#         name_list.append(file)
#         for topic, msg, t in rosbag.Bag(path+file).read_messages(topics=['/gpu_duration']):
#             duration = msg.data
#             duration_list.append(duration)
            
#         all_duration_list.append(duration_list)
    
#     for list,name in zip(all_duration_list, name_list):
#         res = np.array(list)
#         print(name, np.min(res), np.mean(res), np.max(res))

#     dir_count = dir_count + 1


# while True:
#     path = base_path+str(dir_count)+'/'
#     is_dir = os.path.isdir(path)
#     if is_dir is False:
#         break
#     file_list = os.listdir(path)
#     if len(file_list) == 0:
#         dir_count = dir_count + 1   
#         continue
#     file_list.sort()
#     file_count = 0
#     result_list = []
#     result = dict()

#     duration_result = dict()
#     duration_list = []
#     for file in file_list:
#         for topic, msg, t in rosbag.Bag(path+file).read_messages(topics=['/gpu_duration']):
#             msg_id = msg.msg_info[3].msg_id
#             stime = msg.stime
#             etime = msg.etime
#             if result.get(msg_id) is not None:
#                 current_earlist_stime = result[msg_id][0]
#                 current_latest_etime = result[msg_id][1]
#                 if stime < current_earlist_stime:
#                     result[msg_id][0] = stime
                    
#                 if etime > current_latest_etime:
#                     result[msg_id][1] = etime
                    
#                 if (duration_result[msg_id] + msg.elapse).to_nsec() * 1.0e-6 > 20.0:
#                     continue
#                     print(duration_result[msg_id].to_nsec()*1.0e-6, msg.elapse.to_nsec()*1.0e-6)
#                 duration_result[msg_id] = duration_result[msg_id] + msg.elapse
#             else:
#                 result[msg_id] = [stime, etime]
#                 duration_result[msg_id] = msg.elapse
    
    
#     for value in result.values():
#         elapse = (value[1] - value[0]).to_nsec() * 1.0e-6
#         result_list.append(elapse)

#     for duration in duration_result.values():
#         duration_list.append(duration.to_nsec() * 1.0e-6)

#     result_np = np.array(result_list, dtype=np.float32)
#     duration_np = np.array(duration_list, dtype=np.float32)
#     print(path, 'start end', np.min(result_np), np.mean(result_np), np.max(result_np))
#     print(path, 'duration sum', np.min(duration_np), np.mean(duration_np), np.max(duration_np))
#     # break
#     dir_count = dir_count + 1
    



