import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import rosbag
import math
import ros
import rospy
import os
import copy


def get_task_name_shortcut(name):
    
    if name.find('dummy_lidar_kf_contour_track') != -1:
        return 'DUM_K'
    elif name.find('dummy_ndt_matching') != -1:
        return "DUM_N"
    elif name.find('dummy_op_motion_predictor_1') != -1:
        return "DUM_M_1"
    elif name.find('dummy_op_motion_predictor_2') != -1:
        return "DUM_M_2"
    elif name.find('dummy_op_motion_predictor_3') != -1:
        return "DUM_M_3"
    elif name.find('dummy_op_motion_predictor_4') != -1:
        return "DUM_M_4"
    elif name.find('dummy_op_trajectory_generator') != -1:
        return "DUM_G"
    elif name.find('dummy_pure_pursuit') != -1:
        return "DUM_P"
    elif name.find('dummy_op_trajectory_evaluator') != -1:
        return 'DUM_V'
    elif name.find('dummy_ekf_localizer') != -1:
        return 'DUM_E'
    elif name.find('dummy_vision_darknet_detect_f120') != -1:
        return 'DUM_D'
    elif name.find('vlidar_driver_det') != -1:
        return 'LID_D'
    elif name.find('vlidar_driver_loc') != -1:
        return 'LID_L'
    elif name.find('vcamera_driver_f120_1') != -1:
        return 'CAM_1'
    elif name.find('vcamera_driver_f120_2') != -1:
        return 'CAM_2'
    elif name.find('vcamera_driver_f120_3') != -1:
        return 'CAM_3'
    elif name.find('vcamera_driver_f120_4') != -1:
        return 'CAM_4'
    elif name.find('ray_ground_filter') != -1:
        return 'GRF'
    elif name.find('vcan_driver') != -1:
        return 'ODM'
    elif name.find('vehicle_status_converter') != -1:
        return 'STC'
    elif name.find('ekf_localizer') != -1:
        return 'EKF'
    elif name.find('op_trajectory_generator') != -1:
        return 'GEN'
    elif name.find('op_trajectory_evaluator') != -1:
        return 'TEV'
    elif name.find('op_behavior_selector') != -1:
        return 'BSE'
    elif name.find('pure_pursuit') != -1:
        return 'PPS'
    elif name.find('voxel_grid_filter') != -1:
        return 'XGF'
    elif name.find('ndt_matching') != -1:
        return 'NDT'
    elif name.find('lidar_euclidean_cluster_detect') != -1:
        return 'UCL'
    elif name.find('lidar_kf_contour_track') != -1:
        return 'KFT'
    elif name.find('range_vision_fusion_f120_1') != -1:
        return 'RVF_1'
    elif name.find('range_vision_fusion_f120_2') != -1:
        return 'RVF_2'
    elif name.find('range_vision_fusion_f120_3') != -1:
        return 'RVF_3'
    elif name.find('range_vision_fusion_f120_4') != -1:
        return 'RVF_4'
    elif name.find('op_motion_predictor_1') != -1:
        return 'OMP_1'
    elif name.find('op_motion_predictor_2') != -1:
        return 'OMP_2'
    elif name.find('op_motion_predictor_3') != -1:
        return 'OMP_3'
    elif name.find('op_motion_predictor_4') != -1:
        return 'OMP_4'
    elif name.find('vision_darknet_detect_f120_1') != -1:
        return 'DND_1'
    elif name.find('vision_darknet_detect_f120_2') != -1:
        return 'DND_2'
    elif name.find('vision_darknet_detect_f120_3') != -1:
        return 'DND_3'
    elif name.find('vision_darknet_detect_f120_4') != -1:
        return 'DND_4'
    elif name.find('vision_beyond_track_f120_1') != -1:
        return 'VBT_1'
    elif name.find('vision_beyond_track_f120_2') != -1:
        return 'VBT_2'
    elif name.find('vision_beyond_track_f120_3') != -1:
        return 'VBT_3'
    elif name.find('vision_beyond_track_f120_4') != -1:
        return 'VBT_4'
    else:
        return None

def get_sensor_id(type_name):

    sensor_id = -1

    if type_name == 'LIDAR_LOC':
        sensor_id = 0
    elif type_name.find('LIDAR_DET')!=-1:
        sensor_id = 1
    elif type_name == 'ODOM':
        sensor_id = 2
    elif type_name.find('CAMERA_1') != -1:
        sensor_id = 3
    elif type_name.find('CAMERA_2') != -1:
        sensor_id = 4
    elif type_name.find('CAMERA_3') != -1:
        sensor_id = 5
    elif type_name.find('CAMERA_4') != -1:
        sensor_id = 6    
    else:
        sensor_id = -1

    return sensor_id

def find_bag_name_from_shortcut(bag_list, task):
    for j in range(0, len(bag_list)):
        if get_task_name_shortcut(bag_list[j]) == task:
            return bag_list[j]

def make_list(bag_list, bag_dir, task_list, sensor_id, cpu_et_list, list_type):
    et_list = list()
    prev_id = -1
    target_sensor = -1
    topic = ''

    if list_type =='rt':
        topic = '/rtp/response_time'
    elif list_type =='et':
        topic = '/rtp/execution_time'

    for task_name in task_list:
        bag_name = find_bag_name_from_shortcut(bag_list, task_name)
        for topic, msg, t in rosbag.Bag(bag_dir + bag_name).read_messages(topics=[topic]):
            stime = msg.stime.to_time()
            etime = msg.etime.to_time()
            elapse = msg.elapse.to_nsec()
            data = [task_name, stime, etime, elapse, msg.msg_info[sensor_id].msg_id]
            cpu_et_list.append(data)


def process_et(bag_list, task_cpu_info, bag_dir, sensor_info, virtual=True):

    result = dict()
    tasks_info = dict()

    for cpu_id,tasks_temp in enumerate(task_cpu_info):
        tasks_info[cpu_id] = copy.deepcopy(tasks_temp)
        sensor_id = get_sensor_id(sensor_info[cpu_id])

        
        if len(tasks_info[cpu_id]) > 1:
            for i in range(0, len(tasks_info[cpu_id])-1):
                tasks_info[cpu_id][i].append(tasks_info[cpu_id][i+1][0])
        
        for tasks in tasks_info[cpu_id]:
            if len(tasks) > 1:
                
                profiled_list = list()
                cpu_et_list = list()
                target_task_idx = len(tasks)-1
                start_task_name = tasks[0]
                end_task_name = tasks[-1]
                
                make_list(bag_list, bag_dir, tasks, sensor_id, profiled_list, 'et')
                profiled_list.sort(key=lambda x:x[1])

                for i in range(0, len(profiled_list)):
                    if profiled_list[i][0] == start_task_name:
                        selected_task = profiled_list[i]
                        selected_task_stime = selected_task[1]
                        if len(profiled_list) > i + target_task_idx:
                            target_task = profiled_list[i+target_task_idx]
                            if target_task[0] == end_task_name:
                                target_task_stime = target_task[1]
                                stime = rospy.rostime.Time.from_sec(selected_task_stime)
                                etime = rospy.rostime.Time.from_sec(target_task_stime)
                                et = (etime-stime).to_nsec() * 1.0e-6 + 0.5
                                if profiled_list[i][4] == 17833508:
                                    continue
                                cpu_et_list.append(et)


                #result.append([start_task_name, cpu_et_list])
                result[start_task_name] = cpu_et_list

                print('%s to %s avg : %f' %(start_task_name, end_task_name, sum(cpu_et_list)/len(cpu_et_list)))
                print('%s to %s max : %f' %(start_task_name, end_task_name, max(cpu_et_list)))
                print('%s to %s min : %f' %(start_task_name, end_task_name, min(cpu_et_list)))
                print('')
    return result                 


def process_rt(bag_list, task_cpu_info, bag_dir, sensor_info, virtual=True):

    result = dict()
    tasks_info = dict()

    for cpu_id,tasks_temp in enumerate(task_cpu_info):
        tasks_info[cpu_id] = copy.deepcopy(tasks_temp)
        sensor_id = get_sensor_id(sensor_info[cpu_id])

        for tasks in tasks_info[cpu_id]:
            if tasks[-1].find('DUM') != -1:
                tasks.pop(-1)

            if len(tasks) > 1:
                profiled_list = list()
                cpu_rt_list = list()
                target_task_idx = len(tasks)-1
                start_task_name = tasks[0]
                end_task_name = tasks[-1]
    
                make_list(bag_list, bag_dir, tasks, sensor_id, profiled_list, 'rt')
                profiled_list.sort(key=lambda x:x[1])

                for i in range(0, len(profiled_list)):
                    if profiled_list[i][0] == start_task_name:
                        selected_task = profiled_list[i]
                        selected_task_stime = selected_task[1]
                        if len(profiled_list) > i + target_task_idx:
                            target_task = profiled_list[i+target_task_idx]
                            if target_task[0] == end_task_name:
                                target_task_etime = target_task[2]
                                stime = rospy.rostime.Time.from_sec(selected_task_stime)
                                etime = rospy.rostime.Time.from_sec(target_task_etime)
                                rt = (etime-stime).to_nsec() * 1.0e-6 - 0.15
                                if profiled_list[i][4] == 17833508:
                                    continue
                                if start_task_name == 'ODM' and rt > 3.0:
                                    continue
                                    
                                cpu_rt_list.append(rt)

                #result.append([start_task_name, cpu_rt_list])
                result[start_task_name]=cpu_rt_list

                print('%s to %s avg : %f' %(start_task_name, end_task_name, sum(cpu_rt_list)/len(cpu_rt_list)))
                print('%s to %s max : %f' %(start_task_name, end_task_name, max(cpu_rt_list)))
                print('%s to %s min : %f' %(start_task_name, end_task_name, min(cpu_rt_list)))
                print('')
    return result                         



def process_e2e(bag_list, rt_path, bag_dir):
    result = dict()

    for path in rt_path.items():
        rt_list = list()
        start_task = path[0]
        end_task = path[1][0]
        path_name = path[1][1]

        sensor_id = get_sensor_id(path[1][1])
        detection_sensor = 0
        
        if start_task.find('LID_D') != -1:
            detection_sensor=int(start_task.rsplit('_')[-1])
            start_task = 'LID_D'

        start_bag_name = find_bag_name_from_shortcut(bag_list, start_task)
        end_bag_name = find_bag_name_from_shortcut(bag_list, end_task)

        start_bag = rosbag.Bag(bag_dir+start_bag_name)
        end_bag = rosbag.Bag(bag_dir+end_bag_name)

        rt_info = dict()

        prev_id = 0
        detection_sensor = int(detection_sensor)+2
        print(path_name, start_task, detection_sensor)

        for topic, msg, t in end_bag.read_messages(topics=['/rtp/response_time']):
            if start_task=='LID_D':
                if prev_id == msg.msg_info[detection_sensor].msg_id:
                    continue
                else:
                    prev_id = msg.msg_info[detection_sensor].msg_id

            if msg.msg_info[sensor_id].msg_id == 17833508:
                continue
            if rt_info.get(msg.msg_info[sensor_id].msg_id) == None:
                rt_info[msg.msg_info[sensor_id].msg_id] = [msg, False]
        
        for topic, msg, t in start_bag.read_messages(topics=['/rtp/response_time']):
            if rt_info.get(msg.msg_info[sensor_id].msg_id) != None:
                if rt_info[msg.msg_info[sensor_id].msg_id][1] == False:
                    rt = (rt_info[msg.msg_info[sensor_id].msg_id][0].etime - msg.stime).to_nsec() * 1.0e-6 - 0.15
                    rt_info[msg.msg_info[sensor_id].msg_id][1] = True
                    rt_list.append(rt) 

        #result.append([path_name,rt_list])
        result[path_name] = rt_list

        print(start_task+' to '+end_task, 'avg', sum(rt_list)/len(rt_list))
        print(start_task+' to '+end_task, 'max', max(rt_list))
        print(start_task+' to '+end_task, 'min', min(rt_list))
        print('')

    return result


def pearson_test_coefficient(bag_list, task_info, sensor_info, bag_dir):

    tasks_info = dict()
    result = list()

    for id,tasks_temp in enumerate(task_info):
        tasks_info[id] = copy.deepcopy(tasks_temp)
        
        profiled_list = list()
        cpu_et_list =list()
        start_task_name = tasks_info[id][0] 
        end_task_name = tasks_info[id][-1]

        target_task_idx = len(tasks_info[id])-1
        sensor_id=get_sensor_id(sensor_info[id])
        make_list(bag_list, bag_dir, tasks_info[id], sensor_id, profiled_list, 'et')

        profiled_list.sort(key=lambda x:x[1])

        for i in range(0, len(profiled_list)):
            if profiled_list[i][0] == start_task_name:
                selected_task = profiled_list[i]
                selected_task_stime = selected_task[1]
                if len(profiled_list) > i + target_task_idx:
                    target_task = profiled_list[i+target_task_idx]
                    if target_task[0] == end_task_name:
                        target_task_stime = target_task[1]
                        stime = rospy.rostime.Time.from_sec(selected_task_stime)
                        etime = rospy.rostime.Time.from_sec(target_task_stime)
                        et = (etime-stime).to_nsec() * 1.0e-6
                        cpu_et_list.append([et, target_task[4], selected_task_stime])
            else:
                continue

        result.append(cpu_et_list)

    min_index = max(result[0][0][1], result[1][0][1])
    prev_list = list()
    next_list = list()
    
    for i in range(0, len(result[0])):
        if result[0][i][1] >= min_index:
            prev_list.append(result[0][i])
    for i in range(0, len(result[1])):
        if result[1][i][1] >= min_index:
            next_list.append(result[1][i])

    prev_index = 0

    
    prev_new_list = list()
    next_new_list = list()
    for i in range(0, len(prev_list)):
        prev_data = prev_list[i]

        for j in range(prev_index, len(next_list)):
            next_data = next_list[j]

            if abs(next_data[2] - prev_data[2]) < 0.005:
                print(prev_data, next_data)
                print('same')
                prev_index = j
                prev_new_list.append(prev_data[0])
                next_new_list.append(next_data[0])
                break 
    
    corr = np.corrcoef(np.array(prev_new_list), np.array(next_new_list))
    print(corr[0][1])
    # max_index = min(len(prev_list),len(next_list))
    
    # for i in range(0, max_index+1):
    #     if prev_list[0][1] != next_list[0][1]:
    #         print('error')
    # prev_list = np.array(prev_list)[:,0]
    # next_list = np.array(next_list)[:,0]
    # print('%s->%s: '%(paper_name(names[0][0],names[0][-2]),paper_name(tasks[0],tasks[-2])))        
    # corr = np.corrcoef(prev_list[:max_index],np.array(next_list[:max_index]))
    # print(corr[0][1])    

def pearson_correlation_coefficient2(bag_list, task_info, sensor_info, bag_dir):
    if len(task_info) != len(sensor_info):
        print('task info %d, sensor_info %d is not matched!'%( len(task_info), len(sensor_info)))

    tasks_info = dict()

    for id, tasks_temp in enumerate(task_info):
        tasks_info[id] = copy.deepcopy(tasks_temp)
        sensor_id = get_sensor_id(sensor_info[id])
        result = list()
        names = list()
        for tasks in tasks_info[id]:
            names.append(tasks)
            profiled_list = list()
            cpu_et_list = list()
            start_task_name = tasks[0]
            end_task_name = tasks[-1]
            target_task_idx = len(tasks)-1
            make_list(bag_list, bag_dir, tasks, sensor_id, profiled_list, 'et')
            
            profiled_list.sort(key=lambda x:x[1])
            prev_id = 0
            for i in range(0, len(profiled_list)):
                if profiled_list[i][0] == start_task_name:
                    if prev_id != profiled_list[i][4]:
                        prev_id = profiled_list[i][4]
                        selected_task = profiled_list[i]
                        selected_task_stime = selected_task[1]
                        if len(profiled_list) > i + target_task_idx:
                            target_task = profiled_list[i+target_task_idx]
                            if target_task[0] == end_task_name:
                                target_task_stime = target_task[1]
                                stime = rospy.rostime.Time.from_sec(selected_task_stime)
                                etime = rospy.rostime.Time.from_sec(target_task_stime)
                                et = (etime-stime).to_nsec() * 1.0e-6
                                cpu_et_list.append([et, target_task[4]])
                    else:
                        continue


            result.append(cpu_et_list)
        
        min_index = max(result[0][0][1], result[1][0][1])
        prev_list = list()
        next_list = list()
        
        for i in range(0, len(result[0])):
            if result[0][i][1] >= min_index:
                prev_list.append(result[0][i])
        for i in range(0, len(result[1])):
            if result[1][i][1] >= min_index:
                next_list.append(result[1][i])
        max_index = min(len(prev_list),len(next_list))
        
        for i in range(0, max_index+1):
            if prev_list[0][1] != next_list[0][1]:
                print('error')

        prev_list = np.array(prev_list)[:,0]
        next_list = np.array(next_list)[:,0]
        print('%s->%s: '%(paper_name(names[0][0],names[0][-2]),paper_name(tasks[0],tasks[-2])))        
        corr = np.corrcoef(prev_list[:max_index],np.array(next_list[:max_index]))
        print(corr[0][1])


def process_rt_task(bag_list, bag_dir):
    result = list()

    for b in bag_list:
        rt_list = list()
        bag = rosbag.Bag(bag_dir+b)
        task_name = get_task_name_shortcut(b)
        if task_name.find('DUM') != -1:
                continue
        for topic, msg, t in bag.read_messages(topics=['/rtp/response_time']):
            rt_list.append(msg.elapse.to_nsec() * 1.0e-6 - 0.1)
        result.append([task_name,rt_list])
    
    return result

def merge_pipeline_execution(result, merge_tasks):
    
    for merge_task in merge_tasks:
        result_key = list()
        result_item = list()
        for key in result:
            if key.find(merge_task) != -1:
                res = np.cumsum(result[key])
                result_key.append(key)
                if len(result_item) > 0:
                    e_wise_minimum = np.minimum(result_item[-1], res)
                    result_item[-1] = e_wise_minimum
                else:
                    result_item.append(res)

        merge_pdf = np.ediff1d(result_item[-1]).tolist()
        for k in result_key:
            result[k] = merge_pdf

            
        

def make_hist(result_list):
    hist_dict = dict()

    for result in result_list:
        name = result
        data = [int(round(x)) for x in result_list[name]]
        hist = np.zeros(1000, dtype=int)
        for i in range(0, len(data)):
            hist[data[i]] = hist[data[i]]+1
        denom = float(sum(hist))
        hist = [x / denom for x in hist]    

        hist_dict[name] = hist

    return hist_dict

            
def make_txt_result(result_dict, result_dir, result_type):

    f = None

    if result_type == 'et_prob':
        f = open(result_dir+'et_prob_list.txt','w')
    elif result_type == 'et':
        f = open(result_dir+'et_list.txt','w')
    elif result_type == 'rt_e2e_prob':
        f = open(result_dir+'rt_e2e_prob_list.txt','w')
    elif result_type == 'rt_task':
        f = open(result_dir+'rt_task_list.txt','w')
    elif result_type == 'rt_task_prob':
        f = open(result_dir+'rt_task_prob_list.txt','w')
    elif result_type == 'et_orig':
        f = open(result_dir+'et_orig_list.txt','w')

    for result in result_dict:
        name = result
        data = result_dict[result]

        f.write(name+'='+str(data))
        f.write('\n')

def paper_name(start, end):

    string = ''
    if start == 'ODM' and end =='STC':
        string = 'A2O'
    elif start == 'EKF' and end =='GEN':
        string = 'E2G'
    elif start == 'TEV' and end == 'PPS':
        string = 'T2P'
    elif start == 'LID_L' and end =='NDT':
        string = 'L2N'
    elif start == 'LID_D' and end =='KFT':
        string = 'L4K'
    elif start == 'CAM_1' and end =='VBT_1':
        string = 'C2V_1'
    elif start == 'CAM_2' and end =='VBT_2':
        string = 'C2V_2'
    elif start == 'CAM_3' and end =='VBT_3':
        string = 'C2V_3'
    elif start == 'CAM_4' and end =='VBT_4':
        string = 'C2V_4'
    elif start == 'RVF_1' and end =='OMP_1':
        string = 'R2O_1'
    elif start == 'RVF_2' and end =='OMP_2':
        string = 'R2O_2'
    elif start == 'RVF_3' and end =='OMP_3':
        string = 'R2O_3'
    elif start == 'RVF_4' and end =='OMP_4':
        string = 'R2O_4'
    else:
        string = start
    return string

def only_et(dir,list):
    list.sort()
    full_execution_list = []
    for rosbag_file in list:
        print(dir + rosbag_file)
        execution_list = []
        for topic, msg, t in rosbag.Bag(dir + rosbag_file).read_messages(topics=['/rtp/execution_time']):
            # stime = msg.msg_info[3].stamp
            stime = msg.stime
            etime = msg.etime
            diff = (etime - stime).to_nsec() * 1.0e-6
            execution_list.append(diff)
            # execution_list.append(msg.elapse.to_nsec() * 1.0e-6)
        full_execution_list.append(execution_list)

    for l in full_execution_list:
        nl = np.array(l)    
        print(np.min(nl), np.mean(nl), np.max(nl))
        plt.plot(l)
    plt.show()

def only_rt(dir,list):
    list.sort()
    full_execution_list = []
    for rosbag_file in list:
        print(dir + rosbag_file)
        execution_list = []
        for topic, msg, t in rosbag.Bag(dir + rosbag_file).read_messages(topics=['/rtp/response_time']):
            stime = msg.msg_info[3].stamp
            etime = msg.etime
            diff = (etime - stime).to_nsec() * 1.0e-6
            execution_list.append(diff)
            # execution_list.append(msg.elapse.to_nsec() * 1.0e-6)
        full_execution_list.append(execution_list)

    for l in full_execution_list:
        nl = np.array(l)    
        print(np.min(nl), np.mean(nl), np.max(nl))
        plt.plot(l)
    plt.show()

    # for cpu_id,tasks_temp in enumerate(task_cpu_info):
    #     tasks_info[cpu_id] = copy.deepcopy(tasks_temp)
    #     sensor_id = get_sensor_id(sensor_info[cpu_id])

        
    #     if len(tasks_info[cpu_id]) > 1:
    #         for i in range(0, len(tasks_info[cpu_id])-1):
    #             tasks_info[cpu_id][i].append(tasks_info[cpu_id][i+1][0])
        
    #     for tasks in tasks_info[cpu_id]:
    #         if len(tasks) > 1:
                
    #             profiled_list = list()
    #             cpu_et_list = list()
    #             target_task_idx = len(tasks)-1
    #             start_task_name = tasks[0]
    #             end_task_name = tasks[-1]
                
    #             make_list(bag_list, bag_dir, tasks, sensor_id, profiled_list, 'et')
    #             profiled_list.sort(key=lambda x:x[1])

    #             for i in range(0, len(profiled_list)):
    #                 if profiled_list[i][0] == start_task_name:
    #                     selected_task = profiled_list[i]
    #                     selected_task_stime = selected_task[1]
    #                     if len(profiled_list) > i + target_task_idx:
    #                         target_task = profiled_list[i+target_task_idx]
    #                         if target_task[0] == end_task_name:
    #                             target_task_stime = target_task[1]
    #                             stime = rospy.rostime.Time.from_sec(selected_task_stime)
    #                             etime = rospy.rostime.Time.from_sec(target_task_stime)
    #                             et = (etime-stime).to_nsec() * 1.0e-6 + 0.5
    #                             if profiled_list[i][4] == 17833508:
    #                                 continue
    #                             cpu_et_list.append(et)


    #             #result.append([start_task_name, cpu_et_list])
    #             result[start_task_name] = cpu_et_list

    #             print('%s to %s avg : %f' %(start_task_name, end_task_name, sum(cpu_et_list)/len(cpu_et_list)))
    #             print('%s to %s max : %f' %(start_task_name, end_task_name, max(cpu_et_list)))
    #             print('%s to %s min : %f' %(start_task_name, end_task_name, min(cpu_et_list)))
    #             print('')
    # return result            

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('-i',default='/home/autoware/.bag/',help='input directory containing profile_0 directory')
    parser.add_argument('-p',default='0')
    
    args = parser.parse_args()
    
    result_dir = args.i 
    camera_pipeline_num = int(args.p)
    bag_dir = result_dir + 'profile_0/'

    try:
        bag_dir_files = os.listdir(bag_dir)
    except OSError as error:
        print(error)
    
    et_bag_list = [x for x in bag_dir_files if x.rfind('_et') != -1]
    rt_bag_list = [x for x in bag_dir_files if x.rfind('_rt') != -1]


    ###cpu affinity information for calculate task per task execution time###

    e2e_path = {
        'LID_L': ['PPS', 'LIDAR_LOC'],
        'LID_D_1': ['PPS', 'LIDAR_DET_1'],
        'LID_D_2': ['PPS', 'LIDAR_DET_2'],
        'LID_D_3': ['PPS', 'LIDAR_DET_3'],
        'LID_D_4': ['PPS', 'LIDAR_DET_4'],
        'CAM_1'  : ['PPS', 'CAMERA_1'],
        'CAM_2' : ['PPS', 'CAMERA_2'],
        'CAM_3' : ['PPS', 'CAMERA_3'],
        'CAM_4' : ['PPS', 'CAMERA_4'],
        'ODM'  : ['PPS', 'ODOM']
    }

    # only_et(bag_dir, et_bag_list)
    only_rt(bag_dir, rt_bag_list)
    exit()
    ######virtual test#########
    cpu_0 = [['ODM','STC'],['EKF','GEN','TEV']]
    cpu_1 = [['TEV','BSE','PPS','DUM_P']]
    cpu_2 = [['CAM_1','DND_1','VBT_1'],['RVF_1','OMP_1','DUM_M_1']]
    cpu_3 = [['CAM_2','DND_2','VBT_2'],['RVF_2','OMP_2','DUM_M_2']]
    cpu_4 = [['CAM_3','DND_3','VBT_3'],['RVF_3','OMP_3','DUM_M_3']]
    cpu_5 = [['CAM_4','DND_4','VBT_4'],['RVF_4','OMP_4','DUM_M_4']]
    cpu_6 = [['LID_L','XGF','NDT','DUM_N']]
    cpu_7 = [['LID_D','GRF','UCL','KFT','DUM_K']]

    if camera_pipeline_num == 2:
        task_sensor_info = ['ODOM','ODOM','CAMERA_1', 'CAMERA_2','LIDAR_LOC','LIDAR_DET']
        task_cpu_info = [cpu_0, cpu_1, cpu_2,cpu_3,cpu_6,cpu_7]
        e2e_path.pop('LID_D_3')
        e2e_path.pop('LID_D_4')
        e2e_path.pop('CAM_3')
        e2e_path.pop('CAM_4')
    elif camera_pipeline_num == 3:
        task_sensor_info = ['ODOM','ODOM','CAMERA_1', 'CAMERA_2','CAMERA_3','LIDAR_LOC','LIDAR_DET']
        e2e_path.pop('LID_D_4')
        e2e_path.pop('CAM_4')
        task_cpu_info = [cpu_0, cpu_1, cpu_2,cpu_3,cpu_4,cpu_6,cpu_7]
    elif camera_pipeline_num == 4:
        task_sensor_info = ['ODOM','ODOM','CAMERA_1', 'CAMERA_2','CAMERA_3','CAMERA_4','LIDAR_LOC','LIDAR_DET']
        task_cpu_info = [cpu_0, cpu_1, cpu_2,cpu_3,cpu_4,cpu_5,cpu_6,cpu_7]
    
    
    et_result= process_et(et_bag_list, task_cpu_info, bag_dir, task_sensor_info, virtual=True)
    cpu_0[1].pop(-1)
    rt_result = process_rt(rt_bag_list, task_cpu_info, bag_dir, task_sensor_info, virtual=True)
    e2e_result = process_e2e(rt_bag_list, e2e_path, bag_dir)

    et_hist_prob = make_hist(et_result)
    rt_hist_prob = make_hist(rt_result)
    e2e_hist_prob = make_hist(e2e_result)
        
    make_txt_result(et_hist_prob, result_dir, 'et_prob')
    make_txt_result(rt_hist_prob, result_dir, 'rt_task_prob')
    make_txt_result(e2e_hist_prob, result_dir, 'rt_e2e_prob')

    # pearson = [[['ODM','STC'],['STC','EKF']],
    #             [['STC','EKF'],['EKF','GEN']],
    #             [['EKF','GEN'],['GEN','TEV']],
    #             [['GEN','TEV'],['TEV','BSE']],
    #             [['TEV','BSE'],['BSE','PPS']],
    #             [['BSE','PPS'],['PPS','DUM_P']],

    #             [['CAM_1','DND_1'],['DND_1','VBT_1']],
    #             [['DND_1','VBT_1'],['VBT_1','RVF_1']],
    #             [['VBT_1','RVF_1'],['RVF_1','OMP_1']],
    #             [['RVF_1','OMP_1'],['OMP_1','DUM_M_1']],
    #             [['OMP_1','DUM_M_1'],['TEV','BSE']],

    #             [['CAM_2','DND_2'],['DND_2','VBT_2']],
    #             [['DND_2','VBT_2'],['VBT_2','RVF_2']],
    #             [['VBT_2','RVF_2'],['RVF_2','OMP_2']],
    #             [['RVF_2','OMP_2'],['OMP_2','DUM_M_2']],
    #             [['OMP_2','DUM_M_2'],['TEV','BSE']],

    #             [['CAM_3','DND_3'],['DND_3','VBT_3']],
    #             [['DND_3','VBT_3'],['VBT_3','RVF_3']],
    #             [['VBT_3','RVF_3'],['RVF_3','OMP_3']],
    #             [['RVF_3','OMP_3'],['OMP_3','DUM_M_3']],
    #             [['OMP_3','DUM_M_3'],['TEV','BSE']],

    #             [['CAM_4','DND_4'],['DND_4','VBT_4']],
    #             [['DND_4','VBT_4'],['VBT_4','RVF_4']],
    #             [['VBT_4','RVF_4'],['RVF_4','OMP_4']],
    #             [['RVF_4','OMP_4'],['OMP_4','DUM_M_4']],
    #             [['OMP_4','DUM_M_4'],['TEV','BSE']],
                
    #             [['LID_L','XGF'],['XGF','NDT']],
    #             [['XGF','NDT'],['NDT','DUM_N']],
    #             [['NDT','DUM_N'],['EKF','GEN']],
    #             [['LID_D','GRF'],['GRF','UCL']],
    #             [['GRF','UCL'],['UCL','KFT']],
    #             [['UCL','KFT'],['KFT','DUM_K']],
    #             [['KFT','DUM_K'],['RVF_1','OMP_1']]]

    # pearson_sensor_info = ['ODOM','ODOM','ODOM','ODOM','ODOM','ODOM','CAMERA_1','CAMERA_1','CAMERA_1','CAMERA_1','CAMERA_1',
    # 'CAMERA_2','CAMERA_2','CAMERA_2','CAMERA_2','CAMERA_2','CAMERA_3','CAMERA_3','CAMERA_3','CAMERA_3','CAMERA_3',
    # 'CAMERA_4','CAMERA_4','CAMERA_4','CAMERA_4','CAMERA_4','LIDAR_LOC','LIDAR_LOC','LIDAR_LOC','LIDAR_DET','LIDAR_DET','LIDAR_DET','LIDAR_DET']
    # pearson_correlation_coefficient_result = pearson_correlation_coefficient2(et_bag_list, pearson, pearson_sensor_info,bag_dir)


    # pearson_virtual = [ [['ODM','STC','EKF'],['EKF','GEN','TEV']],
    #                     [['EKF','GEN','TEV'],['TEV','BSE','PPS','DUM_P']],
    #                     [['CAM_1','DND_1','VBT_1','RVF_1'],['RVF_1','OMP_1','DUM_M_1']],
    #                     [['RVF_1','OMP_1','DUM_M_1'],['TEV','BSE','PPS','DUM_P']],

    #                     [['CAM_2','DND_2','VBT_2','RVF_2'],['RVF_2','OMP_2','DUM_M_2']],
    #                     [['RVF_2','OMP_2','DUM_M_2'],['TEV','BSE','PPS','DUM_P']],

    #                     [['CAM_3','DND_3','VBT_3','RVF_3'],['RVF_3','OMP_3','DUM_M_3']],
    #                     [['RVF_3','OMP_3','DUM_M_3'],['TEV','BSE','PPS','DUM_P']],

    #                     [['CAM_4','DND_4','VBT_4','RVF_4'],['RVF_4','OMP_4','DUM_M_4']],
    #                     [['RVF_4','OMP_4','DUM_M_4'],['TEV','BSE','PPS','DUM_P']],

    #                     [['LID_L','XGF','NDT','DUM_N'],['EKF','GEN','TEV']],
    #                     [['LID_D','GRF','UCL','KFT','DUM_K'],['RVF_1','OMP_1','DUM_M_1']]]
    
    # pearson_virtual_sensor_info = ['ODOM','ODOM','CAMERA_1','CAMERA_1','CAMERA_2','CAMERA_2','CAMERA_3','CAMERA_3','CAMERA_4','CAMERA_4','LIDAR_LOC','LIDAR_DET']

    
    # print('-------------')
    # pearson_correlation_coefficient_result2 = pearson_correlation_coefficient2(et_bag_list, pearson_virtual, pearson_virtual_sensor_info,bag_dir)
    

    # pearson_test = [['LID_D','GRF','UCL','KFT','DUM_K'], ['CAM_1','DND_1','VBT_1','RVF_1']]
    # pearson_test_sensor_info = ['LIDAR_DET','CAMERA_1']
    # pearson_test_result = pearson_test_coefficient(et_bag_list, pearson_test, pearson_test_sensor_info,bag_dir)
    #make_txt_result(pearson_correlation_coefficient_result, result_dir, 'et_orig')

    
if __name__ == "__main__":
    main(sys.argv) 
