from math import pi
import NGtoOpenSense
import numpy as np
from scipy import signal
from multiprocessing import Process, Queue, Lock
if __name__ == '__main__':
    q = Queue() #maximum 
    lock = Lock()
    imuProc = Process(target=NGtoOpenSense.readIMU, args=(lock, q,))
    imuProc.start()
    sensor_names = ['Left Shank', 'Right Shank']
    num_sensors = len(sensor_names)
    # 50 Hz, 2 seconds of data
    quat_cal_len = 100 
    # standing upright 
    cali_flag = True
    acc_cal = np.zeros((num_sensors, quat_cal_len, 3))
    while cali_flag:
        command = input('Press Y when ready for the standing upright calibration')
        if command == 'Y' or command == 'y':
            cali_flag = False
            while(q.qsize()>0): # clear the queues that may have old messages
                q.get()
            for ilen in range(0, quat_cal_len):
                IMU_list = q.get()
                #Acc index 5:8
                for idx, IMU in enumerate(IMU_list):
                    IMU_acc = np.array(IMU[5: 8])
                    acc_cal[idx,ilen,:] = IMU_acc
        else:
            print('Problems arised, need to fix and reask for the desired action')
    sensors_standingupright_3D_dic = dict()
    for idx, sensor in enumerate(sensor_names):
        #filter
        # temp = acc_cal[idx, :, :]
        # fs = 50; fc = 40; N = 2
        # w = fc/(fs/2)
        # b, a = signal.butter(N, w, 'low')
        # filtered = signal.filtfilt(b, a, temp, axis = 0)
        # sensors_standingupright_3D_dic[sensor] = np.mean(filtered, axis=0)
        sensors_standingupright_3D_dic[sensor] = np.mean(np.asarray(acc_cal[idx, :, :]), axis=0)
    print('Standing upright calibration successfully!')
    
    # sitting down
    cali_flag = True
    acc_cal = np.zeros((num_sensors, quat_cal_len, 3))
    while cali_flag:
        command = input('Press Y when ready for the sitting down calibration')
        if command == 'Y' or command == 'y':
            cali_flag = False
            while(q.qsize()>0): # clear the queues that may have old messages
                q.get()
            for ilen in range(0, quat_cal_len):
                IMU_list = q.get()
                #Acc index 5:8
                for idx, IMU in enumerate(IMU_list):
                    IMU_acc = np.array(IMU[5: 8])
                    acc_cal[idx,ilen,:] = IMU_acc
        else:
            print('Problems arised, need to fix and reask for the desired action')
    sensors_sittingdown_3D_dic = dict()
    for idx, sensor in enumerate(sensor_names):
        # filter
        # temp = acc_cal[idx, :, :]
        # fs = 50; fc = 40; N = 2
        # w = fc/(fs/2)
        # b, a = signal.butter(N, w, 'low')
        # filtered = signal.filtfilt(b, a, temp, axis = 0)
        # sensors_standingupright_3D_dic[sensor] = np.mean(filtered, axis=0)
        sensors_sittingdown_3D_dic[sensor] = np.mean(np.asarray(acc_cal[idx, :, :]), axis=0)
    print('Sitting down calibration successfully!')
    
    #body to sensor rotation calibration
    sensor_to_body = dict()
    for sensor in sensor_names:
        z = sensors_standingupright_3D_dic[sensor]
        z = z/np.linalg.norm(z)
        x = np.cross(sensors_sittingdown_3D_dic[sensor], z)
        x = x/np.linalg.norm(x)
        y = np.cross(z, x)
        y = y/np.linalg.norm(y)
        sensor_to_body[sensor] = np.asarray([x, y, z])
        print('Rotation of '+ sensor + 'is:')
        print(sensor_to_body[sensor])
        
    
    
    

           