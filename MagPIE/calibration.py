import os
import csv
import numpy as np
import allan_variance
import matplotlib.pyplot as plt
import pandas as pd
import yaml

class IntrinsicCalibration:
    def __init__(self, file_paths, cal_path):
        self.num_imus = len(file_paths)
        self.cal_path = cal_path
        self.sensor_data = {'t': {}, 'gx': {}, 'gy': {}, 'gz': {}, 'ax': {}, 'ay': {}, 'az': {}}
        self.params = {'qn': {}, 'wn': {}, 'rw': {}, 'bi': {}, 'rp': {}}
        self.biases = {'gx': {}, 'gy': {}, 'gz': {}, 'ax': {}, 'ay': {}, 'az': {}}
        self.det_params = {'T_i_b': {}, 'acc_M': {}, 'A': {}, 'C_gyro_i': {}, 'gyro_M': {}}

        for i, file_path in enumerate(file_paths):
            df = pd.read_csv(file_path)
            t_key = f'df{i+1}_t'
            self.sensor_data['t'][t_key] = np.array((df['t'] - df['t'][0]) / 1e9)
            
            for axis in ['gx', 'gy', 'gz', 'ax', 'ay', 'az']:
                self.sensor_data[axis][f'df{i+1}_{axis}'] = np.array(df[axis])

    def get_av_params(self):
        for i in range(self.num_imus):
            dt = np.mean(np.diff(self.sensor_data['t'][f'df{i+1}_t']))
            
            for axis in ['gx', 'gy', 'gz', 'ax', 'ay', 'az']:
                tau, avar = allan_variance.compute_avar(self.sensor_data[axis][f'df{i+1}_{axis}'], dt)
                params_axis, _ = allan_variance.estimate_parameters(tau, avar)
                
                for noise_type, param_key in zip(['quantization', 'white', 'flicker', 'walk', 'ramp'], 
                                                ['qn', 'wn', 'bi', 'rw', 'rp']):
                    self.params[param_key][f'df{i+1}_{axis}'] = params_axis[noise_type]

    def get_bias(self):
        gravity = 9.81
        for i in range(self.num_imus):            
            for axis in ['gx', 'gy', 'gz', 'ax', 'ay', 'az']:
                meas = self.sensor_data[axis][f'df{i+1}_{axis}']
                self.biases[axis][f'df{i+1}_{axis}'] = np.mean(meas, axis=0)
                if axis == 'az':
                    self.biases[axis][f'df{i+1}_{axis}'] -= gravity

    def get_det_params(self):
        with open(self.cal_path, "r") as file:
            data = yaml.safe_load(file)
        for i in range(1, self.num_imus+1):
            T_i_b, acc_M, A, C_gyro_i, gyro_M = [], [], [], [], []
            for row in data[f"imu{i}"]["T_i_b"]:
                T_i_b.append(row)
            for row in data[f"imu{i}"]["accelerometers"]['M']:
                acc_M.append(row)
            for row in data[f"imu{i}"]["gyroscopes"]['A']:
                A.append(row)
            for row in data[f"imu{i}"]["gyroscopes"]['C_gyro_i']:
                C_gyro_i.append(row)
            for row in data[f"imu{i}"]["gyroscopes"]['M']:
                gyro_M.append(row)
            self.det_params['T_i_b'][f"imu{i}"] = T_i_b
            self.det_params['acc_M'][f"imu{i}"] = acc_M
            self.det_params['A'][f"imu{i}"] = A
            self.det_params['C_gyro_i'][f"imu{i}"] = C_gyro_i
            self.det_params['gyro_M'][f"imu{i}"] = gyro_M
