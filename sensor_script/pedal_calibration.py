import json
from hipnuc_acc import *
from hipnuc_brake import *
import time
import os

def calibrate_pedals(hip_acc, hip_brake, duration=30):
    min_acc_angle = float('inf')
    max_acc_angle = float('-inf')
    min_brake_angle = float('inf')
    max_brake_angle = float('-inf')
    
    start_time = time.time()
    while time.time() - start_time < duration:
        data_acc = hip_acc.get_module_data(10)
        data_brake = hip_brake.get_module_data(10)

        acc_pitch_angle = data_acc['euler'][0]['Pitch']
        brake_pitch_angle = data_brake['euler'][0]['Pitch']

        print(f"Acc Pitch: {acc_pitch_angle}, Brake Pitch: {brake_pitch_angle}")

        min_acc_angle = min(min_acc_angle, acc_pitch_angle)
        max_acc_angle = max(max_acc_angle, acc_pitch_angle)

        min_brake_angle = min(min_brake_angle, brake_pitch_angle)
        
        max_brake_angle = max(max_brake_angle, brake_pitch_angle)

    return (min_acc_angle, max_acc_angle), (min_brake_angle, max_brake_angle)

script_dir = os.path.dirname(os.path.abspath(__file__))
acc_config_path = os.path.join(script_dir, 'config', 'config_acc.json')
brake_config_path = os.path.join(script_dir, 'config', 'config_brake.json')
calibration_path = os.path.join(script_dir, '..', 'sensor', 'pedal_calibration.json')

hip_acc = hipnuc_acc(acc_config_path)
hip_brake = hipnuc_brake(brake_config_path)
config_file = calibration_path


try:
    (acc_min, acc_max), (brake_min, brake_max) = calibrate_pedals(hip_acc, hip_brake)
    calibration_data = {
        'acc': {'min': acc_min, 'max': acc_max},
        'brake': {'min': brake_min, 'max': brake_max}
    }
    with open(config_file, 'w') as f:
        json.dump(calibration_data, f)
finally:
    hip_acc.close()
    hip_brake.close()
