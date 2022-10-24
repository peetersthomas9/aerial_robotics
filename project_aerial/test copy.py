import logging
import sys
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.syncLogger import SyncLogger
from drone import quad
from utilities import *

"""
Main file to run 

rebase on test branch see how it works
"""
def reset_estimator(scf):
    """ function use to resete the kalman filter so that we have less errors for the landing"""
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def wait_for_position_estimator(scf):

    """ obtain the data of our Kalman filter until we get correct values  """
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_x_history = [1000] * 10
    var_y_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break



# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

cf = Crazyflie(rw_cache='./cache')

#  initialize the drone class
drone = quad(cf, pos_init=np.array([75,150,0]))

# different state that our drone can take 
states = {
    0 :"idle", 
    1 : "take_off",
    2 : "global",
    3 : "local",
    4 : "pad_search",
    5 : "pad_found",
    6 : "land",
    7 : "take_off_back",
    8 : "global_back",
    9 : "local_back",
    10 : "pad_found_back",
    11 : "land_back",
    12 : "finished"

    }

def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(uri, cf=cf) as scf:

        # connect to the drone
        drone.connect(scf, verbose=True)

        # reset kalman filter 
        reset_estimator(scf)
        reset_estimator(scf)
        keep_flying = True
        move = False
        velocity_y = 0.
        velocity_x = 0.

        # loop to controll our drone 
        while keep_flying :

            if states.get(drone.state) == 'idle' : 
                drone.take_off()
                time.sleep(0.1)

            if states.get(drone.state) == 'global' :
                drone.global_nav()
                time.sleep(0.1)
            
            if states.get(drone.state) == 'local' :
                drone.local_nav()
                time.sleep(0.1)

            if states.get(drone.state) == 'pad_search' :
                drone.pad_search()
                time.sleep(0.1)
            
            if states.get(drone.state) == 'pad_found' :
                drone.pad_found(first_pad=True)
                time.sleep(0.1)
                

            if states.get(drone.state) == 'land':
                drone.land(velocity=0.15)
                time.sleep(0.1)
       
            if states.get(drone.state) == 'take_off_back':
                reset_estimator(scf)
                reset_estimator(scf)
                drone.take_off_back()
                time.sleep(0.1)

            if states.get(drone.state) == 'global_back':
                drone.global_back()
                
                time.sleep(0.1)

            if states.get(drone.state) == 'local_back':
                drone.local_back()
                
                time.sleep(0.1)

            if states.get(drone.state) == 'pad_found_back':
                drone.pad_found(first_pad=False)
                
                time.sleep(0.1)

            if states.get(drone.state) == 'land_back':
                drone.land_back(velocity=0.15)
                
                time.sleep(0.1)


            if states.get(drone.state) == 'finished':
                print("ploting logs")
                
                drone.map.plot_map()
                keep_flying = False

        print("finished")
        print("dir",drone.dir)

