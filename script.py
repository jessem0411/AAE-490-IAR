"""
Example script that allows a user to "push" the Crazyflie 2.X around
using your hands while it's hovering.

This examples uses the Flow and Multi-ranger decks to measure distances
in all directions and tries to keep away from anything that comes closer
thn 0.2m by setting a velocity in the opposite direction.

The demo is ended by either pressing Ctrl-C or by holding your hand above the
Crazyflie.
"""
import logging
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement

URI = 'radio://0/80/250K/E7E7E7E718'

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=True)
    cf = Crazyflie(rw_cache='./cache')
    log_config = LogConfig(name='Height', period_in_ms=500)
    log_config.add_variable('kalman.stateZ','float')
    log_config.add_variable('motor.m1','float')
    log_config.add_variable('motor.m2','float')
    log_config.add_variable('motor.m3', 'float')
    log_config.add_variable('motor.m4','float')
    t = []
    zhistory = []
    thrust_hist = []
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf) as motion_commander:
                with SyncLogger(scf, log_config) as logger:
                    #scf.cf.log.add_config(log_config)
                    scf.cf.param.set_value('posCtlPid.zKp','3.5')
                    scf.cf.param.set_value('posCtlPid.zKi','1.3')
                    scf.cf.param.set_value('commander.enHighLevel','1')
                    scf.cf.param.set_value('posCtlPid.zKd','0.5')
                    scf.cf.param.set_value('ctrlMel.mass','0.033')
                    endTime = time.time()+15
                    startTime = time.time()
                    #print('time: {}'.format(endTime))
                    #log_config.data_received_cb.add_callback(position_callback)
                    #log_config.start()
                    for log_entry in logger:
                        timestamp = log_entry[0]
                        data = log_entry[1]
                        logconf_name = log_entry[2]
                        z = data['kalman.stateZ']
                        t1 = data['motor.m1']
                        t2 = data['motor.m2']
                        t3 = data['motor.m3']
                        t4 = data['motor.m4']
                        t_tot = t1+t2+t3+t4
                        t.append(time.time()-startTime)
                        zhistory.append(z)
                        thrust_hist.append(t_tot)
                        if time.time() < (endTime - 10): 
                            motion_commander.start_up(.1)
                            #print('time: {}'.format(time.time()))
                            #print('z: {}',format(z))
                        else: 
                            motion_commander.stop()
                        if time.time() > endTime:
                            print('t')
                            break
                    #motion_commander.move_distance(0,0,0.5)
                    #time.sleep(10)
                    motion_commander.stop()
                    time.sleep(3)
                    motion_commander.land()
                    plt.figure()
                    plt.plot(t,zhistory)
                    plt.grid()
                    plt.xlabel('Time (s)')
                    plt.ylabel('Altitude (m)')
                    plt.figure()
                    plt.plot(t,thrust_hist)
                    plt.grid()
                    plt.xlabel('Time (s)')
                    plt.ylabel('Sum of motor inputs (rpm)')
                    plt.show()
                    print('Demo terminated!)
