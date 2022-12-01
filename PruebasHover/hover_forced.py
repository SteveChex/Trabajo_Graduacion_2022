# -*- coding: utf-8 -*-
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
# Based on the qualisis_hl_commander.py code from the 
# crazyflie-python-lib from Bitcraze.
# Modified by Steve Chex
#
 
"""
Codigo para realizar un vuelo estático del dron basado en el empuje
de las aspas. No se realiza ningun tipo de control en este ejemplo.

Se espera que el dron esté balanceado y con la batería bien 
colocada para evitar los balanceos.
"""

from threading import Thread, Event
import keyboard

import math

# Import Crazyflie Libraries
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/44/2M/AEAEAEAEAE')

# Drone max speed
cf_max_vel = 0.3
step = 0.1
thrustStep = 200

thrust = 45000

def turningOffMotive(event):
    while not keyboard.is_pressed('enter'):
        event.wait(0.1)        
    print("\n-EMERGENCY CALLED- \n DRONE TURNED OFF")
    #logcf.stop()
    event.wait(0.1)
    cf.commander.send_stop_setpoint()
    event.wait(0.1)
    cf.commander.send_stop_setpoint()
    event.wait(0.1)
    cf.commander.send_stop_setpoint()
    event.wait(0.1)
    #while True:
    #    cf.commander.send_stop_setpoint()
    #    event.wait(0.05)
    #    cf.param.set_value('stabilizer.stop', '1')
    #    event.wait(0.05)
       

def estimatorLogData(cf):
    # CHANGE THE LOG ACCORDING TO THE CURRENT NEEDS
    # JUST REMEMBER THE LIMITS
    global logcf
    logcf = LogConfig(name='Kalman Variance', period_in_ms=50)
    logcf.add_variable('stateEstimate.x', 'float')
    logcf.add_variable('stateEstimate.y', 'float')
    logcf.add_variable('stateEstimate.z', 'float')
    logcf.add_variable('stateEstimate.roll', 'float')
    logcf.add_variable('stateEstimate.pitch', 'float')
    logcf.add_variable('stateEstimate.yaw', 'float')
    cf.log.add_config(logcf)
    logcf.data_received_cb.add_callback(log_pose_est_callback)   
    logcf.start()   
    return logcf

def log_pose_est_callback(timestamp, data, logconf):
    global pose_est
    pose_est[0] = data['stateEstimate.x']
    pose_est[1] = data['stateEstimate.y']   
    pose_est[2] = data['stateEstimate.z']
    pose_est[3] = data['stateEstimate.roll']
    pose_est[4] = data['stateEstimate.pitch']
    pose_est[5] = data['stateEstimate.yaw']

    
def landing(cf):
    step = 1000
    currentThrust = thrust
    while currentThrust > 20000 and not keyboard.is_pressed('enter') :
        cf.commander.send_setpoint(0,0,0,currentThrust)
        currentThrust = currentThrust - step
        event.wait(0.1)
    cf.commander.send_setpoint(0,0,0,0)
    event.wait(0.1)


def initParams(cf):
    cf_max_vel_xy = 0.4
    cf_max_vel_z = 0.5
    cf.param.set_value('posCtlPid.xVelMax', cf_max_vel_xy)
    cf.param.set_value('posCtlPid.yVelMax', cf_max_vel_xy)
    cf.param.set_value('posCtlPid.zVelMax', cf_max_vel_z)
    
    # for roll, pitch, yaw controll
    cf.param.set_value('flightmode.stabModeRoll', 1)
    cf.param.set_value('flightmode.stabModePitch', 1)
    cf.param.set_value('flightmode.stabModeYaw', 0)
    cf.param.set_value('flightmode.yawMode', 2)

    # for absolute/relative Positioning
    #cf.param.set_value('flightmode.posSet', 1)

if __name__ == '__main__':
    global event
    event = Event()
    Robotat_shutdown = Thread(target = turningOffMotive, args=(event,),daemon=True)
    Robotat_shutdown.start()
    cflib.crtp.init_drivers()
    for iter in range(0,3):
            print(f"Start in {3-iter:d} seconds")
            event.wait(1)
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        global cf
        cf = scf.cf
        initParams(cf)
        # UNCOMMENT IF LOG POSITION DATA IS NEEDED
        #log_config = estimatorLogData(cf)        
        cf.commander.set_client_xmode(True)
        cf.commander.send_setpoint(0, 0, 0, 0)
        try:      
            print("HOVERING...")
            it=0
            
            roll = 0
            pitch = 0
            yaw = 0
            while not keyboard.is_pressed('space'):
                cf.commander.send_setpoint(roll,pitch,yaw,thrust)
                #cf.commander.send_position_setpoint(0, 0.5, 0.5, 0)                        
                if it > 500:
                    pass
                if keyboard.is_pressed('w'):
                    roll += step
                if keyboard.is_pressed('s'):
                    roll -= step
                if keyboard.is_pressed('d'):
                    pitch += step
                if keyboard.is_pressed('a'):
                    pitch -= step
                if keyboard.is_pressed('e'):
                    yaw += step
                if keyboard.is_pressed('q'):
                    yaw -= step
                if keyboard.is_pressed('r'):
                    thrust += thrustStep
                if keyboard.is_pressed('f'):
                    thrust -= thrustStep
                event.wait(0.1)
                it+=1
            print("Landing and closing connection")
            landing(cf)
            #cf.close_link()
            #log_config.stop()
        except BaseException as theError:
            # Don't execute the control code
            print('--------ERROR ON LOOP -----------') 
            print(f'Leaving because of ')
            print(theError)
        #log_config.stop()

  




# Rejected ideas

#print("putting in althold")
            #cf.param.set_value("flightmode.althold",1)

            #print("Stay in althold for 7s")  
            #cf.param.set_value("flightmode.stabModeRoll",1)
            #cf.param.set_value("flightmode.stabModePitch",1)
            #cf.param.set_value("flightmode.stabModeYaw",1)
            #cf.param.set_value("flightmode.poshold",0)

              #cf.commander.send_position_setpoint(0,0,1.0,0)
            #cf.commander.send_zdistance_setpoint(0, 0, 0, 0.3)        
            #cf.high_level_commander.go_to(0, 0, 0.3, 0, 1, True)
            #cf.commander.send_hover_setpoint(0, 0, 0, 0.3)        

                            #cf.high_level_commander.go_to(0, 0, 0.3, 0, 0.1, True)
                #cf.commander.send_setpoint(0, 0, 0, 40000)
                #cf.commander.send_zdistance_setpoint(0, 0, 0, 0.5)
                #cf.commander.send_hover_setpoint(0, 0, 0, 0.3)        
                #cf.commander.send_position_setpoint(0, 0, 0.3, 0)

                #if keyboard.is_pressed('1'):
                #    cf.param.set_value("flightmode.poshold",1)
                #if keyboard.is_pressed('2'):
                #    cf.param.set_value("flightmode.althold",1)
                #if keyboard.is_pressed('3'):
                #    cf.param.set_value("flightmode.posSet",1)
                #if keyboard.is_pressed('4'):
                #    cf.param.set_value("flightmode.posSet",0)

                #if it > 500:
                #    pass
                    #cf.param.set_value("flightmode.poshold",1)
                    #cf.param.set_value("flightmode.althold",1)
                    #Aparently, Poshold allows the drone to hover at the same 3d position
                    #Next experiment will test this
                    #cf.extpos.send_extpos(0,0,0.5)