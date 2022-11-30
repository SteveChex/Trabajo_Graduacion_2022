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
import socket
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

import math

# Import Crazyflie Libraries
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/44/2M/E7E7E7E7E7')

# Drone max speed
cf_max_vel = 0.3



# for braking
brkIters = 7
brkCounter = 0

# For controlling the brake
movingTicks = 0

# For controlling the advance Impulse
impIters = 4

step = 0.1
thrustStep = 200
thrust = 42000
# Robotat Vars
dataTime_Seconds = 0.1 # Time for updating position

# True: send position and orientation; False: send position only
send_full_pose = False

# Rigid body ID from Optitrack Marker
RigidBodyID = '11' #Object ID
 
# Motive network Data. Make sure to be connected to the right network
HOST = "192.168.50.200"  # The server's hostname or IP address
PORT = 1883  # The port used by the server

class RobotatHandler(Thread):
    def __init__(self, id, event, updateInterval, host, port, fullPose):
        Thread.__init__(self)

        self.rigid_body_id = id
        self.updateInterval = updateInterval
        self.event = event
        self.host = host
        self.port = port
        self.bodypos = [0, 0, 0]
        self.bodyquat = [0, 0, 0, 0]
        self.fullPose = fullPose
        self._stay_open = True
        self.connected = False
        self.socketStat = ''
        self.socketError = ''
        self.robotatSocket = None
        self.cf = None
        self.printPos = False

        self.start()
        self.msgObj("Thread initiated")

    def msgObj(self, msg):
        msg = 'ROBOTAT HANDLER: ' + msg
        print(msg)

    def close(self):
        self.msgObj("Closing Robotat Connection")
        self._stay_open = False

    def run(self):
        self._connect()
        while self._stay_open:   
            self.getPose()
            self.sendPose()
            #self.printPose()
            #self.event.wait(self.updateInterval)
        self.close()    

    def _connect(self):
        self.event.wait(1)
        self.msgObj("Connecting to Motive...")     

        self.socketStat = True
        self.socketError = ''
        try:
            self.robotatSocket = \
                 socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.robotatSocket.connect((self.host, self.port))
            data = self.robotatSocket.recv(1024)
        except BaseException as TheFail:
            self.robotatSocket = None
            self.socketStat = False
            self.socketError = TheFail

        if self.socketStat == False:
            self.msgObj("Couldn't connect to Motive")
            self.msgObj(f"Reason: {self.socketError}")
            self._stay_open = False
            return
        self.connected = True
        self.msgObj("Connected to Motive")
        
    def getPose(self):
        if self._stay_open == False:
            return
        # Request marker data from Motive server, using ID
        self.robotatSocket.sendall(self.rigid_body_id.encode())
        dataHolder = self.robotatSocket.recv(1024)  

        noSpacesData = f"{dataHolder.decode()}"
        noSpacesData = noSpacesData.replace(' ', '')
        items = 0;
        itemsArray = []
        iterator = 0
        lastIteration = 1
        for value in noSpacesData:
            if value == ',' : 
                itemsArray.insert(items, noSpacesData[lastIteration:iterator])
                items +=1
                lastIteration = iterator + 1
            if value == ']':
                itemsArray.insert(items, noSpacesData[lastIteration:iterator])
                items +=1
            iterator += 1
        pose = []
        for item in itemsArray:
            pose.append(float(item))
        self.bodypos, self.bodyquat = pose[0:3], pose[3:7]
    
    def printPose(self):
        if self._stay_open == False:
            return
        if not self.printPose:
            return
        print("\n\
               \nROBOTAT HANDLER: Position XZY\
               \n                 {:.4f}  \t{:.4f}  \t{:.4f}  \
               \n                 Quaternion xi yj zk \u03C9\
               \n                 {:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}" \
            .format(self.bodypos[0], self.bodypos[2], self.bodypos[1], \
            self.bodyquat[0], self.bodyquat[1], self.bodyquat[2], \
            self.bodyquat[3]),\
            end = "")

    def sendPose(self):
        if self.cf == None:
            return
        if self.fullPose == True:  
            # send_extpos(x, y, z, qx, qy, qz, qw)
            self.cf.extpos.send_extpose(\
                self.bodypos[0], self.bodypos[2], self.bodypos[1], \
                self.bodyquat[0], self.bodyquat[1], self.bodyquat[2], \
                self.bodyquat[3])
            return
        else:
            # send_extpos(x, y, z)
            self.cf.extpos.send_extpos(\
                self.bodypos[0], self.bodypos[2], self.bodypos[1])
            return  

def turningOffMotive(event):
    while not keyboard.is_pressed('enter'):
        event.wait(0.1)        
    print("\n-EMERGENCY CALLED- \n DRONE TURNED OFF")
    #logcf.stop()
    while True:
        cf.commander.send_stop_setpoint()
        event.wait(0.05)

def estimatorLogData(cf):
    # CHANGE THE LOG ACCORDING TO THE CURRENT NEEDS
    # JUST REMEMBER THE LIMITS
    global logcf
    logcf = LogConfig(name='Kalman Data', period_in_ms=50)
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

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.01

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
            Robotat.printPose = True

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    time.sleep(1)
    wait_for_position_estimator(cf)

def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')

def initParams(cf):
    cf_max_vel_xy = 0.8
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
    global Robotat
    event = Event()
    Robotat = RobotatHandler(RigidBodyID, event,\
                dataTime_Seconds, HOST, PORT,\
                send_full_pose)
    Robotat_shutdown = Thread(target = turningOffMotive, args=(event,),daemon=True)
    Robotat_shutdown.start()
    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        global cf
        cf = scf.cf
        activate_kalman_estimator(cf)
        activate_high_level_commander(cf)
        reset_estimator(cf)
        #cf.commander.set_client_xmode(True)
        #estimatorLogData(cf)
        print("KEYBOARD CONTROLL FLIGHT")
        for iter in range(0,3):
            print(f"Start in {3-iter:d} seconds")
            event.wait(1)
        cf.commander.send_setpoint(0, 0, 0, 0)
        try:      
            print("HOVERING...")
            it=0
            roll = 0
            pitch = 0
            yaw = 0
            height = 0.1
            print(f"ROLL: {roll:.1f}\tPITCH: {pitch:.1f}\tYAW: {yaw:.1f}\tTHRUST: {height:.2f}")
            while not keyboard.is_pressed('space'):
                roll = 0
                pitch = 0
                yaw = 0
                if keyboard.is_pressed('w'):
                    pitch = 10
                if keyboard.is_pressed('s'):
                    pitch = -12
                if keyboard.is_pressed('d'):
                    roll = 15
                if keyboard.is_pressed('a'):
                    roll = -10
                if keyboard.is_pressed('e'):
                    yaw = 5
                if keyboard.is_pressed('q'):
                    yaw = -5
                if keyboard.is_pressed('r'):
                    height += 0.05
                if keyboard.is_pressed('f'):
                    height -= 0.05
                if keyboard.is_pressed('z'):
                    roll *= -1
                    pitch *= -1
                    yaw *= -1
                    #cf.commander.send_setpoint(roll*2,pitch*2,yaw,height)
                    event.wait(0.1)
                if keyboard.is_pressed('c'):
                    roll = 0
                    pitch = 0
                    yaw = 0
                if keyboard.is_pressed('t'):
                    height = 0.5
                if keyboard.is_pressed('g'):
                    height = 0.3
                if keyboard.is_pressed('b'):
                    height = 0.1
                if keyboard.is_pressed('n'):
                    height = 0.0
                event.wait(0.1)
                if keyboard.is_pressed('x'):
                    roll *= -2
                    pitch *= -2
                    yaw *= -2
                    print("BRAKING")
                    while brkCounter < brkIters:
                        #cf.commander.send_setpoint(roll,pitch,yaw,height)
                        event.wait(0.1)    
                        brkCounter+=1
                    print("restart")
                    brkCounter = 0
                    roll = 0
                    pitch = 0
                    yaw = 0
                    #cf.commander.send_setpoint(roll,pitch,yaw,height)
                cf.commander.send_zdistance_setpoint(roll,pitch,yaw,height) #Más optimo, aunque no tiene control xy 
                print(f"ROLL: {roll:.1f}\tPITCH: {pitch:.1f}\tYAW: {yaw:.1f}\tTHRUST: {height:.2f}")
                event.wait(0.1)
            print("Landing and closing connection")
            landing(cf)
            #cf.close_link()
            #log_config.stop()
        except BaseException as theError:
            # Don't execute the control code
            print('--------ERROR ON LOOP -----------') 
            print(f'Leaving because of ')
            print(theError)
            cf.commander.send_setpoint(0, 0, 0, 0)

        #log_config.stop()