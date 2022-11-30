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
Ejemplo del uso de la libreria Socket para la obtencion de datos de la red
de Motive sobre la pose de un marker en el sistema de captura de movimiento
Optitrack, que luego se trasladan al dron Crazyflie 2.0 para darle sentido
de posicion.

Configurar la direccion de radio de acuerdo al dron en uso.
"""
# Import system Libraries
from cmath import nan, pi
import math
import time
from threading import Thread, Event
import keyboard
import socket
import logging

logging.basicConfig(level=logging.ERROR)

import math

# Import Crazyflie Libraries
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# Network Name to be connected
WifiName = 'Robotat'
overrideWiFi = True

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/44/2M/AEAEAEAEAE')


# time stamps to change the height
increaseTimestamp = 0.2
heightChanges = [0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.3, 0.25, 0.2, 0.15]
heightsNum = len(heightChanges) 
iterVar = 0

# Data holders for Motive Information
pose_est = [0, 0, 0, 0, 0, 0]
pose = [0, 0, 0]
quat = [0, 0, 0, 0]
eulAng = [0, 0, 0]

error = [0, 0, 0]

thrust = 40000

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
            # send_extpos(x, y, z) dron reference point
            self.cf.extpos.send_extpos(\
                self.bodypos[0], self.bodypos[2], self.bodypos[1])
            return 
            
            
def Emergency(event, RobotatObj):
    while not keyboard.is_pressed('enter'):
        event.wait(0.1)
    print("\n-EMERGENCY CALLED- \n DRONE AND MOTIVE CONNECTIONS TERMINATED")
    RobotatObj._stay_open = False
    while True:
        cf.commander.send_setpoint(0, 0, 0, 0)
        cf.commander.send_setpoint(0, 0, 0, 0)
        cf.commander.send_setpoint(0, 0, 0, 0)

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)*180/math.pi
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)*180/math.pi
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)*180/math.pi
    
    return roll_x, pitch_y, yaw_z # in Angles
 
def estimatorLogData(cf):
    log_config = LogConfig(name='Kalman Variance', period_in_ms=100)
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')
    log_config.add_variable('stateEstimate.roll', 'float')
    log_config.add_variable('stateEstimate.pitch', 'float')
    log_config.add_variable('stateEstimate.yaw', 'float')
    cf.log.add_config(log_config)
    log_config.data_received_cb.add_callback(log_pose_est_callback)   
    log_config.start()   
    return log_config

def log_pose_est_callback(timestamp, data, logconf):
    global pose_est

    pose_est[0] = data['stateEstimate.x']
    pose_est[1] = data['stateEstimate.y']
    pose_est[2] = data['stateEstimate.z']
    pose_est[3] = data['stateEstimate.roll']
    pose_est[4] = data['stateEstimate.pitch']
    pose_est[5] = data['stateEstimate.yaw']

    #pose = Robotat.bodypos
    #error = errorCalculation(pose_est, pose)

    #print("\n\
    #    \nInternal Estimated Pose XYZ:\
    #    \n{:.4f}  \t{:.4f}  \t{:.4f}\
    #    \nExternal Pose XZY: \
    #    \n{:.4f}  \t{:.4f}  \t{:.4f}\
    #    \nError Internal-External %: \
    #    \n{:.4f}  \t{:.4f}  \t{:.4f}"\
    #    .format(pose_est[0], pose_est[1], pose_est[2],\
    #            pose[0], pose[2], pose[1], \
    #            error[0], error[1], error[2]), end="")

def set_initial_position(cf, x, y, z, yaw_deg):
    cf.param.set_value('kalman.initialX', x)
    cf.param.set_value('kalman.initialY', y)
    cf.param.set_value('kalman.initialZ', z)
    yaw_radians = math.radians(yaw_deg)
    cf.param.set_value('kalman.initialYaw', yaw_radians)

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

            #print("{} {} {}".
            #      format(max_x - min_x, max_y - min_y, max_z - min_z))

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

def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')

def errorCalculation(estimation, poseData):
    err = [0, 0, 0]
    err[0] = abs(estimation[0]-poseData[0])*100/poseData[0]
    err[1] = abs(estimation[2]-poseData[1])*100/poseData[1]
    err[2] = abs(estimation[1]-poseData[2])*100/poseData[2]
    return err

def landing(cf):
    step = 1000
    currentThrust = thrust
    while currentThrust > 15000 and not keyboard.is_pressed('enter') :
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
    global Robotat
    event = Event()
    Robotat = RobotatHandler(RigidBodyID, event,\
                dataTime_Seconds, HOST, PORT,\
                send_full_pose)
    Robotat_shutdown = Thread(target = Emergency, \
                        args = (event, Robotat),daemon=True)
    Robotat_shutdown.start()
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        global cf
        cf = scf.cf
        Robotat.cf = cf
        #initParams(cf)
        activate_kalman_estimator(cf)
        activate_high_level_commander(cf)
        reset_estimator(cf)
        #activate_mellinger_controller(cf)

        #cf.commander.set_client_xmode(True)
        for iter in range(0,3):
            print(f"Start in {3-iter:d} seconds")
            event.wait(1)

        log_config = estimatorLogData(cf)
        localHeight = heightChanges[iterVar]
        try:
            while not keyboard.is_pressed('p'):
                pose = Robotat.bodypos
                print("\n{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}"\
                    .format(pose_est[0], pose_est[2], pose_est[1],\
                            pose[0], pose[1], pose[2]), end="")
            print("HOVERING...")
            it=0
            cf.param.set_value("kalman.quadIsFlying",1)
            cf.commander.send_setpoint(0, 0, 0, 0)
            while not keyboard.is_pressed('space'):
                #cf.commander.send_setpoint(0, 0, 0, 49000) # Solo controla la potencia y el giro. Complejo.
                cf.commander.send_zdistance_setpoint(0,0,0,0.2) #Más optimo, aunque no tiene control xy 
                #cf.commander.send_hover_setpoint(0,0,0,0.25) # No eleva el dron
                #cf.commander.send_position_setpoint(0, 0, 0.25, 0) # Pierde el sentido de su posición, aparentemente
                #cf.commander.send_velocity_world_setpoint(0,0,0.2,0)
                if it > 500:
                    pass
                    #cf.param.set_value("flightmode.althold",1) 
                #    pass
                it+=1
                #if it > 20:
                #    iterVar += 1
                #    if iterVar > heightsNum-1:
                #        break
                #    else:
                #        localHeight = heightChanges[iterVar] + 0.5
                #        #print(f"\n CHANGING Z to {localHeight:.1f}\n")d
                #        it = 0
                event.wait(0.01)
            cf.param.set_value("kalman.quadIsFlying",0)
            cf.param.set_value("flightmode.poshold",0)
            cf.param.set_value("flightmode.althold",0)
            print("\nClosing Log")
            log_config.stop()
            print("\nLanding and closing connection")
            if not keyboard.is_pressed('space'):
                landing(cf)
            event.wait(0.1)

        except BaseException as theError:
            # Don't execute the control code
            print('--------ERROR ON LOOP -----------') 
            print(f'Leaving because of ')
            print(theError)
        