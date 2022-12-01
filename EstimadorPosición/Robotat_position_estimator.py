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
import pandas as pd
from scipy.spatial.transform import Rotation as R
import numpy as np

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

# Borderlines for pathing.
# intended to be points that should be followed by the drone.
# Straight line

strLn = [\
    [0, 0],
    [-1, 1]
    ]

pathError = 0.1

iterPoints = [0, 0]

# Data holders for Motive Information
pose_est = [0, 0, 0, 0, 0, 0]
pose = [0, 0, 0]
quat = [0, 0, 0, 0]
eulAng = [0, 0, 0]

error = [0, 0, 0]

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
        self._stay_open = False
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
        self._stay_open = True
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
            quat2go = R.from_quat(self.bodyquat)
            quat2go = quat2go * R.from_euler('z', -180, degrees = True)
            quat2go = quat2go.as_quat()
            # send_extpos(x, y, z, qx, qy, qz, qw)
            self.cf.extpos.send_extpose(\
                self.bodypos[0], self.bodypos[2], self.bodypos[1], \
                quat2go[0], quat2go[1], quat2go[2], \
                quat2go[3])
            return
        else:
            # send_extpos(x, y, z)
            self.cf.extpos.send_extpos(\
                self.bodypos[0], self.bodypos[2], self.bodypos[1])
            return 
    
    def sendInitParams(self):
        if self._stay_open == False:
            return
        if self.cf == None:
            return
        self.cf.param.set_value('kalman.initialX',self.bodypos[0])
        self.cf.param.set_value('kalman.initialZ',self.bodypos[1])
        self.cf.param.set_value('kalman.initialY',self.bodypos[2])
        eul = euler_from_quaternion(\
            self.bodyquat[0], self.bodyquat[1], self.bodyquat[2],\
            self.bodyquat[3])
        self.cf.param.set_value('kalman.initialYaw',eul[1])
        #self.cf.param.set_value('imu_sensors.imuPhi',0.0)
        

def Emergency(event, RobotatObj):
    while not keyboard.is_pressed('space'):
        RobotatObj._stay_open = True
        event.wait(0.05)
    print("-EMERGENCY CALLED- \n DRONE AND MOTIVE CONNECTIONS TERMINATED")
    RobotatObj._stay_open = False
    while True:
        cf.commander.send_setpoint(0, 0, 0, 0)
        cf.commander.send_stop_setpoint()

def euler_from_quaternion(x, y, z, w):
    """
    xi yj zk w
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

def rotation_matrix(theta1, theta2, theta3, order='xyz'):
    """
    input
        theta1, theta2, theta3 = rotation angles in rotation order (degrees)
        oreder = rotation order of x,y,z　e.g. XZY rotation -- 'xzy'
    output
        3x3 rotation matrix (numpy array)
    """
    c1 = np.cos(theta1 * np.pi / 180)
    s1 = np.sin(theta1 * np.pi / 180)
    c2 = np.cos(theta2 * np.pi / 180)
    s2 = np.sin(theta2 * np.pi / 180)
    c3 = np.cos(theta3 * np.pi / 180)
    s3 = np.sin(theta3 * np.pi / 180)

    if order == 'xzx':
        matrix=np.array([[c2, -c3*s2, s2*s3],
                         [c1*s2, c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3],
                         [s1*s2, c1*s3+c2*c3*s1, c1*c3-c2*s1*s3]])
    elif order=='xyx':
        matrix=np.array([[c2, s2*s3, c3*s2],
                         [s1*s2, c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1],
                         [-c1*s2, c3*s1+c1*c2*s3, c1*c2*c3-s1*s3]])
    elif order=='yxy':
        matrix=np.array([[c1*c3-c2*s1*s3, s1*s2, c1*s3+c2*c3*s1],
                         [s2*s3, c2, -c3*s2],
                         [-c3*s1-c1*c2*s3, c1*s2, c1*c2*c3-s1*s3]])
    elif order=='yzy':
        matrix=np.array([[c1*c2*c3-s1*s3, -c1*s2, c3*s1+c1*c2*s3],
                         [c3*s2, c2, s2*s3],
                         [-c1*s3-c2*c3*s1, s1*s2, c1*c3-c2*s1*s3]])
    elif order=='zyz':
        matrix=np.array([[c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3, c1*s2],
                         [c1*s3+c2*c3*s1, c1*c3-c2*s1*s3, s1*s2],
                         [-c3*s2, s2*s3, c2]])
    elif order=='zxz':
        matrix=np.array([[c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1, s1*s2],
                         [c3*s1+c1*c2*s3, c1*c2*c3-s1*s3, -c1*s2],
                         [s2*s3, c3*s2, c2]])
    elif order=='xyz':
        matrix=np.array([[c2*c3, -c2*s3, s2],
                         [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                         [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])
    elif order=='xzy':
        matrix=np.array([[c2*c3, -s2, c2*s3],
                         [s1*s3+c1*c3*s2, c1*c2, c1*s2*s3-c3*s1],
                         [c3*s1*s2-c1*s3, c2*s1, c1*c3+s1*s2*s3]])
    elif order=='yxz':
        matrix=np.array([[c1*c3+s1*s2*s3, c3*s1*s2-c1*s3, c2*s1],
                         [c2*s3, c2*c3, -s2],
                         [c1*s2*s3-c3*s1, c1*c3*s2+s1*s3, c1*c2]])
    elif order=='yzx':
        matrix=np.array([[c1*c2, s1*s3-c1*c3*s2, c3*s1+c1*s2*s3],
                         [s2, c2*c3, -c2*s3],
                         [-c2*s1, c1*s3+c3*s1*s2, c1*c3-s1*s2*s3]])
    elif order=='zyx':
        matrix=np.array([[c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2],
                         [c2*s1, c1*c3+s1*s2*s3, c3*s1*s2-c1*s3],
                         [-s2, c2*s3, c2*c3]])
    elif order=='zxy':
        matrix=np.array([[c1*c3-s1*s2*s3, -c2*s1, c1*s3+c3*s1*s2],
                         [c3*s1+c1*s2*s3, c1*c2, s1*s3-c1*c3*s2],
                         [-c2*s3, s2, c2*c3]])
    return matrix


def estimatorLogData(cf):
    log_config = LogConfig(name='Kalman Variance', period_in_ms=50)
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
    #print(data)
    global pose_est
    pose_est[0] = data['stateEstimate.x']
    pose_est[1] = data['stateEstimate.y']
    pose_est[2] = data['stateEstimate.z']
    pose_est[3] = data['stateEstimate.roll']
    pose_est[4] = data['stateEstimate.pitch']
    pose_est[5] = data['stateEstimate.yaw']

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

def errorCalculation(estimation, poseData, eulerData):
    err = [0, 0, 0, 0, 0, 0]
    err[0] = abs(estimation[0]-poseData[0])*100/poseData[0]
    err[1] = abs(estimation[2]-poseData[1])*100/poseData[1]
    err[2] = abs(estimation[1]-poseData[2])*100/poseData[2]
    err[3] = abs(estimation[3]-eulerData[0])*100/eulerData[0]
    err[4] = abs(estimation[4]-eulerData[1])*100/eulerData[1]
    err[5] = abs(estimation[5]-eulerData[2])*100/eulerData[2]
    return err

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
        cf = scf.cf
        Robotat.cf = cf
        #yRobotat.sendInitParams()
        activate_kalman_estimator(cf)
        activate_high_level_commander(cf)
        reset_estimator(cf)
        #activate_mellinger_controller(cf)
        log_config = estimatorLogData(cf)
        cf.commander.send_setpoint(0, 0, 0, 0)
        try:
            while not keyboard.is_pressed('space'):  # if key 'space' is pressed
                pose = Robotat.bodypos
                quat = Robotat.bodyquat
                posR = R.from_quat(quat)
                posR = posR * R.from_euler('z', -180, degrees = True)
                quatR = posR.as_quat()
                #quatR = [0, 0, 0, 0]
                eul = euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])
                eulL = list(eul)
                eulL[1] = eulL[1] + 70
                eul = tuple(eulL)
                eulEst = rotation_matrix(eul[0], eul[1], eul[2], 'xyz')
                eulEst2 = rotation_matrix(pose_est[3], pose_est[4], pose_est[5], 'xyz')
                
                #print("\n\nRotation Matrix Robotat \
                #\n{:.4f}\t{:.4f}\t{:.4f}\t\
                #\n{:.4f}\t{:.4f}\t{:.4f}\t\
                #\n{:.4f}\t{:.4f}\t{:.4f}\t\
                #\nRotation Matrix Crazyflie\
                #\n{:.4f}\t{:.4f}\t{:.4f}\t \
                #\n{:.4f}\t{:.4f}\t{:.4f}\t \
                #\n{:.4f}\t{:.4f}\t{:.4f}\t "\
                #.format(eulEst[0][0], eulEst[0][1], eulEst[0][2],\
                #        eulEst[1][0], eulEst[1][1], eulEst[1][2],\
                #        eulEst[2][0], eulEst[2][1], eulEst[2][2],\
                #        eulEst2[0][0], eulEst2[0][1], eulEst2[0][2],\
                #        eulEst2[1][0], eulEst2[1][1], eulEst2[1][2],\
                #        eulEst2[2][0], eulEst2[2][1], eulEst2[2][2],\
                #), end = "")
                #eulAngEst = R.as_matrix()
                #print(eul.as_matrix())
                #print("")
                #print(eulEst.as_matrix())
                #error = errorCalculation(pose_est, pose, eulAng)

                #print("\n\
                #    \nInternal Estimated Pose RPY:\
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}\
                #    \nExternal Pose  quaternion xi yj zk w: \
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}\
                #    \nExternal Pose Euler RPY: \
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}"\
                #    .format( pose_est[3], pose_est[4], pose_est[5],\
                #            quat[0], quat[1], quat[2], quat[3], \
                #            eul[0], eul[1], eul[2]), end="")
                
                
                #print("\n\
                #    \nInternal Estimated Pose XYZ RPY:\
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}\
                #    \nExternal Pose XZY: \
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}\
                #    \nExternal Pose  quaternion xi yj zk w: \
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}\
                #    \nExternal Pose Euler RPY: \
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}\
                #    \nError Internal-External %: \
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}"\
                #    .format(pose_est[0], pose_est[2], pose_est[1], pose_est[3], pose_est[4], pose_est[5],\
                #            pose[0], pose[1], pose[2], \
                #            quat[0], quat[1], quat[2], quat[3], \
                #            eulAng[0], eulAng[1], eulAng[2], \
                #            error[0], error[1], error[2], error[3], error[4], error[5]), end="")
                #print("\n\
                #    \nInternal Estimated Pose XYZ RPY:\
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}\
                #    \nExternal Pose XZY RPY: \
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}\
                #    \nExternal Pose  quaternion xi yj zk w: \
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}\
                #    \nExternal Pose  Rotated: \
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}  \
                #    \nExternal Pose  quaternion Rotated xi yj zk w: \
                #    \n{:.4f}  \t{:.4f}  \t{:.4f}  \t{:.4f}"\
                #    .format(pose_est[0], pose_est[2], pose_est[1], pose_est[3], pose_est[4], pose_est[5],\
                #            pose[0], pose[1], pose[2], eul[0], eul[1], eul[2],\
                #            quat[0], quat[1], quat[2], quat[3], \
                #            posR.as_euler('xyz', degrees = True)[0], posR.as_euler('xyz', degrees = True)[1], posR.as_euler('xyz', degrees = True)[2],\
                #            quatR[0], quatR[1], quatR[2], quatR[3]), end="")

                print("\n\
                    \nInternal Estimated Pose XYZ RPY:\
                    \n{:.4f}\t{:.4f}\t{:.4f}  \t{:.1f}\t{:.1f}\t{:.1f}\
                    \nExternal Pose XZY RPY: \
                    \n{:.4f}\t{:.4f}\t{:.4f}  \t{:.1f}\t{:.1f}\t{:.1f}"\
                    .format(pose_est[0], pose_est[2], pose_est[1], pose_est[3], pose_est[4], pose_est[5],\
                            pose[0], pose[1], pose[2], eul[0], eul[1], eul[2]), end="")

                # FOR STRAIGHT LINE ON 0 ONLY
                # if abs(pose[0]) > strLn[0][0] + pathError:
                #    print("\nOUT OF PATH on X", end = "")
                if pose[0] > strLn[0][0] - pathError and pose[0] < strLn[0][0] + pathError:
                    pass
                    #print("\nON PATH on X", end = "")
                #if pose[0] < strLn[0][0] - pathError or pose[0] > strLn[0][0] + pathError:
                else:   
                    pass 
                    #print("\nOUT OF PATH on X", end = "")
                if pose[2] > strLn[1][0] - pathError and pose[2] < strLn[1][0] + pathError:
                    pass
                    #print("\nAt Point 1", end = "")
                if pose[2] > strLn[1][1] - pathError and pose[2] < strLn[1][1] + pathError:
                    pass
                    #print("\nAt Point 2", end = "")
                event.wait(0.1)

        except BaseException as theError:
            # Don't execute the control code
            print('--------ERROR ON LOOP -----------') 
            print(f'Leaving because of ')
            print(theError)
        log_config.stop()



#Another controll idea
                #if eulAng[1] > 0.5:
                #    cf.commander.send_setpoint(0, 0, 20, 15000)
                #    time.sleep(0.1)
                #elif eulAng[1] < -0.5: 
                #    cf.commander.send_setpoint(0, 0, -20, 15000)
                #    time.sleep(0.1)
                #else:
                #    cf.commander.send_setpoint(0, 0, 0, 0)
                #    time.sleep(0.1)