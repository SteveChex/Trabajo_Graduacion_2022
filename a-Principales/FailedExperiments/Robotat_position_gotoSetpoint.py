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
uri = uri_helper.uri_from_env(default='radio://0/44/2M/E7E7E7E7E7')

strLn = [\
    [0, 0],
    [-1, 1]
    ]

objetives = [\
    [-1, 0],
    [1, 0]
    ]

# Axis Allign Variable
# 0: for direct allignation
# 1: for reversed allignation
axisAllign = 0

pathError = 0.08

angleValues = [[5,5], [-5,5], [-5,-5], [5,-5]] 

startpoint = [0, 1]
iterPoints = [0, 0]
inBorders = [0, 0]
commingFrom = [0, 0]


# Data holders for Motive Information
pose_est = [0, 0, 0, 0, 0, 0]
pose = [0, 0, 0]
quat = [0, 0, 0, 0]
eulAng = [0, 0, 0]

error = [0, 0, 0]

thrust = 45000
rotationThrust = 28000

itersPulse = 5
iterCounter = 0

# Robotat Vars
dataTime_Seconds = 0.1 # Time for updating position

# True: send position and orientation; False: send position only
send_full_pose = False

# Rigid body ID from Optitrack Marker
RigidBodyID = '11' #Object ID
 
# Motive network Data. Make sure to be connected to the right network
HOST = "192.168.50.200"  # The server's hostname or IP address
PORT = 1883  # The port used by the server

# Flag for emergency Thread
endProgram = False

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
                     
def Emergency(event, RobotatObj):
    while not keyboard.is_pressed('enter') and not endProgram:
        # minz = -1.2
        event.wait(0.1)
    if not endProgram:
        print("\n-EMERGENCY CALLED- \n DRONE AND MOTIVE CONNECTIONS TERMINATED")
    RobotatObj._stay_open = False
    while True:
        cf.commander.send_setpoint(0, 0, 0, 0)
        cf.commander.send_stop_setpoint()
        if endProgram:
            break

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

    pose = Robotat.bodypos
    error = errorCalculation(pose_est, pose)

    print("\n\
        \nInternal Estimated Pose XYZ:\
        \n{:.4f}  \t{:.4f}  \t{:.4f}\
        \nExternal Pose XZY: \
        \n{:.4f}  \t{:.4f}  \t{:.4f}\
        \nError Internal-External %: \
        \n{:.4f}  \t{:.4f}  \t{:.4f}"\
        .format(pose_est[0], pose_est[2], pose_est[1],\
                pose[0], pose[1], pose[2], \
                error[0], error[1], error[2]), end="")

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
    while currentThrust > 20000 and not keyboard.is_pressed('enter') :
        cf.commander.send_setpoint(0,0,0,currentThrust)
        currentThrust = currentThrust - step
        event.wait(0.1)
    cf.commander.send_setpoint(0,0,0,0)
    event.wait(0.1)

def keyboardCheck():
    roll = 0
    pitch = 0
    yawRate = 0
    if keyboard.is_pressed('w'):
        pitch = 5
    if keyboard.is_pressed('s'):
        pitch = -5
    if keyboard.is_pressed('d'):
        roll = 5
    if keyboard.is_pressed('a'):
        roll = -5
    if keyboard.is_pressed('e'):
        yawRate = 5
    if keyboard.is_pressed('q'):
        yawRate = -5
    if keyboard.is_pressed('r'):
        pass
    if keyboard.is_pressed('f'):
        pass
    return roll, pitch, yawRate

def check_inPoint(testPoint, expectedPoint, treshold):
    # testpoint = [num, num]
    # expectedPoint = [num, num]
    # treshold = num
    # Returns:
    #   [0, 0] = onPoint
    #   [1, 0] = Under coord 0
    #   [2, 0] = Over coord 0  
    #   [0, 1] = Under coord 1
    #   [0, 2] = Over coord 1
    test = [0, 0]
    test[0] = check_inCoord(testPoint[0], expectedPoint[0], treshold)
    test[1] = check_inCoord(testPoint[1], expectedPoint[1], treshold)
    return test

def check_inCoord(testPoint, expectedCoord, treshold):
    # testpoint = num
    # expectedCoord = num
    # treshold = num
    # Returns:
    #   0 = onPoint
    #   1 = Under
    #   2 = Over
    if testPoint < expectedCoord - treshold:
        return 1
    if testPoint > expectedCoord - treshold and testPoint < expectedCoord + treshold:
        return 0
    if testPoint > expectedCoord + treshold:
        return 2

def sliceTraj(initPoint, goalPoint, minSlice):
    """
    Cuts a init and end point into an array of points to follow as a a trajectory.
    """
    trajx = []
    trajy = []
    numSlices = [0, 0]
    for item in range(0,len(goalPoint)):
        numSlices[item] = math.ceil((goalPoint[item] - initPoint[item])/minSlice)
    numSlices = abs(min(numSlices))
    for item in range(0, numSlices + 1):
        distx = (goalPoint[0] - initPoint[0])*item/numSlices + initPoint[0]
        disty = (goalPoint[1] - initPoint[1])*item/numSlices + initPoint[1]
        trajx.append(round(distx,4))
        trajy.append(round(disty,4))
    traj = [trajx, trajy]
    return traj

def initParams(cf):
    pass
    # for roll, pitch, yaw controll
    #cf.param.set_value('flightmode.stabModeRoll', 1)
    #cf.param.set_value('flightmode.stabModePitch', 1)
    #cf.param.set_value('flightmode.stabModeYaw', 0)
    #cf.param.set_value('flightmode.yawMode', 2)

    # for absolute/relative Positioning
    #cf.param.set_value('flightmode.posSet', 1)


millis = lambda: int(round(time.time()*1000))

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
        initParams(cf)

        activate_kalman_estimator(cf)
        activate_high_level_commander(cf)
        reset_estimator(cf)

        #activate_mellinger_controller(cf)

        #cf.commander.set_client_xmode(True)
        # for some reason, the firmware detects x mode as T mode in this drone.
        #log_config = estimatorLogData(cf)
        try:
            """
            print("PHASE 1")
            for iter in range(0,3):
                print(f"Start in {3-iter:d} seconds")
                event.wait(1)
            # Phase 1: Rotate to a Yaw 0 position.
            # My personal project only have this orientation angle.
            yawRate = 0
            yawError = 1 #degrees
            cf.commander.send_setpoint(0,0,0,0)
            while not keyboard.is_pressed('space'):
                quat = Robotat.bodyquat
                eul = euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])
                eulL = list(eul)
                eulL[1] = eulL[1] + 70
                eul = tuple(eulL)
                print(eul)
                cf.commander.send_setpoint(0,0,yawRate,rotationThrust)
                if eul[1] > yawError:
                    yawRate = 10
                elif eul[1] < -yawError:
                    yawRate = -10
                if eul[1] > -yawError and eul[1] < yawError:
                    break
                event.wait(0.1)
                pass
            # Stop Commands
            cf.commander.send_setpoint(0,0,0,0)
            event.wait(0.1)
            cf.commander.send_setpoint(0,0,0,0)
            event.wait(1)

            print("PHASE 2")
            for iter in range(0,3):
                print(f"Start in {3-iter:d} seconds")
                event.wait(1)
            
            # Phase 2: Moving drone to check direction.
            it=0
            initPos = Robotat.bodypos
            cf.commander.send_setpoint(0,0,0,0)
            while not keyboard.is_pressed('space'):
                while it < 10:
                    cf.commander.send_setpoint(0,10,0,thrust)
                    event.wait(0.1)
                    it+=1
                break
            endPos = Robotat.bodypos

            cf.commander.send_setpoint(0,0,0,0)
            event.wait(0.1)
            cf.commander.send_setpoint(0,0,0,0)
            event.wait(1)      
                        
            if endPos[0] - initPos[0]:
                axisAllign = 0
            else:
                axisAllign = 1
            """
            print("PHASE 3")
            # Phase 3: Go to Startpoint
            # Startpoint stored at startpoint Variable
            it=0
            pitch = 0
            roll = 2
            yawRate = 0
            yawError = 2
            height = 0.2
            timePulseMS = 100
            timeIntervalMS = 300
            localCounter = 0
            arrived = 0
            brakeIntervalZ = 0
            brakeIntervalX = 0
            current = millis()
            currentX = millis()
            currentZ = millis()
            endFlags = [0, 0]
            initPoint = [Robotat.bodypos[0], Robotat.bodypos[2]]
            minDistance = 0.05
            testOnly = False
            traj = sliceTraj(initPoint, startpoint, minDistance)
            iterVar = 0
            reset = 0
            #cf.param.set_value("flightmode.poshold",0)
            cf.commander.send_setpoint(0,0,0,0)
            traj2show = np.transpose(traj)
            print("Showing Coords to follow (X Y):")
            for x, y in traj2show:
                print("{:.4f}\t{:.4f}".format(x, y))
            
            for iter in range(0,3):
                print(f"Start in {3-iter:d} seconds")
                event.wait(1)
            while not keyboard.is_pressed('space'):
                pos = Robotat.bodypos
                quat = Robotat.bodyquat
                inBorders = check_inPoint([pos[0], pos[2]],\
                                [traj[0][iterVar], traj[1][iterVar]], minDistance)
                eul = euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])
                eulL = list(eul)
                eulL[1] = eulL[1] + 70
                eul = tuple(eulL)
                pulseFlagX = 0
                pulseFlagZ = 0
                endFlags = [0, 0]
                arrived = 0
                if millis() - timeIntervalMS + timePulseMS > currentX:
                    if millis() - timeIntervalMS > currentX:
                        currentX = millis() 
                    pulseFlagX = 1

                pulseFlagZ = 0
                if millis() - timeIntervalMS + timePulseMS > currentZ:
                    if millis() - timeIntervalMS > currentZ:
                        currentZ = millis() 
                    pulseFlagZ = 1

                #if eul[1] > yawError:
                #    yawRate = 20
                #elif eul[1] < -yawError:
                #    yawRate = -20

                # Z align 
                if inBorders[1] == 1:
                    commingFrom[1] = 1
                    if pulseFlagZ:
                        roll = 2
                    else:
                        roll = 0
                elif inBorders[1] == 2:                    
                    commingFrom[1] = 2
                    if pulseFlagZ:
                        roll = -2
                    else:
                        roll = 0

                # X align 
                if inBorders[0] == 1:
                    commingFrom[0] = 1
                    if pulseFlagX:
                        pitch = -2
                    else:
                        pitch = 0
                elif inBorders[0] == 2:
                    commingFrom[0] = 2  
                    if pulseFlagX:
                        pitch = 2
                    else:
                        pitch = 0
                # Z positivo = Roll positivo    
                # X Positivo = Pitch Negativo
                #if inBorders[1] == 1:
                #    commingFrom[1] = 1
                #    roll = 1
                #elif inBorders[1] == 2:
                #    commingFrom[1] = 1
                #    roll = -1
                if inBorders[1] == 0:
                    currentZ = millis() - timeIntervalMS + timePulseMS
                    arrived = 1
                    brakeIntervalZ += 1
                    if brakeIntervalZ >= 5:
                        roll = 0
                        endFlags[1] = 1
                        commingFrom[1] == 0
                    if commingFrom[1] == 1:
                        roll = -3
                        if not testOnly:
                            cf.commander.send_zdistance_setpoint(roll,pitch,0,height)
                    if commingFrom[1] == 2:
                        roll = 3
                        if not testOnly:
                            cf.commander.send_zdistance_setpoint(roll,pitch,0,height)                        
                    
                if inBorders[0] == 0:
                    currentX = millis() - timeIntervalMS + timePulseMS
                    arrived = 1
                    brakeIntervalX += 1
                    if brakeIntervalX >= 5:
                        pitch = 0
                        endFlags[0] = 1
                        commingFrom[0] == 0
                    if commingFrom[0] == 1:
                        pitch = 3
                        if not testOnly:
                            cf.commander.send_zdistance_setpoint(roll,pitch,0,height)
                    if commingFrom[0] == 2:
                        pitch = -3
                        if not testOnly:
                            cf.commander.send_zdistance_setpoint(roll,pitch,0,height)        
                
                if not arrived:
                    if not testOnly:
                        cf.commander.send_zdistance_setpoint(roll,pitch,0,height)
                    if not inBorders[1] == 0:
                        brakeIntervalZ = 0
                    if not inBorders[0] == 0:
                        brakeIntervalZ = 0
                if endFlags[1] == 1 and endFlags[0] == 1:
                    endFlags[1] = 0
                    endFlags[0] = 0
                    brakeIntervalX = 0
                    brakeIntervalZ = 0
                    commingFrom[1] == 0
                    commingFrom[0] == 0
                    iterVar+= 1
                    if iterVar > len(traj[0])-1:
                        break
                print("\nPosition XZ  \t{:.4f}\t{:.4f}\
                        \nGoing to XZ \t{:.4f}\t{:.4f}\
                        \nR P Yr Parameters           \t{:.1f}\t{:.1f}\t{:.1f}\t\
                        \narrived     \t{:d}\
                        \nendFlags                    \t{:d}\t{:d}\t\
                        \npostraj No: \t{:d} of {:d}"\
                        .format(pos[0], pos[2],\
                                traj[0][iterVar], traj[1][iterVar],
                                roll, pitch, pos[1],\
                                    arrived,
                                    endFlags[0], endFlags[1],\
                                    iterVar+1, len(traj[0])),end = "")
                if keyboard.is_pressed('+'):
                    print("\t\tOBSERVATION:", end = "")
                #print("\nPosition XYZ       \t{:.4f}\t{:.4f}\t{:.4f}\t\
                #        \nIn Position Flags \t\t{:d}\t{:d}\
                #        \nR P Yr Parameters           \t{:.4f}\t{:.4f}\t{:.4f}\t\
                #        \nComming From and pulse Flags\t\t{:d}\t{:d}\t{:d}"
                #        .format(pos[0], pos[1], pos[2],\
                #                inBorders[0], inBorders[1],\
                #                roll, pitch, height,\
                #                commingFrom[0], commingFrom[1], pulseFlagX),end = "")
                #print("\nPosition XYZ                 \t\t\t{:.4f}\t{:.4f}\t{:.4f}\t\
                #        \nR P Yr Parameters           \t{:.4f}\t{:.4f}\t{:.4f}\t"
                #        .format(pos[0], pos[1], pos[2],\
                #                roll, pitch, yawRate),end = "")
                event.wait(0.05)

            it = 0
            h = 0.2
            while h <= 0.03:
                cf.commander.send_zdistance_setpoint(0,0,0,height)
                h -= 0.005
                event.wait(0.01)

            cf.commander.send_setpoint(0,0,0,0)
            event.wait(0.1)
            cf.commander.send_setpoint(0,0,0,0)
            event.wait(1)   

            # Phase 4: Pseudotrajectories
            # 
            # Stop Commands
            
            print("\nClosing Log")
            #log_config.stop()
            print("\nLanding and closing connection")
            #landing(cf)
            event.wait(0.1)

        except BaseException as theError:
            # Don't execute the control code
            print('--------ERROR ON LOOP -----------') 
            print(f'Leaving because of ')
            print(theError)
            cf.commander.send_stop_setpoint()
            #log_config.stop()
    
    endProgram = True




