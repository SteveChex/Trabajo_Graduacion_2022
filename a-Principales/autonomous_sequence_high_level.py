# -*- coding: utf-8 -*-
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and uses the high level commander
to send setpoints and trajectory to fly a figure 8.

This example is intended to work with any positioning system (including LPS).
It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints using the high level commander.

Example modded by Steve Chex, to work with Optitrack Mocap.
"""
import sys
import time
from threading import Thread, Event
import socket
import keyboard

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/44/2M/E7E7E7E7E7')

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
figure8 = [
    [2, -1.0, 0.0, 0.0, -7.28583859910259e-16, 4.374999999999999, -5.25, 2.1875, -0.3125, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -5.828670879282072e-16, 5.464378949326942e-15, -6.7394007041698956e-15, 3.1761702642962852e-15, -5.89128355474311e-16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
]

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
            # Sent as XZY from Optitrack
            self.cf.extpos.send_extpos(\
                self.bodypos[0], self.bodypos[2], self.bodypos[1])
            return 

def inBoundaries(poseData, box, celling):
    if abs(poseData[0]) > box or \
            abs(poseData[2]) > box or \
            abs(poseData[1]) > celling:
        return False
    return True

def Emergency(event, RobotatObj):
    # the variable cf should be global at the main definition of the drone in the code
    inside = True
    while not keyboard.is_pressed('enter'):
        event.wait(0.1)
    print("")
    print("-EMERGENCY CALLED- \n DRONE AND MOTIVE CONNECTIONS TERMINATED")
    RobotatObj._stay_open = False
    while True:
        cf.high_level_commander.stop()
        cf.commander.send_stop_setpoint()
        cf.commander.send_stop_setpoint()
        cf.commander.send_stop_setpoint()

def estimatorLogData(cf):
    log_config = LogConfig(name='Kalman Variance', period_in_ms=10)
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

class Uploader:
    def __init__(self):
        self._is_done = False
        self._success = True
    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done, write_failed_cb=self._upload_failed)
        while not self._is_done:
            time.sleep(0.2)
        return self._success
    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True
        self._success = True
    def _upload_failed(self, mem, addr):
        print('Data upload failed')
        self._is_done = True
        self._success = False

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')
    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
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

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(cf)

def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')

def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')

def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []
    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration
    upload_result = Uploader().upload(trajectory_mem)
    if not upload_result:
        print('Upload failed, aborting!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration

def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander
    commander.takeoff(0.4, 1.0)
    event.wait(2.0)
    relative = False
    commander.start_trajectory(trajectory_id, 1.0, relative)
    event.wait(duration)
    commander.land(0.0, 2.0)
    event.wait(2)
    commander.stop()

if __name__ == '__main__':
    event = Event()
    Robotat = RobotatHandler(RigidBodyID, event,\
                 dataTime_Seconds, HOST, PORT, \
                 send_full_pose)
    Robotat_shutdown = Thread(target = Emergency, \
                        args = (event, Robotat),daemon=True)
    Robotat_shutdown.start()
    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        global cf
        cf = scf.cf
        Robotat.cf = cf
        trajectory_id = 1
        activate_high_level_commander(cf)
        try:
            duration = upload_trajectory(cf, trajectory_id, figure8)
            print('The sequence is {:.1f} seconds long'.format(duration))
            reset_estimator(cf)
            for iter in range(0,3):
                print(f"Start in {3-iter:d} seconds")
            event.wait(1)
            run_sequence(cf, trajectory_id, duration)    
        except BaseException as TheFail:
            print("Main program Fail")    
            print(f"Reason: {TheFail}")
        Robotat._stay_open = False

    
