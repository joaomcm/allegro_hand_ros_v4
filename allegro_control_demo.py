import socket
import numpy as np
from threading import Thread,Lock
import time
import re
from copy import deepcopy

import rospy
from sensor_msgs.msg import JointState

import pandas as pd
import klampt
from klampt import WorldModel
from klampt.model import ik
from klampt import vis
from copy import deepcopy
import numpy as np
from klampt.math import se3,so3


class HandController:
    def __init__(self):
        self.state = JointState()
        self.test_pub = rospy.Publisher('/allegroHand_0/joint_cmd', JointState, queue_size=10)
        rospy.init_node('TestNode')
        self.world = WorldModel()

        self.world.loadElement('./src/allegro_hand_description/klampt_allegro_hand_right.urdf')
        vis.init('PyQt')

        vis.add('world',self.world)

        vis.show()
        self.robot = self.world.robot(0)
        self.zero_config = self.robot.getConfig()
        self.get_span()
        self.hand_alignment_and_scaling()
        self.active_dofs_list = [[11,12,13,14],[6,7,8,9],[16,17,18,19],[21,22,23,24]]
        self.to_state_msg = [6,7,8,9,16,17,18,19,21,22,23,24,11,12,13,14]

    def get_span(self,calibration_run = './jingchen_hand_run.pkl'):
        if(type(calibration_run) == str):
            self.hand_df = pd.read_pickle(calibration_run)
        else:
            self.hand_df = calibration_run
        pos = self.hand_df.positions[0]

        max_vals = np.zeros(shape = (4,4))
        max_vals[:] = -np.inf
        min_vals = np.zeros(shape = (4,4))
        min_vals[:] = np.inf
        for angle in self.hand_df.angles:
            angle = angle.astype(int)
            max_vals = np.maximum(max_vals,angle)
            min_vals = np.minimum(min_vals,angle)
            
        span = max_vals-min_vals
        self.span = span
        self.min_vals = min_vals
        self.max_vals = max_vals

    def hand_alignment_and_scaling(self):
        self.pc = klampt.PointCloud()
        # for i in range()
        self.R = so3.from_rotation_vector([0,-np.pi/2+0.25,0])
        self.T = (self.R,[0,0,-0.2])
    def animate_and_command(self,pos,angle,control = False):
        angle = angle.astype(float)
        self.pc.setPoints(1.5*pos.reshape(-1,3)/1000)
        self.pc.transform(self.R,[-0.04,-0.01,-0.05])
        vis.add('pc',self.pc,size = 40,color = [1,0,0,0.5])
        command = np.clip((np.pi/2)*(angle-self.min_vals)/self.span,0,90)
        points = self.pc.getPoints()
    #     robot.setConfig(orig_config)
        prev_config = self.robot.getConfig()
        toe = self.robot.link('link_15.0_tip')
        index = self.robot.link('link_3.0_tip')
        middle = self.robot.link('link_7.0_tip')
        ring = self.robot.link('link_11.0_tip')
        objs = []
        obj_toe = ik.objective(toe,local=[0,0,0.017],world=points[3])
    #     obj_index = ik.objective(index,local=[0,0,0.017],world=points[7])
    #     obj_middle = ik.objective(middle,local=[0,0,0.017],world=points[11])
    #     obj_ring = ik.objective(ring,local=[0,0,0.017],world=points[19])
        iks_to_solve = {'toe':[obj_toe,[11,12,13,14]]}#,'index':[obj_index,[6,7,8,9]],
                        #'middle':[obj_middle,[16,17,18,19]],'ring':[obj_ring,[21,22,23,24]]}
    #     objs = [obj_toe,obj_index,obj_middle,obj_ring]
        for key in iks_to_solve.keys():
            obj,active_dofs = iks_to_solve[key]
            prev_config = self.robot.getConfig()
            vis.lock()
            res = ik.solve_global(obj,activeDofs= active_dofs,numRestarts = 10)
            new_config = self.robot.getConfig()
            self.robot.setConfig(prev_config)
            if(res):
                self.robot.setConfig(new_config)
            vis.unlock()
            conf = self.robot.getConfig()
            conf[7:10] = command[0][1:]
            conf[17:20] = command[1][1:]
            conf[22:25] = command[2][1:]
            conf[21] = 0
            vis.lock()
            self.robot.setConfig(conf)
            vis.unlock()
            if(control):
                self.state = JointState()
                self.state.position = np.array(conf)[[self.to_state_msg]]
                self.test_pub.publish(self.state)

class UDP_Server:
    def __init__(self,record = False,control = False):

        self.localIP     = "0.0.0.0"

        self.localPort   = 54000

        self.bufferSize  = 2048
        self.times = []

        self.array_lock = Lock()
        self.df = None
        self.control = False
        self.msgFromServer       = "Hello UDP Client"

        self.bytesToSend         = str.encode(self.msgFromServer)
        self.positions = []
        self.angles = []
        self.thumb_angles = []
        self.record = record
        # Create a datagram socket
        self.hand_controller = HandController()

        self.serviceThread = Thread(target = self.continuous_service)
        self.serviceThread.start()

        print("UDP server up and listening")
    def continuous_service(self):
        self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

        # Bind to address and ip

        self.UDPServerSocket.bind((self.localIP, self.localPort))
        while(True):
            # print('servicing')
            # output = self.receive_message()
            # with self.array_lock:
            #     self.this_array = output
            #     print('updating!')
            positions,angles,thumb_angles = self.receive_message()
            if self.record:
                self.positions.append(positions)
                self.angles.append(angles)
                self.thumb_angles.append(thumb_angles)
                self.times.append(time.time())
            # if self.control:
            self.hand_controller.animate_and_command(positions,angles,control = self.control)

    
    def receive_message(self):
#         print('waiting for message')
        bytesAddressPair = self.UDPServerSocket.recvfrom(self.bufferSize)

        message = bytesAddressPair[0]

        address = bytesAddressPair[1]
#         print('processing image')
        clientMsg = "Message from Client:{}".format(message)
        clientIP  = "Client IP Address:{}".format(address)
        # print('last_original_character: ',message[-5:-1])
    #     print(clientMsg)
    #     print(clientIP)
#         print('got data')
#         self.message= message
        self.debug_message = deepcopy(message)
        angles,positions,thumb_positions = self.decode_message(deepcopy(message)[:-1])
#         this_str = eval(message[:-1].decode("utf-8"))
#         this_array = np.array(this_str)
#         this_array = this_array
        return angles,positions,thumb_positions
    
    def switch_recording(self,dump = False,name = './example_run.pkl'):
        self.record = not self.record
        df = pd.DataFrame({'time':self.times,'positions':self.positions,'angles':self.angles,'thumb_angles':self.thumb_angles})
        self.df = df
        if(dump):
            df.to_pickle(name)
    def switch_control(self):
        if(self.df is not None):
            pass
        else:
            if(self.record == True):
                self.switch_recording()
            if(self.record == False):
                self.record = True
                time.sleep(30)
                self.switch_recording()

        self.hand_controller.get_span(self.df)
        self.control = not self.control



    def decode_message(self,msg):
        # print('last passed character: ',msg[-5:-1])
        a = msg.decode('utf-8').replace('\t','').replace('\n',';')
        a = eval(a)
        positions = a[1]
        b = a[0].split(';')
        # regex - finds all values between colons and commas
        r = re.compile('(?<=: ).*?(?=,)')
        angles = []
        for c in b:
            g = r.findall(c+',')
        #     print(r.findall(c))
            angles.append(g)
        return np.array(positions),np.array(angles[1:]),np.array(angles[0])

if(__name__ == '__main__'):
    server = UDP_Server(record = False,control = False)    # # time.sleep(5)
    time.sleep(5)
    # ### decrypt message
    # # a = server.message.decode('utf-8').replace('\t','').replace('\n',';')
    # # a = eval(a)
    # # a[0] = a[0].replace('\'','')
    # # a[0].split(';')[1].split(':')


    # print('started!!!!')
    # ani = animation.FuncAnimation(fig, server.update_graph,frames = 10000000, 
    #                             interval=1, blit=False)