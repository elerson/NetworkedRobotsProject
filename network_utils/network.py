#!/usr/bin/python
from socket import *
import threading
import sys
from time import sleep
import cPickle as pickle
import time
from enum import IntEnum


class TYPE(IntEnum):
    WAITREPLY    = 0
    NOWAITREPLY  = 1
    ACKWAITREPLY = 2


class Network:
    
    def __init__(self, id=-1, broadcast_addr = "127.255.255.255", port = 9988):
        self.running = True
        self.port = port
        self.broadcast_addr = broadcast_addr

        #configure send socket
        self.send_socket = socket(AF_INET, SOCK_DGRAM, SOL_UDP)
        self.send_socket.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
        #self.send_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

        #configure rcv socket
        self.rcv_socket = socket(AF_INET, SOCK_DGRAM, SOL_UDP)
        self.rcv_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

        self.rcv_socket.bind(('', self.port))

        self.rcv_thread = threading.Thread(target=self.rcvMessage)
        self.rcv_thread.start()
        self.num_msg = {}
        self.id      = id

        self.rcv_data    = {}
        self.rcv_command = {}

        self.command_callback = None
        self.message_callback = None
        self.msg_num = 0

        self.lock = threading.Lock()

    def addCommandCallback(self, callback):
        self.command_callback = callback

    def addMessageCallback(self, callback):
        self.message_callback = callback

    def __del__(self):
        self.running = False
        self.rcv_thread.join()

        pass
        
    def rcvMessage(self):
        while(self.running):
            data_string = self.rcv_socket.recvfrom(2048)
            #print(data_string[0])
            rcv_data = pickle.loads(data_string[0])

            if not 'id' in rcv_data:
                continue
            if(rcv_data['id'] == self.id and self.id > 0):
                continue


            id = rcv_data['id']
            if(id >= 0):
                #print(rcv_data)
                if(rcv_data['_type_']  == TYPE.WAITREPLY and (rcv_data['_dst_'] == self.id or self.id < 0) ):

                    #print('rcv msg', rcv_data, rcv_data['id'], rcv_data['_dst_'], self.id, rcv_data['_type_'])
                    if not id in self.num_msg:
                        self.num_msg[id] = -1

                    if(rcv_data['_num_'] == self.num_msg[id]):
                        pass
                    else:
                        self.num_msg[id] = rcv_data['_num_']                     

                    if(self.id >= 0):
                        ack_message = {}
                        self.sendAck(rcv_data['id'], ack_message)

                    if self.message_callback != None:
                        #print(rcv_data)
                        self.message_callback(rcv_data)
                    
                elif (rcv_data['_type_']  == TYPE.ACKWAITREPLY and rcv_data['_dst_'] == self.id):
                    self.waiting_reply  = False

                elif (rcv_data['_type_']  == TYPE.ACKWAITREPLY):
                    pass

                else:

                    if(rcv_data['_type_'] == TYPE.NOWAITREPLY):
                        self.lock.acquire()
                        self.rcv_data[id]   = rcv_data
                        self.lock.release()

                        if self.message_callback != None:
                            self.message_callback(rcv_data)
            else:
                self.rcv_command[id] = rcv_data
                if self.command_callback != None:
                    self.command_callback()

    def getDataIds(self):
        self.lock.acquire()
        keys = list(self.rcv_data.keys())
        self.lock.release()
        return keys


    def getData(self, id):
        self.lock.acquire()
        data = self.rcv_data[id].copy()
        self.lock.release()
        return data

    def addMessage(self, message):
        self.rcv_data[message['id']] = message
            #print(self.rcv_data)
    #message must be an dictionary with id field
    def sendMessage(self, message):
        #print(message)
        #save message before sending
        message['_type_'] = TYPE.NOWAITREPLY
        self.rcv_data[message['id']] = message

        data_string = pickle.dumps(message, -1) 

        self.send_socket.sendto(data_string,(self.broadcast_addr, self.port))
        pass

    def sendMessageTo(self, dst_id, message, resend_time = 100):


        message['_type_'] = TYPE.WAITREPLY
        message['_dst_']  = dst_id
        message['_num_']  = self.getMsgNum()
        message['id']     = self.id
        #print(message, 'message')
        data_string = pickle.dumps(message, -1) 
        self.send_socket.sendto(data_string, (self.broadcast_addr, self.port))

        self.waiting_reply = True
        retry_count        = resend_time
        while(self.waiting_reply):
            if(retry_count <= 0):
                retry_count     = resend_time
                self.send_socket.sendto(data_string, (self.broadcast_addr, self.port))
                #print('retry', message)

            retry_count -= 1
            time.sleep(0.01)

    def sendAck(self, dst_id, message):


        message['_type_'] = TYPE.ACKWAITREPLY
        message['_dst_']  = dst_id
        message['_num_']  = self.getMsgNum()
        message['id']     = self.id

        data_string = pickle.dumps(message, -1) 
       
        self.send_socket.sendto(data_string, (self.broadcast_addr, self.port))

        #wait for response


    def getMsgNum(self):
        self.msg_num += 1
        return self.msg_num



# if __name__ == '__main__':
#     id = sys.argv[1]
#     network = Network()
#     data = {}
#     data['id'] = id 
#     data['position'] = (id, id)
#     while(True):

#         network.sendMessage(data)
#         sleep(0.5)
