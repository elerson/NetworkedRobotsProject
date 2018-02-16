#!/usr/bin/python
from socket import *
import threading
import sys
from time import sleep
import cPickle as pickle



class Network:
    
    def __init__(self, broadcast_addr = "127.255.255.255", port = 9988):
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
        

        self.rcv_data = {}

    def destroy(self):
        self.running = False
        print('exit')
        self.rcv_thread.deamon = True 

    def __del__(self):
        self.running = False
        print('exit')
        self.rcv_thread.deamo = True 

        pass
        
    def rcvMessage(self):
        while(self.running):
            data_string = self.rcv_socket.recvfrom(2048)
            rcv_data = pickle.loads(data_string[0])
            id = rcv_data['id']
            self.rcv_data[id] = rcv_data

    def addMessage(self, message):
        self.rcv_data[message['id']] = message
            #print(self.rcv_data)
    #message must be an dictionary with id field
    def sendMessage(self, message):
        #print(message)
        #save message before sending
        self.rcv_data[message['id']] = message

        data_string = pickle.dumps(message, -1) 
        self.send_socket.sendto(data_string,(self.broadcast_addr, self.port))
        pass



# if __name__ == '__main__':
#     id = sys.argv[1]
#     network = Network()
#     data = {}
#     data['id'] = id 
#     data['position'] = (id, id)
#     while(True):

#         network.sendMessage(data)
#         sleep(0.5)
