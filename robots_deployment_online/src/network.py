#!/usr/bin/python
from socket import *
import threading
import sys
from time import sleep
import cPickle as pickle



class Network:
    running = True
    def __init__(self, broadcast_addr = "127.255.255.255", port = 9988):

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

    def __del__(self):
        self.running = False
        self.rcv_thread.join()

        pass
        
    def rcvMessage(self):
        while(self.running):
            data_string = self.rcv_socket.recvfrom(2048)
            rcv_data = pickle.loads(data_string[0])
            id = rcv_data['id']
            self.rcv_data[id] = rcv_data
        
            #print(self.rcv_data)
    #message must be an dictionary with id field
    def sendMessage(self, message):
        #print(message)
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
