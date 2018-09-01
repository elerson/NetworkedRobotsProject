#!/usr/bin/python

import threading
import subprocess as sub
import yaml
import time
import os
from struct import *
import socket

UDP_IP = "0.0.0.0"
PORT   =  4320




class RSSMeasure:
  def __init__(self, essid, yaml_file, max_value = -1000):
    #
    with open(yaml_file, 'r') as stream:
      self.config_data  = yaml.load(stream)['robots']
    #
    self.signal_dict    = dict()
    self.macaddress_map = dict()
    self.callback = None
    self.id_map = dict()
    #create a map from mac to id
    for robot in self.config_data:
      macaddress = self.config_data[robot]['macaddress']
      id_        = self.config_data[robot]['id'] - 1
      self.macaddress_map[id_] = macaddress
      self.id_map[macaddress] = id_
    #  
    #
    self.MAX            = max_value
    self.alpha          = 0.7
    self.essid          = essid
    
    self.sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
    self.sock.bind((UDP_IP, PORT))
    self.thread         = threading.Thread(target=self.readrss)
    self.thread.start()    
    
    #
  def addCallback(self, callback):
    self.callback = callback
  def __del__(self):
    self.thread.stop()
    #
  def readrss(self):
    while True:
    
      message, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
      rss, mac_str = unpack('i6s',message)
      mac = ''.join( [ "%02x:" % ord( x ) for x in mac_str ] ).strip()[:-1]
      self.addRSS(mac, rss)
      #print(mac, rss)
      if self.callback != None:
        try:
          id = self.id_map[mac]
          self.callback(id, rss)
        except:
          pass


    #
  def addRSS(self, mac_addr, signal):
    if(mac_addr in self.signal_dict):
      self.signal_dict[mac_addr] = (self.signal_dict[mac_addr]*(1.0 - self.alpha)) + self.alpha*float(signal)
    else:
      self.signal_dict[mac_addr] = float(signal)      
    #
  def getMeasurement(self, id):
    #
    try:
      return self.signal_dict[self.macaddress_map[id]]
    except:
      return self.MAX



if __name__ == "__main__":

  home = os.path.expanduser("~")
  rssmeasure = RSSMeasure('teste4',home + '/NetworkedRobotsProject/configs/data_real.yaml')
  
  while True:
    for id in range(8):
      print(id, rssmeasure.getMeasurement(id))
    time.sleep(0.1)

#m = RSSMeasure('teste4','data.yaml')

#if __name__ == "__main__":


  
