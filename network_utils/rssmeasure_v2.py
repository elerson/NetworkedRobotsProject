#!/usr/bin/python

import threading
import subprocess as sub
import yaml
import time
import os
import sysv_ipc
from struct import *

class RSSMeasure:
  def __init__(self, essid, yaml_file, max_value = -1000):
    #
    with open(yaml_file, 'r') as stream:
      self.config_data  = yaml.load(stream)['robots']
    #
    self.macaddress_map = dict()
    self.id_map = dict()
    #create a map from mac to id
    for robot in self.config_data:
      macaddress = self.config_data[robot]['macaddress']
      id_        = self.config_data[robot]['id']
      self.macaddress_map[id_] = macaddress
      self.id_map[macaddress] = id_
    #  
    #
    self.MAX            = max_value
    self.alpha          = 0.7
    self.essid          = essid
    self.mq             = sysv_ipc.MessageQueue(1241, sysv_ipc.IPC_CREAT)
    self.thread         = threading.Thread(target=self.readrss)
    self.thread.start()
    self.signal_dict    = dict()
    self.callback = None
    #
  def addCallback(self, callback):
    self.callback = callback
  def __del__(self):
    self.thread.stop()
    #
  def readrss(self):
    while True:
      message = self.mq.receive()
      rss, mac_str = unpack('i6s',message[0])
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
  rssmeasure = RSSMeasure('teste4',home + '/NetworkedRobotsProject/configs/data.yaml')
  
  while True:
    for id in range(8):
      print(id, rssmeasure.getMeasurement(id))
    time.sleep(0.1)

#m = RSSMeasure('teste4','data.yaml')

#if __name__ == "__main__":


  
