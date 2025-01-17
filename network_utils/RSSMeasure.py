#!/usr/bin/python

import threading
import subprocess as sub
import yaml
import time
import os

class RSSMeasure:
  def __init__(self, essid, yaml_file, max_value = -1000):
    #
    with open(yaml_file, 'r') as stream:
      self.config_data  = yaml.load(stream)['robots']
    #
    self.macaddress_map = dict()
    #create a map from mac to id
    for robot in self.config_data:
      macaddress = self.config_data[robot]['macaddress']
      id_        = self.config_data[robot]['id']
      self.macaddress_map[id_] = macaddress
    #  
    #
    self.MAX            = max_value
    self.alpha          = 0.7
    self.essid          = essid
    self.thread         = threading.Thread(target=self.tcpdump)
    self.thread.start()
    self.signal_dict    = dict()
    #
  def __del__(self):
    self.thread.stop()
    #
  def tcpdump(self):
    p = sub.Popen(('sudo', 'tcpdump', '-tt', '-U', '-e', '-i', 'moni0'), stdout=sub.PIPE)
    for row in iter(p.stdout.readline, b''):
      str_data = row.rstrip()
      if(self.essid in str_data and 'Broadcast ' in str_data and ' signal' in str_data):
        #time     = str_data.split(' ')[0]
        signal   = str_data.split(' signal')[0].split(' ')[-1] #str_data.split(' ')[6]

        mac_addr = str_data.split('Broadcast ')[1].split(' ')[0] #str_data.split(' ')[14]
        #
        if('dBm' in signal and 'SA:' in mac_addr):
          self.addRSS(mac_addr[3:], signal[:-3])
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


  
