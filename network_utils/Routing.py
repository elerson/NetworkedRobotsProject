#!/usr/bin/python

import threading
import subprocess as sub
import yaml
from pyroute2 import IPRoute
import netifaces

import hashlib
import json
import time
import os


class Routing:
  def __init__(self, essid, yaml_file, interface):
    #
    with open(yaml_file, 'r') as stream:
      self.config_data  = yaml.load(stream)
      self.config_data  = self.config_data['robots'] 
    #
    self.ifname       = interface
    self.my_macaddr   = self.getMyMacAddr()
    self.my_ip        = self.getMyIp(self.my_macaddr)
    self.my_id        = self.getMyID(self.my_macaddr)
    #
    self.interface    = IPRoute()
    self.network_cell = "76:A9:80:06:38:32"
    self.network_name = essid
    self.channel = '5'
    self.ip_list = dict()
    self.last_routing_hash = ''
    #
    #self.setupInterface(self.ifname, self.my_ip, self.network_name, self.network_cell, self.channel)
    #self.setupInterfaceMonitor(self.ifname)
    #
  def getMyMacAddr(self):
    return netifaces.ifaddresses(self.ifname)[netifaces.AF_LINK][0]['addr']
    #
  def getMyIp(self, macaddress):
    ip = ''
    for robot in self.config_data:
      if(self.config_data[robot]['macaddress'] == macaddress):
        ip = self.config_data[robot]['ip'] 
        break
    return ip
    #
  def getID(self):
    return self.my_id
    #
  def getMyID(self, macaddress):
    id_ = -1
    for robot in self.config_data:
      if(self.config_data[robot]['macaddress'] == macaddress):
        id_ = self.config_data[robot]['id'] 
        break
    return id_
    #
  def setupInterfaceMonitor(self, ifname): 
    sub.Popen(('sudo', 'iw', ifname,'interface', 'add','moni0','type', 'monitor'))
    sub.Popen(('sudo', 'ifconfig', 'moni0', 'up'))
    #
  def setupInterface(self, ifname, ip, network_name, network_cell, channel='5'):
    #
    x = self.interface.link_lookup(ifname=ifname)[0]

    # put link down
    sub.Popen(('sudo', 'ifconfig', ifname, 'down'))
    time.sleep(0.2)
    sub.Popen(('sudo', 'iwconfig', ifname, 'mode', 'ad-hoc'))
    time.sleep(0.2)
    #sub.Popen(('sudo', 'ifconfig', ifname, 'mode', 'monitor'))
    #time.sleep(0.2)

    #self.interface.link("set", index=x, address=network_cell, name=network_name)
    self.interface.link("set", index=x, mtu=1000, txqlen=2000)
    time.sleep(0.2)

    sub.Popen(('sudo', 'iwconfig', ifname, 'ap', network_cell))
    time.sleep(0.2)
    sub.Popen(('sudo', 'iwconfig', ifname, 'channel', channel))
    time.sleep(0.2)
    sub.Popen(('sudo', 'ifconfig', ifname, ip+'/24', 'up'))
    time.sleep(0.2)
    #sub.Popen(('sudo', 'ifconfig', ifname, ip, 'up'))
    sub.Popen(('sudo', 'iwconfig', ifname, 'essid', network_name))
    time.sleep(0.2)
    print('all configured')
   
   
    #
  def flushRouting(self):
    x = self.interface.link_lookup(ifname=self.ifname)[0]
    for robot in self.config_data:
      if(self.config_data[robot]['macaddress'] != self.my_macaddr):
        ip = self.config_data[robot]['ip'] 
        try:
          self.interface.route("del", index=x, dst=ip)
        except:
          pass
    for ip in self.ip_list:
        try:
          self.interface.route("del", index=x, dst=ip)
        except:
          pass
    #
  def addRoute(self, dest, next_hop):
    #
    x = self.interface.link_lookup(ifname=self.ifname)[0]
    self.interface.route("add", index=x, dst=dest+'/32', gateway=next_hop)
    self.ip_list[dest] = 1
  #
  #def createGraph(self, network):
  #  for robot_id in network.
  #
  # def createRoute( neighbors, routing ): # routing = [routing_through_0, routing_through_1]
  #   self.flushRouting()
  #   neighbor_0_ip = ''
  #   neighbor_1_ip = ''
  #   for robot in self.config_data:
  #     #route to the first neighbor and sencond neighbors
  #     if(self.config_data[robot]['id'] == neighbors[0]):
  #       neighbor_0_ip = self.config_data[robot]['ip']
  #     #
  #     if(self.config_data[robot]['id'] == neighbors[1]):
  #       neighbor_1_ip = self.config_data[robot]['ip']
  #   #
  #   for robot in self.config_data:
  #     for throgh_0 in routing[0]:
  #       if(self.config_data[robot]['id'] == throgh_0):
  #         self.addRoute(self.config_data[robot]['ip'], neighbor_0_ip)
  #     #
  #     for throgh_1 in routing[1]:
  #       if(self.config_data[robot]['id'] == throgh_0):
  #         self.addRoute(self.config_data[robot]['ip'], neighbor_1_ip)

  def createRoute(self, graph): # routing = [routing_through_0, routing_through_1]

    #hash = hashlib.sha1(json.dumps(graph, sort_keys=True)).hexdigest()
    #print(hash)
    #if(hash == self.last_routing_hash):
    #  return

    self.last_routing_hash = hash
    self.flushRouting()

    route = {}
    self.createRouteAux(graph, self.my_id, set([]), -1, graph[self.my_id], route)
    
    print(route)
    for next_hop in route:
      if(next_hop != self.my_id and not next_hop in graph[self.my_id]):
        print(self.config_data[next_hop]['ip'], self.config_data[route[next_hop]]['ip'])
        self.addRoute(self.config_data[next_hop]['ip'], self.config_data[route[next_hop]]['ip'])

      

  def createRouteAux(self, graph, id, visited, next_hop, next_hop_list, route):
    for node in graph[id]:
      if(node in next_hop_list):
        next_hop = node
      #
      if(not node in visited):
        self.createRouteAux(graph, node, visited.union([id]), next_hop, next_hop_list, route)
    #
    route[id] = next_hop


#m = Routing('teste4', 'data.yaml')

if __name__ == "__main__":

  home = os.path.expanduser("~")
  routing = Routing('teste4',home + '/NetworkedRobotsProject/configs/data.yaml')
  graph = {1: set([2]), 2: set([1, 3]), 3: set([2, 4]), 4: set([3, 5]), 5: set([4])}
  
  routing.createRoute(graph)
