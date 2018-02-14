#!/usr/bin/python

from Routing import Routing
from network import Network
import time

class Client:
  def __init__(self, essid, yaml_file):
    self.routing  = Routing(essid, yaml_file)
    self.network  = Network()
    self.my_id    = self.routing.getID()
    self.all_ids  = self.getAllIDs(yaml_file)
    #
    self.last_routing_id_ = -1
    #
  def getAllIDs(self, yaml_file):
    ids = []
    with open(yaml_file, 'r') as stream:
      self.config_data = yaml.load(stream)
    for robot in self.config_data:
      id = self.config_data[robot]['id']
      if(self.my_id != id)
        ids.append(id)
    return set(ids)
    #
  def routing(self):
    while self.running:
      for id in self.network.rcv_data:
        routing = self.network.rcv_data[id]['routing']
        if(routing[0] == self.my_id or routing[1] == self.my_id):
          if(self.last_routing_id_ == routing[0]  or self.last_routing_id_ == routing[1])
            break
          print("New Route")
        if(routing[0] == self.my_id):
          self.last_routing_id_ = routing[1]
          self.routing.createRoute([routing[1],-1],[list(self.all_ids - set([routing[1]])),[]] )
          break
        if(routing[1] == self.my_id):
          self.last_routing_id_ = routing[0]
          self.routing.createRoute([routing[0],-1],[list(self.all_ids - set([routing[0]])),[]] )
          break
    

if __name__ == '__main__':
  client = Client('teste4', '../config/data.yaml')
  while True:
    client.routing()
    time.sleep(1)



