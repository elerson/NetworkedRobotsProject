#!/usr/bin/python


config_exp = '''
robots:
  1:
    id: 1
    ip: 192.168.0.1
    macaddress: ec:08:6b:0d:68:ef
    cable_ip: 150.164.212.43

configs:
  map: DIR_/map/ambiente.png
  treefile: DIR_/steinerData1.dat
  resolution: 0.05
  exit: 0
  simulation: 1
  broadcast_address: 127.255.255.255
  algorithm_port: 39988
  configuration_port: 46544
'''


import sys
import os
import math

class Create():
  def create(self, dir_):
    with open("config_sim.yaml", 'w') as f:
      f.write(config_exp.replace('DIR_', dir_))


if __name__ == "__main__":
  dir_ = sys.argv[1]
  exp = Create()
  exp.create(dir_)
