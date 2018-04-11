#!/usr/bin/python

robot_exp = '''


<launch>
  <group ns = "terminal_ID">
    <node pkg="deployment_anchor" type="terminal.py" name="ros_terminal_0" output="screen">
      <param name="radius" value="RADIUS" />
      <param name="xoffset" value="X" />
      <param name="yoffset" value="Y" />
      <param name="id" value="ID" />
      <param name="config_file" value="/home/elerson/NetworkedRobotsProject/configs/data.yaml" />
    </node> 

  </group>
</launch>
'''


#!/usr/bin/python
import sys
import os

class CreateTerminal():
  def __init__(self,radius, x, y):
    self.radius = radius
    self.x      = x
    self.y      = y
    self.test   = 1
  def create(self, numfiles):
    files = ["" for x in range(numfiles)]
    for x in range(1, numfiles):
      files[x] = "terminal"+str(x-1)+".launch"

    with open("all.launch", 'w') as f:
      text = "<launch>\n"
      for x in range(1, numfiles):
        text += "<include file=\"$(find deployment_anchor)/launch/scripts2/"+files[x]+ "\" />\n"
      text += "</launch>\n"
      f.write(text)
      
      for x in range(1, numfiles):
        with open(files[x], 'w') as f:
          f.write(robot_exp.replace('ID', str(x-1)).replace('RADIUS', str(self.radius)).replace('X', str(self.x)).replace('Y', str(self.y)))


if __name__ == "__main__":
  exp = CreateTerminal(75, 20, -1)
  exp.create(5)
