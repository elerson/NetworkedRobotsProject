#!/usr/bin/python

robot_exp = '''
 <launch>
<group ns = "robot_ID">
  <node pkg="robots_deployment_online" type="deployment.py" name="ros_deployment_ID" output="screen">
    <param name="pose_send_time" value="0.3" />
    <param name="id" value="ID" />
    <param name="tree_file" value="/home/elerson/catkin_ws/src/networked_robots_experimetal_data/1euclideanexperiment/steinerData1.dat" />
    <param name="map_resolution" value="0.05" />
  </node> 


 <node pkg="fake_localization" type="fake_localization" name="amclrobot_ID" output="screen">
   <!-- Publish scans from best pose at a max of 10 Hz -->
   <param name="odom_model_type" value="omni"/>
   <param name="base_frame_id" value="robot_ID/base_link"/>
   <param name="odom_frame_id" value="robot_ID/odom"/>
   <param name="global_frame_id" value="/map"/>
  </node>


</group>

 </launch>
'''


#!/usr/bin/python
import sys
import os

class Create():
  def __init__(self):
    self.test = 1
  def create(self, numfiles):
    files = ["" for x in range(numfiles)]
    for x in range(1, numfiles):
      files[x] = "robot"+str(x-1)+".launch"

    with open("all.launch", 'w') as f:
      text = "<launch>\n"
      for x in range(1, numfiles):
        text += "<include file=\"$(find robots_deployment_online)/launch/scripts/"+files[x]+ "\" />\n"
      text += "</launch>\n"
      f.write(text)
      
      for x in range(1, numfiles):
        with open(files[x], 'w') as f:
          f.write(robot_exp.replace('ID', str(x-1)))


if __name__ == "__main__":
  exp = Create()
  exp.create(5)
