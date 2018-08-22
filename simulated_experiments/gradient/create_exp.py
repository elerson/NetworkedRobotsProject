#!/usr/bin/python

robot_exp = '''
<launch>
<group ns = "robot_ID">
  <node pkg="robots_deployment_online" type="deployment.py" name="ros_deployment_ID" output="screen">
    <param name="pose_send_time" value="0.3" />
    <param name="id" value="ID" />
    <param name="config_file" value="$(env EXP_DIR)/../config_sim.yaml" />
    <param name="map_resolution" value="0.05" />
    <param name="ray" value="RADIUS"/>
  </node> 


 <node pkg="fake_localization" type="fake_localization" name="amclrobot_ID" output="screen">
   <!-- Publish scans from best pose at a max of 10 Hz -->
   <param name="odom_model_type" value="omni"/>
   <param name="base_frame_id" value="robot_ID/base_link"/>
   <param name="odom_frame_id" value="robot_ID/odom"/>
   <param name="global_frame_id" value="/map"/>
  </node>


  <node pkg="move_base" type="move_base" respawn="false" name="move_base_ID" output="screen">
   <!--<param name="base_global_planner" value="network_planner/NetworkDeploymentPlanner"/> -->

   <rosparam file="$(find robots_deployment_online)/launch/costmap_common_params.yaml" command="load" ns="global_costmap" />
   <rosparam file="$(find robots_deployment_online)/launch/costmap_common_params.yaml" command="load" ns="local_costmap" />
   <rosparam file="$(find robots_deployment_online)/launch/local_costmap_params.yaml" command="load" />
   <rosparam file="$(find robots_deployment_online)/launch/global_costmap_params.yaml" command="load" />
   <rosparam file="$(find robots_deployment_online)/launch/base_local_planner_params.yaml" command="load" />
   
   <param name="planner_file_dir" value="$(find network_deployment_planner)/experiment/exp1/"/>
   <param name="global_costmap/robot_base_frame" value="robot_ID/base_footprint"/>
   <param name="local_costmap/global_frame" value="robot_ID/odom"/>
   <param name="local_costmap/robot_base_frame" value="robot_ID/base_footprint"/>
   <remap from="/robot_ID/map" to="/map" />
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


  def create(self, numfiles, radius):
    files = ["" for x in range(numfiles)]
    for x in range(1, numfiles):
      files[x] = "robot"+str(x-1)+".launch"

    with open("all.launch", 'w') as f:
      text = "<launch>\n"
      for x in range(1, numfiles):
        text += "<include file=\"$(env EXP_DIR)/../gradient/"+files[x]+ "\" />\n"
      text += "</launch>\n"
      f.write(text)
      
      for x in range(1, numfiles):
        with open(files[x], 'w') as f:
          f.write(robot_exp.replace('ID', str(x-1)).replace('RADIUS', str(radius)))


if __name__ == "__main__":
  num_robots = int(sys.argv[1])
  radius     = int(sys.argv[2])
  exp = Create()
  exp.create(num_robots, radius)
