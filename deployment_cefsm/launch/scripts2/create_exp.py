#!/usr/bin/python

robot_exp = '''


<launch>
<group ns = "robot_ID">
  <node pkg="deployment_cefsm" type="deployment.py" name="ros_deployment_ID" output="screen">
    <param name="pose_send_time" value="0.3" />
    <param name="id" value="ID" />
    <param name="tree_file" value="/home/elerson/catkin_ws/src/networked_robots_experimetal_data/1euclideanexperiment/steinerData1.dat" />
    <param name="lower_threshold" value="-111" />
    <param name="higher_threshold" value="-91" />
  </node> 


  <node pkg="fake_localization" type="fake_localization" name="amclrobot_ID" output="screen">
   <!-- Publish scans from best pose at a max of 10 Hz -->
   <param name="odom_model_type" value="omni"/>
   <param name="base_frame_id" value="robot_ID/base_link"/>
   <param name="odom_frame_id" value="robot_ID/odom"/>
   <param name="global_frame_id" value="/map"/>
  </node>


  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
   <param name="base_global_planner" value="network_planner/NetworkDeploymentPlanner"/>

   <rosparam file="$(find deployment_cefsm)/launch/costmap_common_params.yaml" command="load" ns="global_costmap" />
   <rosparam file="$(find deployment_cefsm)/launch/costmap_common_params.yaml" command="load" ns="local_costmap" />
   <rosparam file="$(find deployment_cefsm)/launch/local_costmap_params.yaml" command="load" />
   <rosparam file="$(find deployment_cefsm)/launch/global_costmap_params.yaml" command="load" />
   <rosparam file="$(find deployment_cefsm)/launch/base_local_planner_params.yaml" command="load" />
   
   <param name="planner_file_dir" value="/home/elerson/catkin_ws/src/networked_robots_experimetal_data/1euclideanexperiment/"/>
   <param name="planner_file_1" value="steinerData1.dat"/>
   <param name="planner_file_2" value="steinerData2.dat"/>

   <param name="global_costmap/footprint" value="robot_ID/base_footprint"/>
   <param name="local_costmap/global_frame" value="robot_ID/odom"/>
   <param name="global_costmap/robot_base_frame" value="robot_ID/base_footprint"/>

   <param name="local_costmap/robot_base_frame" value="robot_ID/base_footprint"/>
   <param name="global_costmap/laser_scan_sensor/sensor_frame" value="robot_ID/base_laser_link"/>
   <param name="local_costmap/obstacle_layer/laser_scan_sensor/sensor_frame" value="robot_ID/base_laser_link"/>


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
  def create(self, numfiles):
    files = ["" for x in range(numfiles)]
    for x in range(1, numfiles):
      files[x] = "robot"+str(x-1)+".launch"

    with open("all.launch", 'w') as f:
      text = "<launch>\n"
      for x in range(1, numfiles):
        text += "<include file=\"$(find deployment_cefsm)/launch/scripts2/"+files[x]+ "\" />\n"
      text += "</launch>\n"
      f.write(text)
      
      for x in range(1, numfiles):
        with open(files[x], 'w') as f:
          f.write(robot_exp.replace('ID', str(x-1)))


if __name__ == "__main__":
  exp = Create()
  exp.create(5)
