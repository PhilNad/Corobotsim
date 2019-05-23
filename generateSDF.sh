#!/usr/bin/env bash

#Files needs to be regenerated every time we make a change to the URDF
rm -f CoroModel.urdf
rm -f CoroModel.sdf

#XACRO to URDF
rosrun xacro xacro CoroModel.urdf.xacro > CoroModel.urdf

#URDF to SDF
gz sdf -p CoroModel.urdf > CoroModel.sdf

#SDF does not support URDF's package:// but only support model:// and file:// as far as I know.
#Models directory is /usr/share/gazebo-9/models/
sed -i -e 's/<uri>model:\/\/ur_description/<uri>file:\/\/..\/..\/..\/..\/home\/pnadeau\/catkin_ws\/src\/universal_robot\/ur_description/g' CoroModel.sdf
sed -i -e 's/<uri>model:\/\/robotiq_2f_model/<uri>file:\/\/..\/..\/..\/..\/home\/pnadeau\/catkin_ws\/src\/beta-robotiq\/robotiq_2f_model/g' CoroModel.sdf

#Effort's limit are a bit strong, lets decrease them.
sed -i -e 's/effort="176/effort="52/g' CoroModel.urdf
sed -i -e 's/effort="60/effort="52/g' CoroModel.urdf
sed -i -e 's/effort>176/effort>52/g' CoroModel.sdf
sed -i -e 's/effort>60/effort>52/g' CoroModel.sdf

#Velocity limits are reduced also
#This doesnt seems to have any impact when using an EffortInterface
#maybe it works only when Gazebo calls setVelocity()
sed -i -e 's/velocity>1.91986/velocity>0.51986/g' CoroModel.sdf
sed -i -e 's/velocity="1.91986177778/velocity="0.51986/g' CoroModel.urdf

#We need to change position_controllers into effort_controllers to allow grasping with Gazebo
#See coro.launch for further details about that issue.
sed -i -e 's/PositionJointInterface/EffortJointInterface/g' CoroModel.urdf

#We need to insert some joints at the end to close the kinematic loop
#If we sucessfully create a closed linkage, we dont need the URDF's "mimic" tags
#UPDATE 18/02/19: Now most mimic statements are commented out in the URDF, except one
#to emulate the wormgear effect
#UPDATE 16/05/19: Mimic tags does not generate forces and is not useful for grasping.
sed -i '/mimic/d' CoroModel.urdf

#Get the line where we want to put the extra joints
lineNb=$(grep -nr static CoroModel.sdf | cut -d ':' -f 1)
#Retrieve every line before that
head -n $(($lineNb -1)) CoroModel.sdf > temporary_file
#Apprend extra joints
#Also added viscuous damping and springs.
#You want the joint to be relative to gripper_right/left_follower
#If that link is the parent, use <use_parent_model_frame>1</use_parent_model_frame> within the axis definition
echo "    <joint name='gripper_left_spring_link_JOINT_1' type='revolute'>" >> temporary_file
echo "      <parent>gripper_left_spring_link</parent>" >> temporary_file
echo "      <child>gripper_left_follower</child>" >> temporary_file
echo "      <pose frame=''>0 -0.018 0.0065 0 -0 0</pose>" >> temporary_file
echo "      <axis>" >> temporary_file
echo "        <xyz>1 0 0</xyz>" >> temporary_file
echo "        <use_parent_model_frame>0</use_parent_model_frame>" >> temporary_file
echo "        <limit>" >> temporary_file
echo "          <lower>-1.79769e+308</lower>" >> temporary_file
echo "          <upper>1.79769e+308</upper>" >> temporary_file
echo "          <effort>-1</effort>" >> temporary_file
echo "          <velocity>0</velocity>" >> temporary_file
echo "        </limit>" >> temporary_file
echo "        <dynamics>" >> temporary_file
echo "          <spring_reference>0</spring_reference>" >> temporary_file
echo "          <spring_stiffness>0</spring_stiffness>" >> temporary_file
echo "          <damping>0.1</damping>" >> temporary_file
echo "          <friction>2</friction>" >> temporary_file
echo "        </dynamics>" >> temporary_file
echo "      </axis>" >> temporary_file
echo "      <physics>" >> temporary_file
echo "        <ode>" >> temporary_file
echo "          <limit>" >> temporary_file
echo "            <cfm>0</cfm>" >> temporary_file
echo "            <erp>0.2</erp>" >> temporary_file
echo "          </limit>" >> temporary_file
echo "          <suspension>" >> temporary_file
echo "            <cfm>0</cfm>" >> temporary_file
echo "            <erp>0.2</erp>" >> temporary_file
echo "          </suspension>" >> temporary_file
echo "        </ode>" >> temporary_file
echo "      </physics>" >> temporary_file
echo "    </joint>" >> temporary_file
echo "    <joint name='gripper_right_spring_link_JOINT_0' type='revolute'>" >> temporary_file
echo "      <parent>gripper_right_spring_link</parent>" >> temporary_file
echo "      <child>gripper_right_follower</child>" >> temporary_file
echo "      <pose frame=''>0 -0.018 0.0065 0 -0 0</pose>" >> temporary_file
echo "      <axis>" >> temporary_file
echo "        <xyz>1 0 0</xyz>" >> temporary_file
echo "        <use_parent_model_frame>0</use_parent_model_frame>" >> temporary_file
echo "        <limit>" >> temporary_file
echo "          <lower>-1.79769e+308</lower>" >> temporary_file
echo "          <upper>1.79769e+308</upper>" >> temporary_file
echo "          <effort>-1</effort>" >> temporary_file
echo "          <velocity>0</velocity>" >> temporary_file
echo "        </limit>" >> temporary_file
echo "        <dynamics>" >> temporary_file
echo "          <spring_reference>0</spring_reference>" >> temporary_file
echo "          <spring_stiffness>0</spring_stiffness>" >> temporary_file
echo "          <damping>0.1</damping>" >> temporary_file
echo "          <friction>2</friction>" >> temporary_file
echo "        </dynamics>" >> temporary_file
echo "      </axis>" >> temporary_file
echo "      <physics>" >> temporary_file
echo "        <ode>" >> temporary_file
echo "          <limit>" >> temporary_file
echo "            <cfm>0</cfm>" >> temporary_file
echo "            <erp>0.2</erp>" >> temporary_file
echo "          </limit>" >> temporary_file
echo "          <suspension>" >> temporary_file
echo "            <cfm>0</cfm>" >> temporary_file
echo "            <erp>0.2</erp>" >> temporary_file
echo "          </suspension>" >> temporary_file
echo "        </ode>" >> temporary_file
echo "      </physics>" >> temporary_file
echo "    </joint>" >> temporary_file
echo "    <static>0</static>" >> temporary_file
echo "    <plugin name='gazebo_ros_control' filename='libgazebo_ros_control.so'/>" >> temporary_file
echo "  </model>" >> temporary_file
echo "</sdf>" >> temporary_file

#Replace the resulting file
mv temporary_file CoroModel.sdf

