sudo rm biped.urdf biped.sdf
rosrun xacro xacro biped.xacro > biped.urdf
gz sdf -p biped.urdf > biped.sdf
