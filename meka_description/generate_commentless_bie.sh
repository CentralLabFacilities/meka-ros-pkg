#!/bin/bash

roscd meka_description
cd robots
rosrun xacro xacro bie_sim.urdf.xacro > bie.urdf   
xmlstarlet ed -d '//comment()' bie.urdf > bie.xml