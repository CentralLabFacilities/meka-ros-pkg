#!/bin/bash

echo "generating bie.xml"
pwd
cd ../robots
rosrun xacro xacro bie_sim.urdf.xacro > bie.urdf
xmlstarlet ed -d '//comment()' bie.urdf > bie.xml