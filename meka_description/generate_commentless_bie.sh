#!/bin/bash

echo "generating bie.xml and bie_floka.xml"
cd $1/robots
rosrun xacro xacro bie_sim.urdf.xacro > bie.urdf
xmlstarlet ed -d '//comment()' bie.urdf > bie.xml

rosrun xacro xacro bie_floka_sim.urdf.xacro > bie_floka.urdf
xmlstarlet ed -d '//comment()' bie_floka.urdf > bie_floka.xml

rm bie.urdf
rm bie_floka.urdf
