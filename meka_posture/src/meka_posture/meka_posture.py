#!/usr/bin/env python

import yaml

import genpy
import copy
import rospy
import os.path
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
FollowJointTrajectoryGoal

# code for loading from yaml inspired by sr_grasp/utils.py, 
# available at Shadow Robot,  author: Mark Pitchless, licence GPL

# author: Guillaume WALCK (2015)

class MekaPosture(object):
    
    def __init__(self, name):        
        
        self._name = name
        self._postures = {}

    def add_posture(self, group_name, posture_name, trajectory, strategy="keep"):
        """
        Add the given posture to the list
        """
        if group_name not in self._postures:
            self._postures[group_name] = {}
        if posture_name not in self._postures[group_name]:
            self._postures[group_name][posture_name] = trajectory
            rospy.loginfo("Adding non existing posture named %s for group %s",
                              posture_name, group_name)
            return True
        else:
            if strategy == "overwrite":
                self._postures[group_name][posture_name] = trajectory
                rospy.loginfo("Overwriting posture named %s for group %s",
                              posture_name, group_name)
                return True
            else:
                rospy.logwarn("A posture named %s exists already for group %s,\
                              Not replacing. stragety=overwrite to replace",
                              posture_name, group_name)
                return False

    def get_posture(self, group_name, posture_name):
        """
        Retrieve the given posture from the list if it exists
        """
        if group_name in self._postures:
            if posture_name in self._postures[group_name]:
                return self._postures[group_name][posture_name]
        return None

    def get_trajectory_goal(self, group_name, posture_name):
        """
        Retrieve a ready to go trajectory goal out of the specific posture
        """
        goal = FollowJointTrajectoryGoal()
        posture = self.get_posture(group_name, posture_name) 
        if posture is not None:
            goal.trajectory = posture
            return goal
        else:
            rospy.logwarn("Posture %s in group %s does not exist", posture_name, group_name)
            return None


    def list_postures(self, group_name=None):
        """
        Retrieve the list of existing posture for the given group
        """
        if group_name is None:
            posture_dict = {}
            for group_name in self._postures:
                if group_name not in posture_dict:
                    posture_dict[group_name] = []
                for posture_name in self._postures[group_name]:
                    posture_dict[group_name].append(posture_name)
            return posture_dict
        else:
            if group_name in self._postures:
                return list(self._postures[group_name].keys())
            else:
                rospy.logwarn("Group %s does not exist in the postures list", group_name)
                return list()

    def load_postures(self, filepath, with_mirrored=0):
        """
        Load the postures from a file (erases current postures)
        """
        # erase current postures
        self._postures = {}
        self.append_postures(filepath, "keep", with_mirrored)
    
    def append_postures(self, filepath, strategy="keep", with_mirrored=0):
        """
        Append the postures from a file to the current list
        strategy keep means postures in memory are not erased
        strategy overwrite means postures in memory are replaced
        """
        yamldoc = None
        # load the yaml
        with open(filepath, 'r') as infile:
            yamldoc = yaml.load(infile)

        if yamldoc is not None:
            # extract trajectories from yaml
            for group_name in yamldoc:
                for posture_name in yamldoc[group_name]:
                    # convert it to a message
                    traj = JointTrajectory()
                    genpy.message.fill_message_args(traj, yamldoc[group_name][posture_name])
                    # store the message
                    self.add_posture(group_name, posture_name, traj, strategy)
                    if(with_mirrored):
                        #only do mirroring if posture has handedness
                        if "arm" not in posture_name: 
                            continue
                        if (group_name == 'left_arm'):
                            group_name_mirrored = 'right_arm'
                        elif (group_name == 'right_arm'):
                            group_name_mirrored = 'left_arm'
                        elif (group_name == 'left_hand'):
                            group_name_mirrored = 'right_hand'
                        elif (group_name == 'right_hand'):
                            group_name_mirrored = 'left_hand'

                        else:
                            group_name_mirrored = group_name


                        posture_name_mirrored = posture_name.replace('right_arm','left_arm',1)
                        if(posture_name_mirrored == posture_name): #if name did not change by replacing right -> left, try replacing left -> right
                            posture_name_mirrored = posture_name.replace('left_arm','right_arm',1)

                        traj_mirrored = self.mirror_posture(group_name,traj)
                        self.add_posture(group_name_mirrored,posture_name_mirrored, traj_mirrored, strategy)
    
    def save_postures(self, filepath, strategy="append"):
        """
        Save the postures to a file
        strategy append means keep all existing postures, add only new ones
        strategy replace means add new ones, replacing other if duplicate
        strategy overwrite, saves only new postures, dropping the one from the file
        """

        # check if file exists
        file_exists = os.path.isfile(filepath)
        posture_strategy = "append"
        if strategy == "append":
            posture_strategy = "overwrite"
            
        if strategy == "replace":
            posture_strategy = "keep"
        
        if strategy != "overwrite":
            if file_exists:
                # load this file
                self.append_postures(filepath, posture_strategy)

        outfile = open(filepath, 'w')

        yamldoc = {}
        for group_name in self._postures:
            yamldoc[group_name] = {}
            for posture_name in self._postures[group_name]:
                #print (genpy.message.strify_message(self._postures[group_name][posture_name]))
                yamldoc[group_name][posture_name] = yaml.load(genpy.message.strify_message(self._postures[group_name][posture_name]))

        output_str = yaml.dump(yamldoc, outfile, default_flow_style=False)
        outfile.close()

    def clear_postures(self):
        self._postures = {}

    def mirror_posture(self, group_name, trajectory):
        traj_mirrored = copy.deepcopy(trajectory)
        if group_name == 'right_arm' or group_name == 'left_arm' :
            for point in traj_mirrored.points:
                point.positions[1] = -point.positions[1]
                point.positions[2] = -point.positions[2]
                point.positions[4] = -point.positions[4]
                point.positions[6] = -point.positions[6]

        if group_name == 'head':
            for point in traj_mirrored.points:
                point.positions[1] = -point.positions[1]

        for i in xrange(0,len(traj_mirrored.joint_names)):
            if(traj_mirrored.joint_names[i].startswith("right")):
                traj_mirrored.joint_names[i] = traj_mirrored.joint_names[i].replace('right', 'left',1)
            else:
                traj_mirrored.joint_names[i] = traj_mirrored.joint_names[i].replace('left', 'right', 1)

        return traj_mirrored

  
def main():
    meka_posture = MekaPosture("whatever")
    trajectory = JointTrajectory()
    test_point1 = JointTrajectoryPoint()
    test_point1.positions = [0,1,2,3,4,5]
    trajectory.points.append(test_point1)
    posture_name='right_arm_test'
    meka_posture.mirror_posture('right_arm', trajectory)
    print(trajectory.points[0].positions[0] ,trajectory.points[0].positions[1] ,trajectory.points[0].positions[2] , posture_name)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["test1","test2"]
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration.from_sec(2.5)
    point.positions=[0.2, 0.5]
    goal.trajectory.points = []
    goal.trajectory.points.append(point)
    
    meka_posture.add_posture('right_arm','test',goal.trajectory)
    meka_posture.add_posture('right_arm','tutu',goal.trajectory)
    meka_posture.add_posture('left_arm','test',goal.trajectory)
    meka_posture.add_posture('left_arm','titi',goal.trajectory)
    
    meka_posture.save_postures("./mytest.yaml")
    
if __name__ == "__main__":
    main()
