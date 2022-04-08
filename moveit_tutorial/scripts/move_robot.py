#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import tf
from geometry_msgs.msg import Pose
from ar_track_alvar_msgs.msg import AlvarMarkers

home_position = [.4, .0, .3]
home_orientation = [.0, .0, .0, .1]

pose1_position = [-0.4, .0, .3]
pose1_oreintation = [.0, .0, .0, .1]

pose2_position = [.4, .1, .2]
pose2_oreintation = [.0, .0, .0, .1]


#set Pose message through lists
def set_pose(xyz = [0, 0, 0], q = [0, 0, 0, 1]):
	pose = Pose() 
	pose.position.x = xyz[0]
	pose.position.y = xyz[1]
	pose.position.z = xyz[2]
	pose.orientation.x = q[0]
	pose.orientation.y = q[1]
	pose.orientation.z = q[2]
	pose.orientation.w = q[3]
	return pose


#plan and execute to given pose; If plan is not confirmed plan again
def plan_and_execute(group, pose):
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


def main():	
    #initialize moveit	
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander("arm")
    group.set_max_velocity_scaling_factor(0.3)
    while not rospy.is_shutdown():
        plan_and_execute(group, set_pose(home_position, home_orientation))
        rospy.sleep(1)
        plan_and_execute(group, set_pose(pose1_position, pose1_oreintation))
        rospy.sleep(1)
        plan_and_execute(group, set_pose(pose2_position, pose2_oreintation))


if __name__ == '__main__':
	rospy.init_node('move_to_marker', anonymous=True)

	main()