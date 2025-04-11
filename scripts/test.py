#!/usr/bin/env python

import rospy
import moveit_python
import baxter_interface

def main():
    """Should move the arms to untuck position."""
    #Define initial parameters
    p = moveit_python.PlanningSceneInterface("base")
    g = moveit_python.MoveGroupInterface("both_arms", "base")
    gr = moveit_python.MoveGroupInterface("right_arm", "base")
    gl = moveit_python.MoveGroupInterface("left_arm", "base")
    right_gripper = baxter_interface.Gripper('right')
    right_gripper.calibrate()
    right_gripper.open()
    joints = ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
    joints_right = ['right_' + j for j in joints]
    joints_left = ['left_' + j for j in joints]
    joints_both = joints_left + joints_right
    pos1r = [-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.5]
    pos1l = [ 0.08,  1.0, -1.19, 1.94, -0.67, 1.03,  0.5]
    pos1 = pos1l + pos1r

    #Clear
    p.clear()

    g.moveToJointPosition(joints_both, pos1, plan_only=False)


if __name__ == '__main__':
    try:
        print("Initializing Node...")
        rospy.init_node("test.py")
        main()

    except rospy.ROSInterruptException:
        pass