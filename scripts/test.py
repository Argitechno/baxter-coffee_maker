#!/usr/bin/env python

import rospy
import moveit_python
import baxter_interface 
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

table_z = -0.1778

def add_keurig(p):
    """Add the keurig to the planning scene given it's location (center)

    Args:
        p PlanningSceneInterface: The planning scene to add the keurig to.+
    """
    #~~~ 9 X 14 Y 16 Z
    height = 0.4318
    keurig = SolidPrimitive(SolidPrimitive.BOX, (0.3302, 0.2286, height))
    keurig_pose = Pose(Point(1, 0, table_z + height/2.0),Quaternion(0, 0, 0, 1))  
    p.addSolidPrimitive("Keurig", keurig, keurig_pose, frame_id = "base")
    #point = get_keurig_position()

def add_cup(p):
    """Add the Cup to the planning scene given it's location (center)

    Args:
        p PlanningSceneInterface: The planning scene to add the Cup to.
    """
    #~1.75 in rad * 4.5 cylinder
    height = 0.1016
    cup = SolidPrimitive(SolidPrimitive.CYLINDER, (height, 0.04064))
    cup_pose = Pose(Point(1, -0.3, table_z + height/2.0),Quaternion(0, 0, 0, 1))  
    p.addSolidPrimitive("Cup", cup, cup_pose, frame_id = "base")
    #point = get_cup_position()


def add_kcup(p):
    """Add the K-Cup to the planning scene at its proper pose.

    Args:
        p PlanningSceneInterface: The planning scene to add the K-Cup to.
    """
    #~1.0 in rad * 2 in cylinder
   
    # Cylinder, height = 2 in, rad = 1 in -> to meters
    height = 0.0508
    kcup = SolidPrimitive(SolidPrimitive.CYLINDER, (height, 0.0254))
    kcup_pose = Pose(Point(1, 0.3, table_z + height/2),Quaternion(0, 0, 0, 1))  
    p.addSolidPrimitive("K-Cup", kcup, kcup_pose, frame_id = "base")

def main():
    """The idea here is to have baxter make some coffee."""
    # General planning scene for moving around the space, will include keurig bounding box so we move around it.
    print("Initializing Planning Scene...")
    general_p = moveit_python.PlanningSceneInterface("base",) # "general_p"
    general_p.clear()
    #Add each object to our general planning scene.
    print("Adding Keurig...")
    add_keurig(general_p)
    print("Adding Cup...")
    add_cup(general_p)
    print("Adding K-Cup...")
    add_kcup(general_p)
    print("Done!")

    #Milestone 1. Great Job if we get this far!


    # Move Group Interfaces for left arm, right arm, and both.
    # left_g = MoveGroupInterface("left_arm", "base")
    # right_g = MoveGroupInterface("left_arm", "base")
    # both_g = MoveGroupInterface("left_arm", "base")



    


if __name__ == '__main__':
    try:
        print("Initializing Node...")
        rospy.init_node("test_coffee_maker")
        main()

    except rospy.ROSInterruptException:
        pass