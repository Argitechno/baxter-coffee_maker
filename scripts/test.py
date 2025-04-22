#!/usr/bin/env python

import rospy
import moveit_python
import baxter_interface
from geometry_msgs.msg import *

def get_keurig_position():
    """Current: Return a set position.
    Future: Use Machine Vision to find the position of the Keurig

    Returns:
        geometry_msgs/Point: The position of the Keurig.
    """
    point = Point()
    point.x = 0
    point.y = 0
    point.z = 0
    return point
    
def get_cup_position():
    """"Current: Return a set position.
    Future: Use Machine Vision to find the position of the cup.

    Returns:
        geometry_msgs/Point: The position of the cup.
    """
    point = Point()
    point.x = 0
    point.y = 0
    point.z = 0
    return point

def get_kcup_position():
    """Current: Return a set position.
    Future: Use Machine Vision to find the position of the K-Cup.

    Returns:
        geometry_msgs/Point: The position of the K-Cup.
    """
    point = Point()
    point.x = 0
    point.y = 0
    point.z = 0
    return point

def add_keurig(p):
    """Add the keurig to the planning scene given it's location.

    Args:
        p PlanningSceneInterface: The planning scene to add the keurig to.+
    """
    point = get_keurig_position()

def add_cup(p):
    """Add the Cup to the planning scene given it's location.

    Args:
        p PlanningSceneInterface: The planning scene to add the Cup to.
    """
    point = get_cup_position()


def add_kcup(p):
    """Add the K-Cup to the planning scene given it's location.

    Args:
        p PlanningSceneInterface: The planning scene to add the K-Cup to.
    """
    kcup_position = get_kcup_position()

def main():
    """The idea here is to have baxter make some coffee."""

    # General planning scene for moving around the space, will include keurig bounding box so we move around it.
    general_p = PlanningSceneInterface("base", "general_p");

    #Add each object to our general planning scene.
    add_keurig(general_p)
    add_cup(general_p)
    add_kcup(general_p)

    #Milestone 1. Great Job if we get this far!


    # Move Group Interfaces for left arm, right arm, and both.
    # left_g = MoveGroupInterface("left_arm", "base")
    # right_g = MoveGroupInterface("left_arm", "base")
    # both_g = MoveGroupInterface("left_arm", "base")



    


if __name__ == '__main__':
    try:
        print("Initializing Node...")
        rospy.init_node("coffee_maker")
        main()

    except rospy.ROSInterruptException:
        pass