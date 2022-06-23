#!/usr/bin/python3
import rospy
import actionlib

#from multinherit.multinherit import multi_super  #pip3 install multinherit

from robotiq_2f_85_control.msg import CommandRobotiqGripperActionFeedback
from robotiq_2f_85_control.msg import CommandRobotiqGripperActionGoal
from robotiq_2f_85_control.msg import CommandRobotiqGripperActionResult
from robotiq_2f_85_control.msg import CommandRobotiqGripperAction
from robotiq_2f_85_control.msg import CommandRobotiqGripperGoal
from robotiq_2f_85_control.msg import CommandRobotiqGripperFeedback
from robotiq_2f_85_control.msg import CommandRobotiqGripperResult


def operate_gripper():

    action_name = 'robotiq_2f85_action_server'
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    rospy.loginfo("Client: Waiting for Server")
    robotiq_client.wait_for_server()
    rospy.loginfo("Client: Connected to Server" + action_name)
    goal = CommandRobotiqGripperGoal()
    goal.emergency_release = False
    goal.stop = False
    goal.position = 0.00
    goal.speed = 0.1
    goal.force = 5.0

    robotiq_client.send_goal(goal)
    robotiq_client.wait_for_result()
    result = robotiq_client.get_result()
    print(result)

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('robotiq_2f_client')
    result = operate_gripper()