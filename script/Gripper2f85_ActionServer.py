#!/usr/bin/python3
import re
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

from smach_ros import SimpleActionState
from Gripper2f85Cmd import GripperCommand

GOAL_DETECTION_THRESHOLD = 0.01 # Max deviation from target goal to consider as goal "reached"

class RobotiqGripperActionServer(actionlib.SimpleActionServer, GripperCommand):

    def __init__(self, name, slave_id = 0, gripper_stroke=0.085, usbComPort='/dev/ttyUSB0',baudRate=115200, max_stroke=0.1, min_stroke=0.0):
        self._action_name = name
        self.__feedback = CommandRobotiqGripperFeedback()
        self.__result = CommandRobotiqGripperResult()
        self._processing_goal = True
        self._seq = 0
        
        #multi_super(GripperCommand, self, id=slave_id, stroke=gripper_stroke, comPort=usbComPort ,baud_rate=baudRate, max_gap=max_stroke, min_gap=min_stroke)
        #multi_super(actionlib.SimpleActionServer, self, name=self._action_name, ActionSpec= CommandRobotiqGripperAction, execute_cb=self.execute_callBack, auto_start=False)
        #super(RobotiqGripperActionServer, self).__init__(id=slave_id, stroke=gripper_stroke, comPort=usbComPort ,baud_rate=baudRate, max_gap=max_stroke, min_gap=min_stroke,
                        #name=self._action_name, ActionSpec= CommandRobotiqGripperAction, execute_cb=self.execute_callBack, auto_start=False)
        GripperCommand.__init__(self, id=slave_id, stroke=gripper_stroke, comPort=usbComPort ,baud_rate=baudRate, max_gap=max_stroke, min_gap=min_stroke)
        actionlib.SimpleActionServer.__init__(self, self._action_name, CommandRobotiqGripperAction, execute_cb=self.execute_callBack, auto_start=False)

        whatchdog_connection = rospy.Timer(rospy.Duration(15.0), self.__connection_timeout, oneshot=True)
        while not rospy.is_shutdown() and not self.initialize():
            rospy.sleep(5)
            rospy.logwarn_throttle(5, self._action_name + ": Waiting for gripper to be ready...")
        
        whatchdog_connection.shutdown()
        if self.is_ready():
            self.start()
            rospy.loginfo("Action server is Active")
        else:
            rospy.loginfo("Gripper Is Connected but Can't Activate ")

    def __connection_timeout(self, event):
        rospy.logfatal("Gripper on port %s seems not to respond" % (self.com_port))
        rospy.signal_shutdown("Gripper on port %s seems not to respond" % (self.com_port))
        self._processing_goal = False

    def __movement_timeout(self, event):
        rospy.logerr("%s: Achieving goal is taking too long, dropping current goal")
    
    def __getStatusFeedback(self):
        status = CommandRobotiqGripperFeedback()
        status.header.stamp         = rospy.get_rostime()
        status.header.seq           = 0
        status.is_ready             = super().is_ready()
        status.is_reset             = super().is_reset()
        status.is_moving            = super().is_moving()
        status.obj_detected         = super().object_detected()
        status.fault_status         = super().get_fault_status()
        status.position             = super().get_pos()
        status.requested_position   = super().get_req_pos()
        status.current              = super().get_current()
        print(status)
        return status
    
    def __PosError(self):
        return abs(self.__feedback.requested_position - self.__feedback.position)

    def __abortingActionServer(self, abort_error):
        rospy.logerr("%s: Dropping current goal -> " + abort_error )
        self.set_aborted(self.__feedback , (self._action_name))

    def execute_callBack(self, goal):
        if not self.is_gripper_connected:
            self.__abortingActionServer("Connection Lost")
            
        
        self.__feedback = self.__getStatusFeedback()
        rospy.loginfo( (": New goal received Pos:%.3f Speed: %.3f Force: %.3f Force-Stop: %r") % (goal.position, goal.speed, goal.force, goal.stop) )

        success = False
        rate = rospy.Rate(1)

        
        if not goal.stop:
            is_modbus_msg_sent = self.goTo(goal.position, goal.speed, goal.force)
            self._processing_goal = True  
        else:
            rospy.logwarn_throttle(5, self._action_name + ": stop command is active")

        if not is_modbus_msg_sent:
            self.__abortingActionServer("Unable to Send Modbus MSG")
            
        watchdog_move = rospy.Timer(rospy.Duration(5.0), self.__movement_timeout, oneshot=True)

        while not rospy.is_shutdown() and self._processing_goal: 
            
            if not self.is_gripper_connected:
                self.__abortingActionServer("Connection Lost")
            
            self.__feedback = self.__getStatusFeedback()
            rospy.logdebug("Error = %.5f Requested position = %.3f Current position = %.3f" % (abs(self.__feedback.requested_position - self.__feedback.position), self.__feedback.requested_position, self.__feedback.position))
            
            if self.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.set_preempted()
                break

            if self.__feedback.fault_status != 0:
                self.__abortingActionServer("Fault status (gFLT) is: %d" % self.__feedback.fault_status)
                self._processing_goal = False
                break
            if( self.__PosError() < GOAL_DETECTION_THRESHOLD or self.__feedback.obj_detected):
                self._processing_goal = False
                success = True
                print(success)
                break
            self.publish_feedback(self.__feedback)
        
            rate.sleep()
        

        self.__result = self.__feedback
        watchdog_move.shutdown()
        if success:
            rospy.logdebug(self._action_name + ": Goal reached or object detected Pos: %.3f PosRequested: %.3f ObjectDetected: %r" % (goal.position, self.__feedback.requested_position, self.__feedback.obj_detected) )
            self.set_succeeded(self.__result)
            
        

if __name__ == "__main__":

    rospy.init_node('robotiq_2f85_action_server')
    server = RobotiqGripperActionServer(rospy.get_name())


    rospy.spin()
    