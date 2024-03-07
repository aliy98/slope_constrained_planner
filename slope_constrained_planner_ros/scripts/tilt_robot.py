#!/usr/bin/env python 
"""
.. module:: tilt_robot
    :platform: Unix
    :synopsis: the tilt_robot python script in slope_constrained_planner_ros package

Subscribes to:
    /tf

Publishes to:
    /wolf_controller/twist



A helper script based on PID control to be used for tilting the robot base frame
"""
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
import roslib
import rospy
import math
import tf
import numpy as np
from std_srvs.srv import Trigger

ROBOT_FRAME = 'base_link'
GOAL_THRES_POS = 0.01
GOAL_THRES_ANG = 0.01
FACE_GOAL_DIST = 1.0
COUNTER = 0


def getRPY(quat):
    """
        Gets roll, pitch, and yaw angles from quaternoid representation

        Args: 
            quat

        Returns:
            [roll, pitch, yaw]

    """
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
    return [roll, pitch, yaw]


def getConstrainedAngle(angle):
    """
        Gets constrained value of angle between -2pi and 2pi

        Args: 
            angle
        
        Returns:
            angle
    """
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle


def getAngleError(target, current):
    """
        Gets the difference between two given angles

        Args:
            target
            current
        
        Returns:
            dyaw
    """
    dangle = target - current
    dangle = getConstrainedAngle(dangle)
    return dangle


def getQuatFromRPY(RPY):
    """
        Gets quaternoid representation from euler representation

        Args:
            RPY
        
        Returns:
            quat
    """
    return tf.transformations.quaternion_from_euler(RPY[0], RPY[1], RPY[2])


class TiltRobot:
    """
    Defines the corresponding node with its initial configurations 
    and member functions.
    """
    def __init__(self):
        self.listener = tf.TransformListener()
        self.pub_twist = rospy.Publisher('/wolf_controller/twist', Twist, queue_size=1)
        self.sub_goal = rospy.Subscriber('/goal', PoseStamped, self.goal_clbk)
        self.current_pose = None
        self.goal_pose = None
        self.fixed_frame = "odom"
        self.gain_pid_pos = [0.3, 0.0, 0.0]
        self.gain_pid_ang = [0.5, 0.0, 0.0]
        self.i = [0, 0, 0]
        success = self.standUp()
        while not success:
            rospy.sleep(1)
        self.computeAndPublishTwist()

    def goal_clbk(self, msg):
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.goal_pose = [msg.pose.position.x, msg.pose.position.y, getRPY(quat)[0], getRPY(quat)[1], getRPY(quat)[2]]

    def standUp(self):
        rospy.wait_for_service('wolf_controller/stand_up')
        try:
            stand_up = rospy.ServiceProxy('wolf_controller/stand_up', Trigger)
            response = stand_up()
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)


    def updateCurrentPose(self):
        """
            Updates robot current position using tf tree data
        """
        if self.fixed_frame is not None:
            self.listener.waitForTransform(self.fixed_frame, ROBOT_FRAME, rospy.Time(), rospy.Duration(4.0))
            try:
                now = rospy.Time.now()
                self.listener.waitForTransform(self.fixed_frame, ROBOT_FRAME, now, rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform(self.fixed_frame, ROBOT_FRAME, now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr('TF lookup of pose failed')
                return
            RPY = getRPY(rot)
            self.current_pose = [trans[0], trans[1], RPY[0], RPY[1], RPY[2]]
        else:
            rospy.logerr('Fixed frame not set.')
        

    def getYawTarget(self):
        """
            Gets yaw angle of target frame

            Returns:
                yaw_target
        """
        dx = self.goal_pose[0] - self.current_pose[0]
        dy = self.goal_pose[1] - self.current_pose[1]
        dist = (dx**2 + dy**2)**0.5

        if dist < FACE_GOAL_DIST:
            return self.goal_pose[2]
        else:
            # Face towards goal.
            yaw_target = math.atan2(dy, dx)
            error = getAngleError(yaw_target, self.current_pose[2])
            if abs(error) > math.pi*0.5:
                # Face dat booty towards the goal.
                yaw_target = getConstrainedAngle(yaw_target + math.pi)
            return yaw_target

    def alignToGoal(self):
        yaw_target = self.getYawTarget()
        dyaw = getAngleError(yaw_target, self.current_pose[4])
        msg = Twist()
        rospy.loginfo("Rotating towards goal...")
        while abs(dyaw) > GOAL_THRES_ANG:
            self.updateCurrentPose()
            # print("yaw: " + str(self.current_pose[4] * 180 / math.pi))
            dyaw = getAngleError(yaw_target, self.current_pose[4])
            self.i[2] += dyaw
            yaw_rate = dyaw * self.gain_pid_ang[0] + self.i[0] * self.gain_pid_ang[1]
            msg.angular.z = yaw_rate
            self.pub_twist.publish(msg)

    def tiltAlongpitch(self):
        rospy.loginfo("Tilting pitch to +25deg...")
        self.goal_pose = self.current_pose
        pitch_target = 25 * math.pi / 180
        dpitch = getAngleError(pitch_target, self.current_pose[3])
        msg = Twist()
        COUNTER = 0
        while abs(dpitch) > GOAL_THRES_ANG and COUNTER < 200:
            self.updateCurrentPose()
            # print("pitch: " + str(self.current_pose[3] * 180 / math.pi))
            dpitch = getAngleError(pitch_target, self.current_pose[3])
            self.i[1] += dpitch
            pitch_rate = dpitch * self.gain_pid_ang[0] + self.i[1] * self.gain_pid_ang[1]
            msg.angular.y = pitch_rate
            self.pub_twist.publish(msg)
            COUNTER += 1

        rospy.loginfo("Tilting pitch to -25deg...")
        self.goal_pose = self.current_pose
        pitch_target = -25 * math.pi / 180
        dpitch = getAngleError(pitch_target, self.current_pose[3])
        COUNTER = 0
        while abs(dpitch) > GOAL_THRES_ANG and COUNTER < 200:
            self.updateCurrentPose()
            # print("pitch: " + str(self.current_pose[3] * 180 / math.pi))
            dpitch = getAngleError(pitch_target, self.current_pose[3])
            self.i[1] += dpitch
            pitch_rate = dpitch * self.gain_pid_ang[0] + self.i[1] * self.gain_pid_ang[1]
            msg.angular.y = pitch_rate
            self.pub_twist.publish(msg)
            COUNTER += 1

        rospy.loginfo("Tilting pitch to 0deg...")
        self.goal_pose = self.current_pose
        pitch_target = 0
        dpitch = getAngleError(pitch_target, self.current_pose[3])
        COUNTER = 0
        while abs(dpitch) > GOAL_THRES_ANG and COUNTER < 200:
            self.updateCurrentPose()
            # print("pitch:" + str(self.current_pose[3] * 180 / math.pi))
            dpitch = getAngleError(pitch_target, self.current_pose[3])
            self.i[1] += dpitch
            pitch_rate = dpitch * self.gain_pid_ang[0] + self.i[1] * self.gain_pid_ang[1]
            msg.angular.y = pitch_rate
            self.pub_twist.publish(msg)
            COUNTER += 1


    def moveBackward(self):
        rospy.loginfo("Moving backward...")
        x_target = self.current_pose[0] - 1
        dx =  x_target - self.current_pose[0]
        msg = Twist()
        COUNTER = 0
        while abs(dx) > GOAL_THRES_POS and COUNTER < 200:
            self.updateCurrentPose()
            # print("dx: " + str(dx))
            dx =  x_target - self.current_pose[0]
            self.i[0] += dx
            x_rate = dx * self.gain_pid_pos[0] + self.i[0] * self.gain_pid_pos[1]
            msg.linear.x = x_rate
            self.pub_twist.publish(msg)
            COUNTER += 1

    def moveForward(self):
        rospy.loginfo("Moving forward...")
        x_target = self.current_pose[0] + 1
        dx =  x_target - self.current_pose[0]
        msg = Twist()
        COUNTER = 0
        while abs(dx) > GOAL_THRES_POS and COUNTER < 200:
            self.updateCurrentPose()
            # print("dx: " + str(dx))
            dx =  x_target - self.current_pose[0]
            self.i[0] += dx
            x_rate = dx * self.gain_pid_pos[0] + self.i[0] * self.gain_pid_pos[1]
            msg.linear.x = x_rate
            self.pub_twist.publish(msg)
            COUNTER += 1
        
    def computeAndPublishTwist(self):
        """
            Computes the twist value and publishes it on the ``/wolf_controller/twist`` topic
        """
        self.updateCurrentPose()
        if self.current_pose is not None:
            self.alignToGoal()
            self.tiltAlongpitch()
        else:
            rospy.logerr("Cant get robot pose!")
            

if __name__ == '__main__':
    rospy.init_node('path_follower_pid')

    tilt_robot = TiltRobot()

