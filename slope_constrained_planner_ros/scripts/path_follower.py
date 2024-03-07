#!/usr/bin/env python 
"""
.. module:: path_follower
    :platform: Unix
    :synopsis: the path_follower python script in slope_constrained_planner package

Subscribes to:
    /slope_constrained_planner/path

Publishes to:
    /spot/wolf_controller/twist

    /slope_constrained_planner/followed_path


A provided path follower, used only in simulation mode, based on PID control to be used for testing on a mobile robot
"""
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse
import rospy
import math
import tf
import numpy as np



ROBOT_FRAME = 'base_link'
GOAL_THRES_POS = 0.1
GOAL_THRES_ANG = 0.2
FACE_GOAL_DIST = 1.0



def getYaw(quat):
    """
        Gets yaw angle from quaternoid representation

        Args: 
            quat

        Returns:
            yaw

    """
    _, _, yaw = tf.transformations.euler_from_quaternion(quat)
    return yaw


def getRoll(quat):
    """
        Gets roll angle from quaternoid representation

        Args: 
            quat

        Returns:
            roll

    """
    roll, _, _ = tf.transformations.euler_from_quaternion(quat)
    return roll


def getPitch(quat):
    """
        Gets pitch angle from quaternoid representation

        Args: 
            quat

        Returns:
            pitch

    """
    _, pitch, _ = tf.transformations.euler_from_quaternion(quat)
    return pitch


def getConstrainedYaw(yaw):
    """
        Gets constrained value of yaw angle between -2pi and 2pi

        Args: 
            yaw
        
        Returns:
            yaw
    """
    while yaw > math.pi:
        yaw -= 2*math.pi
    while yaw < -math.pi:
        yaw += 2*math.pi
    return yaw


def getAngleError(target, current):
    """
        Gets the difference between two given angles

        Args:
            target
            current
        
        Returns:
            dyaw
    """
    dyaw = target - current
    dyaw = getConstrainedYaw(dyaw)
    return dyaw

def getQuatFromYaw(yaw):
    """
        Gets quaternoid representation from euler representation

        Args:
            yaw
        
        Returns:
            quat
    """
    return tf.transformations.quaternion_from_euler(0, 0, yaw)



class PathFollower:
    """
    Path follower class which defines the corresponding node with its initial configurations 
    and member functions.
    """
    def __init__(self):
        self.listener = tf.TransformListener()
        self.pub_twist = rospy.Publisher('/wolf_controller/twist', Twist, queue_size=1)
        self.pub_path = rospy.Publisher('/slope_constrained_planner/followed_path', Path, queue_size=1, latch=True)
        self.sub = rospy.Subscriber('/slope_constrained_planner/path', Path, self.pathCallback)
        self.dummy_srv = rospy.Service('/path_follower/dummy_service', Empty, self.dummyCallback)
        self.current_pose = None
        self.goal_pose = None
        self.fixed_frame = None
        self.path = None
        self.path_ros = None
        self.largest_valid_index = 0
        self.gain_pid_pos = [0.3, 0.0, 0.0]
        self.gain_pid_ang = [0.2, 0.0, 0.0]
        self.i = [0, 0, 0]
        rospy.set_param("/path_follower/init_time", rospy.Time.now().to_sec())



    def pathCallback(self, path_msg):
        """
            Callback function which triggers once the found path is available 

            Args:
                path_msg
        """
        self.fixed_frame = path_msg.header.frame_id

        self.path = []
        self.path_ros = path_msg
        self.goal_pose = None

        if len(path_msg.poses) > 1:

            for ros_pose in path_msg.poses:
                pos = ros_pose.pose.position
                rot = ros_pose.pose.orientation
                yaw = getYaw([rot.x, rot.y, rot.z, rot.w])
                roll = getRoll([rot.x, rot.y, rot.z, rot.w])
                pitch = getPitch([rot.x, rot.y, rot.z, rot.w])
                self.path.append([pos.x, pos.y, yaw, roll, pitch])

            self.removePathNodesBeforeIndex(1)
            # rospy.loginfo('Got path: ' + str(self.path))
            self.i = [0, 0, 0]    # Reset integrators.

        else:
            rospy.logwarn('Path message is too short')

    def dummyCallback(self, request):
        """
            A dummy rosservice, used for notifying the presence of this node to the planner.
        """
        rospy.loginfo("Empty service called.")
        return EmptyResponse()

    def updateCurrentPose(self):
        """
            Updates robot current position using tf tree data
        """
        if self.fixed_frame is not None:
            try:
                (trans,rot) = self.listener.lookupTransform(self.fixed_frame, ROBOT_FRAME, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn_throttle('TF lookup of pose failed')
                return

            self.current_pose = [trans[0], trans[1], getYaw(rot), getRoll(rot), getPitch(rot)]
        else:
            rospy.logwarn_throttle(1, 'Fixed frame not set.')



    def publishPath(self):
        """
            Publishes the followed path so far, with its frame id
        """
        if self.fixed_frame is not None:
            msg = Path()
            msg.header.frame_id = self.fixed_frame
            if self.path_ros is not None:
                msg.poses = self.path_ros.poses

            self.pub_path.publish(msg)




    def removePathNodesBeforeIndex(self, index):
        """
            Removes the path points before current index

            Args:
                index
        """
        self.path = self.path[index:]
        self.path_ros.poses = self.path_ros.poses[index:]



    def updateCurrentGoalPose(self):
        """
            Updates current goal pose considering the remaining path points
        """
        if self.goal_pose is not None:
            dx = self.goal_pose[0] - self.current_pose[0]
            dy = self.goal_pose[1] - self.current_pose[1]
            dist = (dx**2 + dy**2)**0.5
            dyaw = getAngleError(self.goal_pose[2], self.current_pose[2])
            if dist < GOAL_THRES_POS and abs(dyaw) < GOAL_THRES_ANG:
                if len(self.path) > 1:
                    self.removePathNodesBeforeIndex(1)
                else:
                    self.path = None
                    self.path_ros = None
                    self.publishPath()
                self.goal_pose = None

        # Only get new goal pose if we don't have one.
        if self.goal_pose is None:
            if self.current_pose is not None and self.path is not None:
                # Set goal to final path segment in case we have a weird path
                # and all checks fail.
                for i in range(len(self.path)):
                    # path_segment = np.array([self.path[i+1][0] - self.path[i][0],
                    #                          self.path[i+1][1] - self.path[i][1]])
                    robot_from_node = np.array([self.current_pose[0] - self.path[i][0],
                                                self.current_pose[1] - self.path[i][1]])
                    # dist_along_path = robot_from_node.dot(path_segment)
                    # if (dist_along_path > 0):
                    #     # Robot is "in front of" the current node.

                    if abs(robot_from_node[0]) < GOAL_THRES_POS and abs(robot_from_node[1]) < GOAL_THRES_ANG:
                        rospy.loginfo('Target reached!')
                        self.largest_valid_index = i+1
                    
                rospy.loginfo('Current goal state index in the path: ' + str(self.largest_valid_index))
                self.removePathNodesBeforeIndex(self.largest_valid_index)
                if(len(self.path)>0):
                    self.goal_pose = self.path[0]
                self.publishPath()



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
                yaw_target = getConstrainedYaw(yaw_target + math.pi)
            return yaw_target



    def computeAndPublishTwist(self):
        """
            Computes the twist value and publishes it on the ``/cmd_vel`` topic
        """
        self.updateCurrentPose()
        if self.path is not None and self.current_pose is not None:
            self.updateCurrentGoalPose()

            if self.goal_pose is None:
                return

            msg = Twist()
            # msg.header.frame_id = ROBOT_FRAME
            # msg.header.stamp = rospy.Time.now()

            yaw = self.current_pose[2]
            yaw_target = self.getYawTarget()

            dx = self.goal_pose[0] - self.current_pose[0]
            dy = self.goal_pose[1] - self.current_pose[1]
            dyaw = getAngleError(yaw_target, self.current_pose[2])
            droll = getAngleError(self.goal_pose[3], self.current_pose[3])
            dpitch = getAngleError(self.goal_pose[4], self.current_pose[4])

            dlon = math.cos(yaw)*dx + math.sin(yaw)*dy
            dlat = -math.sin(yaw)*dx + math.cos(yaw)*dy

            # Update integrator.
            self.i[0] += dlon
            self.i[1] += dlat
            self.i[2] += dyaw

            lon_rate = dlon * self.gain_pid_pos[0] + self.i[0] * self.gain_pid_pos[1]
            lat_rate = dlat * self.gain_pid_pos[0] + self.i[1] * self.gain_pid_pos[1]
            yaw_rate = dyaw * self.gain_pid_ang[0] + self.i[2] * self.gain_pid_ang[1]
            roll_rate = droll  * self.gain_pid_ang[0] 
            pitch_rate = dpitch * self.gain_pid_ang[0] 
        
            msg.linear.x = lon_rate
            msg.linear.y = lat_rate
            msg.angular.z = yaw_rate
            msg.angular.x = roll_rate
            msg.angular.y = pitch_rate

            self.pub_twist.publish(msg)






if __name__ == '__main__':
    rospy.init_node('path_follower_pid')

    follower = PathFollower()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        follower.computeAndPublishTwist()
        rate.sleep()