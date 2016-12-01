#!/usr/bin/env python

"""
Inverse Kinematics
"""
import argparse
import struct
import sys
import baxter_interface
import rospy

import time

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)



class ik_service(object):
    def __init__(self, limb, speed=0.3):
        rospy.init_node("ik_service", anonymous=True)
        self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)

        # Starts the Libm interface
        self.arm = baxter_interface.Limb(limb)
        self.arm.set_command_timeout(2.0)
        self.arm.set_joint_position_speed(speed)


    def ik_call(self, xyz, quat, seed_angles = None):
        '''
        xyz: list with  cartesian position
        quat: list with quaternion (the robust equivalent of roll, pitch and yaw)
        
        Optional:
        seed_angles: list with all seven joint angles to start the IK algorithm
        '''
        self.ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose = PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=xyz[0],
                        y=xyz[1],
                        z=xyz[2],
                    ),
                    orientation=Quaternion(
                        x=quat[0],
                        y=quat[1],
                        z=quat[2],
                        w=quat[3],
                    ),
                ),
            )
        if seed_angles:
            self.ikreq.pose_stamp.append(pose,seed_angles=seed_angles)
        else:
            self.ikreq.pose_stamp.append(pose)

        try:
            rospy.wait_for_service(self.ns, 5.0) # blocks until a service is available
            resp = self.iksvc(self.ikreq)

        except Exception as err:
            print err
            return 1

        # Check if result valid
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            # Format solution into Limb API-compatible dictionary
            self.joint_names = resp.joints[0].name
            self.joint_values = resp.joints[0].position
            self.limb_joints = dict(zip(self.joint_names, self.joint_values))
        else:
            print "Invalid IK"
            return 2

        return 0


    def ik_move(self):
        self.arm.set_joint_positions(self.limb_joints)
        
    def ik_move_to(self, timeout=15.0):
        self.arm.move_to_joint_positions(self.limb_joints, timeout)