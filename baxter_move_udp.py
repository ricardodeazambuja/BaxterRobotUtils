"""
Moves BAXTER (JointCommand) according to the signal received from a topic

http://ricardodeazambuja.com

"""

from multiprocessing import Pipe, Process

import rospy
import std_msgs.msg

import baxter_interface
from baxter_core_msgs.msg import JointCommand


class BAXTER_slave(object):
    def __init__(topic_to_subscribe_to,
                 topic_to_publish_to,
                 joints,
                 limb_name='left',
                 msg='OK',
                 msg_type='String',
                 max_freq=50):
        """
        Args:
            topic_to_subscribe_to (str): name of the topic to wait for the msg.
            topic_to_publish_to (str): name of the topic to wait for the msg.
            joints (list(list)): a list with what 'rostopic echo /robot/joint_states' outputs.
            limb_name (str): right or left.
            msg (str): the system will wait for this string.
            msg_type (str): ROS std_msgs.msg
            max_freq (int): max frequency to publish.
        """

        assert type(joints[0]) != list, "Joints must be a list of lists!"

        self.received_msg = None

        joint_names_simpl = ['_e0', '_e1', '_s0', '_s1', '_w0', '_w1', '_w2']

        joint_names = [limb_name+si for si in joint_names_simpl]
        self.joints_dict = dict(zip(joint_names,[None]*len(joint_names)))

        self.joints_list = joints

        self.msg = msg

        self.counter = 0

        rospy.init_node('move_BAXTER', anonymous=False)

        self.limb = baxter_interface.Limb(limb_name)

        self.available_joints = self.limb.joint_names()

        self.msg_type = getattr(std_msgs.msg, msg_type)

        self.write_msg = rospy.Publisher(
                                    topic_to_publish_to,
                                    self.msg_type,
                                    tcp_nodelay=True,
                                    queue_size=1)

        self.read_msg = rospy.Subscriber(
                                    topic_to_subscribe_to,
                                    self.msg_type,
                                    self.move_robot,
                                    tcp_nodelay=True,
                                    queue_size=1)

    def move_robot(self, data):

        joint_names = ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
        joints_dict_conv = dict(zip(joint_names,self.joints_list[self.counter]))

        if data.data == self.msg:
            for ji in self.joints_dict:
                self.joints_dict[ji]=joints_dict_conv[ji]
            limb.move_to_joint_positions(self.joints_dict, timeout=15.0, threshold=0.008726646)

            if self.counter<(len(self.joints_list)-1):
                self.counter += 1
            else:
                self.counter = 0


            self.write_msg.publish(self.msg_type(self.msg))
