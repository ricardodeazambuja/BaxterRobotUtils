'''
Movement Recorder

Records some topics if the user presses both dash and circle buttons


I need to record:
 /robot/limb/'arm_name'/endpoint_state
 => baxter_core_msgs/EndpointState
 and
 /robot/joint_states
 => sensor_msgs/JointState

Pressing both buttons will start / stop the recording.
/robot/digital_io/left_lower_button/state
/robot/digital_io/left_upper_button/state

'''
import rospy

import numpy

from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import EndpointState
from baxter_core_msgs.msg import DigitalIOState


class read_from_ros(object):

    def __init__(self, base_filename='learned_curve',  joint_names = ['left_s0','left_s1','left_e1','left_w1']):

        self.base_filename = base_filename

        self.joint_names = joint_names

        self.joints = dict(zip(joint_names,[0]*len(joint_names)))

        self.joints_save = []

        self.endpoint_save = []

        self.dash_state = False
        self.circle_state = False

        self.record = False

        rospy.loginfo("Initializing Movement Recorder node... ")

        rospy.init_node('movement_recorder', anonymous=False) #I don't want to have multiple "children" nodes

        joint_state_topic = 'robot/joint_states'

        endpoint_state_topic = '/robot/limb/left/endpoint_state'

        dash_button_state_topic = '/robot/digital_io/left_upper_button/state'
        circle_button_state_topic = '/robot/digital_io/left_lower_button/state'

        _joint_state_sub = rospy.Subscriber(
            joint_state_topic,
            JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)

        _endpoint_state_sub = rospy.Subscriber(
            endpoint_state_topic,
            EndpointState,
            self._on_endpoint_states,
            queue_size=1,
            tcp_nodelay=True)

        _dash_button_state_sub = rospy.Subscriber(
            dash_button_state_topic,
            DigitalIOState,
            self._on_dash_button_states,
            queue_size=1,
            tcp_nodelay=True)

        _circle_button_state_sub = rospy.Subscriber(
            circle_button_state_topic,
            DigitalIOState,
            self._on_circle_button_states,
            queue_size=1,
            tcp_nodelay=True)

        rospy.on_shutdown(self.clean_shutdown)

        state_changed = False

        print "Movement Recorder initialization completed!"
        print "Press Baxter's circle button to start and stop recording..."

        # while not rospy.is_shutdown():
        #     if self.dash_state and self.circle_state and (not self.record):
        #         self.record = True
        #         print "Recording started!"
        #
        #     if (not self.dash_state) and (not self.circle_state) and self.record:
        #         state_changed = True
        #
        #     if self.dash_state and self.circle_state and state_changed:
        #         print "Recording finished!"
        #         return
        #
        #     rospy.sleep(.1)

        while not rospy.is_shutdown():
            if self.circle_state and (not self.record):
                self.record = True
                print "Recording started!"

            if (not self.circle_state) and self.record:
                state_changed = True

            if self.circle_state and state_changed:
                print "Recording finished!"
                return

            rospy.sleep(.1)


    def _on_joint_states(self,msg):

        for idx, key in enumerate(msg.name):
            if key in self.joint_names:
                self.joints[key]=msg.position[idx]

        if self.record:
            self.joints_save.append([self.joints[idx] for idx in self.joint_names])

    def _on_endpoint_states(self,msg):
        if self.record:
            self.endpoint_save.append([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])

    def _on_circle_button_states(self, msg):
        self.circle_state = msg.state

    def _on_dash_button_states(self, msg):
        self.dash_state = msg.state

    def clean_shutdown(self):

        if self.record:
            print "Saving files..."
            numpy.save(self.base_filename+'_joints',self.joints_save)
            print self.base_filename+'_joints'+'.npy', "...saved!"
            numpy.save(self.base_filename+'_endpoints',self.endpoint_save)
            print self.base_filename+'_endpoints'+'.npy', "...saved!"

        print "The end!"

if __name__ == '__main__':
    r = read_from_ros()
