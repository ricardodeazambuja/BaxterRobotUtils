'''
ZeroG

Allows the user to loosen some joints while stiffen others.

Based on "Baxter RSDK Joint Trajectory Action Server" and the other examples...


rostopic echo /robot/joint_states/position[6] 2>/dev/null | rostopic pub left_w0 std_msgs/Float64 -r 150 >/dev/null&
rostopic echo /robot/joint_states/position[2] 2>/dev/null | rostopic pub left_e0 std_msgs/Float64 -r 150 >/dev/null&
2>/dev/null => redirects the stderr (the pipe receives only stdout)
-r 150 => must be faster than the result of rostopic hz /robot/joint_states

proc = subprocess.Popen(...)
proc.send_signal(subprocess.signal.SIGINT)

'''
import operator
import subprocess
from multiprocessing import Process

import rospy

from std_msgs.msg import (
    UInt16,
    Bool,
    Empty,
)

import baxter_control
import baxter_interface

# The gains are QUITE HIGH, so it's important to start at the initial position!
starting_pos_example = dict(zip(['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2'],[0.0, 1.85, -0.54, -1.1, 0.0, 0.78, 0.64]))

stiff_joints_names = ['left_w0', 'left_e0'] #I will keep left_w2 loose, because the pen should be aligned with that axis anyway...
joint_gains = [[30,21,.4],[60,42,1.2]]

stiff_joints_example = dict(zip(stiff_joints_names,[0.0,0.0])) # Set point angles for the stiff joints
PID_gains_example = dict(zip(stiff_joints_names,[dict(zip(['kp','ki','kd'],jg)) for jg in joint_gains]))

class ZeroG(object):
    def __init__(self, limb_name='left', PID_gains=PID_gains_example, rate=1000.0,
                 stiff_joints=stiff_joints_example, starting_positions=starting_pos_example):

        rospy.loginfo("Initializing node... ")
        rospy.init_node("ZeroG")

        self._stiff_joints = stiff_joints # {'joint_name':angle,...}
        self.stiff_joints_names = stiff_joints.keys() # ['joint_name1',...]
        self.stiff_joints_angles = stiff_joints.values() # [angle1,...]

        self._gains = PID_gains # {'joint_name':{'kp':value,'ki':value,'kd':value},...}

        self._missed_cmds = 20

        self._ns = 'robot/limb/' + limb_name
        self._limb = baxter_interface.Limb(limb_name)
        self._joint_names = self._limb.joint_names()
        self._torques = dict(zip(self._joint_names,[0.0]*len(self._joint_names)))

        self._loose_joints = []

        self._enable = baxter_interface.RobotEnable()
        self._name = limb_name
        self._cuff = baxter_interface.DigitalIO('%s_lower_cuff' % (limb_name,))
        # DigitalIO('%s_upper_button' % (limb,))  # 'dash' btn
        # DigitalIO('%s_lower_button' % (limb,))   # 'circle' btn
        # DigitalIO('%s_lower_cuff' % (limb,))    # cuff squeeze
        self._cuff.state_changed.connect(self._cuff_cb) #callback for buttons
                                                        #it pass the buttons state to _cuff_cb

        self._cuff_state = False

        # Controller parameters
        self._control_rate = rate  # Hz

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._control_rate) * self._missed_cmds)

        # Set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher(
            '/robot/joint_state_publish_rate',
             UInt16,
             queue_size=10)
        self._pub_rate.publish(self._control_rate)

        print "Initialization completed!"

        rospy.on_shutdown(self.clean_shutdown)

        print "Moving to starting position!"
        self._limb.move_to_joint_positions(starting_positions, timeout=15.0, threshold=0.008726646)

    def _disable_cuff(self):
        cmd="rostopic pub -r 10 /robot/limb/left/suppress_cuff_interaction std_msgs/Empty"
        print "Normal Zero-G disabled!"
        return subprocess.Popen(cmd.split())

    def _start_controller(self):
        control_rate = rospy.Rate(self._control_rate)
        print "Hold Baxter's cuff sensors to start..."
        # Waits until ctrl+c or someone holds the cuff
        while not rospy.is_shutdown():
            if not self.robot_is_enabled():
                return
            if self._cuff_state:
                break
            control_rate.sleep()

        # Create PID controllers for all joints
        # Only the stiff joints are going to be controlled, the others will get torque ZERO.
        self._pid = dict()
        for joint in self.stiff_joints_names:
            self._pid[joint] = baxter_control.PID()
            self._pid[joint].set_kp(self._gains[joint]['kp'])
            self._pid[joint].set_ki(self._gains[joint]['ki'])
            self._pid[joint].set_kd(self._gains[joint]['kd'])
            self._pid[joint].initialize()

        self._enable_pub = rospy.Publisher('robot/set_super_enable',
                                           Bool, queue_size=10)

        print "Baxter is under control..."
        # Does the control magic :D
        while not rospy.is_shutdown():
            if not self.robot_is_enabled():
                rospy.logerr("ZeroG problem with _start_controller...")
                break
            if not self._cuff_state:
                print "The user released the cuff sensor...exiting!"
                break
            # self._disable_original_zero_g.publish()
            self._update_command()
            control_rate.sleep()

        self.clean_shutdown()
        print "The end!"

    def _update_command(self):
        error = self._get_current_error(self.stiff_joints_names,self.stiff_joints_angles)
        # print error
        for joint in error:
                self._torques[joint] = self._pid[joint].compute_output(error[joint])
        self._limb.set_joint_torques(self._torques)

    def robot_is_enabled(self):
        return self._enable.state().enabled

    def clean_shutdown(self):
        self._limb.exit_control_mode()
        if hasattr(self, 'disable_orig_zerog'):
            self.disable_orig_zerog.kill()


    def _cuff_cb(self, value):
        self._cuff_state = value
        print "Cuff state:",value

    def _get_current_position(self, joint_names):
        return [self._limb.joint_angle(joint) for joint in joint_names]

    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position(joint_names)
        error = map(operator.sub, set_point, current)
        return dict(zip(joint_names, error))

if __name__ == '__main__':
    zo = ZeroG()

    # Launches a second process to disable the original zero-g:
    zo.disable_orig_zerog = zo._disable_cuff()

    # Starts the controller
    zo._start_controller()
