'''
ZeroG

Allows the user to loosen some joints while stiffen others.

Based on "Baxter RSDK Joint Trajectory Action Server" and the other examples...


If you want to see joint angles using rqt_plot, you can create new topics like these:
rostopic echo /robot/joint_states/position[6] 2>/dev/null | rostopic pub left_w0 std_msgs/Float64 -r 150 >/dev/null&
rostopic echo /robot/joint_states/position[2] 2>/dev/null | rostopic pub left_e0 std_msgs/Float64 -r 150 >/dev/null&
2>/dev/null => redirects the stderr (the pipe receives only stdout)
-r 150 => must be faster than the result of rostopic hz /robot/joint_states


BE VERY VERY CAREFUL WITH THIS SCRIPT. IT USES TORQUE CONTROL (SAFETY DISABLED)
AND ALSO QUITE HIGH GAINS (CRAZY ROBOT)! I WILL TAKE NO RESPONSIBILITIES!!!

It is very important to have a fast connection to the robot or instabilities will occur!

'''

import operator # https://docs.python.org/2/library/operator.html
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

from baxter_interface import CHECK_VERSION


stiff_joints_names = ['left_w0', 'left_e0'] #I will keep left_w2 loose, because the pen should be aligned with that axis anyway...
joint_gains = [[30*2.,21/2.,.4],[60*2.,42/2.,1.2]]
# I've reduced the ki by 2 to avoid the oscillations and increased the kp by 2 to keep it as stiff.

stiff_joints_example = dict(zip(stiff_joints_names,[0.0,0.0])) # Set point angles for the stiff joints
PID_gains_example = dict(zip(stiff_joints_names,[dict(zip(['kp','ki','kd'],jg)) for jg in joint_gains]))

class ZeroG(object):
    def __init__(self, limb_name='left', PID_gains=PID_gains_example, rate=1000.0,
                 stiff_joints=stiff_joints_example):

        rospy.loginfo("Initializing ZeroG node... ")
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
        self._dash = baxter_interface.DigitalIO('%s_upper_button' % (limb_name,))
        self._circle = baxter_interface.DigitalIO('%s_lower_button' % (limb_name,))
        # DigitalIO('%s_upper_button' % (limb,))  # 'dash' btn
        # DigitalIO('%s_lower_button' % (limb,))   # 'circle' btn
        # DigitalIO('%s_lower_cuff' % (limb,))    # cuff squeeze
        self._cuff.state_changed.connect(self._cuff_cb) #callback for cuff sensor
                                                        #it pass the cuff sensor state to _cuff_cb

        self._dash.state_changed.connect(self._dash_cb) #callback for dash button
                                                        #it pass the dash button state to _dash_cb

        self._circle.state_changed.connect(self._circle_cb) #callback for circle button
                                                            #it pass the circle button state to _circle_cb

        self._cuff_state = False
        self._dash_state = False
        self._dash_value = False
        self._circle_state = False

        self._gripper = baxter_interface.Gripper('%s' % (limb_name,), CHECK_VERSION)

        # Verify Grippers Have No Errors and are Calibrated
        if self._gripper.error():
            self._gripper.reset()
        if (not self._gripper.calibrated() and
            self._gripper.type() != 'custom'):
            self._gripper.calibrate()

        # Controller parameters
        self._control_rate = rate  # Hz

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._control_rate) * self._missed_cmds)

        # also for safety, sets the maximum speed
        self._limb.set_joint_position_speed(0.3)  # it's a percentage of the arm speed


        # Set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher(
            '/robot/joint_state_publish_rate',
             UInt16,
             queue_size=10)
        self._pub_rate.publish(self._control_rate)

        print "ZeroG initialization completed!"

        rospy.on_shutdown(self.clean_shutdown)

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

    def _dash_cb(self, value):
        self._dash_value = value
        if self._dash_value:
            if self._dash_state:
                self._dash_state = False
            else:
                self._dash_state = True

    def _circle_cb(self, value):
        if value and not self._dash_value:
            if self._circle_state:
                self._gripper.open()
                self._circle_state=False
            else:
                self._gripper.close()
                self._circle_state=True

    def _get_current_position(self, joint_names):
        return [self._limb.joint_angle(joint) for joint in joint_names]

    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position(joint_names)
        error = map(operator.sub, set_point, current)
        return dict(zip(joint_names, error))

    def initial_position(self):
        print "Press the dash button to start the controller."
        # Waits until ctrl+c or someone holds the cuff
        while not rospy.is_shutdown():
            if not self.robot_is_enabled():
                return False
            if self._dash_state:
                self._dash_state = False
                break
            rospy.sleep(.1)

        if rospy.is_shutdown():
            self.clean_shutdown()

        return True

if __name__ == '__main__':
    zo = ZeroG()

    # Launches a second process to disable the original zero-g:
    zo.disable_orig_zerog = zo._disable_cuff()

    starting_positions=dict(zip(['left_w0', 'left_w1', 'left_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1'],
                                [0.0, 0.20943951023931953, 0.0, 0.0, 2.0420352248333655, -0.598647933434055, -0.640535835481919-0.1]))
                                # the -0.1 keeps the arm a little bit above the table

    for key, value in zo._stiff_joints.iteritems():
        starting_positions[key]=value  # changes the values for the stiff joints

    print "Moving to starting joint positions!"
    zo._limb.move_to_joint_positions(starting_positions, timeout=15.0, threshold=0.008726646)

    if zo.initial_position():
        # Starts the controller
        zo._start_controller()
