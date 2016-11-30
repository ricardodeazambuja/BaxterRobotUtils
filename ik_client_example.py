import time

import rospy

# http://api.rethinkrobotics.com/baxter_interface/html/index.html
import baxter_interface

# https://github.com/ricardodeazambuja/BaxterRobotUtils/blob/master/ik_client.py
from ik_client import ik_service

limb = 'right'

gripper_force_threshold = 30 # in percentage
gripper_vacuum_threshold = 18 # in percentage

# Receives a list of poses / gripper commands
ik = ik_service(limb, speed=0.3)

gripper = baxter_interface.Gripper(limb)

print "Using the " + gripper.type() + " gripper."

if gripper.type() == 'electric':
    print "Calibrating the electric gripper"
    gripper.calibrate()
    gripper.set_holding_force(gripper_force_threshold)
else:
    gripper.set_vacuum_threshold(gripper_vacuum_threshold)
    
print "Gripper parameters: ",gripper.parameters()

# Creates two empty lists
movements = []
gripper_state = []

# The values can be easily copied from the command:
# rostopic echo /robot/limb/<left/right>/endpoint_state/pose -n 1 -p

# Position 1
movements.append([0.58095804878,0.180697202191,0.104661163813,0.138913336717,
                  0.989859457319,0.0104028059438,0.0278050582852])
# Gripper state (0=>open, 1=>close)
gripper_state.append(0)

# Position 2
movements.append([-0.232847216127,0.869765405092,-0.159673189854,0.308104056409,
                  0.9484955968,0.0451304514572,-0.0582343165775])

# Gripper state (0=>open, 1=>close)
gripper_state.append(1)

#
# Goes through the lists commanding the robot
#

for mv_i,g_i in zip(movements,gripper_state):
    if not ik.ik_call(mv_i[:3], mv_i[3:]):
        ik.ik_move_to(timeout=15)
        if g_i:
            gripper.close()
        else:
            gripper.open()
    else:
        print "IK returned an error..."