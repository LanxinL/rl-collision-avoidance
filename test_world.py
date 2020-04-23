import os
import numpy as np
import rospy

from collections import deque
from mpi4py import MPI
from circle_world import StageWorld

import time

NUM_ENV = 1
OBS_SIZE = 512
ACT_SIZE = 2

comm = MPI.COMM_WORLD
rank = comm.Get_rank()
size = comm.Get_size()
env = StageWorld(OBS_SIZE, index=rank, num_env=NUM_ENV)

def my_callback(event):
    print 'Timer called at ' + str(event.current_real)

#rospy.Timer(rospy.Duration(2), my_callback, reset=True)

if env.index == 0:
    env.reset_world()

env.reset_pose()
env.generate_goal_point()

#for i in range(5000):
while not rospy.is_shutdown():
    env.control_vel([np.random.random(), np.random.random() - 1])
    #time.sleep(10)
    #time.sleep(1)
