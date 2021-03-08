#!/usr/bin/env python

from time import sleep
from manipulator import Manipulator

#manipulator = Ur3("150.254.47.149", 30003, 30002)
manipulator = Manipulator("150.254.47.149", 30003, 30002)
print("XFD")

for i in range(10):
    pose = manipulator.get_pose()
    print(pose)
    pose = None
    sleep(1.)


