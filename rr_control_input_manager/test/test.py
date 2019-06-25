#!/usr/bin/env python
from __future__ import print_function

PKG = 'rr_control_input_manager'
import roslib

roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import os
import sys
import rospkg

rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path(PKG), 'scripts'))
import unittest
import rosunit

from control_input_manager import CommandHandle, ControlInputManager


## A sample python unit test
class TestTwistCommand(unittest.TestCase):
    def test_default_timeout_0_float(self):  # only functions with 'test_'-prefix will be run!



class TestLatency(unittest.TestCase):
    pass


rosunit.unitrun(PKG, 'test_bare_bones', TestTwistCommand)
