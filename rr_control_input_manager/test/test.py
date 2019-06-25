#!/usr/bin/env python
PKG = 'rr_control_input_manager'
import roslib
import rostest

import sys
import unittest

## A sample python unit test
class TestBareBones(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones)