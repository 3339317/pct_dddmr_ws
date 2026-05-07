#! /usr/bin/env python3

# BSD 3-Clause License

# Copyright (c) 2024, DDDMobileRobot

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Bool
import time
import copy
import math
import sys

class MappingTest(Node):

    def __init__(self):
        super().__init__('mapping_test')

        self.trigger_bag_mapping = self.create_publisher(Bool, 'lego_loam_bag_pause', 2)

def main(args=None):

    rclpy.init(args=args)
    mt = MappingTest()
    executor = MultiThreadedExecutor()
    executor.add_node(mt) 

    mt.get_logger().info("Spin start!")
    
    pause_mapping_cmd = Bool()
    pause_mapping_cmd.data = False
    while rclpy.ok():

        if os.path.isdir("/tmp/testing_pg"):
            mt.get_logger().info("Detect /tmp/testing_pg dir, exit waiting")
            break
        else:
            pass

        mt.trigger_bag_mapping.publish(pause_mapping_cmd)
        executor.spin_once(timeout_sec=0.1)
    
    print('DoneSuccess') #TestGoodProcess can not get ros log0


if __name__ == '__main__':
    main()