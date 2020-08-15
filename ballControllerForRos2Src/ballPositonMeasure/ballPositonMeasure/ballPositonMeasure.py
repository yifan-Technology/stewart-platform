# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import sys, serial, struct, time
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point

port = '/dev/ttyUSB0'

class ballPositonMeasure(Node):

    def __init__(self):
        super().__init__('ballPositonMeasure')
        self.publisher_ = self.create_publisher(Point, 'stewart2/ballPositon', 10)
        timer_period = 0.02 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sp = serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
             xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)
        self.sp.setDTR(True) # dsrdtr is ignored on Windows.

    def timer_callback(self):
        self.sp.write(b'send')
        self.sp.flush()
        rawData = self.sp.read(12)
        data = struct.unpack("<lll",rawData)
        msg = Point()
        msg.x = float(data[0])
        msg.y = float(data[1])
        msg.z = float(data[2])
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(data))
    def closeSerial(self):
        self.sp.close()


def main(args=None):
    rclpy.init(args=args)

    ballPositonMeasurement = ballPositonMeasure()

    rclpy.spin(ballPositonMeasurement)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ballPositonMeasurement.closeSerial()
    ballPositonMeasurement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
