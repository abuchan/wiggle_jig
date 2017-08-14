#!/usr/bin/python

from packet_parser import *

import rospy
import math
import threading

from sensor_msgs.msg import JointState

modes = [MODE_POS, MODE_VEL, MODE_PWM, MODE_NONE]

class WiggleJig(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    self.daemon = True
    rospy.init_node('wiggle_jig')
    self.pp = PacketParser()
    self.rate = rospy.Rate(200.0)
    self.state_pub = rospy.Publisher('state',JointState,queue_size=1)
    rospy.Subscriber('command', JointState, self.command_callback, queue_size=1)

  def command_callback(self, cmd_msg):
    cmds = [list(c) + 2*[float('nan')] for c in
      [cmd_msg.position, cmd_msg.velocity, cmd_msg.effort, 2*[0.0]]]
    cmds_0, cmds_1 = zip(*cmds)
    
    for c0, m0 in zip(cmds_0, modes):
      if not math.isnan(c0):
        break
    
    for c1, m1 in zip(cmds_1, modes):
      if not math.isnan(c1):
        break
  
    self.pp.put(get_command_packet(c0, c1, m0, m1))

  def set_positions(self, p0, p1):
    self.pp.put(get_command_packet(p0, p1, MODE_POS, MODE_POS))

  def set_velocities(self, v0, v1):
    self.pp.put(get_command_packet(v0, v1, MODE_VEL, MODE_VEL))

  def run(self):
    self.pp.start()

    while not rospy.is_shutdown():
      pkt = self.pp.get()
      
      if pkt:
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.seq = pkt['time']
        js.header.frame_id = 'wiggle_jig'
        js.position = [pkt['p0'], pkt['p1']]
        js.velocity = [pkt['v0'], pkt['v1']]
        js.effort   = [pkt['w0'], pkt['w1']]
        self.state_pub.publish(js)
      
      self.rate.sleep()

if __name__ == '__main__':
  wj = WiggleJig()
  wj.run()
