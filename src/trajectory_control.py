#/usr/bin/python

import rospy
import numpy
from sensor_msgs.msg import JointState

class TrajectoryControl:
  def __init__(self):
    rospy.init_node('trajectory_control')
    self.cmd_pub = rospy.Publisher('command', JointState, queue_size=1)
    rospy.Subscriber('state', JointState, self.state_callback, queue_size=1)
    self.last_state = JointState()

  def state_callback(self, state_msg):
    self.last_state = state_msg

  def set_positions(self, p0, p1):
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.position = [p0, p1]
    self.cmd_pub.publish(js)

  def set_velocities(self, v0, v1):
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.velocity = [v0, v1]
    self.cmd_pub.publish(js)
    
  def random_position_demo(self):
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
      p0, p1 = numpy.random.rand(2) - 0.5
      self.set_positions(p0,p1)
      rate.sleep()

  def sinusoid_demo(self):
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
      if (rospy.Time.now() - self.last_state.header.stamp).to_sec() < 0.5:
        p0, p1 = self.last_state.position
        self.set_velocities(0.0, -numpy.cos(rospy.Time.now().to_sec()))
      rate.sleep()
    
if __name__ == '__main__':
  tc = TrajectoryControl()
  tc.random_position_demo()
