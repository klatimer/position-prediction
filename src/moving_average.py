#!/usr/bin/env python

# github - klatimer/position_prediction
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from collections import deque	
import numpy

class MovingAverage:
	def __init__(self):
		# Fields
		self.time_range = 2 # seconds
		self.n_time_increments = 100
		self.n_data_pts = 10 # how many data points to use for average
		self.buffer_filled = False
		self.xbuf = deque('', maxlen=self.n_data_pts) # sliding window to store position
		self.ybuf = deque('', maxlen=self.n_data_pts)
		self.tbuf = deque('', maxlen=self.n_data_pts)

		self.pub_predicted = rospy.Publisher('predicted_position', Int64, queue_size=10)
		rospy.Subscriber("/ar_single_board/position", Vector3Stamped, self.callback)
		rospy.init_node('moving_average', anonymous=True)
		rospy.spin()

	def callback(self, data):
		self.xbuf.appendleft(data.vector.x)
		self.ybuf.appendleft(data.vector.y)
		self.tbuf.appendleft(data.header.stamp.secs + data.header.stamp.nsecs)
		if not self.buffer_filled:
			if len(self.xbuf) == self.n_data_pts: 
				self.buffer_filled = True
		else: 
			def generate_predicted_states(data):
				# p = polynomial coefficients in descending order
				p = numpy.polyfit(self.tbuf, data, 1, rcond = None, full = False, w = None, cov = False)
				slope = p[0]
				latest_point = [self.tbuf[0], data[0]]
				intercept = latest_point[0] - slope * latest_point[1]
				time_step = self.time_range / self.n_time_increments
				dq = deque('', maxlen = self.n_time_increments)
				for i in range(0, self.n_time_increments):
					dq.appendleft(latest_point[1] + slope * time_step)
				return dq
			xpred = generate_predicted_states(self.xbuf)
			ypred = generate_predicted_states(self.ybuf)
			self.xbuf.pop()
			self.ybuf.pop()
			self.tbuf.pop()
			self.pub_predicted.publish([xpred, ypred])		

if __name__ == '__main__':
	try:
		MovingAverage()
	except rospy.ROSInterruptException:
		pass
