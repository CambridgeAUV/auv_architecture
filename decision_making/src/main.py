#!/usr/bin/env python

import rospy
import std_msgs

class DecisionMaking():

    def __init__(self):

        self.river_side_data = []
        
        rospy.init_node('decision_making')
        self.yaw_publisher = rospy.Publisher('yaw_demand', std_msgs.msg.String, queue_size = 10)
        rospy.Subscriber('river_sides', std_msgs.msg.Float32MultiArray, self.set_river_side_data)

        self.rate = rospy.Rate(10)

    def set_river_side_data(self, data):
        if not(len(data.data) == 0):
            self.river_side_data = data.data
            

    def run(self):
        while not rospy.is_shutdown():
            print "Decision making river side data: "
            print self.river_side_data

            self.rate.sleep()


if __name__ == '__main__':
    decision_making = DecisionMaking()
    try:
        decision_making.run()
    except rospy.ROSInterruptException:
        pass
