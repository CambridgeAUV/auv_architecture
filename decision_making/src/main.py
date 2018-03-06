#!/usr/bin/env python

import rospy
import std_msgs
import cauv_msgs.msg

class DecisionMaking():

    def __init__(self):

        self.river_side_data = []
        self.current_yaw = 0
        self.desired_yaw = 0

        # These are hysteresis boundaries (radians) for keeping the river sides vertical (on the sonar image) 
        self.vertical_river_angle_lower_thresh = 0.1
        self.vertical_river_angle_higher_thresh = 0.2
        self.use_upper_bound = False #False if using lower bound, true if using upper bound
        
        rospy.init_node('decision_making')
        self.yaw_publisher = rospy.Publisher('yaw_demand', std_msgs.msg.String, queue_size = 10)
        rospy.Subscriber('river_sides', std_msgs.msg.Float32MultiArray, self.river_sides_callback)
        rospy.Subscriber('imu_data', cauv_msgs.msg.imu, self.imu_data_callback)

        self.rate = rospy.Rate(10)

    def river_sides_callback(self, data):
        if not(len(data.data) == 0):
            self.river_side_data = data.data

    def imu_data_callback(self, data):
        print "imu_data_callback received data: ", "roll: ", data.roll, " pitch: ", data.pitch, " yaw: ", data.yaw
        self.current_yaw = data.yaw


    def update_desired_yaw(self):
        if not(len(self.river_side_data) == 0):
            avg_river_side_angle = (self.river_side_data[2] + self.river_side_data[3]) / 2            

            if self.use_upper_bound:
                if abs(avg_river_side_angle) > self.vertical_river_angle_higher_thresh:
                    self.use_upper_bound = False
                    self.desired_yaw = self.current_yaw  - avg_river_side_angle
            else:
                if abs(avg_river_side_angle) > self.vertical_river_angle_lower_thresh:
                    self.use_upper_bound = True
                    self.desired_yaw = self.current_yaw - avg_river_side_angle

            

    def run(self):
        while not rospy.is_shutdown():
            self.update_desired_yaw() 
            self.yaw_publisher.publish(str(self.desired_yaw)) #Why can't we just use a Float32 as opposed to a string? 

            self.rate.sleep()


if __name__ == '__main__':
    print "Decision making running"

    decision_making = DecisionMaking()
    try:
        decision_making.run()
    except rospy.ROSInterruptException:
        pass
