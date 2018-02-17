#!/usr/bin/env python
import rospy
import json
from pid import PIDControl
import std_msgs

class Controller():

    def __init__(self):

        self.euler_angles = [0,0,0]
        self.external_demands = [0,0,0,0,0,0]
        self.motor_demands = [0,0,0,0,0,0]
        self.depth_demand = 0
        self.depth_real_value = 0
        self.yaw_demand = 0
        self.pitch_demand = 0
        self.roll_demand = 0

        rospy.init_node('cauv_controller')
        self.pub = rospy.Publisher('motor_demand', std_msgs.msg.String, queue_size = 5)
        rospy.Subscriber("imu_euler_angles", std_msgs.msg.String, self.euler_angle_callback)
        rospy.Subscriber("external_demands", std_msgs.msg.String, self.external_demand_callback)
        rospy.Subscriber("depth_demand", std_msgs.msg.String, self.set_depth_demand)
        rospy.Subscriber("depth_value", std_msgs.msg.String, self.set_depth_real_value)
        rospy.Subscriber("pitch_demand", std_msgs.msg.String, self.set_pitch_demand)
        rospy.Subscriber("yaw_demand", std_msgs.msg.String, self.set_yaw_demand)
        rospy.Subscriber("roll_demand", std_msgs.msg.String, self.set_roll_demand)

        self.rate = rospy.Rate(10) # hz
        self.yaw_pid = PIDControl(0, 0, 0, True)
        self.pitch_pid = PIDControl(0, 0, 0, True)
        self.roll_pid = PIDControl(0, 0, 0, True)
        self.depth_pid = PIDControl(0, 0, 0, False)

    def set_depth_real_value(self, data):
        self.depth_real_value = data.data

    def set_depth_demand(self, data):
        self.depth_demand = data.data

    def set_pitch_demand(self, data):
        self.pitch_demand = data.data

    def set_yaw_demand(self, data):
        self.yaw_demand = data.data

    def set_roll_demand(self, data):
        self.row_demand = data.data

    def euler_angle_callback(self, data):
        print(data)
        # TODO Fetch the appropriate angles from the data message

    def external_demand_callback(self, data):
        print(data)
        # TODO Do something with the data
        


    def add_external_demands(self):
        for i in range(6):
            self.motor_demands[i] += self.external_demands[i]


    def run(self):
        while not rospy.is_shutdown():

            self.motor_demands = [0,0,0,0,0,0]

            
            # Need some way of converting output of pid to motor controls here
            # motor_demand_msg = MotorDemand()
            # motor_demand_msg.motor0 = self.motor_demands[0]
            # motor_demand_msg.motor1 = self.motor_demands[0]
            # motor_demand_msg.motor2 = self.motor_demands[0]
            # motor_demand_msg.motor3 = self.motor_demands[0]
            # motor_demand_msg.motor4 = self.motor_demands[0]
            # motor_demand_msg.motor5 = self.motor_demands[0]

            # rospy.loginfo(motor_demand_msg)
            # pub.publish(motor_demand_msg)
            self.rate.sleep()

if __name__ == '__main__':
    pid_controller = Controller()
    try:
        pid_controller.run()
    except rospy.ROSInterruptException:
        pass
