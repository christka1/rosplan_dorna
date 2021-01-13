#!/usr/bin/env python

import rospy
import roslib
import json
from std_msgs.msg import String


class mylistener():

    def __init__(self):

        rospy.init_node('mylistener', anonymous=False)

        rospy.Subscriber('state', String, self.cmd_callback)

        rospy.sleep(2)

        self.main()

    def cmd_callback(self, msg):

        json_msg = json.loads(msg.data)
        r1_act_pos = json_msg["r1"]["act_pos"]
        r3_act_pos = json_msg["r3"]["act_pos"]
        control_box_blue_light = json_msg["control_box"]["blue_light_on"]
        camera_scanning = json_msg["camera"]["scanning"]
        camera_done = json_msg["camera"]["done"]
        camera_result = json_msg["camera"]["result"]
        gripper_closed = json_msg["gripper"]["closed"]
        gripper_sensor = json_msg["gripper"]["part_sensor"]
        cubes_sensor = json_msg["cubes"]["sensor"]

        print('r1 act pos: \t{}'.format(r1_act_pos))
        print('r3 act pos: \t{}'.format(r3_act_pos))
        print('Blue light: \t{}'.format(control_box_blue_light))
        print('Scanning: \t{}'.format(camera_scanning))
        print('Camera done: \t{}'.format(camera_done))
        print('Result: \t{}'.format(camera_result))
        print('Gripper closed: {}'.format(gripper_closed))
        print('Gripper sensor: {}'.format(gripper_sensor))
        print('Cubes sensor: \t{}'.format(cubes_sensor))
        print('-------------------------')

    def main(self):

        rospy.spin()


if __name__ == '__main__':
    try:
        mylistener()
    except rospy.ROSInterruptException:
        pass
