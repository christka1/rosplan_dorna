#!/usr/bin/env python

import rospy
import roslib
import json
from std_msgs.msg import String


class mytalker():

    def __init__(self):

        rospy.init_node('mytalker', anonymous=False)

        self.main_pub = rospy.Publisher('cmd', String, queue_size=10)

        self.pub_rate = rospy.Rate(1000)

        self.msg = String()

        rospy.sleep(2)

        self.main()

    def main(self):

        while not rospy.is_shutdown():

            r1_pos = "pre_take"
            r3_pos = "leave"
            blue_light = False
            do_scan = False
            gripper_close = False
            make_cube = True
            remove_cube = False

            message = {
                "r1": {"ref_pos": r1_pos},
                "r3": {"ref_pos": r3_pos},
                "control_box": {"blue_light": blue_light},
                "camera": {"do_scan": do_scan},
                "gripper": {"close": gripper_close},
                "cubes": {"make_cube": make_cube,
                "remove_cube": remove_cube}
            }
            json_message = json.dumps(message)
            self.msg.data = json_message

            self.main_pub.publish(self.msg)
            self.pub_rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    try:
        mytalker()
    except rospy.ROSInterruptException:
        pass
