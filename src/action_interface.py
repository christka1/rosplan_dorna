#!/usr/bin/env python

import rospy
import roslib
import json
from std_msgs.msg import String
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import ActionFeedback
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue


class myparser():

    def __init__(self):

        rospy.init_node('myparser', anonymous=False)

        self.feedback_pub = rospy.Publisher('/rosplan_plan_dispatcher/action_feedback', ActionFeedback, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd', String, queue_size=10)

        self.pub_rate = rospy.Rate(10)

        self.feedback_msg = ActionFeedback()
        self.cmd_msg = String()

        self.r1_pos = "pre_take"
        self.r3_pos = "pre_take"
        self.blue_light = False
        self.do_scan = False
        self.gripper_close = False
        self.make_cube = False
        self.remove_cube = False

        self.task_started = False
        self.task_done = False
        self.wait_for_gripper = False
        self.read_gripper = False

        rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch', ActionDispatch, self.action_callback)
        rospy.Subscriber('/state', String, self.state_callback)

        rospy.sleep(2.)

        self.main()

    def action_callback(self, msg):

        #print(msg)
        #print("Task_started: {}".format(self.task_started))
        #print("Task_done: {}".format(self.task_done))

        #if not self.task_started and not self.task_done:

        print("Starting task {}".format(msg.name))

        self.feedback_msg.action_id = msg.action_id

        if msg.name == "move_conveyor":
            to = msg.parameters[1].value
            robot = msg.parameters[2].value
            cube = msg.parameters[3].value

            if to == "shared_location":
                self.r3_pos = "leave"
            
            elif to == "remove_location":
                self.r3_pos = "take1"

            self.update_knowledge('holding', ['robot', 'cube'], [robot, cube], False)
            self.update_knowledge('not_holding', ['robot', 'cube'], [robot, cube], True)
            self.update_knowledge('cube_at', ['cube', 'loc'], [cube, to], True)

        elif msg.name == "move_robot":
            robot = msg.parameters[0].value
            to = msg.parameters[2].value

            if robot == "scanner" and to == "scan_location":
                self.r1_pos = "scan"
            elif robot == "scanner" and to == "shared_location":
                self.r1_pos = "leave"
            elif robot == "scanner" and to == "red_location":
                self.r1_pos = "take1"
            elif robot == "scanner" and to == "green_location":
                self.r1_pos = "take2"
            elif robot == "scanner" and to == "blue_location":
                self.r1_pos = "take3"
            elif robot == "conv" and to == "shared_location":
                self.r3_pos = "leave"
            elif robot == "conv" and to == "start_location":
                self.r3_pos = "pre_take"
            elif robot == "conv" and to == "remove_location":
                self.r3_pos = "take1"
            elif robot == "conv" and to == "take_location":
                self.r3_pos = "take2"

        elif msg.name == "pick-up":

            self.gripper_close = True

            # The predicate updates are being done in the state_callback method

            self.wait_for_gripper = True

        elif msg.name == "open_gripper":

            self.gripper_close = False

        elif msg.name == "drop":
            cube = msg.parameters[1].value
            loc = msg.parameters[2].value
            
            self.gripper_close = False

            self.update_knowledge('cube_at', ['cube', 'loc'], [cube, loc], True)

        elif msg.name == "scan":

            self.blue_light = True
            self.do_scan = True

            # The predicate updates are being done in the state_callback method

        elif msg.name == "make_cube":
            cube = msg.parameters[0].value
            loc = msg.parameters[1].value
            robot = msg.parameters[2].value

            self.r3_pos = "after_homing"
            rospy.sleep(1.)
            self.make_cube = True
            rospy.sleep(1.)
            self.r3_pos = "pre_take"
            self.make_cube = False
            rospy.sleep(3.)

            self.update_knowledge('no_cube_exist', [], [], False)
            self.update_knowledge('holding', ['robot', 'cube'], [robot, cube], True)
            self.update_knowledge('not_holding', ['robot', 'cube'], [robot, cube], False)
            self.update_knowledge('cube_removed', [], [], False)

            self.task_done = True
            self.feedback_msg.status = "action achieved"

        elif msg.name == "prepare_removal":
            robot = msg.parameters[0].value
            cube = msg.parameters[1].value
            shared_loc = msg.parameters[2].value

            self.update_knowledge('cube_at', ['cube', 'loc'], [cube, shared_loc], False)
            self.update_knowledge('holding', ['robot', 'cube'], [robot, cube], True)
            self.update_knowledge('not_holding', ['robot', 'cube'], [robot, cube], False)

            self.task_done = True
            self.feedback_msg.status = "action achieved"

        elif msg.name == "remove_cube":
            cube = msg.parameters[0].value
            loc = msg.parameters[1].value

            self.remove_cube = True

            self.update_knowledge('cube_at', ['cube', 'loc'], [cube, loc], False)
            self.update_knowledge('no_cube_exist', [], [], True)
            self.update_knowledge('scanned', [], [], False)
            self.update_knowledge('cube_removed', [], [], True)

        elif msg.name == "store_cube":
            cube = msg.parameters[0].value
            loc = msg.parameters[1].value

            self.update_knowledge('cube_at', ['cube', 'loc'], [cube, loc], False)
            self.update_knowledge('cube_stored', ['cube', 'loc'], [cube, loc], True)
            self.update_knowledge('no_cube_exist', [], [], True)
            self.update_knowledge('scanned', [], [], False)

            self.task_done = True
            self.feedback_msg.status = "action achieved"

        '''
        elif msg.name == "get_cube":
            cube = msg.parameters[0].value
            robot = msg.parameters[1].value
            loc = msg.parameters[3].value

            if to == "start_location":
                self.r3_pos = "pre_take"

            self.update_knowledge('cube_at', ['cube', 'loc'], [cube, loc], False)
        '''

        self.task_started = True

    def state_callback(self, msg):

        state_msg = json.loads(msg.data)
        r1_act_pos = state_msg["r1"]["act_pos"]
        r3_act_pos = state_msg["r3"]["act_pos"]
        control_box_blue_light = state_msg["control_box"]["blue_light_on"]
        camera_scanning = state_msg["camera"]["scanning"]
        camera_done = state_msg["camera"]["done"]
        camera_result = state_msg["camera"]["result"]
        gripper_closed = state_msg["gripper"]["closed"]
        gripper_sensor = state_msg["gripper"]["part_sensor"]
        cubes_sensor = state_msg["cubes"]["sensor"]

        if self.task_started and not self.task_done:

            if camera_done:
                self.do_scan = False
                self.blue_light = False

                self.update_knowledge('result', [], [], True, camera_result, KnowledgeItem.FUNCTION)
                self.update_knowledge('scanned', [], [], True)

            if not cubes_sensor:
                self.remove_cube = False

            expected_state = {
                "r1": {"act_pos": self.r1_pos},
                "r3": {"act_pos": self.r3_pos},
                "control_box": {"blue_light_on": self.blue_light},
                "camera": {"done": self.do_scan,
                        "scanning": camera_scanning,
                        "result": camera_result},
                "gripper": {"closed": self.gripper_close,
                            "part_sensor": self.gripper_close},
                "cubes": {"sensor": cubes_sensor}
            }

            expected_state_json = json.loads(json.dumps(expected_state))

            if expected_state_json == state_msg:
                self.task_done = True
                self.feedback_msg.status = "action achieved"

        if gripper_closed and self.read_gripper:
            self.read_gripper = False

            if gripper_sensor:
                self.update_knowledge('cube_at', ['cube', 'loc'], ['cube', "shared_location"], False)
            else:
                self.task_done = True
                self.feedback_msg.status = "action failed"

    def update_knowledge(self, attribute_name, keys, values, is_true, function_value=0.0, knowledge_type=KnowledgeItem.FACT):

        msg = KnowledgeItem()
        msg.knowledge_type = knowledge_type
        msg.instance_type = ''
        msg.instance_name = ''
        msg.attribute_name = attribute_name
        for key, value in zip(keys, values):
            msg.values.append(KeyValue(key, value))
        msg.function_value = function_value
        msg.is_negative = not is_true


        update_kb = rospy.ServiceProxy('rosplan_knowledge_base/update', KnowledgeUpdateService)
        response = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, msg)

    def main(self):

        while not rospy.is_shutdown():

            message = {
                "r1": {"ref_pos": self.r1_pos},
                "r3": {"ref_pos": self.r3_pos},
                "control_box": {"blue_light": self.blue_light},
                "camera": {"do_scan": self.do_scan},
                "gripper": {"close": self.gripper_close},
                "cubes": {"make_cube": self.make_cube,
                          "remove_cube": self.remove_cube}
            }
            json_message = json.dumps(message)
            if not self.cmd_msg.data == json_message:
                pub_cmd = True
                self.cmd_msg.data = json_message

                while pub_cmd:
                    connections = self.cmd_pub.get_num_connections()
                    if connections > 0:
                        self.cmd_pub.publish(self.cmd_msg)
                        pub_cmd = False
                        if self.wait_for_gripper:
                            rospy.sleep(1.)
                            self.wait_for_gripper = False
                            self.read_gripper = True
                    else:
                        self.pub_rate.sleep()

            if self.task_started and self.task_done and not self.read_gripper:
                pub_feedback = True

                while pub_feedback:
                    connections = self.feedback_pub.get_num_connections()
                    if connections > 0:
                        pub_feedback = False
                        self.task_started = False
                        self.task_done = False
                        self.feedback_pub.publish(self.feedback_msg)
                        self.feedback_msg.status = ""
                    else:
                        self.pub_rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    try:
        myparser()
    except rospy.ROSInterruptException:
        pass
