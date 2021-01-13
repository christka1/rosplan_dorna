#!/usr/bin/env python
import json
from std_msgs.msg import String


def gripper_closed(msg, params):

    state_msg = json.loads(msg.data)
    return state_msg["gripper"]["closed"]


def gripper_open(msg, params):

    state_msg = json.loads(msg.data)
    return not state_msg["gripper"]["closed"]


def robot_at(msg, params):

    state_msg = json.loads(msg.data)
    ret_value = []

    for robot in params[0]:

        if robot == "scanner":
            act_robot = "r1"
        elif robot == "conv":
            act_robot = "r3"

        act_pos = state_msg[act_robot]["act_pos"]

        for location in params[1]:

            #act_location = "NA"

            if robot == "scanner":
                if location == "start_location":
                    act_location = "pre_take"
                elif location == "shared_location":
                    act_location = "leave"
                elif location == "scan_location":
                    act_location = "scan"
                elif location == "take_location":
                    act_location = "NA"
                elif location == "remove_location":
                    act_location = "NA"
                elif location == "red_location":
                    act_location = "take1"
                elif location == "green_location":
                    act_location = "take2"
                elif location == "blue_location":
                    act_location = "take3"
                else:
                    act_location = "unknown"

            elif robot == "conv":
                if location == "start_location":
                    act_location = "pre_take"
                elif location == "shared_location":
                    act_location = "leave"
                elif location == "scan_location":
                    act_location = "NA"
                elif location == "take_location":
                    act_location = "take2"
                elif location == "remove_location":
                    act_location = "take1"
                elif location == "red_location":
                    act_location = "NA"
                elif location == "green_location":
                    act_location = "NA"
                elif location == "blue_location":
                    act_location = "NA"
                else:
                    act_location = "unknown"

            ret_value.append((robot + ":" + location, act_pos == act_location))
    
    return ret_value

def holding(msg, params):

    state_msg = json.loads(msg.data)
    
    return state_msg["gripper"]["part_sensor"]

def not_holding(msg, params):

    state_msg = json.loads(msg.data)
    
    return not state_msg["gripper"]["part_sensor"]
