#!/usr/bin/env python

import rospy
import sys
from std_srvs.srv import Empty
from rosplan_dispatch_msgs.srv import DispatchService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import GetAttributeService
from diagnostic_msgs.msg import KeyValue


class scenarios():

    def __init__(self):

            #rospy.init_node('scenarios', anonymous=False)

            #rospy.sleep(2)

            self.fail_limit = 5

            self.main()

    def dispatch(self):

        # TODO
        # Some pauses might be needed since the planning can fail even at easy tasks, but we still handle it,
        # assuming that this is because it is executed to fast

        goal_achieved = False
        fail_counter = 0

        while not goal_achieved and fail_counter < self.fail_limit:

            #print("Generating a Problem")
            problem_generation_server = rospy.ServiceProxy('rosplan_problem_interface/problem_generation_server', Empty)
            problem_generation_server()

            rospy.sleep(0.5)

            #print("Planning")
            planning_server = rospy.ServiceProxy('rosplan_planner_interface/planning_server', Empty)
            planning_server()

            rospy.sleep(0.5)

            print("Executing the generated Plan")
            parse_plan = rospy.ServiceProxy('rosplan_parsing_interface/parse_plan', Empty)
            parse_plan()

            rospy.sleep(0.5)

            dispatch_plan = rospy.ServiceProxy('rosplan_plan_dispatcher/dispatch_plan', DispatchService)
            response = dispatch_plan()
            goal_achieved = response.goal_achieved

            if goal_achieved:
                fail_counter = 0
            else:
                fail_counter += 1
                print("\033[93m" + "Failed, replanning" + "\033[0m")

        if goal_achieved:
            print("Goal achieved!")
        else:
            print("Goal not achieved, replanned 5 times in a row without success")

        return goal_achieved

    def update_goal(self, attribute_name, keys, values, is_true, 
    request=KnowledgeUpdateServiceRequest.ADD_GOAL, function_value=0.0, knowledge_type=KnowledgeItem.FACT):

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
        response = update_kb(request, msg)

    def get_result(self):

        predicate_name = "result"

        read_kb = rospy.ServiceProxy('rosplan_knowledge_base/state/functions', GetAttributeService)
        response = read_kb(predicate_name)

        result = int(response.attributes[0].function_value)
        return result

    def store_cube(self, location):

        self.update_goal('cube_stored', ['cube', 'loc'], ['cube', location], True, request=KnowledgeUpdateServiceRequest.ADD_GOAL)

        result = self.dispatch()

        self.update_goal('cube_stored', ['cube', 'loc'], ['cube', location], True, request=KnowledgeUpdateServiceRequest.REMOVE_GOAL)

        return result

    def main(self):

        goal_achieved = False
        fail_counter = 0
        fail_limit = 5

        try:
            scenario = sys.argv[1]
        except IndexError as error:
            sys.exit("Input a argument between 1-3. 1 = scan test, 2 = pick-up test, 3 = sorting test")

        #raw_input("Press Enter to add goal")

        if scenario == "1":

            # Add goal "scanned = True"
            self.update_goal('scanned', [], [], True)
            #raw_input("Goal 'scanned' added. Press Enter to continue")

            self.dispatch()

        elif scenario == "2":

            for i in range(5):

                # Add goal (holding scanner cube) = True
                self.update_goal('holding', ['robot', 'cube'], ['scanner', 'cube'], True, request=KnowledgeUpdateServiceRequest.ADD_GOAL)
                #raw_input("Goal 'holding scanner cube' added. Press Enter to continue")
                print("Goal 'holding scanner cube' added")

                if not self.dispatch():
                    break

                # Add goal (holding scanner cube) = False
                self.update_goal('holding', ['robot', 'cube'], ['scanner', 'cube'], True, request=KnowledgeUpdateServiceRequest.REMOVE_GOAL)
                self.update_goal('not_holding', ['robot', 'cube'], ['scanner', 'cube'], True, request=KnowledgeUpdateServiceRequest.ADD_GOAL)
                print("Holding removed, not_holding added")

                if not self.dispatch():
                    break

                self.update_goal('not_holding', ['robot', 'cube'], ['scanner', 'cube'], True, request=KnowledgeUpdateServiceRequest.REMOVE_GOAL)
                print("Not_holding removed")

        elif scenario == "3":

            red = False
            green = False
            blue = False

            continue_run = True
            
            while continue_run and not (red and green and blue):

                #raw_input("Hit Enter to get a new cube scanned")
                
                # Add goal "scanned = True"
                self.update_goal('scanned', [], [], True, request=KnowledgeUpdateServiceRequest.ADD_GOAL)

                continue_run = self.dispatch()

                self.update_goal('scanned', [], [], True, request=KnowledgeUpdateServiceRequest.REMOVE_GOAL)

                result = self.get_result()

                #raw_input("Hit Enter to do the correct thing with the cube")
                
                if result == 1 and not red:

                    continue_run = self.store_cube("red_location")

                    red = True

                elif result == 2 and not green:

                    continue_run = self.store_cube("green_location")

                    green = True

                elif result == 3 and not blue:

                    continue_run = self.store_cube("blue_location")

                    blue = True

                else:

                    self.update_goal('cube_removed', [], [], True, request=KnowledgeUpdateServiceRequest.ADD_GOAL)

                    continue_run = self.dispatch()

                    self.update_goal('cube_removed', [], [], True, request=KnowledgeUpdateServiceRequest.REMOVE_GOAL)
            

if __name__ == '__main__':
    try:
        scenarios()
    except rospy.ROSInterruptException:
        pass
