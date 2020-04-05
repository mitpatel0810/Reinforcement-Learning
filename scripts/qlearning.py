#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Abhyudaya Srinet"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
from std_msgs.msg import String
import problem
import random
import matplotlib.pyplot as plt
import json
import os
import math
import argparse
from action_server import RobotActionsServer

parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
# parser.add_argument('-task', help="Task to execute:\n1. Q learning on sample trajectories\n2. Q learning without pruned actions\n3. Q learning with pruned actions", metavar='1', action='store', dest='task', default="1", type=int)
# parser.add_argument("-sample", metavar="1", dest='sample', default='1', help="which trajectory to evaluate (with task 1)", type=int)
parser.add_argument('-episodes', help="Number of episodes to run (with task 2 & 3)", metavar='1', action='store', dest='episodes', default="1", type=int)
parser.add_argument('-headless', help='1 when running in the headless mode, 0 when running with gazebo', metavar='1', action='store', dest='headless', default=1, type=int)


class QLearning:

    def __init__(self,  headless=1, episodes=1):
        rospy.init_node('qlearning', anonymous=True)
        root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
        
        self.books_json_file = root_path + "/books.json"
        self.books = json.load(open(self.books_json_file))
        self.a_ser = RobotActionsServer(self.books, root_path, args.headless ==1)
        self.helper = problem.Helper()
        self.helper.reset_world()
        self.headless = headless
        self.alpha = 0.2
        self.gamma = 0.9

        q_values = self.qlearning(episodes)

        with open(root_path + "/q_values.json", "w") as fout:
            json.dump(q_values, fout)

    def qlearning(self, episodes):
        
        q_values = {}

        # Your code here   
       
        gamma = self.gamma
        alpha = self.alpha
        episodes_list = []
        cum_rewards_list = []
        for i in range(1, episodes+1):
            
            self.helper.reset_world()
            reward = 0

            current_state = self.helper.get_current_state()
            print(current_state)
            
            if str(current_state) not in q_values:
                q_values[str(current_state)]={}
                all_actions = self.helper.get_all_actions()
                for a in all_actions:
                    if a not in q_values[str(current_state)]:
                        q_values[str(current_state)][a]=0
            steps = 0
            while not self.helper.is_terminal_state(current_state):
                
                current_actions = self.helper.get_all_actions()
                x_traffic_light = current_state['traffic_stop']['x']
                y_traffic_light = current_state['traffic_stop']['y']
                traffic_direction = current_state['traffic_stop']['orientation']

                # code changes
                action_list = []
                x_robot = current_state['robot']['x']
                y_robot = current_state['robot']['y']
                robot_direction = current_state['robot']['orientation']
                # case 1
                if robot_direction == 'EAST' and x_traffic_light-x_robot==0.5:
                    if abs(y_robot-y_traffic_light) <= 1.0:
                        for action in current_actions:
                            if action == 'moveF':
                                continue
                            else:
                                action_list.append(action)
                        current_actions = action_list

                # case 2
                if x_robot==x_traffic_light:
                    current_actions = ['moveF']

                # case 3
                if robot_direction == 'WEST' and x_robot-x_traffic_light ==0.5:
                    for action in current_actions:
                            if action == 'moveF':
                                continue
                            else:
                                action_list.append(action)
                    current_actions = action_list

                print(current_state)
                epsilon = max(0.05, 0.7 - (0.05 * i))
                choice = random.uniform(0, 1)
                if choice < epsilon:
                    # get random action
                    random_action = current_actions[random.randint(0, len(current_actions)-1)]
                else:
                    # get best action
                    best_value = max(q_values[str(current_state)].values())
                    for a in q_values[str(current_state)]:
                        if q_values[str(current_state)][str(a)]==best_value:
                            random_action = a
                            break
                    # random_action = sorted(q_values[str(current_state)].items(), key =lambda kv:(kv[1], kv[0]))[-1][0]
                    # random_action = max([(key, value) for key, value in q_values[str(current_state)].items()])[0]
                # action_name = random_action.split(' ')[0]
                # action_name_list = random_action.split(' ')
                
                action_params = {}
                # action_len = len(action_name_list)
                # if action_len==2:
                action_params['book_name'] = 'book_1'
                # elif action_len==3:
                #     action_params['book_name'] = action_name_list[1]
                #     action_params['bin_name'] = action_name_list[2]
                if 'pick' in random_action:
                    flag, next_state = self.helper.execute_action(random_action, action_params)
                else:
                    flag, next_state = self.helper.execute_action(random_action, {})
                

                if y_traffic_light == 0.0 and traffic_direction=='SOUTH':
                    traffic_direction="NORTH"

                if y_traffic_light == 3.0 and traffic_direction=='NORTH':
                    traffic_direction= "SOUTH"

                if traffic_direction=="NORTH":
                    y_traffic_light = y_traffic_light + 0.5
                    traffic_stop_pos = [x_traffic_light,y_traffic_light]
                    self.a_ser.change_gazebo_state('traffic_stop',traffic_stop_pos)

                if traffic_direction =='SOUTH':
                    y_traffic_light = y_traffic_light - 0.5
                    traffic_stop_pos = [x_traffic_light,y_traffic_light]
                    self.a_ser.change_gazebo_state('traffic_stop',traffic_stop_pos)
                    
                next_state['traffic_stop']['y']=y_traffic_light
                next_state['traffic_stop']['orientation'] = traffic_direction
                # print(next_state)
                if str(next_state) not in q_values:
                    q_values[str(next_state)] = {}
                    all_actions = self.helper.get_all_actions()
                    for a in all_actions:
                        if a not in q_values[str(next_state)]:
                            q_values[str(next_state)][a] = 0

                # calculating cumulative reward
                intermediate_reward = math.pow(gamma, steps)*(self.helper.get_reward(current_state, random_action, next_state))
                reward = reward + intermediate_reward

                # updating q values
                intermediate_value = (1 - alpha) * q_values[str(current_state)][random_action]
                # max_q_value = sorted(q_values[str(next_state)].items(), key =lambda kv:(kv[1], kv[0]))[-1][-1]
                # max_q_value = max([(key, value) for key, value in q_values[str(next_state)].items()])[1]
                max_q_value = max(q_values[str(next_state)].values())
                q_values[str(current_state)][random_action] = intermediate_value + alpha * (
                        self.helper.get_reward(current_state,random_action,next_state) + (gamma * max_q_value))
                current_state = next_state
                print(steps)
                steps += 1
            
            print('episode',i)
            print('steps',steps)
            print('reward',reward)
            # plotting graph
            episodes_list.append(i)
            cum_rewards_list.append(reward)
        plt.plot(episodes_list, cum_rewards_list)
        plt.xlabel('Number of Episodes')
        plt.ylabel('Cumulative reward')
        plt.title('Plot for Task 2 with complete action list')
        plt.show()
        return q_values


if __name__ == "__main__":

    args = parser.parse_args()
    QLearning(headless=args.headless, episodes=args.episodes)
