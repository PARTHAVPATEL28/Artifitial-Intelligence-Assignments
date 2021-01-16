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
import json
import os
import argparse
import random


parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument('-task', help="Task to execute:\n1. Q learning on sample trajectories\n2. Q learning without pruned actions\n3. Q learning with pruned actions", metavar='1', action='store', dest='task', default="1", type=int)
parser.add_argument("-sample", metavar="1", dest='sample', default='1', help="which trajectory to evaluate (with task 1)", type=int)
parser.add_argument('-episodes', help="Number of episodes to run (with task 2 & 3)", metavar='1', action='store', dest='episodes', default="1", type=int)
parser.add_argument('-headless', help='1 when running in the headless mode, 0 when running with gazebo', metavar='1', action='store', dest='headless', default=1, type=int)


class QLearning:

    def __init__(self, task, headless=1, sample=1, episodes=1):
        rospy.init_node('qlearning', anonymous=True)
        root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
        
        self.books_json_file = root_path + "/books.json"
        self.books = json.load(open(self.books_json_file))
        self.helper = problem.Helper()
        self.helper.reset_world()
        self.headless = headless
        self.alpha = 0.3
        self.gamma = 0.9

        if(task == 1):
            trajectories_json_file = root_path + "/trajectories{}.json".format(sample)
            q_values = self.task1(trajectories_json_file)
        elif(task == 2):
            q_values = self.task2(episodes)
        elif(task == 3):
            q_values = self.task3(episodes)

        with open(root_path + "/q_values.json", "w") as fout:
            json.dump(q_values, fout)

    def task3(self, episodes):

        q_values = {}
        actions = self.helper.get_all_actions()
        books_location = []
        bins_location = []
        for key in self.books['books'].keys():
            books_location.extend(self.books['books'][key]['load_loc'])

        for key in self.books['bins'].keys():
            bins_location.extend(self.books['bins'][key]['load_loc'])

        init_state = self.helper.get_current_state()
        init_str_state = json.dumps(init_state)
        q_values[init_str_state] = {}
        prun_action_list=[]
        for action in actions:
            if len(action.split())==2:
                if [init_state['robot']['x'], init_state['robot']['y']] not in books_location:
                    pass
                else:
                    prun_action_list.append(action)

            elif len(action.split())==3:
                if [init_state['robot']['x'], init_state['robot']['y']] not in bins_location:
                    pass
                else:
                    prun_action_list.append(action)

            else:
                prun_action_list.append(action)

        for action in prun_action_list:
            q_values[init_str_state][action] = 0



        for episode in range(1, episodes + 1):
            self.helper.reset_world()
            step = 0
            cur_state = init_state
            cumm_reward = 0
            epsilon = max(0.05, 0.7 - 0.05 * episode)

            while not self.helper.is_terminal_state(cur_state):
                cur_str_state = json.dumps(cur_state)

                new_actions=[]
                for action in q_values[cur_str_state]:
                    new_actions.append(action)


                if random.uniform(0, 1) < epsilon:
                    cur_action = random.choice(new_actions)


                else:
                    cur_action = max(q_values[cur_str_state], key=q_values[cur_str_state].get)

                l = cur_action.split()
                ac = l[0].split('_')
                action_params = {}
                if ac[1] == 'pick':
                    action_params['book_name'] = l[1]
                elif ac[1] == 'place':
                    action_params['book_name'] = l[1]
                    action_params['bin_name'] = l[2]
                elif ac[1] not in ['pick', 'place']:
                    action_params = {}
                else:
                    continue

                success, next_state = self.helper.execute_action(l[0], action_params)
                next_str_state = json.dumps(next_state)
                if next_str_state not in q_values.keys():
                    q_values[next_str_state] = {}
                    prun_action_list = []
                    for action in actions:
                        if len(action.split()) == 2:
                            if [next_state['robot']['x'], next_state['robot']['y']] not in books_location:
                                pass
                            else:
                                prun_action_list.append(action)

                        elif len(action.split()) == 3:
                            if [next_state['robot']['x'], next_state['robot']['y']] not in bins_location:
                                pass
                            else:
                                prun_action_list.append(action)

                        else:
                            prun_action_list.append(action)
                    for action in prun_action_list:
                        q_values[next_str_state][action] = 0


                reward = self.helper.get_reward(cur_state, l[0], next_state)
                q_values[cur_str_state][cur_action] = (1 - self.alpha) * q_values[cur_str_state][
                    cur_action] + self.alpha * (reward + self.gamma * max(q_values[next_str_state].values()))
                cumm_reward = cumm_reward + pow(self.gamma, step) * reward

                cur_state = next_state
                step += 1

        return q_values

    def task2(self, episodes):
        
        q_values = {}
        actions=self.helper.get_all_actions()

        init_state = self.helper.get_current_state()
        init_str_state = json.dumps(init_state)
        q_values[init_str_state] = {}
        for action in actions:
            q_values[init_str_state][action] = 0


        for episode in range(1, episodes+1):
            self.helper.reset_world()
            step=0
            cur_state=init_state
            cumm_reward = 0
            epsilon = max(0.05, 0.7 - 0.05 * episode)


            while not self.helper.is_terminal_state(cur_state):
                cur_str_state = json.dumps(cur_state)

                if random.uniform(0, 1) < epsilon:
                    cur_action = random.choice(actions)

                else:
                    cur_action=max(q_values[cur_str_state], key=q_values[cur_str_state].get)


                l = cur_action.split()
                ac = l[0].split('_')
                action_params={}

                if ac[1] == 'pick':
                    action_params['book_name']=l[1]
                elif ac[1]=='place':
                    action_params['book_name']=l[1]
                    action_params['bin_name']=l[2]

                success, next_state = self.helper.execute_action(l[0], action_params)
                next_str_state=json.dumps(next_state)
                if next_str_state not in q_values.keys():
                    q_values[next_str_state] = {}
                    for action in actions:
                        q_values[next_str_state][action] = 0

                reward = self.helper.get_reward(cur_state, l[0], next_state)
                q_values[cur_str_state][cur_action] = (1 - self.alpha)*q_values[cur_str_state][cur_action] + self.alpha * (reward + self.gamma * max(q_values[next_str_state].values()))
                cumm_reward = cumm_reward + pow(self.gamma, step) * reward

                cur_state=next_state
                step+=1

        return q_values

    def task1(self, trajectories_json_file):

        q_values = {}
        h=problem.Helper()
        actions = h.get_all_actions()
        reward=1
        with open(trajectories_json_file,'r') as file:
            data = json.load(file)
        for index in data:
            q_values[index['state']]={}
            for action in actions:
                q_values[index['state']][action]=0
        while reward<=1:
            for i in range(len(data)-1):
                q_values[data[i]['state']][data[i]['action']]=q_values[data[i]['state']][data[i]['action']]*(1-self.alpha)+self.alpha*(data[i]['reward']+self.gamma*max(q_values[data[i+1]['state']].values()))
            reward+=1
        return q_values



if __name__ == "__main__":

    args = parser.parse_args()

    if args.task == 1:
        QLearning(args.task, headless=args.headless, sample=args.sample)
    elif args.task == 2 or args.task == 3:
        QLearning(args.task, headless=args.headless, episodes=args.episodes)
