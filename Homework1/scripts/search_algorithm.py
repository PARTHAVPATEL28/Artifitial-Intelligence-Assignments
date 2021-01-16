#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
import time

rospy.init_node("search_algorithms")
publisher = rospy.Publisher("/actions", String, queue_size=10)
parser = argparse.ArgumentParser()
parser.add_argument('-a', help="Please mention algorithm to use. Possible arguments = {bfs, ucs, gbfs, astar}. Default value is bfs.", metavar='bfs', action='store', dest='algorithm', default="bfs", type=str)
parser.add_argument('-c', help="Use custom heuristic function. No value needed.", action='store_true', dest='custom_heuristic')


def bfs(use_custom_heuristic):
    '''
    Perform BFS to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)
    import Queue as queue
    visited=[]
    q3=queue.Queue()
    if not helper.is_goal_state(init_state):
        visited.append(init_state)
        for key,values in state_dictionary.items():
            list_1=[]
            temp=[]
            list_1.extend(key.split())
            temp.append(list_1)
            temp.append(values[0])
            q3.put(temp)
        while not q3.empty():
            cur_node=q3.get()
            if cur_node[1].x<0 or cur_node[1].y<0:
                continue
            elif helper.is_goal_state(cur_node[1]):
                action_list=cur_node[0]
                break
            elif cur_node[1] in visited:
                continue
            else:
                temp = helper.get_successor(cur_node[1])
                visited.append(cur_node[1])
                for key,values in temp.items():
                    list_1=[]
                    temp=[]
                    list_1.extend(cur_node[0])
                    list_1.extend(key.split())
                    temp.append(list_1)
                    temp.append(values[0])
                    q3.put(temp)
    return action_list


def ucs(use_custom_heuristic):
    '''
    Perform UCS to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)
    visited = []
    q3 = []
    entry_count = 0
    if not helper.is_goal_state(init_state):
        visited.append(init_state)
        for key, values in state_dictionary.items():
            list_1 = []
            temp = []
            list_1.extend(key.split())
            temp.append(list_1)
            temp.append(values[0])
            heapq.heappush(q3, (values[1], entry_count, temp))
            entry_count += 1
        while q3:
            cur_node = heapq.heappop(q3)
            if cur_node[2][1].x < 0 or cur_node[2][1].y < 0:
                continue
            elif helper.is_goal_state(cur_node[2][1]):
                action_list = cur_node[2][0]
                break
            elif cur_node[2][1] in visited:
                continue
            else:
                temp = helper.get_successor(cur_node[2][1])
                visited.append(cur_node[2][1])
                for key, values in temp.items():
                    cost = 0
                    list_1 = []
                    temp = []
                    cost = values[1] + cur_node[0]
                    list_1.extend(cur_node[2][0])
                    list_1.extend(key.split())
                    temp.append(list_1)
                    temp.append(values[0])
                    heapq.heappush(q3, (cost, entry_count, temp))
                    entry_count += 1

    return action_list
def gbfs(use_custom_heuristic):
    '''
    Perform GBFS to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)
    if not helper.is_goal_state(init_state):
        entry_count=0
        if use_custom_heuristic == True:
            visited = []
            q3 = []
            visited.append(init_state)
            for key, values in state_dictionary.items():
                list_1 = []
                temp = []
                heu_cost = cus_heu(values[0], goal_state)
                list_1.extend(key.split())
                temp.append(list_1)
                temp.append(values[0])
                heapq.heappush(q3, (heu_cost,entry_count,temp))
                entry_count+=1
            while q3:
                cur_node = heapq.heappop(q3)
                if cur_node[2][1].x < 0 or cur_node[2][1].y < 0:
                    continue
                elif helper.is_goal_state(cur_node[2][1]):
                    action_list = cur_node[2][0]
                    break
                elif cur_node[2][1] in visited:
                    continue

                else:
                    temp = helper.get_successor(cur_node[2][1])
                    visited.append(cur_node[2][1])
                    for key, values in temp.items():
                        list_1 = []
                        temp = []
                        heu_cost = cus_heu(values[0], goal_state)
                        list_1.extend(cur_node[2][0])
                        list_1.extend(key.split())
                        temp.append(list_1)
                        temp.append(values[0])
                        heapq.heappush(q3, (heu_cost,entry_count, temp))
                        entry_count+=1
        else:
            visited = []
            q3 = []
            visited.append(init_state)
            for key, values in state_dictionary.items():
                list_1 = []
                temp = []
                heu_cost = manhattan_heu(values[0], goal_state)
                list_1.extend(key.split())
                temp.append(list_1)
                temp.append(values[0])
                heapq.heappush(q3, (heu_cost, entry_count, temp))
                entry_count += 1
            while q3:
                cur_node = heapq.heappop(q3)
                if cur_node[2][1].x < 0 or cur_node[2][1].y < 0:
                    continue
                elif helper.is_goal_state(cur_node[2][1]):
                    action_list = cur_node[2][0]
                    break
                elif cur_node[2][1] in visited:
                    continue

                else:
                    temp = helper.get_successor(cur_node[2][1])
                    visited.append(cur_node[2][1])
                    for key, values in temp.items():
                        list_1 = []
                        temp = []
                        heu_cost = manhattan_heu(values[0], goal_state)
                        list_1.extend(cur_node[2][0])
                        list_1.extend(key.split())
                        temp.append(list_1)
                        temp.append(values[0])
                        heapq.heappush(q3, (heu_cost, entry_count, temp))
                        entry_count += 1

    return action_list
def cus_heu(cur_state,goal_state):
    import math
    if cur_state.x>=0 or cur_state.y>=0:
        if cur_state.orientation == 'NORTH' or cur_state.orientation == 'EAST':
            heu_val = (goal_state.x - cur_state.x) + (goal_state.y - cur_state.y) - 1
        else:
            heu_val = (goal_state.x - cur_state.x) + (goal_state.y - cur_state.y) + 1
    else:
        heu_val = (goal_state.x - cur_state.x) + (goal_state.y - cur_state.y) + 1

    return heu_val
def manhattan_heu(cur_state,goal_state):
    import math
    heu_val=(goal_state.x-cur_state.x)+(goal_state.y-cur_state.y)
    return heu_val

def astar(use_custom_heuristic):
    '''
    Perform A* search to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)
    if not helper.is_goal_state(init_state):
        entry_count=0
        if use_custom_heuristic==True:
            visited = []
            q3 = []
            visited.append(init_state)
            for key, values in state_dictionary.items():
                list_1 = []
                temp = []
                heu_cost = cus_heu(values[0], goal_state)
                fun_cost= heu_cost+values[1]
                list_1.extend(key.split())
                temp.append(list_1)
                temp.append(values[0])
                heapq.heappush(q3,(fun_cost,entry_count,temp,values[1]))
                entry_count+=1
            while q3:
                cur_node = heapq.heappop(q3)
                if cur_node[2][1].x < 0 or cur_node[2][1].y < 0:
                    continue
                elif helper.is_goal_state(cur_node[2][1]):
                    action_list = cur_node[2][0]
                    break
                elif cur_node[2][1] in visited:
                    continue
                else:
                    temp = helper.get_successor(cur_node[2][1])
                    visited.append(cur_node[2][1])
                    for key, values in temp.items():
                        list_1 = []
                        temp = []
                        heu_cost = cus_heu(values[0], goal_state)
                        cost = values[1] + cur_node[3]
                        fun_cost=heu_cost+cost
                        list_1.extend(cur_node[2][0])
                        list_1.extend(key.split())
                        temp.append(list_1)
                        temp.append(values[0])
                        heapq.heappush(q3,(fun_cost,entry_count, temp,cost))
                        entry_count+=1
        else:
            visited = []
            q3 = []
            visited.append(init_state)
            for key, values in state_dictionary.items():
                list_1 = []
                temp = []
                heu_cost = manhattan_heu(values[0], goal_state)
                fun_cost = heu_cost + values[1]
                list_1.extend(key.split())
                temp.append(list_1)
                temp.append(values[0])
                heapq.heappush(q3, (fun_cost, entry_count, temp, values[1]))
                entry_count += 1
            while q3:
                cur_node = heapq.heappop(q3)
                if cur_node[2][1].x < 0 or cur_node[2][1].y < 0:
                    continue
                elif helper.is_goal_state(cur_node[2][1]):
                    action_list = cur_node[2][0]
                    break
                elif cur_node[2][1] in visited:
                    continue
                else:
                    temp = helper.get_successor(cur_node[2][1])
                    visited.append(cur_node[2][1])
                    for key, values in temp.items():
                        list_1 = []
                        temp = []
                        heu_cost = manhattan_heu(values[0], goal_state)
                        cost = values[1] + cur_node[3]
                        fun_cost = heu_cost + cost
                        list_1.extend(cur_node[2][0])
                        list_1.extend(key.split())
                        temp.append(list_1)
                        temp.append(values[0])
                        heapq.heappush(q3, (fun_cost, entry_count, temp, cost))
                        entry_count += 1

    return action_list


def exec_action_list(action_list):
    '''
    publishes the list of actions to the publisher topic
    action_list: list of actions to execute
    '''
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))


if __name__ == "__main__":
    # DO NOT MODIFY BELOW CODE 
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print("Incorrect Algorithm name.")
        exit(1)
    if args.algorithm in ["bfs", "ucs"] and args.custom_heuristic == True:
        print ("Error: "+args.algorithm+" called with heuristic")
        exit(1)

    start_time = time.time()
    actions = algorithm(args.custom_heuristic)
    time_taken = time.time() - start_time
    print("Time Taken = " + str(time_taken))
    print("Plan = " + str(actions))
    exec_action_list(actions)
