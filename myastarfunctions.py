#!/usr/bin/env python
# -*- coding: utf-8 -*-

import openravepy
import numpy as np
from Queue import PriorityQueue
if not __openravepy_build_doc__:
    from openravepy import *

step_size = np.array([0.35,0.35,np.pi/2])
pointsize = 5
handles = []
height = 0.1

def getcurrentconfig(robot):
    T_initial = robot.GetTransform()
    X_initial = T_initial[0][3]
    Y_initial = T_initial[1][3]
    height = T_initial[2][3]
    theta_initial = axisAngleFromRotationMatrix(T_initial[0:3][0:3])[2]
    return [X_initial, Y_initial, theta_initial], height


class Node:
    def __init__(self,config_in, cost_in, parent_in):
        self.config = config_in
        self.cost = cost_in
        self.parent = parent_in


def checkgoal(point1,point2,tol=step_size/2):
	if (abs(point1-point2) < tol).all():
		return True
	else:
		return False


def getneighborsconfig(env,robot,current_config,neighbors,step_size=step_size):
	candidates = []
	if neighbors == 4:
		for i in range(3):
			for j in (-1,1):
				tmp = np.copy(current_config)
				tmp[i] += j*step_size[i]
				if i==2:
					tmp[2] = tmp[2]%(2*np.pi)-np.pi
				candidates.append(tmp)
	else :
		for i in range(-1,2):
			for j in range(-1,2):
				for k in range(-1,2):
					if i==0 and j==0 and k==0:
						continue
					tmp = np.copy(current_config)
					tmp += [i,j,k]*step_size
					candidates.append(tmp)

	neighbors = []
	for config in candidates:
		robot.SetActiveDOFValues(config)
		if env.CheckCollision(robot):
			handles.append(env.plot3(points=(config[0],config[1],height),pointsize=pointsize,colors=(1,0,0)))
		else:
			handles.append(env.plot3(points=(config[0],config[1],height),pointsize=pointsize,colors=(0,0,1)))
			neighbors.append(config)
	return neighbors


def astar(env,robot,goalconfig,neighbors=8):

	initialconfig, height = getcurrentconfig(robot)
	height = 0.1
	initialconfig = np.array(initialconfig)
	goalconfig = np.array(goalconfig)

	cost_new = 0
	cost_current = 0
	nodes_list = []

	start = Node(initialconfig,0,None)
	end = Node(goalconfig,0,None)
	nodes = PriorityQueue()
	nodes.put((0,start))

	while not nodes.empty():
		node_current = nodes.get()[1]
		nodes_list.append(node_current)

		if checkgoal(node_current.config,goalconfig):
			end.parent = node_current.parent
			break
		for config in getneighborsconfig(env,robot,node_current.config,neighbors):
			cost_new = node_current.cost+0.00001*np.linalg.norm(node_current.config==config)
			theta_dif = np.minimum(abs(goalconfig[2]-config[2]),2*np.pi-abs(goalconfig[2]-config[2]))
			heuristic = np.linalg.norm([goalconfig[0]-config[0],goalconfig[1]-config[1],theta_dif])
			flag = 1
			for node in nodes_list:
				if (node.config == config).all():
					if node.cost > cost_new:
						node_new = Node(config,cost_new,node_current)
						nodes.put((cost_new+heuristic,node_new))
					flag = 0
					break
			if flag :
				node_new = Node(config,cost_new,node_current)
				nodes.put((cost_new+heuristic,node_new))
	node = end
	path_nodes = []
	while node.parent is not None:
		config = node.config
		handles.append(env.plot3(points=(config[0],config[1],height),pointsize=pointsize,colors=(0,0,0)))
		path_nodes.append(config)
		print config,'\n'
		node = node.parent
	config = node.config
	handles.append(env.plot3(points=(config[0],config[1],height),pointsize=pointsize,colors=(0,0,0)))
	path_nodes.append(config)
	path_nodes.reverse()
	return path_nodes


if __name__ == '__main__':
	print 'Please call astar_template.py!'