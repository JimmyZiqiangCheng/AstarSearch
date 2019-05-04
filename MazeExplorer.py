
# Astar search implementation with python 3

# Author: Jimmy Ziqiang Cheng
# Date: 2, May, 2019

import sys
import copy 

# basic unit that contains the position of itself, the cumulated cost and actions
class Node:
	def __init__(self, pos, cost):
		self.pos = pos
		self.cost = cost
		self.actions =[]

# the main class that implements the search algorithm
class Robotplanner:
	def __init__(self, start, goal, grid_file):

		self.goal = [int(goal[0]),int(goal[1])]
		self.grid_file = grid_file
		self.start= [int(start[0]), int(start[1])]
		self.grid = []
		self.fringe = []

	# function that loads the grid_file
	def load_file(self):
		data = []
		with open(self.grid_file,'r') as g:
			for line in g:
				line = line.strip()
				data.append(line)
		return data

	# get the size of the grid
	def get_size(self):
		data = self.load_file()
		size_string = data[0].split()
		size = [int(size_string[0]), int(size_string[1])]
		return size

	# transform the grid_file into a 2d list
	def get_grid(self):
		data = self.load_file()
		grid = []
		for i in range(1,len(data)):
			line_string = data[i].split()
			grid.append([])
			for ele in line_string:
				grid[i-1].append(int(ele))

		return grid

	# moves the node and mutates the corresponding attributs within the node
	def move_left(self, node):
		node.pos[0]-=1
		node.cost +=1
		node.actions.append("L")
		return node
	def move_right(self, node):
		node.pos[0]+=1
		node.cost +=1
		node.actions.append("R")
		return node
	def move_up(self, node):
		node.pos[1]-=1
		node.cost +=1
		node.actions.append("U")
		return node
	def move_down(self, node):
		node.pos[1]+=1
		node.cost +=1
		node.actions.append("D")
		return node

	# the actual search function which returns the result node if a route can be established, none otherwise
	def search(self):
		self.grid = self.get_grid()
		self.grid[self.start[1]][self.start[0]]=2

		self.fringe.append(Node(self.start,0))
		
		while len(self.fringe) >0:
			best = self.get_size()[0]+self.get_size()[1]
			to_pop = 0

			for i in range(len(self.fringe)):	
				evaluation = self.evaluate2(self.fringe[i])
				if evaluation <  best:
					best = evaluation
					to_pop = i

			test_node = self.fringe.pop(to_pop)
			if test_node.pos == self.goal:
				return test_node
			else:
				self.fringe = self.fringe + self.expand(test_node)
		return None

	# evaluation function based on euclidean distance
	def evaluate1(self, cur_node):		
		heuristic = ((self.goal[0] - cur_node.pos[0])**2 + (self.goal[1] - cur_node.pos[1])**2)**(1/2.0)
		return cur_node.cost + heuristic

	# evaluation function based on manhattan distance
	def evaluate2(self, cur_node):	
		heuristic = abs(self.goal[0] - cur_node.pos[0]) + abs(self.goal[1] - cur_node.pos[1])
		return cur_node.cost + heuristic

	# expand the input node towards all directions if possible, set explored grid value to 2
	def expand(self, node):
		nodes = []
		x = node.pos[0]
		y = node.pos[1]

		if (x+1 <self.get_size()[0] and self.grid[y][x+1]==0):
			node_toAdd = copy.deepcopy(node)
			nodes.append(self.move_right(node_toAdd))
			self.grid[y][x+1]=2
		if (x-1 >=0 and self.grid[y][x-1]==0):
			node_toAdd = copy.deepcopy(node)
			nodes.append(self.move_left(node_toAdd))
			self.grid[y][x-1]=2
		if (y+1 <self.get_size()[1] and self.grid[y+1][x]==0):
			node_toAdd = copy.deepcopy(node)
			nodes.append(self.move_down(node_toAdd))
			self.grid[y+1][x]=2
		if (y-1 >=0 and self.grid[y-1][x]==0):
			node_toAdd = copy.deepcopy(node)
			nodes.append(self.move_up(node_toAdd))
			self.grid[y-1][x]=2
		return nodes
	# count then number of nodes explored
	def count_nodes_explored(self):
		return sum(i.count(2) for i in self.grid)

	def print_grid(self):
		for line in self.grid:
			print (line)


# take command line parameters
grid_file = sys.argv[1]
start = sys.argv[2],sys.argv[3]
goal = sys.argv[4],sys.argv[5]

# execute the search
my_robot = Robotplanner(start, goal, grid_file)
node = my_robot.search()

nodes_explored = my_robot.count_nodes_explored()

# if the goal cannot be reached
if node == None:
	print(nodes_explored," ",0)
	print("X")
	
# otherwise print out the number of nodes explored, total cost and the actions taken to get to the goal
else:
	total_cost = node.cost
	actions = ' '.join(node.actions)

	print(nodes_explored," ",total_cost)
	print(actions)