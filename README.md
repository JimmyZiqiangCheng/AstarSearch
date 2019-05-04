# Maze exploration using Astar search implemented in python 3
An optimized maze searching algorithm using astar search

1. How to run the file?

./MazeExplorer gird_file x1 y1 x2 y2

details:
grid_file: a text file consisting of 1s and 0s, with 1s indicating obstacles and 0s indicating pathes

x1, y1: the starting column and row values

x2, y2: the goal column and row values


2. What is the output format?

If a path cannot be found:
"num of nodes explored", "total cost=0"
"X"

If a path can be found:
"num of nodes explored", "total cost"
"actions taken (e.g. LRLLD")

details:
L:left
R:right
U:up
D:down
