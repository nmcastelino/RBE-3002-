#!/usr/bin/env python
import rospy, numpy, math
from nav_msgs.msg import GridCells, OccupancyGrid

def listener():
	rospy.init_node('A*')
	rospy.Subscriber("nav_msgs/GridCells.msg", GridCells, aStar)

def findDist(point, goal):  #Finds the distance between a point and the goal point
	return (math.sqrt((((int(point[1]))-(int(goal[1])))^2)+(((int(point[2]))-(int(goal[2])))^2)))

def findNeighbor(point, goal):   #Finds the closest neighbor to goal
	closest = array(0,0)
	if (abs((point[1]-goal[1]))>(abs((point[2]-goal[2])))):
		if (point[1] > goal[1]):
			val = array((point[1]-1),point[2])
		else:
			closest = array((point[1]+1),point[2])
	else:
		if (point[2] > goal[2]):
			closest = array((point[1]),(point[2]-1))
		else:
			closest = array((point[1]),(point[2]+1))
            

def aStar(gridData, start, goal):
	notEvaluated = gridData.cells   #Cells that are not yet evaluated
	Evaluated = [start]                  #Cells that are evaluated (none so far)

	gScore[start] = 0

	fScore[start] = findDist(start, goal)

	current = start
    
	while (pathFound == FALSE):
		findNeighbor(current, goal)

