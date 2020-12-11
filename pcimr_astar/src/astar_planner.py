#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from visualization_msgs.msg import Marker

class Node():
    '''
    A node class for A* Pathfinding
    '''

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

class Astar_planner:
    """
    implement of Astar planner, neccessary subscribers and publishers
    """

    def __init__(self):

        # Initialize Subscribers
        self.sub_pos = rospy.Subscriber('/robot_pos', Point, self.callback_pos)
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback_goal)

        # Initialize Publisher
        self.pub_path = rospy.Publisher('/global_path', Path, queue_size=10)
        self.pub_goal = rospy.Publisher('/visualization/goal', Marker, queue_size=10)
        self.pub_plan = rospy.Publisher('/visualization/plan', Marker, queue_size=10)

        # Initialize messages
        self.msg_goal_marker = Marker()
        self.msg_goal_marker.header.frame_id = "map"
        self.msg_goal_marker.ns = "navigation"
        self.msg_goal_marker.id = 0
        self.msg_goal_marker.type = Marker.CUBE
        self.msg_goal_marker.action = Marker.ADD
        self.msg_goal_marker.scale.x = 1
        self.msg_goal_marker.scale.y = 1
        self.msg_goal_marker.scale.z = 0.2
        self.msg_goal_marker.color.a = 1.0
        self.msg_goal_marker.color.r = 1.0
        self.msg_goal_marker.color.g = 0.0
        self.msg_goal_marker.color.b = 0.0
        self.msg_goal_marker.pose.orientation = Quaternion(0, 0, 0, 1)

        self.msg_path_marker = Marker()
        self.msg_path_marker.header.frame_id = "map"
        self.msg_path_marker.ns = "navigation"
        self.msg_path_marker.id = 0
        self.msg_path_marker.type = Marker.LINE_STRIP
        self.msg_path_marker.action = Marker.ADD
        self.msg_path_marker.scale.x = 0.3
        self.msg_path_marker.color.a = 0.5
        self.msg_path_marker.color.r = 0.0
        self.msg_path_marker.color.g = 0.0
        self.msg_path_marker.color.b = 1.0
        self.msg_path_marker.pose.orientation = Quaternion(0, 0, 0, 1)

        # self.msg_path = Path()
        # # print(self.msg_path)
        # self.msg_path.header.stamp = rospy.Time.now()
        # self.msg_path.header.frame_id = "path"

    # callback of position
    def callback_pos(self, Point):
        self.pos_x = Point.x
        self.pos_y = Point.y

    # callback of map
    def callback_map(self, OccupancyGrid):
        self.map_input = np.array(OccupancyGrid.data)
        self.map_width = OccupancyGrid.info.width
        self.map_height = OccupancyGrid.info.height
        self.map = self.map_input.reshape(self.map_width, self.map_height)
        self.map = np.transpose(self.map)

    # callback of goal
    def callback_goal(self, PoseStamped):
        self.goal_x = PoseStamped.pose.position.x
        self.goal_y = PoseStamped.pose.position.y

    def getMinNode(self):
        '''
        try to find the node with minimal f in openlist
        '''
        currentNode = self.open_list[0]
        for node in self.open_list:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode

    def pointInCloseList(self, position):
        '''
        determine if a node is in closedlist
        '''
        for node in self.closed_list:
            if node.position == position:
                return True
        return False

    def pointInOpenList(self, position):
        '''
        determine if a node is in openlist
        '''
        for node in self.open_list:
            if node.position == position:
                return node
        return None

    def endPointInCloseList(self):
        '''
        determine if a node is endnode
        '''
        for node in self.closed_list:
            if node.position == self.endnode.position:
                return node
        return None

    def check_valid(self, goalx, goaly):
        '''
        check the validility of goal
        '''
        if self.map[int(goalx)][int(goaly)] == 0:
            return True
        else:
            return None

    def search(self, minF, offsetX, offsetY):
        '''
        search action with minimal f for next step
        '''

        node_pos = (minF.position[0] + offsetX, minF.position[1] + offsetY)

        # if the offset out of boundary
        if node_pos[0] < 0 or node_pos[0] > self.map_width - 1 or node_pos[1] < 0 or node_pos[1] > self.map_height - 1:
            return

        # if the offset valid
        elif self.map[int(node_pos[0])][int(node_pos[1])] != 0:
            return

        # if the node is in closed set, then pass
        elif self.pointInCloseList(node_pos):
            return

        else:
            # if it is not in openlist, add it to openlist
            currentNode = self.pointInOpenList(node_pos)
            if not currentNode:
                currentNode = Node(minF, node_pos)
                currentNode.g = minF.g + 1
                currentNode.h = (abs(node_pos[0] - self.endnode.position[0]) + abs(node_pos[1] - self.endnode.position[1])) * 0.8
                currentNode.f = currentNode.g + currentNode.h
                self.open_list.append(currentNode)
                return
            else:
                # if it is in openlist, determine if g of currentnode is smaller
                if minF.g + 1 < currentNode.g:
                    currentNode.g = minF.g + 1
                    currentNode.parent = minF
                    return

    # astar search function
    def astar(self, start, end):
        '''
        main function of astar search
        '''

        # Initialize end node and start node
        self.startnode = Node(None, start)
        self.startnode.g = self.startnode.h = self.startnode.f = 0
        self.endnode = Node(None, end)
        self.endnode.g = self.endnode.h = self.endnode.f = 0

        # Initialize open and closed list
        self.open_list = [self.startnode] # store f of next possible step
        self.closed_list = [] # store f of minimal path

        # try to find the path with minimal cost
        while True:

            # find the node with minimal f in openlist
            minF = self.getMinNode()

            # add this node to closed_list and delete this node from open_list
            self.closed_list.append(minF)
            self.open_list.remove(minF)

            # apply search to add node for next step
            self.search(minF, 0, 1)
            self.search(minF, 1, 0)
            self.search(minF, 0, -1)
            self.search(minF, -1, 0)

            # determine if it the endpoint
            endnode = self.endPointInCloseList()
            if endnode:
                path = []
                current = endnode
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]


    # run astar node
    def run(self, rate: float = 1):

        while not rospy.is_shutdown():

            rospy.wait_for_message('/move_base_simple/goal', PoseStamped)

            start = (self.pos_x, self.pos_y)

            if self.check_valid(self.goal_x, self.goal_y):
                end = (int(self.goal_x), int(self.goal_y))

                path = self.astar(start, end)

                # publish goal
                self.msg_goal_marker.pose.position.x = end[0] + 0.5
                self.msg_goal_marker.pose.position.y = end[1] + 0.5
                self.pub_goal.publish(self.msg_goal_marker)

                # publish plan
                self.msg_goal_marker.points.clear()
                for p in path:
                    self.msg_path_marker.points.append(Point(p[0]+0.5, p[1]+0.5, 0))
                self.pub_plan.publish(self.msg_path_marker)

                # # publish path
                # self.msg_path.poses.pose.position.clear()
                # for pa in path:
                #     self.msg_path.poses.pose.position.x.append(pa[0])
                #     self.msg_path.poses.pose.position.x.append(pa[1])
                # self.pub_path.publish(self.msg_path, latch=True)

            else:
                rospy.loginfo('Goal is not valid')



if __name__ == "__main__":
   rospy.init_node('Astar_planner')

   astar = Astar_planner()
   astar.run(rate=1)
