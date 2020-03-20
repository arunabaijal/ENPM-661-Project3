# *********************************** Rigid Robot ********************************#
import numpy as np
import queue
import math
import time
import cv2


class Node():
    def __init__(self, start_point, parent, cost2come, cost2go, clear_val, step):
        self.current = start_point
        self.parent = parent
        self.cost2come = cost2come
        self.cost2go = cost2go
        self.clear_val = clear_val
        self.step = step
        
    def __lt__(self, other):
        return (self.cost2go + self.cost2come) < (other.cost2go + other.cost2come)
    
    def moveForward(self):
        new_position = [self.current[0] + self.step * math.cos(math.radians(self.current[2])), self.current[1] + self.step * math.sin(math.radians(self.current[2])), self.current[2]]
        if not check_node(new_position, self.clear_val):
            return False
        else:
            return new_position, 'right'
    def moveUp30(self):
        new_position = [self.current[0] + self.step * math.cos(math.radians(self.current[2] + 30)), self.current[1] + self.step * math.sin(math.radians(self.current[2] + 30)), self.current[2] + 30]
        if not check_node(new_position, self.clear_val):
            return False
        else:
            return new_position, 'right'
    def moveUp60(self):
        new_position = [self.current[0] + self.step * math.cos(math.radians(self.current[2] + 60)), self.current[1] + self.step * math.sin(math.radians(self.current[2] + 60)), self.current[2] + 60]
        if not check_node(new_position, self.clear_val):
            return False
        else:
            return new_position, 'right'
    def moveDown30(self):
        new_position = [self.current[0] + self.step * math.cos(math.radians(self.current[2] + 330)), self.current[1] + self.step * math.sin(math.radians(self.current[2] + 330)), self.current[2] + 330]
        if not check_node(new_position, self.clear_val):
            return False
        else:
            return new_position, 'right'
    def moveDown60(self):
        new_position = [self.current[0] + self.step * math.cos(math.radians(self.current[2] + 300)), self.current[1] + self.step * math.sin(math.radians(self.current[2] + 300)), self.current[2] + 300]
        if not check_node(new_position, self.clear_val):
            return False
        else:
            return new_position, 'right'
    
    def findRegion(self, current):
        region = [round(current[0] * 2) / 2, round(current[1] * 2) / 2, (current[2] / 30) % 12]
        return region
    
    def astar(self, goal, step_size):
        open('Nodes.txt', 'w').close()  # clearing files
        # accepted = dict()
        visited = np.zeros((600, 400, 12))  # TODO change value and make it generic
        accepted = np.full((600, 400, 12), None)  # TODO change value and make it generic
        toBeVisited = queue.PriorityQueue()
        toBeVisited.put(self)
        region = self.findRegion(self.current)
        accepted[int(region[0]*2)][int(region[1]*2)][int(region[2])] = self
        f = open("Nodes.txt", "a+")
        while not toBeVisited.empty():
            visitingNode = toBeVisited.get()
            node = visitingNode.current
            region = self.findRegion(node)
            if visited[int(region[0]*2)][int(region[1]*2)][int(region[2])] == 1:  # check if node already visited
                continue
            else:
                toWrite = str(node)
                f.write(toWrite[1:len(toWrite) - 1] + '\n')
                if goalReached(node, goal):  # check if goal found
                    f.close()
                    print('FINAL COST', visitingNode.cost2go + visitingNode.cost2come)
                    return visitingNode
                # create all possible children
                if visitingNode.moveForward():
                    new, up = visitingNode.moveForward()
                    new_node = Node(new, visitingNode, visitingNode.cost2come + step_size, calc_cost(new, goal, step_size), visitingNode.clear_val, step_size)
                    new_region = self.findRegion(new)
                    if visited[int(new_region[0]*2)][int(new_region[1]*2)][int(new_region[2])] == 0:
                        if accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] is not None:
                            if accepted[int(new_region[0]*2)][int(new_region[1]*2)][int(new_region[2])].cost2come + accepted[int(new_region[0]*2)][int(new_region[1]*2)][int(new_region[2])].cost2go > new_node.cost2come + new_node.cost2go:
                                accepted[int(new_region[0]*2)][int(new_region[1]*2)][int(new_region[2])] = new_node
                                toBeVisited.put(new_node)
                        else:
                            accepted[int(new_region[0]*2)][int(new_region[1]*2)][int(new_region[2])] = new_node
                            toBeVisited.put(new_node)
                if visitingNode.moveUp30():
                    new, up = visitingNode.moveUp30()
                    new_node = Node(new, visitingNode, visitingNode.cost2come + step_size, calc_cost(new, goal, step_size), visitingNode.clear_val, step_size)
                    new_region = self.findRegion(new)
                    if visited[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] == 0:
                        if accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] is not None:
                            if accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])].cost2come + \
                                    accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][
                                        int(new_region[2])].cost2go > new_node.cost2come + new_node.cost2go:
                                accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] = new_node
                                toBeVisited.put(new_node)
                        else:
                            accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] = new_node
                            toBeVisited.put(new_node)
                if visitingNode.moveUp60():
                    new, up = visitingNode.moveUp60()
                    new_node = Node(new, visitingNode, visitingNode.cost2come + step_size, calc_cost(new, goal, step_size), visitingNode.clear_val, step_size)
                    new_region = self.findRegion(new)
                    if visited[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] == 0:
                        if accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] is not None:
                            if accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])].cost2come + \
                                    accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][
                                        int(new_region[2])].cost2go > new_node.cost2come + new_node.cost2go:
                                accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] = new_node
                                toBeVisited.put(new_node)
                        else:
                            accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] = new_node
                            toBeVisited.put(new_node)
                if visitingNode.moveDown30():
                    new, up = visitingNode.moveDown30()
                    new_node = Node(new, visitingNode, visitingNode.cost2come + step_size, calc_cost(new, goal, step_size), visitingNode.clear_val, step_size)
                    new_region = self.findRegion(new)
                    if visited[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] == 0:
                        if accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] is not None:
                            if accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])].cost2come + \
                                    accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][
                                        int(new_region[2])].cost2go > new_node.cost2come + new_node.cost2go:
                                accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] = new_node
                                toBeVisited.put(new_node)
                        else:
                            accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] = new_node
                            toBeVisited.put(new_node)
                if visitingNode.moveDown60():
                    new, up = visitingNode.moveDown60()
                    new_node = Node(new, visitingNode, visitingNode.cost2come + step_size, calc_cost(new, goal, step_size), visitingNode.clear_val, step_size)
                    new_region = self.findRegion(new)
                    if visited[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] == 0:
                        if accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] is not None:
                            if accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])].cost2come + \
                                    accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][
                                        int(new_region[2])].cost2go > new_node.cost2come + new_node.cost2go:
                                accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] = new_node
                                toBeVisited.put(new_node)
                        else:
                            accepted[int(new_region[0] * 2)][int(new_region[1] * 2)][int(new_region[2])] = new_node
                            toBeVisited.put(new_node)
                # accepted[int(region[0]*2)][int(region[1]*2)][int(region[2])] = None
                visited[int(region[0]*2)][int(region[1]*2)][int(region[2])] = 1
        f.close()
        return False


"""
# Function to check if the given point lies outside the trial map or in the obstacle space
def check_node(node):
    if node[0] >= 200 or node[0] < 0 or node[1] >= 100 or node[1] < 0:
        # print('Sorry the point you entered is out of bounds! Try again.')
        return False
    elif 90 <= node[0] <= 110 and 40 <= node[1] <= 60:
        # print('Sorry the point you entered is in the obstacle space! Try again.')
        return False
    elif (node[0]-160)**2 + (node[1]-50)**2 < 15**2:
        # print('Sorry the point you entered is in the obstacle space! Try again.')
        return False
    else:
        return True
"""

def goalReached(node, goal):
    if (node[0]-goal[0])**2 + (node[1]-goal[1])**2 < 1.5**2:
        return True
    return False

# Function to generate new points
def new_points(point, clearance, direction):
    if direction == 0:
        point[1] = point[1] - np.sqrt(2) * clearance
    elif direction == 1:
        point[0] = point[0] + np.sqrt(2) * clearance
    elif direction == 2:
        point[1] = point[1] + np.sqrt(2) * clearance
    else:
        point[0] = point[0] - np.sqrt(2) * clearance
    return point


# Function to generate new points for the concave polygon
def new_points_pol(point, point0, point1, point3, clearance, direction):
    if direction == 0:
        slope1 = (point1[1] - point[1]) / (point1[0] - point[0])
        a1 = (90 - math.degrees(math.atan(slope1))) / 2
        l1 = clearance / math.sin(math.radians(a1))
        x1 = l1 * math.sin(math.radians(a1))
        y1 = l1 * math.cos(math.radians(a1))
        point[0] = point[0] - x1
        point[1] = point[1] - y1
        # point[1] = point[1] - np.sqrt(2)*clearance
    elif direction == 1:
        point[1] = point[1] - np.sqrt(2) * clearance
    elif direction == 2:
        # slope3 = (point1[1] - point[1]) / (point1[0] - point[0])
        # a3 = 90 - (180 - math.degrees(math.atan(slope3)))
        # l3 = clearance/math.sin(math.radians(a3))
        # x3 = l3*math.sin(math.radians(a3))
        # y3 = l3*math.cos(math.radians(a3))
        # point[0] = point[0] + x3
        # point[1] = point[1] - y3
        point[1] = point[1] - np.sqrt(2) * clearance
    elif direction == 3:
        point[0] = point[0] + np.sqrt(2) * clearance
    elif direction == 4:
        slope4 = (point[1] - point1[1]) / (point[0] - point1[0])
        a4 = math.degrees(math.atan(slope4))
        l4 = clearance / math.sin(math.radians(a4))
        x4 = l4 * math.cos(math.radians(a4))
        y4 = l4 * math.sin(math.radians(a4))
        point[0] = point[0] + x4
        point[1] = point[1] + y4
    else:
        slope5 = (point[1] - point1[1]) / (point[0] - point1[0])
        a5 = 180 - math.degrees(math.atan(slope5))
        l5 = clearance / math.sin(math.radians(a5))
        x5 = l5 * math.cos(math.radians(a5))
        y5 = l5 * math.sin(math.radians(a5))
        point[0] = point[0] - x5
        point[1] = point[1] + y5
    return point


# Function to generate new equation of line acc. to new points

def eqn(point1, point2):
    a = point1[1] - point2[1]
    b = point2[0] - point1[0]
    c = point1[1] * b - point1[0] * (-a)
    return a, b, c


# Function to check if the given point lies outside the final map or in the obstacle space
def check_node(node, clearance):
    u = 150.  # x-position of the center
    v = 100.  # y-position of the center
    a = 40.  # radius on the x-axis
    b = 20.  # radius on the y-axis
    
    clear_val = clearance
    
    # New points for diamond obstacle
    p1 = new_points([225, 15], clear_val, 0)
    p2 = new_points([250, 30], clear_val, 1)
    p3 = new_points([225, 45], clear_val, 2)
    p4 = new_points([200, 30], clear_val, 3)
    
    # New lines for diamond obstacle
    
    a1, b1, c1 = eqn(p1, p2)
    a2, b2, c2 = eqn(p2, p3)
    a3, b3, c3 = eqn(p3, p4)
    a4, b4, c4 = eqn(p4, p1)
    
    # New points for tilted cuboid
    p5 = new_points([95, 30], clear_val, 0)
    p6 = new_points([100, 38.66], clear_val, 1)
    p7 = new_points([35.05, 76.16], clear_val, 2)
    p8 = new_points([30.05, 67.5], clear_val, 3)
    
    # New lines for tilted cuboid
    
    a5, b5, c5 = eqn(p5, p6)
    a6, b6, c6 = eqn(p6, p7)
    a7, b7, c7 = eqn(p7, p8)
    a8, b8, c8 = eqn(p8, p5)
    
    # New points for concave polygon
    p9 = new_points_pol([20, 120], [20, 120], [50, 150], [100, 150], clear_val, 0)
    p10 = new_points_pol([50, 150], [20, 120], [50, 150], [100, 150], clear_val, 1)
    p11 = new_points_pol([75, 120], [20, 120], [50, 150], [100, 150], clear_val, 2)
    p12 = new_points_pol([100, 150], [20, 120], [50, 150], [100, 150], clear_val, 3)
    p13 = new_points_pol([75, 185], [20, 120], [50, 150], [100, 150], clear_val, 4)
    p14 = new_points_pol([25, 185], [20, 120], [50, 150], [100, 150], clear_val, 5)
    
    # New lines for concave polygon
    
    a9, b9, c9 = eqn(p9, p10)
    a10, b10, c10 = eqn(p10, p11)
    a11, b11, c11 = eqn(p11, p12)
    a12, b12, c12 = eqn(p12, p13)
    a13, b13, c13 = eqn(p13, p14)
    a14, b14, c14 = eqn(p14, p9)
    a15, b15, c15 = eqn(p14, p11)
    
    if node[0] + clearance >= 300 or node[0] - clearance < 0 or node[1] + clearance >= 200 or node[1] - clearance < 0:
        # print('Sorry the point is out of bounds! Try again.')
        return False
    elif (node[0] - 225) ** 2 + (node[1] - 150) ** 2 <= (25 + clear_val) ** 2:
        # print('Sorry the point is in the obstacle space! Try again.1')
        return False
    elif ((node[0] - 150) ** 2) / (a + clear_val) ** 2 + ((node[1] - 100) ** 2) / (b + clear_val) ** 2 <= 1:
        # print('Sorry the point is in the obstacle space! Try again.2')
        return False
    elif (a1 * node[0] + b1 * node[1] >= c1) and (a2 * node[0] + b2 * node[1] >= c2) and (
            a3 * node[0] + b3 * node[1] >= c3) and (a4 * node[0] + b4 * node[1] >= c4):
        # print('Sorry the point is in the obstacle space! Try again.3')
        return False
    elif (a5 * node[0] + b5 * node[1] >= c5) and (a6 * node[0] + b6 * node[1] >= c6) and (
            a7 * node[0] + b7 * node[1] >= c7) and (a8 * node[0] + b8 * node[1] >= c8):
        # print('Sorry the point is in the obstacle space! Try again. hello')
        return False
    # Dividing concave shape into 2 convex shapes
    elif (a10 * node[0] + b10 * node[1] >= c10) and (a11 * node[0] + b11 * node[1] >= c11) and (
            a12 * node[0] + b12 * node[1] >= c12) and (a13 * node[0] + b13 * node[1] >= c13):
        # print('Sorry the point is in the obstacle space! Try again.4')
        return False
    elif (a10 * node[0] + b10 * node[1] <= c10) and (a14 * node[0] + b14 * node[1] >= c14) and (
            a9 * node[0] + b9 * node[1] >= c9):
        # print('Sorry the point is in the obstacle space! Try again.5')
        return False
    else:
        return True

def generate_path(node, root):
    while (not np.array_equal(node.current, root)):
        f = open("nodePath.txt", "r+")
        content = f.read()
        f.seek(0, 0)
        toWrite = str(node.current)
        node = node.parent
        f.write(toWrite[1:len(toWrite) - 1] + '\n' + content)
        f.close()
    
    f = open("nodePath.txt", "r+")
    content = f.read()
    f.seek(0, 0)
    toWrite = str(node.current)
    f.write(toWrite[1:len(toWrite) - 1] + '\n' + content)
    f.close()


def get_line(x1, y1, x2, y2):
    points = []
    issteep = abs(y2 - y1) > abs(x2 - x1)
    if issteep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    rev = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        rev = True
    deltax = x2 - x1
    deltay = abs(y2 - y1)
    error = int(deltax / 2)
    y = y1
    ystep = None
    if y1 < y2:
        ystep = 1
    else:
        ystep = -1
    for x in range(x1, x2 + 1):
        if issteep:
            points.append((y, x))
        else:
            points.append((x, y))
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax
    # Reverse the list if the coordinates were reversed
    if rev:
        points.reverse()
    return points


# Functions for the action space - Up, Down, Left, Right, Up-Right, Down-Right, Up-left, Down-left
def calc_cost(current, goal, step):
    return np.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2) + step

def main():
    # Taking start point and goal point from the user
    # radius = eval(input('Please enter robot radius value: '))
    # clearance = eval(input('Please enter robot clearance value: '))
    # start_point = eval(input('Please enter the start point in this format - [x,y]: '))
    # while not check_node(start_point, radius + clearance):
    #     start_point = eval(input('Please enter the start point in this format - [x,y]: '))
    #
    # print('The start point you gave is:', start_point)
    # print('')
    #
    # goal_point = eval(input('Please enter the goal point in this format - [x,y]: '))
    # while not check_node(goal_point, radius + clearance):
    #     goal_point = eval(input('Please enter the goal point in this format - [x,y]: '))
    #
    # print('The goal point you gave is:', goal_point)
    start_time = time.time()
    start_pt = [10, 10, 60]
    end_pt = [150, 150]
    step_size = 2
    start = Node(start_pt, None, 0, calc_cost(start_pt, end_pt, step_size), 1 + 1, step_size)
    goal = start.astar(end_pt, step_size)
    open('nodePath.txt', 'w').close()
    generate_path(goal, start_pt)
    end_time = time.time()
    print('Time taken to find path: ' + str(end_time - start_time))
    grid = np.ones((201, 301, 3), dtype=np.uint8) * 255
    lines = []
    lines.append(get_line(0, 0, 300, 0))
    lines.append(get_line(0, 0, 0, 200))
    lines.append(get_line(300, 0, 300, 200))
    lines.append(get_line(0, 200, 300, 200))
    
    lines.append(get_line(200, 30, 225, 45))
    lines.append(get_line(225, 45, 250, 30))
    lines.append(get_line(250, 30, 225, 15))
    lines.append(get_line(225, 15, 200, 30))
    
    lines.append(get_line(95, 30, 100, 38))
    lines.append(get_line(100, 38, 35, 76))
    lines.append(get_line(35, 76, 30, 67))
    lines.append(get_line(30, 67, 95, 30))
    
    lines.append(get_line(20, 120, 25, 185))
    lines.append(get_line(25, 185, 75, 185))
    lines.append(get_line(75, 185, 100, 150))
    lines.append(get_line(100, 150, 75, 120))
    lines.append(get_line(75, 120, 50, 150))
    lines.append(get_line(50, 150, 20, 120))
    
    circle = []
    for i in range(200, 250):
        for j in range(125, 175):
            if (i - 225) ** 2 + (j - 150) ** 2 <= 25 ** 2:
                circle.append([i, j])
    lines.append(circle)
    
    ellipse = []
    for i in range(110, 190):
        for j in range(80, 120):
            if (i - 150) ** 2 / 40 ** 2 + (j - 100) ** 2 / 20 ** 2 <= 1:
                ellipse.append([i, j])
    lines.append(ellipse)
    
    # node[0]-150)**2)/a**2 + ((node[1]-100)**2)/b**2 <= 1:
    for line in lines:
        for l in line:
            # print(l)
            grid[l[1]][l[0]] = [0, 0, 0]
    file = open('Nodes.txt', 'r')
    points = file.readlines()
    for point in points:
        pts = point.split(',')
        grid[int(float(pts[1]))][int(float(pts[0]))] = [255, 0, 0]
        cv2.imshow('Path', np.flip(grid, 0))
        cv2.waitKey(1)
    file = open('nodePath.txt', 'r')
    points = file.readlines()
    for point in points:
        pts = point.split(',')
        grid[int(float(pts[1]))][int(float(pts[0]))] = [0, 255, 0]
        cv2.imshow('Path', np.flip(grid, 0))
        cv2.waitKey(1)
    cv2.waitKey()
    graph_end_time = time.time()
    print('Time taken to animate paths: ' + str(graph_end_time - end_time))


if __name__ == '__main__':
    main()
