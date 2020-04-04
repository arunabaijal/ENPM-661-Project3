# *********************************** Rigid Robot ********************************#
import numpy as np
import queue
import math
import time
import cv2


class Node():
    r = 1*5
    L = 2*5
    dt = 1
    def __init__(self, parent, cost2come, cost2go, clear_val, x, y, theta):
        self.parent = parent
        self.cost2come = cost2come
        self.cost2go = cost2go
        self.clear_val = clear_val
        self.x = x
        self.y = y
        self.theta = theta
        self.current = [self.x, self.y, self.theta]
        
    def __lt__(self, other):
        return (self.cost2go + self.cost2come) < (other.cost2go + other.cost2come)

    def possible_steps(self, rpm1, rpm2):
        steps_with_cost = np.array([
            [0, rpm1, 1],  # Move left
            [rpm1, 0, 1],  # Move right
            [rpm1, rpm1, 1],  # Move forward
            [0, rpm2, 1],  # Move left
            [rpm2, 0, 1],  # Move right
            [rpm2, rpm2, 1],  # Move forward
            [rpm1, rpm2, 1],  # Move diagonal
            [rpm2, rpm1, 1]])  # Move diagonal
        return steps_with_cost
    
    def do_action(self, LW, RW):
        dx=Node.r*(LW+RW)*math.cos(np.deg2rad(self.theta))*Node.dt
        dy=Node.r*(LW+RW)*math.sin(np.deg2rad(self.theta))*Node.dt
        dtheta=(Node.r/Node.L)*(RW-LW)*Node.dt
        step = np.sqrt(dx**2 + dy**2)
        # print('dy', dy)
        return self.x + dx, self.y + dy, self.theta + np.rad2deg(dtheta), step
    
    def findRegion(self, current):
        region = [current[0], current[1], (current[2] + 360) % 360]
        return region
    
    def astar(self, goal, step_size, rpm1, rpm2):
        open('Nodes.txt', 'w').close()  # clearing files
        # visited = np.zeros((600*100, 400*100, 360), dtype=np.uint8)
        # accepted = np.full((600*100, 400*100, 360), None, dtype=np.uint8)
        visited = {}
        accepted = {}
        toBeVisited = queue.PriorityQueue()
        toBeVisited.put(self)
        region = self.findRegion(self.current)  # creating unique indexing value
        accepted[(int(region[0]), int(region[1]), int(region[2]))] = self
        f = open("Nodes.txt", "a+")
        steps_with_cost = self.possible_steps(rpm1, rpm2)
        while not toBeVisited.empty():
            visitingNode = toBeVisited.get()
            print(visitingNode.current)
            node = visitingNode.current
            region = self.findRegion(node)  # creating unique indexing value
            key = (int(region[0]), int(region[1]), int(region[2]))
            if key in visited.keys():  # check if node already visited
                continue
            else:
                toWrite = str(node)
                f.write(toWrite[1:len(toWrite) - 1] + '\n')
                if goalReached(node, goal):  # check if goal found
                    f.close()
                    return visitingNode
                
                for i in steps_with_cost:
                    new_x, new_y, new_theta, new_step = visitingNode.do_action(i[0], i[1])
                    # print(new_x, new_y, new_theta)
                    new_region = self.findRegion([new_x, new_y, new_theta])
                    # print(new_region)
                    new_keys = (int(new_region[0]), int(new_region[1]), int(new_region[2]))
                    if check_node([new_x, new_y], visitingNode.clear_val) and (new_keys not in visited.keys()):
                        new_node = Node(visitingNode, visitingNode.cost2come + new_step, calc_cost([new_x, new_y], goal,new_step), visitingNode.clear_val, new_x, new_y, new_theta)
                        if new_keys in accepted.keys():
                            if accepted[new_keys].cost2come + accepted[new_keys].cost2go > new_node.cost2come + new_node.cost2go:
                                accepted[new_keys] = new_node
                                toBeVisited.put(new_node)
                        else:
                            accepted[new_keys] = new_node
                            toBeVisited.put(new_node)
                visited[(int(region[0]), int(region[1]), int(region[2]))] = visitingNode
        f.close()
        return False

# Function to check if point is within goal threshold
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


# Function to generate new equation of line acc. to new points
def eqn(point1, point2):
    a = point1[1] - point2[1]
    b = point2[0] - point1[0]
    c = point1[1] * b - point1[0] * (-a)
    return a, b, c


# Function to check if the given point lies outside the final map or in the obstacle space
# def check_node(node, clearance):
#     u = 150.  # x-position of the center
#     v = 100.  # y-position of the center
#     a = 40.  # radius on the x-axis
#     b = 20.  # radius on the y-axis
#
#     clear_val = clearance
#
#     # New points for diamond obstacle
#     p1 = new_points([225, 15], clear_val, 0)
#     p2 = new_points([250, 30], clear_val, 1)
#     p3 = new_points([225, 45], clear_val, 2)
#     p4 = new_points([200, 30], clear_val, 3)
#
#     # New lines for diamond obstacle
#
#     a1, b1, c1 = eqn(p1, p2)
#     a2, b2, c2 = eqn(p2, p3)
#     a3, b3, c3 = eqn(p3, p4)
#     a4, b4, c4 = eqn(p4, p1)
#
#     # New points for tilted cuboid
#     p5 = new_points([95, 30], clear_val, 0)
#     p6 = new_points([100, 38.66], clear_val, 1)
#     p7 = new_points([35.05, 76.16], clear_val, 2)
#     p8 = new_points([30.05, 67.5], clear_val, 3)
#
#     # New lines for tilted cuboid
#
#     a5, b5, c5 = eqn(p5, p6)
#     a6, b6, c6 = eqn(p6, p7)
#     a7, b7, c7 = eqn(p7, p8)
#     a8, b8, c8 = eqn(p8, p5)
#
#     l1 = node[1] - 13*node[0] - (-140 + clearance*np.sqrt(1 + 13**2))
#     l2 = node[1] - (185 + clearance)
#     l3 = node[1] + 1.4*node[0] - (290 + clearance*np.sqrt(1 + 1.4**2))
#     l4 = node[1] - 1.2*node[0] - (30 - clearance*np.sqrt(1 + 1.2**2))
#     l5 = node[1] + 1.2*node[0] - (210 - clearance*np.sqrt(1 + 1.2**2))
#     l6 = node[1] - 1*node[0] - (100 - clearance*np.sqrt(1 + 1**2))
#
#     if node[0] + clearance >= 300 or node[0] - clearance < 0 or node[1] + clearance >= 200 or node[1] - clearance < 0:
#         return False
#
#     elif (node[0] - 225) ** 2 + (node[1] - 150) ** 2 <= (25 + clear_val) ** 2:
#         return False
#
#     elif ((node[0] - 150) ** 2) / (a + clear_val) ** 2 + ((node[1] - 100) ** 2) / (b + clear_val) ** 2 <= 1:
#         return False
#
#     elif (a1 * node[0] + b1 * node[1] >= c1) and (a2 * node[0] + b2 * node[1] >= c2) and (
#             a3 * node[0] + b3 * node[1] >= c3) and (a4 * node[0] + b4 * node[1] >= c4):
#         return False
#
#     elif (a5 * node[0] + b5 * node[1] >= c5) and (a6 * node[0] + b6 * node[1] >= c6) and (
#             a7 * node[0] + b7 * node[1] >= c7) and (a8 * node[0] + b8 * node[1] >= c8):
#         return False
#
#     elif (l1 <= 0) & (l2 <= 0) & (l3 <= 0) & (l4 >= 0) & ((l5 >= 0) | (l6 >= 0)):
#         return False
#
#     else:
#         return True

def check_node(node, clearance):
    

    if (node[0]/100.0 < -5.0) or (node[0]/100.0 > 5.0) or (node[1]/100.0 < -5.0) or (node[1]/100.0 > 5.0):
        # print('1')
        return False

    circle_bottom_left = (node[0]/100.0 - (-2.0))**2 + (node[1]/100.0 - (-3.0))**2
    if  circle_bottom_left < (1**2):
        # print('2')
        return False

    circle_bottom_right = (node[0]/100.0 - (2.0))**2 + (node[1]/100.0 - (-3.0))**2
    if  circle_bottom_right < (1**2):
        # print('3')
        return False

    circle_top_right = (node[0]/100.0 - (2.0))**2 + (node[1]/100.0 - (3.0))**2
    if circle_top_right < (1**2):
        # print('4')
        return False

    circle_center = (node[0]/100.0 - (0.0))**2 + (node[1]/100.0 - (0.0))**2
    if circle_center < (1**2):
        # print(circle_center, node)
        # print('5')
        return False

    # Left Square
    if (node[0]/100.0 > -4.75) and (node[0]/100.0 < -3.25) and (node[1]/100.0 < 0.75) and (node[1]/100.0 > -0.75):
        # print('6')
        return False

    # Left Top Square
    if (node[0]/100.0 > -2.75) and (node[0]/100.0 < -1.25) and (node[1]/100.0 < 3.75) and (node[1]/100.0 > 2.25):
        # print('7')
        return False

    # Right square
    if (node[0]/100.0 < 4.75) and (node[0]/100.0 > 3.25) and (node[1]/100.0 < 0.75) and (node[1]/100.0 > -0.75):
        # print('8')
        return False

    return True

# Function to find selected optimal path
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

# Function to generate points on a line
def get_line(x1, y1, x2, y2):
    points = []
    x1 = int(100*(x1 + 5.1))
    y1 = int(100*(y1 + 5.1))
    x2 = int(100*(x2 + 5.1))
    y2 = int(100*(y2 + 5.1))
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
    return np.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

def main():
    # Taking start point and goal point from the user
    radius = 1
    step_size = 1
    clearance = 1
    start_point = [0,150,0]
    while not check_node([0,150,0], radius + clearance):
        print('Invalid start point given')
        exit(-1)
        # start_point = eval(input('Please enter the start point in this format - [x,y,theta (in deg)]: '))
    
    print('The start point you gave is:', start_point)
    print('')
    
    goal_point = [400,500]
    while not check_node([400,500], radius + clearance):
        print('Invalid end point given')
        exit(-1)
    #     goal_point = eval(input('Please enter the goal point in this format - [x,y]: '))
    
    print('The goal point you gave is:', goal_point)
    start_time = time.time()
    start = Node(None, 0, calc_cost(start_point, goal_point, step_size), radius + clearance, start_point[0], start_point[1], start_point[2])
    print('Finding path...')
    goal = start.astar(goal_point, step_size, 1, 5)
    if not goal:
        print('Path not found')
        exit(-1)
    open('nodePath.txt', 'w').close()
    generate_path(goal, start_point)
    end_time = time.time()
    print('Time taken to find path: ' + str(end_time - start_time))
    grid = np.ones((1021, 1021, 3), dtype=np.uint8) * 255
    lines = []
    # Left Square
    lines.append(get_line(-4.75, 0.75, -3.25, 0.75))
    lines.append(get_line(-4.75, 0.75, -4.75, -0.75))
    lines.append(get_line(-4.75, -0.75, -3.25, -0.75))
    lines.append(get_line(-3.25, -0.75, -3.25, 0.75))
    
    lines.append(get_line(-2.75, 3.75, -1.25, 3.75))
    lines.append(get_line(-2.75, 3.75, -2.75, 2.25))
    lines.append(get_line(-2.75, 2.25, -1.25, 2.25))
    lines.append(get_line(-1.25, 2.25, -1.25, 3.75))
    
    lines.append(get_line(4.75, 0.75, 3.25, 0.75))
    lines.append(get_line(4.75, 0.75, 4.75, -0.75))
    lines.append(get_line(4.75, -0.75, 3.25, -0.75))
    lines.append(get_line(3.25, -0.75, 3.25, 0.75))
    
    index = np.mgrid[0:1021, 0:1021]

    # Left Bottom Circle
    result_left_bottom = (index[0] - (-300+510))**2 + (index[1] - (-200+510))**2
    inds = np.where(result_left_bottom < 10000.0)
    grid[inds] = [0,0,0]

    # Right Bottom Circle
    result_right_bottom = (index[1] - (200+510))**2 + (index[0] - (-300+510))**2
    inds = np.where(result_right_bottom < 10000.0)
    grid[inds] = [0,0,0]

    # Center
    result_center = (index[1] - (0+510))**2 + (index[0] - (0+510))**2
    inds = np.where(result_center < 10000.0)
    grid[inds] = [0,0,0]

    # Right Top Circle
    result_top = (index[1] - (200+510))**2 + (index[0] - (300+510))**2
    inds = np.where(result_top < 10000.0)
    grid[inds] = [0,0,0]

    # circle = []
    # for i in range(200, 250):
    #     for j in range(125, 175):
    #         if (i - 225) ** 2 + (j - 150) ** 2 <= 25 ** 2:
    #             circle.append([i, j])
    # lines.append(circle)
    
    # ellipse = []
    # for i in range(110, 190):
    #     for j in range(80, 120):
    #         if (i - 150) ** 2 / 40 ** 2 + (j - 100) ** 2 / 20 ** 2 <= 1:
    #             ellipse.append([i, j])
    # lines.append(ellipse)
    
    vidWriter = cv2.VideoWriter("./video_output.mp4",cv2.VideoWriter_fourcc(*'mp4v'), 500, (1021, 1021))
    for line in lines:
        for l in line:
            grid[l[1]][l[0]] = [0, 0, 0]
    file = open('Nodes.txt', 'r')
    points = file.readlines()
    for point in points:
        pts = point.split(',')
        grid[int(float(pts[1]))+510][int(float(pts[0]))+510] = [255, 0, 0]
        vidWriter.write(np.flip(grid, 0))
    file = open('nodePath.txt', 'r')
    points = file.readlines()
    pr_point = start_point
    for point in points:
        pts = point.split(',')
        grid = cv2.arrowedLine(grid, (int(float(pr_point[0])) + 510, int(float(pr_point[1])) + 510),
                               (int(float(pts[0])) + 510, int(float(pts[1])) + 510), (0, 255, 0), 1)
        pr_point = pts
        vidWriter.write(np.flip(grid, 0))
    for i in range(2000):
        vidWriter.write(np.flip(grid, 0))
    vidWriter.release()
    graph_end_time = time.time()
    print('Time taken to animate paths: ' + str(graph_end_time - end_time))


if __name__ == '__main__':
    main()
