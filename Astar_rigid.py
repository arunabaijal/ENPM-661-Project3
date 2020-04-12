# *********************************** Rigid Robot ********************************#
import numpy as np
import queue
import math
import time
from decimal import *

getcontext().prec = 18
class Node():
    # Radius of wheels of turtlebot, divided by 1000 to convert to metres
    r = Decimal(38 * 10 / 1000.0)
    # Radius / Distance between the wheels of the turtlebot, divided by 1000 to convert to metres
    L = Decimal(354 * 10 / 1000.0)
    # curve_list_start = []
    # curve_list_end = []
    
    def __init__(self, parent, cost2come, cost2go, clear_val, x, y, theta):
        self.parent = parent
        self.cost2come = Decimal(cost2come)
        self.cost2go = Decimal(cost2go)
        self.clear_val = Decimal(clear_val)
        self.x = Decimal(x)
        self.y = Decimal(y)
        self.theta = Decimal(theta)
        self.current = [self.x, self.y, self.theta]
        # self.kids = []
    
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
    
    # Functions for the action space - Up, Down, Left, Right, Up-Right, Down-Right, Up-left, Down-left
    def do_action(self, LW, RW):
        LW = Decimal(LW * 2 * np.pi / 60)
        RW = Decimal(RW * 2 * np.pi / 60)
        dt = Decimal(0.1)
        t = Decimal(0)
        theta = Decimal(np.deg2rad(float(self.theta)))
        accum_dx = Decimal(0)
        accum_dy = Decimal(0)
        step = Decimal(0)
        # Loop to iterate 10 steps over 1sec time period
        # print("THETA: {}".format(theta))
        while t < 1:
            t += dt
            dx = Decimal((Node.r / 2) * (LW + RW) * (Decimal(math.cos(theta))) * dt)
            dy = Decimal((Node.r / 2) * (LW + RW) * (Decimal(math.sin(theta))) * dt)
            accum_dx += dx
            accum_dy += dy
            dtheta = Decimal((Node.r / Node.L) * (RW - LW) * dt)
            theta += dtheta
            # print("dx: {}, dy: {}, theta: {}, rw: {}, lw: {}".format(dx, dy, dtheta, RW, LW))
            # raw_input()
            node = [self.x + accum_dx, self.y + accum_dy]
            if not check_node(node, self.clear_val):
                break
            step += np.sqrt(dx ** 2 + dy ** 2)
        
        # print('-----------------------------')
        # print("acum_dx: {}, accum_dy: {}, theta: {}".format(accum_dx, accum_dy, theta))
        # raw_input()
        return self.x + accum_dx, self.y + accum_dy, Decimal(np.rad2deg(float(theta))), step
    
    def findRegion(self, current):
        region = [current[0], current[1], (current[2] + 360) % 360]
        return region
    
    def astar(self, goal, step_size, rpm1, rpm2):
        open('Nodes.txt', 'w').close()
        visited = {}
        accepted = {}
        toBeVisited = queue.PriorityQueue()
        toBeVisited.put(self)
        region = self.findRegion(self.current)  # creating unique indexing value
        accepted[(int(region[0]), int(region[1]), int(region[2]))] = self
        f = open("Nodes.txt", "a+")
        steps_with_cost = self.possible_steps(rpm1, rpm2)
        j = 0
        # print(toBeVisited[0])
        while not toBeVisited.empty():
            visitingNode = toBeVisited.get()
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
                    # print("Cost: {}, Heuristic: {}, total: {}".format(visitingNode.cost2come + new_step, calc_cost([new_x, new_y], goal, new_step), \
                    #                                             visitingNode.cost2come + new_step + calc_cost([new_x, new_y], goal, new_step)))
                    new_region = self.findRegion([new_x, new_y, new_theta])
                    new_keys = (int(new_region[0]), int(new_region[1]), int(new_region[2]))
                    # Check if the new node is already visited or not
                    if check_node([new_x, new_y], visitingNode.clear_val) and (new_keys not in visited.keys()):
                        new_node = Node(visitingNode, visitingNode.cost2come + new_step,
                                        calc_cost([new_x, new_y], goal, new_step), visitingNode.clear_val, new_x, new_y,
                                        new_theta)
                        if new_keys in accepted.keys():
                            if accepted[new_keys].cost2come + accepted[
                                new_keys].cost2go > new_node.cost2come + new_node.cost2go:
                                accepted[new_keys] = new_node
                                toBeVisited.put(new_node)
                                # Node.curve_list_start, Node.curve_list_end = plot_curve(visitingNode.x, visitingNode.y,
                                #                                                         visitingNode.theta, i[0], i[1],
                                #                                                         visitingNode.clear_val,
                                #                                                         Node.curve_list_start,
                                #                                                         Node.curve_list_end)
                        else:
                            accepted[new_keys] = new_node
                            toBeVisited.put(new_node)
                            # Node.curve_list_start, Node.curve_list_end = plot_curve(visitingNode.x, visitingNode.y,
                            #                                                         visitingNode.theta, i[0], i[1],
                            #                                                         visitingNode.clear_val,
                            #                                                         Node.curve_list_start,
                            #                                                         Node.curve_list_end)
                visited[(int(region[0]), int(region[1]), int(region[2]))] = visitingNode
        f.close()
        return False


# Function to check if point is within goal threshold
def goalReached(node, goal):
    if (node[0] - Decimal(goal[0])) ** 2 + (node[1] - Decimal(goal[1])) ** 2 < 1.5 ** 2:
        return True
    return False


# def plot_curve(Xi, Yi, Thetai, UL, UR, clear_val, curve_list_start, curve_list_end):
#     UL = UL * 2 * np.pi / 60
#     UR = UR * 2 * np.pi / 60
#     t = 0
#     r = Node.r
#     L = Node.L
#     dt = 0.1
#     Xn = Xi
#     Yn = Yi
#     Thetan = 3.14 * Thetai / 180
#     # Xi, Yi,Thetai: Input point's coordinates
#     # Xs, Ys: Start point coordinates for plot function
#     # Xn, Yn, Thetan: End point coordintes
#     while t < 1:
#         t = t + dt
#         Xs = Xn
#         Ys = Yn
#         Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
#         Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
#         Thetan += (r / L) * (UR - UL) * dt
#         if check_node([Xn, Yn], clear_val):
#             curve_list_start.append([Xs, Ys])
#             curve_list_end.append([Xn, Yn])
#     Thetan = 180 * (Thetan) / 3.14
    
#     return curve_list_start, curve_list_end


def check_node(node, clearance):
    # Checking border
    clearance = Decimal(clearance)
    node[0] = Decimal(node[0])
    node[1] = Decimal(node[1])
    # print(type(node[0]))
    if (node[0] / Decimal(10.0) < -Decimal(5.0) + Decimal(clearance) / Decimal(10.0)) or (node[0] / Decimal(10.0) > (Decimal(5.0) - Decimal(clearance) / Decimal(10.0))) or \
            (node[1] / Decimal(10.0) < -Decimal(5.0) + Decimal(clearance) / Decimal(10.0)) or (node[1] / Decimal(10.0) > Decimal(5.0) - Decimal(clearance) / Decimal(10.0)):
        return False
    
    # Bottom Left Circle
    circle_bottom_left = (node[0] / Decimal(10.0) - Decimal((-2.0))) ** 2 + (node[1] / Decimal(10.0) - Decimal((-3.0))) ** 2
    if circle_bottom_left < ((Decimal(1) + Decimal(clearance) / Decimal(10.0)) ** 2):
        return False
    
    # Bottom Right Circle
    circle_bottom_right = (node[0] / Decimal(10.0) - Decimal((2.0))) ** 2 + (node[1] / Decimal(10.0) - Decimal((-3.0))) ** 2
    if circle_bottom_right < ((Decimal(1) + Decimal(clearance) / Decimal(10.0)) ** 2):
        return False
    
    # Circle top right
    circle_top_right = (node[0] / Decimal(10.0) - Decimal((2.0))) ** 2 + (node[1] / Decimal(10.0) - Decimal((3.0))) ** 2
    if circle_top_right < ((Decimal(1) + Decimal(clearance) / Decimal(10.0)) ** 2):
        return False
    
    # Circle Center
    circle_center = (node[0] / Decimal(10.0) - Decimal((0.0))) ** 2 + (node[1] / Decimal(10.0) - Decimal((0.0))) ** 2
    if circle_center < ((Decimal(1) + Decimal(clearance) / Decimal(10.0)) ** 2):
        return False
    
    # Left Square
    if (node[0] / Decimal(10.0) > -(Decimal(4.75) + Decimal(clearance) / Decimal(10.0))) and (node[0] / Decimal(10.0) < -Decimal(3.25) + Decimal(clearance) / Decimal(10.0)) and \
            (node[1] / Decimal(10.0) < Decimal(0.75) + Decimal(clearance) / Decimal(10.0)) and (node[1] / Decimal(10.0) > -(Decimal(0.75) + Decimal(clearance) / Decimal(10.0))):
        return False
    
    # Left Top Square
    if (node[0] / Decimal(10.0) > -(Decimal(2.75) + Decimal(clearance) / Decimal(10.0))) and (node[0] / Decimal(10.0) < -Decimal(1.25) + Decimal(clearance) / Decimal(10.0)) \
            and (node[1] / Decimal(10.0) < Decimal(3.75) + Decimal(clearance) / Decimal(10.0)) and (node[1] / Decimal(10.0) > Decimal(2.25) - Decimal(clearance) / Decimal(10.0)):
        return False
    
    # Right square
    if (node[0] / Decimal(10.0) < Decimal(4.75) + Decimal(clearance) / Decimal(10.0)) and (node[0] / Decimal(10.0) > Decimal(3.25) - Decimal(clearance) / Decimal(10.0)) \
            and (node[1] / Decimal(10.0) < Decimal(0.75) + Decimal(clearance) / Decimal(10.0)) and (node[1] / Decimal(10.0) > -(Decimal(0.75) + Decimal(clearance) / Decimal(10.0))):
        return False
    
    return True


# Function to find selected optimal path
def generate_path(node, root):
    while (not np.array_equal(node.current, root)):
        dt = 1
        f = open("nodePath.txt", "r+")
        content = f.read()
        f.seek(0, 0)
        dx = (node.x - node.parent.x) / (dt * 10)
        dy = (node.y - node.parent.y) / (dt * 10)
        dtheta = (node.theta - node.parent.theta) / (dt * 10)
        toWrite = str(node.current)[1:-1] + ', ' + str(dx) + ', ' + str(dy) + ', ' + str(dtheta)
        node = node.parent
        f.write(toWrite + '\n' + content)
        f.close()
    
    f = open("nodePath.txt", "r+")
    content = f.read()
    f.seek(0, 0)
    dx = 0
    dy = 0
    dtheta = 0
    toWrite = str(node.current)[1:-1] + ', ' + str(dx) + ', ' + str(dy) + ', ' + str(dtheta)
    f.write(toWrite + '\n' + content)
    print(toWrite + '\n' + content)
    f.close()


# Function to generate points on a line
def get_line(x1, y1, x2, y2):
    points = []
    x1 = int(100 * (x1 + 5.1))
    y1 = int(100 * (y1 + 5.1))
    x2 = int(100 * (x2 + 5.1))
    y2 = int(100 * (y2 + 5.1))
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


def calc_cost(current, goal, step):
    return np.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)


def main():
    # All the values below are multiplied by 10 because of the scale factor of the grid
    # but for plotting purpose we scale the values by 100 for better plotting
    
    # Take the clearance value as input from the user
    clearance = float(input('Please enter robot clearance value: '))
    # Bot radius in meters and scaled to value of 10
    radius = 3.54 / 2
    step_size = 1
    # Multiplied by 10 to scale values by 10
    clearance = clearance * 10 + 1
    # Take the start point value as input from the user
    x = float(input('Please enter the x coordinate of the start point: '))
    y = float(input('Please enter the y coordinate of the start point: '))
    theta = float(input('Please enter the theta value of the robot (in deg): '))
    start_point = [-1*x * 10, -1*y * 10, theta]
    
    # Check if the start node is valid or not
    while not check_node(start_point, radius + clearance):
        print('Invalid start point given')
        x = float(input('Please enter the x coordinate of the start point: '))
        y = float(input('Please enter the y coordinate of the start point: '))
        theta = float(input('Please enter the theta value of the robot (in deg): '))
        start_point = [-1*x * 10, -1*y * 10, theta]
        # Take the goal point value as input from the user
    x_goal_point = float(input('Please enter the x coordinate of the goal point: '))
    y_goal_point = float(input('Please enter the y coordinate of the goal point: '))
    goal_point = [-1*x_goal_point * 10, -1*y_goal_point * 10]
    
    # Check if the goal node is valid or not
    while not check_node(goal_point, radius + clearance):
        print('Invalid end point given')
        x_goal_point = float(input('Please enter the x coordinate of the goal point: '))
        y_goal_point = float(input('Please enter the y coordinate of the goal point: '))
        goal_point = [-1*x_goal_point * 10, -1*y_goal_point * 10]
    rpm1 = float(input('Please enter value of RPM1: '))
    rpm2 = float(input('Please enter value of RPM2: '))
    start_time = time.time()
    start = Node(None, 0, calc_cost(start_point, goal_point, step_size), radius + clearance, start_point[0],
                 start_point[1], start_point[2])
    print('Finding path...')
    goal = start.astar(goal_point, step_size, rpm1, rpm2)
    if not goal:
        print('Path not found')
        exit(-1)
    open('nodePath.txt', 'w').close()
    end_time = time.time()
    print('Time taken to find path: ' + str(end_time - start_time))
    generate_path(goal, start_point)


if __name__ == '__main__':
    main()
