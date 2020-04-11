# *********************************** Rigid Robot ********************************#
import numpy as np
import queue
import math
import time
import cv2
import matplotlib.pyplot as plt

class Node():
    # Radious of wheeles of turtlebot, divided by 1000 to conver to metres
    r = 38*10/1000 
    # Radius / Distance between the wheels of the turtlebot, divided by 1000 to conver to metres
    L = 354*10/1000 
    curve_list_start = []
    curve_list_end = []
    def __init__(self, parent, cost2come, cost2go, clear_val, x, y, theta):
        self.parent = parent
        self.cost2come = cost2come
        self.cost2go = cost2go
        self.clear_val = clear_val
        self.x = x
        self.y = y
        self.theta = theta
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
        LW = LW * 2 * np.pi / 60
        RW = RW * 2 * np.pi / 60
        dt = 0.1
        t = 0
        theta = np.deg2rad(self.theta)
        accum_dx = 0
        accum_dy = 0
        step = 0
        # Loop to iterate 10 steps over 1sec time period
        while t < 1:
            t += dt
            dx = (Node.r / 2) * (LW + RW) * math.cos(theta) * dt
            dy = (Node.r / 2) * (LW + RW) * math.sin(theta) * dt
            accum_dx += dx
            accum_dy += dy
            dtheta = (Node.r / Node.L) * (RW - LW) * dt
            theta += dtheta
            node = [self.x + accum_dx, self.y + accum_dy]
            if not check_node(node, self.clear_val):
                break
            step += np.sqrt(dx ** 2 + dy ** 2)

        return self.x + accum_dx, self.y + accum_dy, np.rad2deg(theta), step

    def findRegion(self, current):
        region = [current[0], current[1], (current[2] + 360) % 360]
        return region
    
    def astar(self, goal, step_size, rpm1, rpm2):

        visited = {}
        accepted = {}
        toBeVisited = queue.PriorityQueue()
        toBeVisited.put(self)
        region = self.findRegion(self.current)  # creating unique indexing value
        accepted[(int(region[0]), int(region[1]), int(region[2]))] = self
        steps_with_cost = self.possible_steps(rpm1, rpm2)
        j = 0
        while not toBeVisited.empty():
            visitingNode = toBeVisited.get()
            node = visitingNode.current
            region = self.findRegion(node)  # creating unique indexing value
            key = (int(region[0]), int(region[1]), int(region[2]))

            if key in visited.keys():  # check if node already visited
                continue
            else:
                if goalReached(node, goal):  # check if goal found
                    return visitingNode
                
                for i in steps_with_cost:
                    new_x, new_y, new_theta, new_step = visitingNode.do_action(i[0], i[1])
                    new_region = self.findRegion([new_x, new_y, new_theta])
                    new_keys = (int(new_region[0]), int(new_region[1]), int(new_region[2]))

                    # Check if the new node is already visited or not
                    if check_node([new_x, new_y], visitingNode.clear_val) and (new_keys not in visited.keys()):
                        new_node = Node(visitingNode, visitingNode.cost2come + new_step, calc_cost([new_x, new_y], goal,new_step), visitingNode.clear_val, new_x, new_y, new_theta)
                        if new_keys in accepted.keys():
                            if accepted[new_keys].cost2come + accepted[new_keys].cost2go > new_node.cost2come + new_node.cost2go:
                                accepted[new_keys] = new_node
                                toBeVisited.put(new_node)
                                Node.curve_list_start, Node.curve_list_end = plot_curve(visitingNode.x, visitingNode.y, visitingNode.theta, i[0], i[1], visitingNode.clear_val, Node.curve_list_start, Node.curve_list_end)
                        else:
                            accepted[new_keys] = new_node
                            toBeVisited.put(new_node)
                            Node.curve_list_start, Node.curve_list_end = plot_curve(visitingNode.x, visitingNode.y, visitingNode.theta, i[0], i[1], visitingNode.clear_val, Node.curve_list_start, Node.curve_list_end)
                visited[(int(region[0]), int(region[1]), int(region[2]))] = visitingNode

        return False

# Function to check if point is within goal threshold
def goalReached(node, goal):
    if (node[0]-goal[0])**2 + (node[1]-goal[1])**2 < 1.5**2:
        return True
    return False

def plot_curve(Xi,Yi,Thetai,UL,UR, clear_val, curve_list_start, curve_list_end):
    UL = UL * 2 * np.pi / 60
    UR = UR * 2 * np.pi / 60
    t = 0
    r = Node.r
    L = Node.L
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        if check_node([Xn, Yn], clear_val):
            curve_list_start.append([Xs, Ys])
            curve_list_end.append([Xn, Yn])
    Thetan = 180 * (Thetan) / 3.14

    return curve_list_start, curve_list_end

def check_node(node, clearance):
   
    # Checking border
    if (node[0]/10.0 < -5.0+clearance/10) or (node[0]/10.0 > (5.0-clearance/10)) or \
    					(node[1]/10.0 < -5.0+clearance/10) or (node[1]/10.0 > 5.0-clearance/10):

        return False

    # Bottom Left Circle
    circle_bottom_left = (node[0]/10.0 - (-2.0))**2 + (node[1]/10.0 - (-3.0))**2
    if  circle_bottom_left < ((1+clearance/10)**2):


        return False

    # Bottom Right Circle
    circle_bottom_right = (node[0]/10.0 - (2.0))**2 + (node[1]/10.0 - (-3.0))**2
    if  circle_bottom_right < ((1+clearance/10)**2):


        return False

    # Circle top right
    circle_top_right = (node[0]/10.0 - (2.0))**2 + (node[1]/10.0 - (3.0))**2
    if circle_top_right < ((1+clearance/10)**2):

        return False

    # Circle Center
    circle_center = (node[0]/10.0 - (0.0))**2 + (node[1]/10.0 - (0.0))**2
    if circle_center < ((1+clearance/10)**2):


        return False

    # Left Square
    if (node[0]/10.0 > -(4.75+clearance/10)) and (node[0]/10.0 < -3.25+clearance/10) and \
    			(node[1]/10.0 < 0.75+clearance/10) and (node[1]/10.0 > -(0.75 + clearance/10)):

        return False

    # Left Top Square
    if (node[0]/10.0 > -(2.75 + clearance/10)) and (node[0]/10.0 < -1.25 + clearance/10) \
    and (node[1]/10.0 < 3.75 + clearance/10) and (node[1]/10.0 > 2.25 - clearance/10):

        return False

    # Right square
    if (node[0]/10.0 < 4.75 + clearance/10) and (node[0]/10.0 > 3.25 - clearance/10) \
    	and (node[1]/10.0 < 0.75 + clearance/10) and (node[1]/10.0 > -(0.75 + clearance/10)):

        return False

    return True

# Function to find selected optimal path
def generate_path(node, root):
    while (not np.array_equal(node.current, root)):
        dt = 1
        f = open("nodePath.txt", "r+")
        content = f.read()
        f.seek(0, 0)
        dx = (node.x - node.parent.x)/(dt*10) 
        dy = (node.y - node.parent.y)/(dt*10) 
        dtheta = (node.theta - node.parent.theta)/(dt*10) 
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


def calc_cost(current, goal, step):
    return np.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

def main():

    # All the values below are multiplied by 10 because of the scale factor of the grid 
    # but for plotting purpose we scale the values by 100 for better plotting
    
    # Take the clearance value as input from the user
    clearance = eval(input('Please enter robot clearance value: '))
    # Bot radius in meters and scaled to value of 10
    radius = 3.54/2 
    step_size = 1
    # Multiplied by 10 to scale values by 10
    clearance = clearance*10 
    # Take the start point value as input from the user
    start_point = eval(input('Please enter the start point in this format - [x,y,theta (in deg)]: '))
    start_point = [start_point[0]*10, start_point[1]*10, start_point[2]] 

    # Check if the start node is valid or not
    while not check_node(start_point, radius + clearance):
        print('Invalid start point given')
        start_point = eval(input('Please enter the start point in this format - [x,y,theta (in deg)]: '))
        start_point = [start_point[0] * 10, start_point[1] * 10, start_point[2]] 

    # Take the goal point value as input from the user
    goal_point = eval(input('Please enter the goal point in this format - [x,y]: '))
    goal_point = [goal_point[0]*10, goal_point[1]*10] 

    # Check if the goal node is valid or not
    while not check_node(goal_point, radius + clearance):
        print('Invalid end point given')
        goal_point = eval(input('Please enter the goal point in this format - [x,y]: '))
        goal_point = [goal_point[0] * 10, goal_point[1] * 10] 
    rpm1 = eval(input('Please enter value of RPM1: '))
    rpm2 = eval(input('Please enter value of RPM2: '))
    # print('The goal point you gave is:', goal_point)
    start_time = time.time()
    start = Node(None, 0, calc_cost(start_point, goal_point, step_size), radius + clearance, start_point[0], start_point[1], start_point[2])
    print('Finding path...')
    goal = start.astar(goal_point, step_size, rpm1, rpm2)
    if not goal:
        print('Path not found')
        exit(-1)
    open('nodePath.txt', 'w').close()
    generate_path(goal, start_point)
    end_time = time.time()
    print('Time taken to find path: ' + str(end_time - start_time))
    grid = np.ones((1021, 1021, 3), dtype=np.uint8) * 255 
    lines = []
    
    # Square equations in the grid
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

    # Border
    grid[:, 0:10] = [0,0,0] 
    grid[:, -10:] = [0,0,0] 
    grid[0:10, :] = [0,0,0] 
    grid[-10:, :] = [0,0,0] 
    
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
    
    # Video writing initialzaion
    vidWriter = cv2.VideoWriter("./video_output.mp4",cv2.VideoWriter_fourcc(*'mp4v'), 500, (1021, 1021))
    for line in lines:
        for l in line:
            grid[l[1]][l[0]] = [0, 0, 0]

    pr_point = start_point
    # Plot the explored nodes with dt = 0.1 in time period of 1
    for i in range(len(Node.curve_list_start)):

        grid = cv2.line(grid, (int(float(Node.curve_list_start[i][0])*10) + 510, int(float(Node.curve_list_start[i][1])*10) + 510), 
                               (int(float(Node.curve_list_end[i][0])*10) + 510, int(float(Node.curve_list_end[i][1])*10) + 510), (255, 0, 0), 1) 
        # Write every tenth frame for faster video writing
        if i % 10 == 0:
            vidWriter.write(np.flip(grid, 0))

    file = open('nodePath.txt', 'r')
    points = file.readlines()
    pr_point = start_point

    # Draw the final path
    for point in points:

        pts = point.split(',')
        grid = cv2.arrowedLine(grid, (int(float(pr_point[0])*10) + 510, int(float(pr_point[1])*10) + 510), 
                               (int(float(pts[0])*10) + 510, int(float(pts[1])*10) + 510), (0, 255, 0), 1) 
        pr_point = pts
        vidWriter.write(np.flip(grid, 0))

    # Added 2000 frames so that the path can be seen in the end
    for i in range(2000):
        vidWriter.write(np.flip(grid, 0))
    vidWriter.release()
    graph_end_time = time.time()
    print('Time taken to animate paths: ' + str(graph_end_time - end_time))


if __name__ == '__main__':
    main()
