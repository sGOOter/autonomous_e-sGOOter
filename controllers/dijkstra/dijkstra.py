from time import sleep
import cv2
import numpy as np
from controller import Supervisor

# 定义图中的点
nodes = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'X', 'Y']

# 定义邻接矩阵表示图的连通性,1表示连通,0表示不连通
graph = [
    [0, 1, 0, 1, 1, 0, 0, 0, 1, 0],
    [1, 0, 1, 0, 0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 1, 0],
    [1, 0, 0, 0, 0, 1, 0, 1, 1, 0],
    [0, 1, 1, 0, 1, 0, 1, 0, 0, 1],
    [0, 0, 0, 1, 0, 1, 0, 1, 0, 1],
    [0, 0, 1, 0, 1, 0, 1, 0, 0, 0],
    [1, 0, 0, 1, 1, 0, 0, 0, 0, 1],
    [0, 0, 1, 0, 0, 1, 1, 0, 1, 0]
]

def floyd(graph):
    '''佛洛依德算法计算所有点对最短路径和路径'''
    n = len(graph)
    dist = [[float('inf')] * n for _ in range(n)]
    path = [[None] * n for _ in range(n)]
    
    for i in range(n):
        for j in range(n):
            if i == j:
                dist[i][j] = 0
                path[i][j] = [nodes[i]]
            elif graph[i][j]:
                dist[i][j] = graph[i][j]
                path[i][j] = [nodes[i], nodes[j]]
    
    for k in range(n):
        for i in range(n):
            for j in range(n):
                if dist[i][j] > dist[i][k] + dist[k][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    path[i][j] = path[i][k] + path[k][j][1:]
    
    return dist, path

# 计算所有点对最短路径和路径
shortest_paths, shortest_paths_nodes = floyd(graph)

# 打印结果
print("All-pairs shortest paths:")
print("    " + " ".join(nodes))
for i, row in enumerate(shortest_paths):
    print(nodes[i], row)

print("\nAll-pairs shortest paths with nodes:")    
for i in range(len(nodes)):
    for j in range(len(nodes)):
        print(f"{nodes[i]} -> {nodes[j]}: {' -> '.join(shortest_paths_nodes[i][j])}")

robot = Supervisor()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

robotNode = robot.getSelf()

preview = 0 # Mask Preview 1

Kp = 0.01
Ki = 0.02 
Kd = 0.0001

P = 0
I = 0
D = 0
oldP = 0
PID = 0

# Bicycle speed and handlebar settings
maxS = 4 # max speed 
minS = 2 # min speed
bcyS = maxS
acceleration = 2  # Acceleration factor
deceleration = 0.5  # Deceleration factor

hMax = 0.1 # rads (11°) Max
hndB = 0 # center initially 
maxV = 0 # Max Velocity

# Motor and handlebar controls
whemotor = robot.getDevice('motor::crank')
whemotor.setPosition(float('inf'))
whemotor.setVelocity(maxS)  # Set initial velocity to move forward

hndmotor = robot.getDevice('handlebars motor')
hndmotor.setPosition(0)

# Keyboard setup
robot.keyboard.enable(timestep)
robot.keyboard = robot.getKeyboard()

# Camera and display setup
camera = robot.getDevice('camera')
camera.enable(timestep*4)

display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0x00FF00)
display.setFont('Verdana', 16, True)

if preview == 1:
    cv2.startWindowThread()

last_error = 0  # Initialize the last error as a global variable outside the function

def getError():
    global last_error  # Declare it as global to modify the variable outside the function scope

    # Get the image from the robot's camera
    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))

    # Convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define thresholds for detecting white
    lower_white = np.array([0, 0, 200], dtype=np.uint8)
    upper_white = np.array([180, 50, 255], dtype=np.uint8)

    # Create mask based on thresholds
    mask = cv2.inRange(hsv, lower_white, upper_white)

 
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Find the largest contour assumed to be the line
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            # Calculate error based on the center of the line
            center_x = camera.getWidth() // 2
            last_error = (cx - center_x)*0.03  # Update the last known error
            # print('detect error:', last_error)
            return last_error
    # print('no detect error:')
    return 0  # Return the last known error if no line is detected



manual_control = True
def keyCtrl():
    global bcyS, hndB, manual_control
    key = robot.keyboard.getKey()

    if key == 315:
        print("forward")
        bcyS = maxS  # Increase speed by a factor
    elif key == 317:
        bcyS = minS  # Decrease speed by a factor
        print("backward")
    elif key == ord('S'):
        bcyS = 0  # Stop

    if key == 314:  # Left arrow
        hndB = hMax
        print("left")
    elif key == 316:  # Right arrow
        hndB = -hMax
    else:
        hndB = 0  # Center handlebar if no left/right keys are pressed

    whemotor.setVelocity(max(minS, min(bcyS, maxS)))  # Ensure velocity stays within bounds
    hndmotor.setPosition(hndB)
    
    
def hms(sec):
    # Convert seconds to hours:minutes:seconds
    h = sec // 3600
    m = (sec % 3600) // 60
    s = (sec % 3600) % 60
    return f'{h:02d}:{m:02d}:{s:02d}'

def printStatus():
    global maxV
    velo = robotNode.getVelocity()
    velocity = np.sqrt(np.sum(np.square(velo[:3]))) * 3.6  # Convert m/s to km/h
    if velocity > maxV:
        maxV = (velocity + maxV) / 2

    timer = int(robot.getTime())
    strP = hms(timer)
    
    robot.setLabel(0, f'Robot: {robot.getName()} Speed: {velocity:.2f} km/h Max: {maxV:.2f} km/h', 0, 0.05, 0.06, 0x000000, 0, 'Lucida Console')



STEERING_RESET_TOLERANCE = 0.0015

manual_control = False
key_pressed = False



stop_point = (-35, 84)  # 定义停车点的坐标(x, y)
def isAtStopPoint():
    # 获取机器人的当前位置
    position = robotNode.getPosition()
    x, y = position[0], position[1]  # 假设机器人在平面上移动,y坐标对应于3D空间的z坐标

    # 检查机器人是否在停车点附近(可以根据需要调整阈值)
    if abs(x - stop_point[0]) < 5 and abs(y - stop_point[1]) < 5:
        return True
    else:
        return False

# 定义节点的坐标位置
node_positions = {
    'A': (97.84, -37.29),  # A点的坐标(x, y)
    'B': (97.84, 89.50),  # B点的坐标(x, y)
    'C': (-35.53, 89.18),  # C点的坐标(x, y)
    'D': (-35.53, -37.29),  # D点的坐标(x, y)
    'E': (34.86, -100.35),  # E点的坐标(x, y)
    'F': (34.86, 33.08),  # F点的坐标(x, y)
    'G': (-94.84, 33.08),  # G点的坐标(x, y)
    'H': (-94.84, -92.09),  # H点的坐标(x, y)
    'X': (45.39, -46.16),  # X点的坐标(x, y)
    'Y': (-48.04, 44.51)   # Y点的坐标(x, y)
}

start_point = 'B'  # 定义起点为B
target_point = 'A'  # 定义目标点为A

# 获取从起点到目标点的最短路径节点列表
start_index = nodes.index(start_point)
target_index = nodes.index(target_point)
shortest_path = shortest_paths_nodes[start_index][target_index]

print(f"Shortest path from {start_point} to {target_point}: {' -> '.join(shortest_path)}")

# 初始化当前节点为起点
current_node = start_point
# Main control loop
stopped = False
while robot.step(timestep) != -1:
    #print('当前位置', robotNode.getPosition())
    
    # print(isAtStopPoint())
    key = robot.keyboard.getKey()
    if key != -1:
        pass  # to do
    else:
        # Autonomous control based on line detection
        current_error = getError()
        P = current_error
        I += (2 / 3) * P * (timestep / 1000)
        D = 0.5 * (P - oldP) / (timestep / 1000)
        
        PID = Kp * P + Ki * I + Kd * D
        oldP = P
        
        # Adjust steering based on the PID output
        if abs(PID) < STEERING_RESET_TOLERANCE:
            hndB = 0  # Reset handlebar to center if error is within tolerance
        else:
            # Adjust the maximum handlebar angle based on the error
            max_handlebar_angle = np.clip(abs(PID), 0, -hMax)  # Ensures the angle is within a dynamic range
            hndB = np.sign(PID) * max_handlebar_angle * 0.65  # Adjusts direction based on the sign of PID
        
        # 获取小车的当前位置
        position = robotNode.getPosition()
        x, y = position[0], position[1]

        # 检查小车是否到达了当前目标节点
        current_node_index = nodes.index(current_node)
        if abs(x - node_positions[current_node][0]) < 1 and abs(y - node_positions[current_node][1]) < 1:
            # 如果到达了当前节点,就更新目标节点为路径中的下一个节点
            current_node_index += 1
            if current_node_index < len(shortest_path):
                current_node = shortest_path[current_node_index]
            else:
                print("Reached the target point")
                bcyS = 0  # 停止小车
                break

        # 根据当前节点和目标节点,计算小车的运动方向
        target_node_position = node_positions[current_node]
        direction = np.arctan2(target_node_position[1] - y, target_node_position[0] - x)

        # 根据运动方向调整小车的转向角度
        hndB = direction

        # 调整速度
        whemotor.setVelocity(max(minS, min(bcyS, maxS)))  # 确保速度在合理范围内
        hndmotor.setPosition(hndB)


    printStatus()