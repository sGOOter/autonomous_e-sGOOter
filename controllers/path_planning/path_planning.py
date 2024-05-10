import cv2
import numpy as np
from controller import Supervisor
from time import sleep
import math
from heapq import heappop, heappush


############################################################
#------------rOBOT sETUP--------------------------##########
############################################################
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robotNode = robot.getSelf()

preview = 0  # Mask Preview 1

Kp = 0.01
Ki = 0.02
Kd = 0.0001

P = 0
I = 0
D = 0
oldP = 0
PID = 0

# Bicycle speed and handlebar settings
maxS = 4  # max speed
minS = 2  # min speed
bcyS = maxS
acceleration = 2  # Acceleration factor
deceleration = 0.5  # Deceleration factor

hMax = 0.1  # rads (11°) Max
hndB = 0  # center initially
maxV = 0  # Max Velocity

# Motor and handlebar controls
whemotor = robot.getDevice('motor::crank')
whemotor.setPosition(float('inf'))
whemotor.setVelocity(maxS)  # Set initial velocity to move forward

hndmotor = robot.getDevice('handlebars motor')
hndmotor.setPosition(0)

# Keyboard setup
robot.keyboard.enable(timestep)

# Camera and display setup
camera = robot.getDevice('camera')
camera.enable(timestep*4)

display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0x00FF00)
display.setFont('Verdana', 16, True)

# LiDAR setup
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()




############################################################
#------------------ Path Planning -------------------------#
############################################################
import math
from heapq import heappop, heappush
def load_paths_from_file(input_file):
    all_paths = []
    current_path = []
    with open(input_file, 'r') as file:
        for line in file:
            if line.strip() == "# New Path":
                if current_path:
                    all_paths.append(current_path)
                    current_path = []
            else:
                parts = line.strip().split(',')
                if len(parts) == 2:
                    x, y = float(parts[0].strip()), float(parts[1].strip())
                    current_path.append((x, y))
        if current_path:  # Add the last path if file does not end with delimiter
            all_paths.append(current_path)
    return all_paths

# Now you can load all paths with a single call
all_paths = load_paths_from_file('/Users/apple/Desktop/robotic_project/sGOOter_project/controllers/path_planning/map_routes.txt')

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.neighbors = []
        self.cost = float('inf')
        self.estimate = float('inf')
        self.parent = None

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)
    
    def __lt__(self, other):
        return self.estimate < other.estimate

def euclidean_distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

def find_closest_node(nodes, x, y):
    min_distance = float('inf')
    closest_node = None
    for node in nodes:
        distance = euclidean_distance(node, Node(x, y))
        if distance < min_distance:
            min_distance = distance
            closest_node = node
    return closest_node

def connect_nodes(nodes, connection_range):
    for node in nodes:
        for other in nodes:
            if node != other and euclidean_distance(node, other) <= connection_range:
                node.add_neighbor(other)

def a_star(start, goal, y_constraint_length=20):
    open_set = []
    heappush(open_set, (start.estimate, start))
    start.cost = 0
    start.estimate = euclidean_distance(start, goal)

    while open_set:
        current = heappop(open_set)[1]

        if current == goal:
            potential_path = reconstruct_path(goal)
            if is_path_valid(potential_path, y_constraint_length):
                return potential_path
            else:
                continue  # Skip this path as it doesn't meet the y-value constraint

        for neighbor in current.neighbors:
            # Ensure the path remains valid with the y-direction constraint
            if len(reconstruct_path(current)) < y_constraint_length and neighbor.y < current.y:
                continue  # Skip adding this neighbor if it violates the increasing y constraint

            tentative_cost = current.cost + euclidean_distance(current, neighbor)
            if tentative_cost < neighbor.cost:
                neighbor.cost = tentative_cost
                neighbor.estimate = tentative_cost + euclidean_distance(neighbor, goal)
                neighbor.parent = current
                heappush(open_set, (neighbor.estimate, neighbor))

    return None  # Return None if no valid path is found


def reconstruct_path(goal):
    path = []
    current = goal
    while current:
        path.append((current.x, current.y))
        current = current.parent
    path.reverse()
    return path

def is_path_valid(path, y_constraint_length):
    # Check if the first 30 nodes (or the whole path if shorter) have increasing y values
    if len(path) >= y_constraint_length:
        constrained_part = path[:y_constraint_length]
    else:
        constrained_part = path

    return all(constrained_part[i][1] < constrained_part[i + 1][1] for i in range(len(constrained_part) - 1))



# Create all nodes for each path
all_nodes = [Node(coord[0], coord[1]) for path in all_paths for coord in path]

# Connect nodes within a specified connection range
connection_range = 5  # Set the connection range within which nodes can interconnect
connect_nodes(all_nodes, connection_range)


def follow_path():
    global path  # Refer to the global path variable
    print(path)
    if not path:
        print("No path or path completed")
        whemotor.setVelocity(0.0)  # Stop the scooter
        return

    # Get current position and orientation of the robot
    position = robotNode.getPosition()
    orientation = robotNode.getOrientation()
    current_pos = np.array([position[0], position[1]])  # X and Z coordinates in Webots are flat plane coordinates

    # Calculate the robot's current facing direction
    robot_front_vector = np.array([math.cos(orientation[3]), math.sin(orientation[3])])
    robot_angle = np.arctan2(robot_front_vector[1], robot_front_vector[0])

    # Process the path
    next_wp = np.array(path[0])
    print(next_wp)
    target_vector = next_wp - current_pos
    distance = np.linalg.norm(target_vector)
    angle_to_target = np.arctan2(target_vector[1], target_vector[0])

    # Calculate the needed change in heading
    angle_diff = angle_to_target - robot_angle
    print(angle_diff)
    # angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

    if distance < 5:  # Proximity threshold to consider waypoint "reached"
        print(f"Reached waypoint: {next_wp}")
        path.pop(0)  # Remove the waypoint that has been reached
        if not path:
            print("All waypoints reached")
            whemotor.setVelocity(0.0)  # Stop the scooter if all waypoints are reached
            return

    # Update to the next waypoint in path after popping
    if path:
        next_wp = np.array(path[0])

    # Steering control
    steering_adjustment = np.clip(angle_diff, -hMax, hMax)
    print(steering_adjustment)
    hndmotor.setPosition(steering_adjustment)

    # Speed control
    # speed = max(0.5, min(maxS, distance * 0.5))
    # whemotor.setVelocity(speed)


############################################################
#-------------Autonomous Control------------------##########
############################################################

def get_error():
    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_white, upper_white = np.array([0, 0, 200], dtype=np.uint8), np.array([180, 50, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            center_x = camera.getWidth() // 2
            return (cx - center_x) * 0.03
    return 0




############################################################
#--------------- Key Control----------------------##########
############################################################

def handle_keys():
    global bcyS, hndB, manual_control
    key = robot.keyboard.getKey()
    if key == -1:
        return False

    if key == ord('M'):
        manual_control = not manual_control
        print("Switched to", "manual" if manual_control else "automatic", "control")

    if key == 315:  # Up
        bcyS = maxS
    elif key == 317:  # Down
        bcyS = minS
    elif key == ord('S'):
        bcyS = 0

    if key == 314:  # Left
        hndB = hMax
    elif key == 316:  # Right
        hndB = -hMax
    else:
        hndB = 0  # Center

    whemotor.setVelocity(max(minS, min(bcyS, maxS)))
    hndmotor.setPosition(hndB)
    return True




############################################################
#---------------Other Setting --------------------##########
############################################################

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

stop_point = (-38, -86)
# Define a function to determine if near the stop point
def nearStopPoint():
    position = robotNode.getPosition()
    x, y = position[0], position[1]
    if abs(x - stop_point[0]) < 10 and abs(y - stop_point[1]) < 10:
        return True
    else:
        return False
    
# Main control loop
stopped = False
STEERING_RESET_TOLERANCE = 0.0015
manual_control = False
key_pressed = False
toggle_pressed = False
DECELERATION_THRESHOLD = 1

# Define the path globally
goal_x, goal_y = -39.51, -87.47  # Goal coordinates
goal_node = find_closest_node(all_nodes, goal_x, goal_y)
start_node = find_closest_node(all_nodes, 50.05, -106.24)
path = a_star(start_node,goal_node)
############################################################
#---------Main cONTROL loop ----------------------##########
############################################################


while robot.step(timestep) != -1:
    if manual_control or handle_keys():
        continue  # Skip automatic controls if in manual mode
    
    key = robot.keyboard.getKey()
    

    # Autonomous control based on line detection
    current_error = get_error()
    P = current_error
    I += (2 / 3) * P * (timestep / 1000)
    D = 0.5 * (P - oldP) / (timestep / 1000)
    PID = Kp * P + Ki * I + Kd * D
    oldP = P

    if abs(PID) < STEERING_RESET_TOLERANCE:
        hndB = 0
    else:
        max_handlebar_angle = np.clip(abs(PID), 0, -hMax)
        hndB = np.sign(PID) * max_handlebar_angle * 0.65
        
    # LiDAR obstacle detection
    range_image = lidar.getRangeImage()
    min_distance = min(range_image)

    # Determine behavior based on obstacle proximity
    if nearStopPoint():
        if min_distance < 2:
            whemotor.setVelocity(0)  # Stop the motor immediately
            continue  # Skip further processing in this cycle
    else:
        if min_distance < 7:
            whemotor.setVelocity(minS)  # Reduce speed but do not stop
            hndB = -hMax if min_distance == min(range_image[:len(range_image)//2]) else hMax  # Basic obstacle avoidance: turn away from obstacle
            hndmotor.setPosition(hndB)
            continue  # Skip further processing to prioritize obstacle avoidance

    if manual_control or handle_keys():
        continue
    
    follow_path()
        
    # Set motor and handlebar positions based on PID calculation
    if not min_distance < 1 or nearStopPoint():
        whemotor.setVelocity(max(minS, min(bcyS, maxS)))  # Only adjust speed if not reacting to an obstacle
    
    if bcyS == 0:
        whemotor.setVelocity(0)  # Ensure the motor is stopped when bcyS is 0
        continue
    hndmotor.setPosition(hndB)
