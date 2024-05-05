from time import sleep
import cv2
import numpy as np
from controller import Supervisor

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



manual_control = False
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

# Main control loop
stopped = False
while robot.step(timestep) != -1:
    #print('当前位置', robotNode.getPosition())
    
    print(isAtStopPoint())
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
        
        # Check if the robot has reached the stop point
        if isAtStopPoint():
            print('Reached the stop point')
            bcyS = 0  # Stop the robot if it's at the stop point
            if(stopped == False):
                sleep(5)
                stopped = True
                bcyS = maxS
            
        else:
            # Adjust velocity based on the line detection
            whemotor.setVelocity(max(minS, min(bcyS, maxS)))  # Ensure velocity stays within bounds
        
        hndmotor.setPosition(hndB)  

    printStatus()


