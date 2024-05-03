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
maxS = 6 # max speed
minS = 3 # min speed
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

def getError(act_error):
    # This function would include your image processing logic
    return 0  # Placeholder return

def keyCtrl():
    global bcyS, hndB
    key = robot.keyboard.getKey()

    if key == 315:
        bcyS = maxS  # Increase speed by a factor
    elif key == 317:
        bcyS = minS  # Decrease speed by a factor
    elif key == ord('S'):
        bcyS = 0  # Stop

    if key == 314:  # Left arrow
        hndB = hMax
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

# Main loop
while robot.step(timestep) != -1:
    keyCtrl()  # Handle direction and speed control via keyboard

    P = getError(P)
    I += (2 / 3) * P * (timestep / 1000)
    D = 0.5 * (P - oldP) / (timestep / 1000)
    
    PID = Kp * P + Ki * I + Kd * D
    oldP = P

    hndmotor.setPosition(hndB + PID)  # Adjust handlebar based on PID and key control

    printStatus()