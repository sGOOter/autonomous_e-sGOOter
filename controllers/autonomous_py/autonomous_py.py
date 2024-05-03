from controller import Robot, Camera, Display, GPS, Lidar, Keyboard
import math

# Constants and settings
TIME_STEP = 50
UNKNOWN = 99999.99

# PID coefficients
KP = 0.25
KI = 0.006
KD = 2
FILTER_SIZE = 3

# Initialize the Robot
robot = Robot()

# Enable devices
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
camera_width = camera.getWidth()
camera_height = camera.getHeight()
camera_fov = camera.getFov()

lidar = robot.getDevice('Sick LMS 291')
lidar.enable(TIME_STEP)
lidar_width = lidar.getHorizontalResolution()
lidar_range = lidar.getMaxRange()
lidar_fov = lidar.getFov()

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

display = robot.getDevice('display')
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# Helper functions
def process_camera_image(image):
    num_pixels = camera_height * camera_width
    REF = [95, 187, 203]  # BGR format for yellow
    sumx = 0
    pixel_count = 0

    for x in range(num_pixels):
        pixel = image[x]
        if color_diff(pixel, REF) < 30:
            sumx += x % camera_width
            pixel_count += 1

    if pixel_count == 0:
        return UNKNOWN
    return ((sumx / pixel_count / camera_width - 0.5) * camera_fov)

def color_diff(a, b):
    return sum(abs(a[i] - b[i]) for i in range(3))

def filter_angle(new_value):
    old_values = [0.0] * FILTER_SIZE
    if new_value == UNKNOWN:
        old_values = [0.0] * FILTER_SIZE
    else:
        old_values.pop(0)
        old_values.append(new_value)

    return sum(old_values) / FILTER_SIZE if new_value != UNKNOWN else UNKNOWN

# Main loop
while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()
    while key > 0:
        if key == keyboard.UP:
            speed += 5
        elif key == keyboard.DOWN:
            speed -= 5
        key = keyboard.getKey()

    camera_image = camera.getImage()
    lidar_data = lidar.getRangeImage()

    yellow_line_angle = filter_angle(process_camera_image(camera_image))
    # Implement similar logic to handle Lidar data and vehicle control as per your need

    # Display updates or other logic here
