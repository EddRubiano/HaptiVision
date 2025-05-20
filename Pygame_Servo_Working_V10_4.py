import pygame
import numpy as np
import threading
import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from smbus2 import SMBus, i2c_msg
from statistics import mean
from time import sleep

# Servo setup
address = 0x10
factory = PiGPIOFactory()
servo1 = Servo(17, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)
servo2 = Servo(18, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

# Vibration motor GPIOs
LEFT_MOTOR = 5
CENTER_MOTOR = 13
RIGHT_MOTOR = 6
motor_pins = {"left": LEFT_MOTOR, "center": CENTER_MOTOR, "right": RIGHT_MOTOR}

# Zone to motor map
zone_motor_map = {
    "1st": "left", "6th": "left", "7th": "left",
    "2nd": "center", "5th": "center", "8th": "center",
    "3rd": "right", "4th": "right", "9th": "right"
}

# GPIO Setup
GPIO.setmode(GPIO.BCM)
for pin in motor_pins.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Pygame init
pygame.init()
WIDTH, HEIGHT = 1720, 1000
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("HaptiVision Point Cloud + Navigation Zones V.1.0")

# Colors
BLACK = (0, 0, 0) # Set different colors to be use
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GREY = (50, 50, 50)
GREY2 = (128, 128, 128)
WHITE = (255, 255, 255)
MAGENTA = (255, 0, 255)
YELLOW = (255, 255, 102) 
RED = (255, 102, 102) 
CENTER = (600, HEIGHT - 50)

zone_1, zone_2, zone_3 = [], [], [] # Distance storage in 12 different arrays
zone_4, zone_5, zone_6 = [], [], []
zone_7, zone_8, zone_9 = [], [], []
zone_10, zone_11, zone_12 = [], [], []

# Zones
zone_labels = [f"{i+1}st" if i == 0 else f"{i+1}th" for i in range(12)]
zone_colors = {label: GREEN for label in zone_labels}

 # Angle ranges for each zone (degrees) 12 in total ( 4 different passes)
zone_angles = {       
    "1st": (135, 105),
    "2nd": (105, 75),
    "3rd": (75, 45),
    "4th": (45, 75),
    "5th": (75, 105),
    "6th": (105, 135),
    "7th": (135, 105),
    "8th": (105, 75),
    "9th": (75, 45),
    "10th": (45, 75),
    "11th": (75, 105),
    "12th": (105, 135)
}

def polar_to_screen(center, angle_deg, distance, scale=1): # Converts polar to screen rectangular to draw with pygame x, y
    
    if distance < 400:
        angle_rad = np.radians(angle_deg)
        x = center[0] + scale * distance * 2 * np.cos(angle_rad)
        y = center[1] - scale * distance * 2 * np.sin(angle_rad)
        return int(x), int(y)
    else:
        angle_rad = np.radians(angle_deg) # Everything above 4 meters is limited to 4 meters for ease of visualization 
        x = center[0] + scale * 400 * 2 * np.cos(angle_rad)
        y = center[1] - scale * 400 * 2 * np.sin(angle_rad)
        return int(x), int(y)

def get_color_for_distance(d): # Function Determine the color of the points in the points cloud
    
    if idx == 0 or idx ==1  or idx ==2 or idx ==3 or idx == 4 or idx==5 or idx == 9 or idx==10 or idx==11:
        if d < 100:
            return RED
        elif 100 <= d < 200:
            return YELLOW
        elif 200 <= d < 400:
            return GREEN
        else:
            return GREEN  # Anything beyond 2m
    elif idx == 6 or idx ==7  or idx ==8:
        if d < 80:         ##### Test bench values
            return RED
        elif d > 200:      ####### Code to detect a Hole or Gap in front
            return GREY
        else:
            return GREEN
        

def read_lidar_points(write, read, count=20, max_retries=3):
    distance_values = []
    attempt = 0
    while attempt < max_retries:
        try:
            with SMBus(1) as bus:
                for _ in range(count):
                    bus.i2c_rdwr(write, read)
                    data = list(read)
                    dist = ((data[3] << 8) | data[2])
                    distance_values.append(dist)
                    sleep(0.008)
            return distance_values
        except OSError as e:
            print(f"[Retry {attempt+1}/{max_retries}] I2C Error: {e}")
            attempt += 1
            sleep(0.01)
    return [0] * count

servo1.value = 0.5 # Initial servo positions (Home)
servo2.value = 0.1
sleep(0.5)

zone_1 = np.array([[1330, 120], [1430, 120], [1430, 220], [1330, 220]]) # Square 1 vertices (top-left, top-right, bottom-right, bottom-left) for visualization
zone_2 = np.array([[1440, 120], [1540, 120], [1540, 220], [1440, 220]])
zone_3 = np.array([[1550, 120], [1650, 120], [1650, 220], [1550, 220]])
zone_4 = np.array([[1330, 230], [1430, 230], [1430, 330], [1330, 330]])
zone_5 = np.array([[1440, 230], [1540, 230], [1540, 330], [1440, 330]])
zone_6 = np.array([[1550, 230], [1650, 230], [1650, 330], [1550, 330]])

zone_7 = np.array([[1325, 360], [1426, 360], [1356, 430], [1255, 430]]) # Rhomboid 1 vertices (top-left, top-right, bottom-right, bottom-left) for visualization
zone_8 = np.array([[1440, 360], [1537, 360], [1467, 430], [1370, 430]])
zone_9 = np.array([[1551, 360], [1650, 360], [1580, 430], [1481, 430]])

zone_labels = ["1st", "2nd", "3rd", "4th", "5th", "6th",
               "7th", "8th", "9th", "10th", "11th", "12th"]

zone_to_points = {label: [] for label in zone_labels}

zone_colors = { # Dictionary to store colors for the Navigation Visualization
    "1st": GREEN,
    "2nd": GREEN,
    "3rd": GREEN,
    "4th": GREEN,
    "5th": GREEN,
    "6th": GREEN,
    "7th": GREEN,
    "8th": GREEN,
    "9th": GREEN,
    "10th": GREEN,
    "11th": GREEN,
    "12th": GREEN,
}


def vibration_feedback():
    while True:
        motor_states = {label: GPIO.LOW for label in motor_pins}
        for zone, color in zone_colors.items():
            motor = zone_motor_map.get(zone)
            if not motor:
                continue
            if color == RED:
                motor_states[motor] = GPIO.HIGH
            elif color == YELLOW:
                GPIO.output(motor_pins[motor], GPIO.HIGH)
                sleep(0.1)
                GPIO.output(motor_pins[motor], GPIO.LOW)
            elif color == GREY:
                for _ in range(2):
                    GPIO.output(motor_pins[motor], GPIO.HIGH)
                    sleep(0.05)
                    GPIO.output(motor_pins[motor], GPIO.LOW)
                    sleep(0.05)
        for label, pin in motor_pins.items():
            GPIO.output(pin, motor_states[label])
        sleep(0.1)

def draw_zones():
    clock = pygame.time.Clock()
    while True:
        screen.fill(BLACK)
        for i, shape in enumerate(zone_shapes):
            label = zone_labels[i]
            color = zone_colors.get(label, GREEN)
            pygame.draw.polygon(screen, color, shape, 0)
            pygame.draw.polygon(screen, WHITE, shape, 2)
        pygame.display.flip()
        clock.tick(30)

# Start threads
threading.Thread(target=vibration_feedback, daemon=True).start()
threading.Thread(target=draw_zones, daemon=True).start()

# Home servos
servo1.value = 0.5
servo2.value = 0.1
sleep(0.5)

# Main loop
positions = [
    (0.16, 0.125, "1st"), (-0.16, 0.125, "2nd"), (-0.5, 0.125, "3rd"),
    (-0.16, 0.125, "4th"), (0.16, 0.125, "5th"), (0.5, 0.125, "6th"),
    (0.16, 0.125, "7th"), (-0.16, 0.125, "8th"), (-0.5, 0.125, "9th"),
    (-0.16, 0.125, "10th"), (0.16, 0.125, "11th"), (0.5, 0.125, "12th")
]

while True:
    for idx, (pos, delay, label) in enumerate (positions):
        print(f"\nLiDAR Zone: {label}")
        servo1.value = pos
        write = i2c_msg.write(address, [1, 2, 7])
        read = i2c_msg.read(address, 7)
        dist = read_lidar_points(write, read)
        sleep(delay)
        avg_distance = mean(dist)
        print(f"Zone {label} distances: {dist}")
        print(f"Average distance for Zone {label}: {avg_distance:.2f} mm")

            
        if label == "1st": # Code to store the color values of each one of the zones for zone_colors dictionary
            count_1 = sum(1 for v in dist if 100 <= v < 200)
            count_2 = sum(1 for v in dist if v < 100)
            if count_2 >= 3:
                print("Between 200 and 100" )
                zone_colors["1st"] = RED
                #color = RED
            elif count_1 >= 3:
                print("Less than 100")
                zone_colors["1st"] = YELLOW
                #color = YELLOW
            else:
                zone_colors["1st"] = GREEN
                #color = GREEN
   
        elif label == "2nd": 
            count_1 = sum(1 for v in dist if 100 <= v < 200)
            count_2 = sum(1 for v in dist if v < 100)
            if count_2 >= 3:
                zone_colors["2nd"] = RED
                #color = RED
            elif count_1 >= 3:
                zone_colors["2nd"] = YELLOW
                #color = YELLOW
            else:
                zone_colors["2nd"] = GREEN
                #color = GREEN
       
        elif label == "3rd": 
            count_1 = sum(1 for v in dist if 100 <= v < 200)
            count_2 = sum(1 for v in dist if v < 100)
            if count_2 >= 3:
                zone_colors["3rd"] = RED
                #color = RED
            elif count_1 >= 3:
                zone_colors["3rd"] = YELLOW
                #color = YELLOW
            else:
                zone_colors["3rd"] = GREEN
                #color = GREEN
            
        elif label == "4th": 
            count_1 = sum(1 for v in dist if 100 <= v < 200)
            count_2 = sum(1 for v in dist if v < 100)
            if count_2 >= 3:
                zone_colors["4th"] = RED
                #color = RED
            elif count_1 >= 3:
                zone_colors["4th"] = YELLOW
                #color = YELLOW
            else:
                zone_colors["4th"] = GREEN
                #color = GREEN
                
        elif label == "5th": 
            count_1 = sum(1 for v in dist if 100 <= v < 200)
            count_2 = sum(1 for v in dist if v < 100)
            if count_2 >= 3:
                zone_colors["5th"] = RED
                #color = RED
            elif count_1 >= 3:
                zone_colors["5th"] = YELLOW
                #color = YELLOW
            else:
                zone_colors["5th"] = GREEN
                #color = GREEN
                
        elif label == "6th": 
            count_1 = sum(1 for v in dist if 100 <= v < 200)
            count_2 = sum(1 for v in dist if v < 100)
            if count_2 >= 3:
                zone_colors["6th"] = RED
                #color = RED
            elif count_1 >= 3:
                zone_colors["6th"] = YELLOW
                #color = YELLOW
            else:
                zone_colors["6th"] = GREEN
                #color = GREEN
                
        elif label == "7th":  # These are the zones to detect the floor in front, Test Bench values, they need to be replaced for a stand up position 
            count_1 = sum(1 for v in dist if v < 80) # Here we know that everything would be below 200mm, reason why we are only detecting distances < 100mm
            count_2 = sum(1 for v in dist if v > 200) # Anything above a Trheshold, will be consider a GAP in front of the person if v > 150
            if count_1 >= 3:                          # Theres no warning for these zones 7th, 8th, 9th
                zone_colors["7th"] = RED
                #color = RED
            elif count_2 >= 3:
                zone_colors["7th"] = GREY
                #color = GREY
            else:
                zone_colors["7th"] = GREEN
                #color = GREEN
        
        elif label == "8th":   
            count_1 = sum(1 for v in dist if v < 80) 
            count_2 = sum(1 for v in dist if v > 200) 
            if count_1 >= 3:                          
                zone_colors["8th"] = RED
                #color = RED
            elif count_2 >= 3:
                zone_colors["8th"] = GREY
                #color = GREY
            else:
                zone_colors["8th"] = GREEN
                #color = GREEN
                
        elif label == "9th":   
            count_1 = sum(1 for v in dist if v < 80) 
            count_2 = sum(1 for v in dist if v > 200) 
            if count_1 >= 3:                          
                zone_colors["9th"] = RED
                #color = RED
            elif count_2 >= 3:
                zone_colors["9th"] = GREY
                #color = GREY
            else:
                zone_colors["9th"] = GREEN
                #color = GREEN
                
        zone_to_points[label].clear() # Clear the previous points of that zone

        start_angle, end_angle = zone_angles[label] # Calculate new points for this zone
        angle_step = (end_angle - start_angle) / len(dist)

        for i, d in enumerate(dist):
            angle = start_angle + i * angle_step
            scaled_d = min(d, 350)
            point = polar_to_screen(CENTER, angle, scaled_d, scale=1)
            #zone_to_points[label].append((point, color))
            zone_to_points[label].append((point, get_color_for_distance(d)))
    
     # Draw Points Clous + Navigation Zones Visualizations
        screen.fill(BLACK)
        pygame.draw.circle(screen, GREY, CENTER, 600, 1)
        pygame.draw.circle(screen, GREY, CENTER, 400, 1)
        pygame.draw.circle(screen, GREY, CENTER, 200, 1)
        pygame.draw.line(screen, GREY, CENTER, (777,260))
        pygame.draw.line(screen, GREY, CENTER, (90,460))
        pygame.draw.line(screen, GREY, CENTER, (403,260))
        pygame.draw.line(screen, GREY, CENTER, (1120,440))
        pygame.draw.line(screen, GREY, (000,950), (1200,950))
        
        font1 = pygame.font.SysFont(None, 45)
        font2 = pygame.font.SysFont(None, 25)
        font3 = pygame.font.SysFont(None, 22)
        
        screen.blit(font1.render("POINT CLOUD V.1.0", True, WHITE), (450,15))
        screen.blit(font2.render("3m", True, WHITE), (1170,955))
        screen.blit(font2.render("2m", True, WHITE), (970,955))
        screen.blit(font2.render("1m", True, WHITE), (770,955))
        screen.blit(font2.render("0m", True, WHITE), (570,955))
        screen.blit(font3.render("-45°", True, GREY2), (100,450))
        screen.blit(font3.render("-15°", True, GREY2), (415,250))
        screen.blit(font3.render("15°", True, GREY2), (750,250))
        screen.blit(font3.render("45°", True, GREY2), (1085,440))
      
        
        screen.blit(font1.render("NAVIGATION ZONES V.1.0", True, WHITE), (1250, 15))
        screen.blit(font3.render("z", True, GREY2), (1309, 95))
        screen.blit(font3.render("x", True, GREY2), (1658, 330))
        screen.blit(font3.render("y", True, GREY2), (1230, 400))
        
        screen.blit(font2.render("COLOR Coding Key: ", True, WHITE), (1260, 510))
        screen.blit(font3.render("Green =  No Obstacles Detected, Distance > 2 Meters", True, GREEN), (1260, 540))
        screen.blit(font3.render("Yellow =  Obstacle Detected, Distance <= 2 Meters", True, YELLOW), (1260, 560))
        screen.blit(font3.render("Red =  Obstacle Detected, Distance <= 1 Meter", True, RED), (1260, 580))
        screen.blit(font3.render("Grey =  GAP Detected, Floor Level", True, GREY2),(1260, 600))
        
        screen.blit(font2.render("SHAPE Coding Key (Markers):", True, WHITE), (1260,660))
        screen.blit(font3.render("X (Diagonal Cross) =  Top Tilt Level ", True, BLUE), (1260,690))
        screen.blit(font3.render("+ (Orthogonal Cross) =  Middle Tilt Level ", True, BLUE), (1260,710))
        screen.blit(font3.render("O (Circle) =  Bottom Tilt Level", True, BLUE), (1260,730))
        screen.blit(font3.render("NOTE 1: Color encoding is used to visually represent distance", True, WHITE), (1220,820))
        screen.blit(font3.render("thresholds, While different geometric markers represent 3 different", True, WHITE), (1220,840))
        screen.blit(font3.render("vertical positions at where the LiDAR is aiming.", True, WHITE), (1220,860))
        screen.blit(font3.render("NOTE 2: Zones 7, 8 and 9 do not include a warning region (Yellow). ", True, WHITE), (1220,900))
        screen.blit(font3.render("NOTE 3: Distances above 4 meters are limited to 4 meters. ", True, WHITE), (1220,940))

    
        pygame.draw.line(screen, GREY2,(1305, 105), (1305, 345)) # Draw Lines for the 3D Plot
        pygame.draw.line(screen, GREY2,(1305, 345), (1660, 345)) 
        pygame.draw.aaline(screen, GREY2,(1305, 345), (1228, 425)) 
 
        #pygame.draw.rect(screen, GREY2, (60, 100, 210, 95), width=2) # Draw Grey Frames
        pygame.draw.rect(screen, GREY2, (25, 70, 1180, 910), width=2)
        #pygame.draw.rect(screen, GREY2, (1250, 865, 460, 95), width=2)
        pygame.draw.rect(screen, GREY2, (1220, 70, 470, 410), width=2)
        
        pygame.draw.polygon(screen, zone_colors["1st"], zone_1) # Remember its color until its get updated with LiDAR dist values
        pygame.draw.polygon(screen, zone_colors["2nd"], zone_2) # Draw the Rhomboids using the vertices
        pygame.draw.polygon(screen, zone_colors["3rd"], zone_3)
        pygame.draw.polygon(screen, zone_colors["4th"], zone_4)
        pygame.draw.polygon(screen, zone_colors["5th"], zone_5)
        pygame.draw.polygon(screen, zone_colors["6th"], zone_6)
        pygame.draw.polygon(screen, zone_colors["7th"], zone_7)
        pygame.draw.polygon(screen, zone_colors["8th"], zone_8)
        pygame.draw.polygon(screen, zone_colors["9th"], zone_9)
        
        
        screen.blit(font3.render("Zone 1", True, GREY2), (1355, 200))
        screen.blit(font3.render("Zone 2", True, GREY2), (1465, 200))
        screen.blit(font3.render("Zone 3", True, GREY2), (1575, 200))
        screen.blit(font3.render("Zone 6", True, GREY2), (1355, 310))
        screen.blit(font3.render("Zone 5", True, GREY2), (1465, 310))
        screen.blit(font3.render("Zone 4", True, GREY2), (1575, 310))
        screen.blit(font3.render("Zone 7", True, GREY2), (1300, 410))
        screen.blit(font3.render("Zone 8", True, GREY2), (1410, 410))
        screen.blit(font3.render("Zone 9", True, GREY2), (1520, 410))
        
        
        for label, zone_points in zone_to_points.items():
            for point, color in zone_points:
                if label in ("1st", "2nd", "3rd"):  # Draw 2 lines to form a diagonal cross
                    pygame.draw.line(screen, color, (point[0] - 3, point[1] - 3), (point[0] + 3, point[1] + 3), 1)
                    pygame.draw.line(screen, color, (point[0] - 3, point[1] + 3), (point[0] + 3, point[1] - 3), 1)
                    
                elif label in ("4th", "5th", "6th"):  # Draw 2 lines to form a orthogonal cross
                    pygame.draw.rect(screen, color, [point[0], point[1], 1, 8], 1)
                    pygame.draw.rect(screen, color, [point[0]-4, point[1]+4, 8, 1], 1)
                elif label in ("7th", "8th", "9th"):  # Draw a circle
                    pygame.draw.circle(screen, color, point, 4,1)
             


        pygame.display.flip()
                
        if idx == 2:
            servo2.value = -0.2 # Values for Servo2, which gives the proper Tilt (Vertical Scan)
            sleep(0.25)
        elif idx == 5:
            servo2.value = -0.6
            sleep(0.25)  
        elif idx == 8:
            servo2.value = -0.2 
            sleep(0.25)
        elif idx == 11:
            servo2.value = 0.1 
            sleep(0.25)

    sleep(0.01)
