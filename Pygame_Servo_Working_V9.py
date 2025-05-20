from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
from smbus2 import SMBus, i2c_msg
import pygame
import numpy as np

address = 0x10 # Servo setup
factory = PiGPIOFactory()
servo1 = Servo(17, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)
servo2 = Servo(18, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

pygame.init() # Pygame initial setup
WIDTH, HEIGHT = 1530, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("HaptiVision Point Cloud + Navigation Zones V.1.0")

BLACK = (0, 0, 0) # Set different colors to be use
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GREY = (50, 50, 50)
GREY2 = (128, 128, 128)
WHITE = (255, 255, 255)
MAGENTA = (255, 0, 255)
YELLOW = (255, 255, 102) 
RED = (255, 102, 102) 
CENTER = (400, HEIGHT - 50)

zone_1, zone_2, zone_3 = [], [], [] # Distance storage in 12 different arrays
zone_4, zone_5, zone_6 = [], [], []
zone_7, zone_8, zone_9 = [], [], []
zone_10, zone_11, zone_12 = [], [], []

zone_angles = {        # Angle ranges for each zone (degrees) 12 in total ( 4 different passes)
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
    angle_rad = np.radians(angle_deg)
    x = center[0] + scale * distance * np.cos(angle_rad)
    y = center[1] - scale * distance * np.sin(angle_rad)
    return int(x), int(y)


def read_lidar_points(write, read, count=15, max_retries=3): # Function to continuosly read the i2c buffer
    distance_values = []
    attempt = 0

    while attempt < max_retries:
        try:
            with SMBus(1) as bus:
                for i in range(count):
                    bus.i2c_rdwr(write, read)
                    data = list(read)
                    TrigFlag = data[0]
                    Dist = ((data[3] << 8) | data[2])
                    distance_values.append(Dist)
                    #sleep(0.01)  # Give sensor a small break
                    
            return distance_values  # Success, return results

        except OSError as e:
            print(f"[Retry {attempt+1}/{max_retries}] I2C Error: {e}")
            attempt += 1
            sleep(0.01)  # Wait before retrying
    
    print("LiDAR read failed after retries. Returning fallback values.") # If all attempts fail
    return [0] * count  

servo1.value = 0.5 # Initial servo positions (Home)
servo2.value = 0.1
sleep(0.5)

zone_1 = np.array([[1000, 120], [1100, 120], [1100, 220], [1000, 220]]) # Square 1 vertices (top-left, top-right, bottom-right, bottom-left) for visualization
zone_2 = np.array([[1110, 120], [1210, 120], [1210, 220], [1110, 220]])
zone_3 = np.array([[1220, 120], [1320, 120], [1320, 220], [1220, 220]])
zone_4 = np.array([[1000, 230], [1100, 230], [1100, 330], [1000, 330]])
zone_5 = np.array([[1110, 230], [1210, 230], [1210, 330], [1110, 330]])
zone_6 = np.array([[1220, 230], [1320, 230], [1320, 330], [1220, 330]])

zone_7 = np.array([[995, 360], [1096, 360], [1026, 430], [925, 430]]) # Rhomboid 1 vertices (top-left, top-right, bottom-right, bottom-left) for visualization
zone_8 = np.array([[1110, 360], [1207, 360], [1137, 430], [1040, 430]])
zone_9 = np.array([[1221, 360], [1320, 360], [1250, 430], [1151, 430]])

zone_labels = ["1st", "2nd", "3rd", "4th", "5th", "6th",
               "7th", "8th", "9th", "10th", "11th", "12th"]

zone_to_points = {label: [] for label in zone_labels}

# Main loop
while True:
    positions = [(0.16, 1, "1st"), (-0.16, 1, "2nd"), (-0.5, 1, "3rd"),
                 (-0.16, 1, "4th"), (0.16, 1, "5th"), (0.5, 1, "6th"),
                 (0.16, 1, "7th"), (-0.16, 1, "8th"), (-0.5, 1, "9th"),
                 (-0.16, 1, "4th"), (0.16, 1, "5th"), (0.5, 1, "6th")]

    for idx, (pos, delay, label) in enumerate(positions):
        print(f"\nLiDAR Zone: {label}")
        servo1.value = pos
        
        write = i2c_msg.write(address, [1, 2, 7]) # Prepare I2C comms
        read = i2c_msg.read(address, 7)
        dist = read_lidar_points(write, read)
        sleep(delay)
        print(f"Zone {label} distances:", dist)

        zone_to_points[label].clear() # Clear the previous points of that zone    
        if label in ["1st", "2nd", "3rd"]: # Determine the color of the points in the points cloud
            color = BLUE
        elif label in ["4th", "5th", "6th"]:
            color = GREEN
        elif label in ["7th", "8th", "9th"]:
            color = MAGENTA

        start_angle, end_angle = zone_angles[label] # Calculate new points for this zone
        angle_step = (end_angle - start_angle) / len(dist)

        for i, d in enumerate(dist):
            angle = start_angle + i * angle_step
            scaled_d = min(d, 350)
            point = polar_to_screen(CENTER, angle, scaled_d, scale=1)
            zone_to_points[label].append((point, color))

        # Draw Points Clous + Navigation Zones Visualizations
        screen.fill(BLACK)
        pygame.draw.circle(screen, GREY, CENTER, 300, 1)
        pygame.draw.circle(screen, GREY, CENTER, 200, 1)
        pygame.draw.circle(screen, GREY, CENTER, 100, 1)
        pygame.draw.line(screen, GREY, CENTER, (650,300))
        pygame.draw.line(screen, GREY, CENTER, (145,300))
        pygame.draw.line(screen, GREY, CENTER, (305,200))
        pygame.draw.line(screen, GREY, CENTER, (490,200))
        pygame.draw.line(screen, GREY, (100,550), (700,550))
        
        font1 = pygame.font.SysFont(None, 45)
        font2 = pygame.font.SysFont(None, 25)
        font3 = pygame.font.SysFont(None, 22)
        
        screen.blit(font1.render("POINT CLOUD V.1.0", True, WHITE), (250,15))
        screen.blit(font2.render("3m", True, WHITE), (670,555))
        screen.blit(font2.render("2m", True, WHITE), (570,555))
        screen.blit(font2.render("1m", True, WHITE), (470,555))
        screen.blit(font2.render("0m", True, WHITE), (370,555))
        screen.blit(font3.render("-45°", True, GREY2), (115,300))
        screen.blit(font3.render("-15°", True, GREY2), (270,200))
        screen.blit(font3.render("15°", True, GREY2), (495,200))
        screen.blit(font3.render("45°", True, GREY2), (655,300))
        screen.blit(font2.render("TILT LEVELS", True, GREY2), (75,80))
        screen.blit(font3.render("BLUE         = Top Tier", True, BLUE), (75,100))
        screen.blit(font3.render("GREEN      = Middle Tier", True, GREEN), (75,120))
        screen.blit(font3.render("MAGENTA = Bottom Tier", True, MAGENTA), (75,140))
        
        screen.blit(font1.render("NAVIGATION ZONES V.1.0", True, WHITE), (960, 15))
        screen.blit(font3.render("z", True, GREY2), (979, 85))
        screen.blit(font3.render("x", True, GREY2), (1348, 333))
        screen.blit(font3.render("y", True, GREY2), (875, 440))
        screen.blit(font3.render("GREEN =     No Obstacles Detected, Distance > 2 Meters", True, GREEN), (930, 480))
        screen.blit(font3.render("YELLOW =  Obstacle Detected, Distance <= 2 Meters", True, YELLOW), (930, 500))
        screen.blit(font3.render("RED =          Obstacle Detected, Distance <= 1 Meter", True, RED), (930, 520))
        screen.blit(font3.render("GREY =       GAP Detected, Floor Level", True, GREY2),(930, 540)) 
    
        pygame.draw.line(screen, GREY2,(985, 105), (985, 345)) # Draw Lines for the 3D Plot
        pygame.draw.line(screen, GREY2,(985, 345), (1340, 345)) 
        pygame.draw.aaline(screen, GREY2,(985, 345), (888, 440)) 

        pygame.draw.polygon(screen, GREEN, zone_1) # Draw the squares using the vertices
        pygame.draw.polygon(screen, GREEN, zone_2)
        pygame.draw.polygon(screen, GREEN, zone_3)
        pygame.draw.polygon(screen, GREEN, zone_4)
        pygame.draw.polygon(screen, GREEN, zone_5)
        pygame.draw.polygon(screen, GREEN, zone_6)
    
        pygame.draw.polygon(screen, GREEN, zone_7) # Draw the Rhomboids using the vertices
        pygame.draw.polygon(screen, GREEN, zone_8)
        pygame.draw.polygon(screen, GREEN, zone_9)
        
        pygame.draw.rect(screen, GREY2, (60, 70, 210, 95), width=2) # Draw Grey Frames
        pygame.draw.rect(screen, GREY2, (40, 50, 710, 530), width=2)
        pygame.draw.rect(screen, GREY2, (910, 465, 440, 95), width=2)
        pygame.draw.rect(screen, GREY2, (770, 50, 720, 530), width=2)

        # Draw all points from all zones
        for zone_points in zone_to_points.values():
            for point, color in zone_points:
                pygame.draw.circle(screen, color, point, 2)
                
        pygame.display.flip()
                
        if idx == 2:
            servo2.value = -0.25 # Values for Servo2, which gives the proper Tilt (Vertical Scan)
            sleep(1)
        elif idx == 5:
            servo2.value = -0.65 ###### proper value 0.1 change only for testing
            sleep(1)  
        elif idx == 8:
            servo2.value = -0.25 
            sleep(1)
        elif idx == 11:
            servo2.value = 0.1 
            sleep(1)

    sleep(0.01)
