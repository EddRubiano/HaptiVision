from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
from smbus2 import SMBus, i2c_msg
import pygame
import numpy as np

# Servo + I2C setup
address = 0x10
factory = PiGPIOFactory()
servo1 = Servo(17, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)
servo2 = Servo(18, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

# Pygame setup
pygame.init()
WIDTH, HEIGHT = 750, 500
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("HaptiVision Radar Sweep V.1.0")
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
GREY = (50, 50, 50)
WHITE = (255, 255, 255)
CENTER = (WIDTH // 2, HEIGHT - 50)
#RADIUS = 350


# Distance storage
zone_1, zone_2, zone_3 = [], [], []
zone_4, zone_5, zone_6 = [], [], []

# Angle ranges for each zone (degrees)
zone_angles = {
    "1st": (135, 105),
    "2nd": (105, 75),
    "3rd": (75, 45),
    "4th": (45, 75),
    "5th": (75, 105),
    "6th": (105, 135)
}

# Converts polar to screen x, y
def polar_to_screen(center, angle_deg, distance, scale=1):
    angle_rad = np.radians(angle_deg)
    x = center[0] + scale * distance * np.cos(angle_rad)
    y = center[1] - scale * distance * np.sin(angle_rad)
    return int(x), int(y)


def read_lidar_points(write, read, count=20, max_retries=3):
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
                    #print("   LiDAR cms: ", Dist)
                    #sleep(0.01)  # Give sensor a small break
            return distance_values  # Success! return results

        except OSError as e:
            print(f"[Retry {attempt+1}/{max_retries}] I2C Error: {e}")
            attempt += 1
            sleep(0.01)  # Wait before retrying

    # If all attempts fail
    print("LiDAR read failed after retries. Returning fallback values.")
    return [0] * count  # You can also use None, or -1s if preferred


# Initial servo positions
servo1.value = 0.5
servo2.value = 0.15
sleep(1)

# Main loop
while True:
    positions = [(0.16, 0.1, "1st"), (-0.16, 0.1, "2nd"), (-0.5, 0.1, "3rd"),
                 (-0.16, 0.1, "4th"), (0.16, 0.1, "5th"), (0.5, 0.1, "6th")]
    #screen.fill(BLACK)
    #pygame.draw.circle(screen, GRAY, CENTER, RADIUS, 1)

    for idx, (pos, delay, label) in enumerate(positions):
        print(f"\nLiDAR Zone: {label}")
        servo1.value = pos
        sleep(delay)

        if idx == 2:
            servo2.value = -0.25 ########### for reading the floor it will be -0.65
            sleep(0.2)
        elif idx == 5:
            servo2.value = 0.1
            sleep(0.2)

        # Prepare I2C transaction
        write = i2c_msg.write(address, [1, 2, 7])
        read = i2c_msg.read(address, 7)

        dist = read_lidar_points(write, read)

        # Store zone data and print
        if label == "1st":
            zone_1 = dist
            print("Zone 1 distances:", zone_1)
        elif label == "2nd":
            zone_2 = dist
            print("Zone 2 distances:", zone_2)
        elif label == "3rd":
            zone_3 = dist
            print("Zone 3 distances:", zone_3)
        elif label == "4th":
            zone_4 = dist
            print("Zone 4 distances:", zone_4)
        elif label == "5th":
            zone_5 = dist
            print("Zone 5 distances:", zone_5)
        elif label == "6th":
            zone_6 = dist
            print("Zone 6 distances:", zone_6)

        # Plot distances in pygame
        start_angle, end_angle = zone_angles[label]
        angle_step = (end_angle - start_angle) / len(dist)

        # Clear screen and draw radar circle
        screen.fill(BLACK)
        pygame.draw.circle(screen, GREY, CENTER, 300, 1)
        pygame.draw.circle(screen, GREY, CENTER, 200, 1)
        pygame.draw.circle(screen, GREY, CENTER, 100, 1)
        pygame.draw.line(screen, GREY, CENTER, (625,200))
        pygame.draw.line(screen, GREY, CENTER, (125,200))
        pygame.draw.line(screen, GREY, (50,450), (700,450))
        font1= pygame.font.SysFont(None, 55)
        text1 = font1.render("RADAR SWEEP",True, WHITE)
        screen.blit(text1, (230,30))
        font2= pygame.font.SysFont(None, 25)
        text2 = font2.render("3m",True, WHITE)
        screen.blit(text2, (650,455))
        font3= pygame.font.SysFont(None, 25)
        text3 = font3.render("2m",True, WHITE)
        screen.blit(text3, (550,455))
        font4= pygame.font.SysFont(None, 25)
        text4 = font4.render("1m",True, WHITE)
        screen.blit(text4, (450,455))
        font5= pygame.font.SysFont(None, 25)
        text5 = font5.render("0m",True, WHITE)
        screen.blit(text5, (350,455))

        # Draw points
        for i, d in enumerate(dist):
            angle = start_angle + i * angle_step
            scaled_d = min(d, 350)
            point = polar_to_screen(CENTER, angle, scaled_d, scale=1)
            pygame.draw.circle(screen, GREEN, point, 2)

        pygame.display.flip()

    sleep(0.05)
