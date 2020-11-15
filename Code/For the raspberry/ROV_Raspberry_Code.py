#Last Edited from Alex Ch on 29/Jan/2020
#PREPI NA EKTELESO THN ENTOLI TERMINAL $sudo pigpiod
#An thelo na elegxo an einai connected i camera ektelo sto terminal $vcgencmd get_camera
import pygame
try:
    import pygame.camera
except:
    print('The camera is not connected')
import RPi.GPIO as GPIO
import time
try:
    import pigpio
except:
    print("Run $sudo pigpiod, from the command window")
import sys
try:
    import ms5837
    sensor = ms5837.MS5837_30BA()# Default I2C bus is 1 (Raspberry Pi 3)
except:
    print('Try finding the ms5837.py file and place it in the same folder as this .py file')
    
#sensor = ms5837.MS5837_30BA()# Default I2C bus is 1 (Raspberry Pi 3)
# We must initialize the sensor before reading it
if not sensor.init():
        print("Sensor could not be initialized")
        exit()

sensor.setFluidDensity(ms5837.DENSITY_SALTWATER) #1029kg/m3
P_zero = 101325 #Pa
h_zero = 0; #m

pi = pigpio.pi()
#pi.set_mode(23, pigpio.OUTPUT) #set pin 23 as OUTPUT
#pi.set_mode(24, pigpio.OUTPUT)
pygame.init()
pygame.camera.init()

#Screen Options
font = pygame.font.Font('freesansbold.ttf', 20) #font file, font size
screen_size = [480,400]
screen = pygame.display.set_mode(screen_size,0)

#pygame.display.flip()
pygame.display.set_caption('Submarine Camera Preview 2020') #submarine Camera Preview 2020
BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)

pi = pigpio.pi()
GPIO.setmode(GPIO.BCM) #the GPIO numbers (not the pin number)
GPIO.setwarnings(False)

#Camera Stuff
cam_list = pygame.camera.list_cameras()
cam = pygame.camera.Camera(cam_list[0],screen_size)
cam.start()

#GPIO pins
motor1=23
motor2=24
motor3=25
motor4=8
led1=20
led2=16


# Stepper Motor for CAMERA, THIS WORKS PHOTO OF PINLAYOUT in DRIVE
A = 17
B = 18
C = 21
D = 22
GPIO.setup(A, GPIO.OUT)
GPIO.setup(B, GPIO.OUT)
GPIO.setup(C, GPIO.OUT)
GPIO.setup(D, GPIO.OUT)


def GPIO_SETUP(a,b,c,d):
    GPIO.output(A, a)
    GPIO.output(B, b)
    GPIO.output(C, c)
    GPIO.output(D, d)
    time.sleep(0.001)
    
# Moving the Camera right    
def RIGHT_TURN(degrees):
    full_circle_steps = 512.0
    steps = (full_circle_steps * degrees) / 360
    GPIO_SETUP(0,0,0,0)
    while steps > 0.0:
        GPIO_SETUP(1,0,0,0)
        GPIO_SETUP(1,1,0,0)
        GPIO_SETUP(0,1,0,0)
        GPIO_SETUP(0,1,1,0)
        GPIO_SETUP(0,0,1,0)
        GPIO_SETUP(0,0,1,1)
        GPIO_SETUP(0,0,0,1)
        GPIO_SETUP(1,0,0,1)      
        steps -= 1

# Moving the camera left
def LEFT_TURN(degrees):
    full_circle_steps = 512.0
    steps = (full_circle_steps * degrees) / 360
    GPIO_SETUP(0,0,0,0)
    while steps > 0.0:
        GPIO_SETUP(1,0,0,1)
        GPIO_SETUP(0,0,0,1)
        GPIO_SETUP(0,0,1,1)
        GPIO_SETUP(0,0,1,0)
        GPIO_SETUP(0,1,1,0)
        GPIO_SETUP(0,1,0,0)
        GPIO_SETUP(1,1,0,0)
        GPIO_SETUP(1,0,0,0)
        steps -= 1
        
# def move
def STOP_MOTOR(): # [Reverce=1100 - Stopped=1500 - Forword=1900]
    pi.set_servo_pulsewidth(motor1, 1500)
    pi.set_servo_pulsewidth(motor2, 1500)
    pi.set_servo_pulsewidth(motor3, 1500)
    pi.set_servo_pulsewidth(motor4, 1500)
    
def FRONT(a,b): # [Reverce=1100 - Stopped=1500 - Forword=1900]
    pi.set_servo_pulsewidth(motor1, a)
    pi.set_servo_pulsewidth(motor2, b)
    
def LEFT(a,b): # [Reverce=1100 - Stopped=1500 - Forword=1900]
    pi.set_servo_pulsewidth(motor1, a)
    pi.set_servo_pulsewidth(motor2, b)
    
def BACK(a,b): # [Reverce=1100 - Stopped=1500 - Forword=1900]
    pi.set_servo_pulsewidth(motor1, a)
    pi.set_servo_pulsewidth(motor2, b)
    
def RIGHT(a,b): # [Reverce=1100 - Stopped=1500 - Forword=1900]
    pi.set_servo_pulsewidth(motor1, a)
    pi.set_servo_pulsewidth(motor2, b)
    
    
# DOWN-UP-ROTATE SUBMARINE
def DOWN(a,b): # [Reverce=1100 - Stopped=1500 - Forword=1900]
    pi.set_servo_pulsewidth(motor3, a)
    pi.set_servo_pulsewidth(motor4, b)
    
def UP(a,b): # [Reverce=1100 - Stopped=1500 - Forword=1900]
    pi.set_servo_pulsewidth(motor3, a)
    pi.set_servo_pulsewidth(motor4, b)

def ROTATE_CLOCK(a,b):
    pi.set_servo_pulsewidth(motor3, a)
    pi.set_servo_pulsewidth(motor4, b)

def ROTATE_ANTI_CLOCK(a,b):
    pi.set_servo_pulsewidth(motor3, a)
    pi.set_servo_pulsewidth(motor4, b)
    
# LED ON
def LED_ON(): # [OFF=1100 - ON=1900]
    pi.set_servo_pulsewidth(led1, 1500)
    pi.set_servo_pulsewidth(led2, 1500)
    
# LED OFF
def LED_OFF(): # [OFF=1100 - ON=1900]
    pi.set_servo_pulsewidth(led1, 1100)
    pi.set_servo_pulsewidth(led2, 1100)


def draw_stuff(P_zero,h_zero):
    #Get The Temp and Display it
    tFile = open('/sys/class/thermal/thermal_zone0/temp')
    temp = float(tFile.read()) / 1000 #[Celsius]
    #Display On the screen what the camera sees
    image1 = cam.get_image()
    image1 = pygame.transform.scale(image1,screen_size)
    screen.blit(image1,(0,0))
    if temp < 62: color = BLUE
    elif temp >= 62: color = RED
    elif temp >= 75:
        color = RED
        warn_text = font.render("Too HOT ABORT Mission", True, BLACK, RED)
        warn_textRect = warn_text.get_rect()
        warn_textRect.center = (screen_size[0]/2,screen_size[1]/5) 
        screen.blit(warn_text, warn_textRect)
    if sensor.read():
        #Pressure print(str(sensor.pressure(ms5837.UNITS_kPa))+ " [mbar],  "+str(sensor.temperature())+"  [C]")
        sensor_text = font.render('P: '+str(round(sensor.pressure(ms5837.UNITS_kPa),2)) + '[kPa]', True, WHITE, BLACK)
        sensor_textRect = sensor_text.get_rect()
        sensor_textRect.center = (370,screen_size[1]-20) 
        screen.blit(sensor_text, sensor_textRect)
        #Temp OUT
        sensorT_text = font.render('Tout: '+str(round(sensor.temperature(),2))+'[C]', True, WHITE, BLACK)
        sensorT_textRect = sensorT_text.get_rect()
        sensorT_textRect.center = (70,screen_size[1]-20) 
        screen.blit(sensorT_text, sensorT_textRect)
        #Depth
        depth = sensor.pressure(ms5837.UNITS_Pa) / (1028 * 9.81)
        if sensor.pressure(ms5837.UNITS_Pa) <= P_zero:
            depth = abs(depth-h_zero)
        else:
            depth = -(depth-h_zero)
        depth_text = font.render('Depth: '+str(round(depth))+'[m]', True, WHITE, BLACK)
        depth_textRect = depth_text.get_rect()
        depth_textRect.center = (70,20) 
        screen.blit(depth_text, depth_textRect)        
    else:
        print("Pressure Sensor Failed")
    
    text = font.render('Tin: '+str(round(temp,2))+ '[C]', True, GREEN, color) #'CPU Temp: '+temp+' [C]'
    textRect = text.get_rect()
    textRect.center = (200,screen_size[1]-20) 
    screen.blit(text, textRect)
    #screen.blit(pygame.transform.rotate(screen, 180), (0, 0)) #flips the display with everything on it
    pygame.display.update()
 
    

# Loop
print ("Starting loop")
running = True
led = 1
try:
    while running:
        draw_stuff(P_zero, h_zero) #updates the screen object with everything displayed on it
        STOP_MOTOR()
           
        for e in pygame.event.get():
            if e.type == pygame.KEYDOWN:
                if (e.key == pygame.K_o): #Calibrate the depth
                    P_zero = sensor.pressure(ms5837.UNITS_Pa) #Pa, Pressure at current sea level
                    h_zero = sensor.pressure(ms5837.UNITS_Pa) / (1028 * 9.81) #m, wrong depth at curernt sea level 
                if (e.key == pygame.K_UP): #Left JoyStick Y axis forward
                    print ("Left JoyStick Y axis(going forward): ", e.key)
                    a=1600
                    b=1600
                    FRONT(a,b)
                if (e.key == pygame.K_DOWN): #Left JoyStick Y axis backward
                    print ("Left JoyStick Y axis(going backward): ", e.key)
                    a=1400
                    b=1400
                    BACK(a,b)
                if (e.key == pygame.K_RIGHT): #Left JoyStick X axis right
                    print ("Left JoyStick X axis(going right): ", e.key)
                    a=1500
                    b=1600
                    RIGHT(a,b)
                if (e.key == pygame.K_LEFT): #Left JoyStick X axis left
                    print ("Left JoyStick X axis(going left): ", e.key)
                    a=1600
                    b=1500
                    LEFT(a,b)
                    # plunge down rov
                if (e.key == pygame.K_s): #Right JoyStick Y axis backward(going down)
                    print ("Right JoyStick Y axis(backward)(going down): ", e.key)
                    a=1600
                    b=1600
                    DOWN(a,b)
                    # plunge up rov
                if (e.key == pygame.K_w): #Right JoyStick Y axis forward(going up)
                    print ("Right JoyStick Y axis(forward)(going up): ", e.key)
                    a=1400
                    b=1400
                    UP(a,b)
                if (e.key == pygame.K_d): #Right JoyStick X axis Right(rotating clockwise)
                    print ("Right JoyStick Y axis(Right)(rotating clockwise): ", e.key)
                    a=1600
                    b=1400
                    ROTATE_CLOCK(a,b)
                if (e.key == pygame.K_a): #Right JoyStick X axis Left(rotating anticlockwise)
                    print ("Right JoyStick Y axis(Left)(rotating anticlockwise): ", e.key)
                    a=1400
                    b=1600
                    ROTATE_ANTI_CLOCK(a,b)
                # led
                if (e.key == pygame.K_l): #button Delta
                    if led > 0:
                        print("LED ON, Delta Button value: ", e.key)
                        led = led * -1
                        LED_ON()
                    else:
                        print("LED OFF, Delta Button value: ", e.key)
                        led = led * -1
                        LED_OFF()
                # Camera Movement
                if (e.key == pygame.K_r):
                    print("Camera Tilt Up", e.key)
                    u = 5
                    LEFT_TURN(u)
                    GPIO_SETUP(0,0,0,0)
                if (e.key == pygame.K_f):
                    print("Camera Tilt Down",e.key)
                    y = 5
                    RIGHT_TURN(y)
                    GPIO_SETUP(0,0,0,0)
                # Exit Program
                if (e.key == pygame.K_ESCAPE): #button PS
                    print("Stop and Close, PS Button value: ", e.key)
                    STOP_MOTOR()
                    LED_OFF()
                    running = False
                    pygame.quit()
                    exit()
            elif e.type == pygame.QUIT:
                STOP_MOTOR()
                LED_OFF()
                running = False
                pygame.quit()
                exit()

except KeyboardInterrupt: #Press ctrl+c to exit
    pygame.quit()
    GPIO.output(STANDBY, False)
    exit()




