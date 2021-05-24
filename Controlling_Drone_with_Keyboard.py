import pygame
from dronekit import connect,LocationGlobalRelative,VehicleMode,LocationGlobal,Command
import  time 
from pymavlink import mavutil
pygame.init()
win=pygame.display.set_mode((500,500))
pygame.display.set_caption("first game")


print('Connecting to vehicle on: "udp:127.0.0.1:14550" ')
vehicle = connect("udp:127.0.0.1:14550", wait_ready=True)
print("connected successfully")
gnd_speed=40


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # WAIT UNTIL THE AUTOPILOT IS READY
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

 
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       
        0, 0,    
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000111111000111, 
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 
        0, 0)    

    # send command to vehicle 
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 
    else:
        is_relative=0 
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,
        heading,    
        0,         
        1,         
        is_relative, 
        0, 0, 0)   
    # send command to vehicle
    vehicle.send_mavlink(msg)

    
print("control the vehicle with keyboard keys")
## keyboard inputs
def keyboard():
    while True:  
        event = pygame.event.wait()  
        if event.type == pygame.QUIT:  
            break  
   #PRESS THE KEYS TO MOVE THE DRONE
        if  event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:  #UP ARROW=FORWARD
                    print("moving forward") 
                    send_ned_velocity(gnd_speed,0,0,4)  
                elif event.key == pygame.K_DOWN:         #DOWN ARROW=BACKWARD
                    print("moving backward")
                    send_ned_velocity(-gnd_speed,0,0,4)  
                elif event.key == pygame.K_LEFT:          #LEFT ARROW=LEFT
                    print("moving left")
                    send_ned_velocity(0,-gnd_speed,0,4)  #RIGHT ARROW=RIGHTR
                elif event.key == pygame.K_RIGHT:
                    print("moving right ")
                    send_ned_velocity(0,gnd_speed,0,4)  #
                elif event.key == pygame.K_SPACE:             #W BAR= UP
                    print("moving up ")
                    send_ned_velocity(0,0,-gnd_speed,4)
                elif event.key == pygame.K_s:                  #S BUTTON= DOWN
                    print("moving dowm")
                    send_ned_velocity(0,0,gnd_speed,4)
                elif event.key == pygame.K_a:               #A button=yaw left
                    print(" yaw left")
                    condition_yaw(300,1)
                elif event.key == pygame.K_d:              #D button=yaw right
                    print("start yaw right")
                    condition_yaw(60,1)
            

    ###WHEN THE KEY IS RELEASED
        if  event.type == pygame.KEYUP:
                if event.key == pygame.K_UP:
                    print("STOP moving forward") 
                    send_ned_velocity(0,0,0,1)
                elif event.key == pygame.K_DOWN:
                    print(" STOP moving backward")
                    send_ned_velocity(0,0,0,1)
                elif event.key == pygame.K_LEFT:
                    print(" STOP moving left")
                    send_ned_velocity(0,0,0,1)
                elif event.key == pygame.K_RIGHT:
                    print(" STOP moving right ")
                    send_ned_velocity(0,0,0,1)
                elif event.key == pygame.K_w:
                    print(" STOP moving up ")
                    send_ned_velocity(0,0,0,1)
                elif event.key == pygame.K_s:
                    print(" STOP moving dowm")
                    send_ned_velocity(0,0,0,1)
                elif event.key == pygame.K_a:               #A button=yaw left
                    print("STOP yaw left")
                    condition_yaw(0,1)
                elif event.key == pygame.K_d:              #D button=yaw right
                    print("STOP yaw right")
                    condition_yaw(0,1)
arm_and_takeoff(50) 
keyboard()
