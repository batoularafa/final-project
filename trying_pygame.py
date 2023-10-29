import pygame
import serial
import time

#SerialObj = serial.Serial('COM4')

#SerialObj.baudrate = 9600  # set Baud rate to 9600
#SerialObj.bytesize = 8     # Number of data bits = 8
#SerialObj.parity   ='N'    # No parity
#SerialObj.stopbits = 1

#time.sleep(3)

pygame.joystick.init() #detect controller
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joysticks found.")
else:
     joysticks = [pygame.joystick.Joystick(x) for x in range(joystick_count)]
     print(joysticks)
     pygame.init()
     while True:
# xspeed = round(pygame.joystick.Joystick(0).get_axis(0))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break
            if event.type == pygame.JOYBUTTONDOWN:
                print(event)
            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 0:
                    if event.value > 0.5:
                        print("right")
                    elif event.value < -0.5:
                        print("left")
                elif event.axis == 1:
                    if event.value > 0.5:
                        print("back")
                    elif event.value < -0.5:
                        print("forward")
pygame.quit()


        #   print(event.axis)



        #Joy1Hat = pygame.joystick.Joystick(0).get_hat(0)
    # print(Joy1Hat)



        