from websocket import create_connection
import pygame

esp32_ip = "192.168.1.4"
websocket_url = f"ws://{esp32_ip}/ws"

ws = create_connection(websocket_url)

pygame.joystick.init() #detect controller
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joysticks found.")
else:
     joysticks = [pygame.joystick.Joystick(x) for x in range(joystick_count)]
     print(joysticks)
     pygame.init()
     while True:
          for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break
            if event.type == pygame.JOYBUTTONDOWN:
                print(event)
            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 0:
                    if event.value > 0.5:
                        #print("right")
                        message = "right"
                        ws.send(message)
                        response = ws.recv()
                        print("Received:", response)
                    elif event.value < -0.5:
                        #print("left")
                        message = "left"
                        ws.send(message)
                        response = ws.recv()
                        print("Received:", response)
                elif event.axis == 1:
                    if event.value > 0.5:
                        #print("back")
                        message = "back"
                        ws.send(message)
                        response = ws.recv()
                        print("Received:", response)
                    elif event.value < -0.5:
                        #print("forward")
                        message = "forward"
                        ws.send(message)
                        response = ws.recv()
                        print("Received:", response)
            
     '''   message = input("Enter a message to send: ")
        ws.send(message)
        response = ws.recv()
        print("Received:", response)'''
        
pygame.quit()
ws.close()
