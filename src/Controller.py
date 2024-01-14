'''Module to handle a controller. Could be extended to other controllers with other layouts, currently only a PS4 controller is implemented.'''

import os
import pygame
from threading import Thread
import time



os.environ["SDL_VIDEODRIVER"] = "dummy"

class PS4Controller():
    '''Class to handle a PS4 controller. Upon initialization starts a new thread that asynchronously keeps input values up to date.
    
    Input values are accessible through a variety of fields named after their corresponding Input'''
    
    run=True
    def init(self):
        '''Initializes all dependencies and starts the thread'''
        pygame.init()
        screen = pygame.display.set_mode((1,1))
        pygame.joystick.init()
        
        print("Waiting for controller")
        while True:
            pygame.joystick.init()
            if pygame.joystick.get_count() != 0:
                break
            
            print(".")
            time.sleep(5)
        print("Connected!")
    
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        
        self.RX_Axis = 0.0
        self.RY_Axis = 0.0
        self.LX_Axis = 0.0
        self.LY_Axis = 0.0
        self.R_trigger = -1.0
        self.L_trigger = -1.0
        self.X_button = False
        self.Opt_button=False
        
        self.L1_Button=False
        self.R1_Button=False
        
        
        self.Up_button=False
        self.Down_button=False
        self.Right_button=False
        self.Left_button=False
        
        self.L3_Button=False
            
        t = Thread(target=self.__listen)
        t.start()  
        t.join


    def __listen(self):
        '''Private function that handels the input events in a thread'''
        while self.run:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION: 
                    if event.axis == 3:
                        self.RX_Axis = float(round(event.value,2))
                    elif event.axis == 4:
                        self.RY_Axis = float(round(event.value,2))
                    elif event.axis == 0:
                        self.LX_Axis = float(round(event.value,2))
                    elif event.axis == 1:
                        self.LY_axis = float(round(event.value,2))
                    elif event.axis ==5:
                        self.R_trigger= float(round(event.value,2))
                    elif event.axis ==2:
                        self.L_trigger=float(round(event.value,2))
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button==0:
                        self.X_button=True
                    if event.button==9:
                        self.Opt_button=True
                    if event.button==4:
                        self.L1_Button=True
                    if event.button==5:
                        self.R1_Button=True
                    if event.button==11:
                        self.L3_Button=True
                elif event.type == pygame.JOYBUTTONUP:
                    if event.button==0:
                        self.X_button=False
                    if event.button==9:
                        self.Opt_button=False
                    if event.button==4:
                        self.L1_Button=False
                    if event.button==5:
                        self.R1_Button=False
                    if event.button==11:
                        self.L3_Button=False
                elif event.type == pygame.JOYHATMOTION:
                    if event.hat==0:
                        x,y=event.value
                        if y==0:
                            self.Up_button=False
                            self.Down_button=False
                        if x==0:
                            self.Right_button=False
                            self.Left_button=False
                        if y==1:
                            self.Up_button=True
                        if y==-1:
                            self.Down_button=True
                        if x==-1:
                            self.Left_button=True
                        if x==1:
                            self.Right_button=True
                    
    
    def quit(self):
        '''Deallocates everything. Required!'''
        self.run=False
        pygame.quit()


if __name__ == "__main__":
    ps4 = PS4Controller()
    ps4.init()
    while True:
        print()
        print(ps4.R1_Button)
        print(ps4.L1_Button)
        print(ps4.Up_button)
        print(ps4.Down_button)
        print(ps4.Left_button)
        print(ps4.Right_button)
        time.sleep(1)