'''Module designed to handle the movement capabilities of the HTL Braunau HexaBot.'''

from __future__ import annotations
import time
import math
from adafruit_motor import servo
from BetterServoKit import ServoKit
from Controller import PS4Controller
from threading import Thread
from abc import ABC, abstractmethod

J2L = 76 #length between shoulderjoint and middlejoint
J3L = 112 #length between middle joint and tip
J3OffsetAngle=-5 #the offset angle between where the tip actually is and where it's supposed to be

Y_rest = 70 #resting Y position
Z_rest = -110 #resting X position


class HBUserIO(ABC):
    '''Abstract base class for IO. Use as an interface to control the HexaBot's movement. 
    
    Functions ending with "con" are for the FSM. They get called to check wether the HexaBot should switch into another state. They also define behaivoral values like movement speed.'''
    
    @abstractmethod
    def init(self):
        '''Initialize your required fields if needed here. Gets automatically called by the HexaBot on start.'''
        return None
    @abstractmethod
    def quit(self):
        '''Cleanup function if your chosen interface needs it. Gets automatically called by the HexaBot on exit.'''
        return None
    @abstractmethod
    def jumpCon(self):
        '''Returns a boolean. If True, the HexaBot will jump. '''
        return False
    @abstractmethod
    def decreaseSpeedCon(self):
        '''Returns a boolean. If True, the HexaBot's global speed will decrease.'''
        return False
    @abstractmethod
    def increaseSpeedCon(self):
        '''Returns a boolean. If True, the HexaBot's global speed will increase.'''
        return False
    @abstractmethod
    def increaseHeightCon(self):
        '''Returns a boolean. If True, the Hexabot's height will increase.'''
        return False
    @abstractmethod
    def decreaseHeightCon(self):
        '''Returns a boolean. If True, the HexaBot's height will decrease.'''
        return False
    @abstractmethod
    def resetCon(self):
        '''Returns a boolean. If True, global speed and height will reset to the default value.'''
        return False
    @abstractmethod
    def forwardSpeedCon(self):
        '''Returns a float between 0-1. If not 0, the HexaBot will move forward. The value itself is used as the forwards speed.'''
        return 0.0
    @abstractmethod
    def backwardsSpeedCon(self):
        '''Returns a float between 0-1. If not 0, the HexaBot will move backwards. The value itself is used as the backwards speed.'''
        return 0.0
    @abstractmethod
    def exitCon(self):
        '''Returns a boolean. If True, the HexaBot will deallocate everything and stop all functionality.'''
        return False
    @abstractmethod
    def turnCon(self):
        '''Returns a float between 0-1. If not 0, the HexaBot will start turning. The value itself is used as the turning speed.'''
        return 0.0

class HBPS4IO(HBUserIO):
    '''Specific implementation of the HBUserIO class using a PS4 controller.'''
    def init(self):
        self.__ps4 = PS4Controller()
        self.__ps4.init()
    def quit(self):
        self.__ps4.quit()
    def jumpCon(self):
        return self.__ps4.X_button
    def decreaseSpeedCon(self):
        return self.__ps4.Down_button
    def increaseSpeedCon(self):
        return self.__ps4.Up_button
    def increaseHeightCon(self):
        return self.__ps4.L1_Button
    def decreaseHeightCon(self):
        return self.__ps4.R1_Button
    def resetCon(self):
        return self.__ps4.L3_Button
    def forwardSpeedCon(self):
        return self.__ps4.R_trigger
    def backwardsSpeedCon(self):
        return self.__ps4.L_trigger
    def exitCon(self):
        return self.__ps4.Opt_button
    def turnCon(self):
        return self.__ps4.LX_Axis
        
class Joint():
    '''A class representing a single rotational joint using a servo motor.'''
    def __init__(self, servo : servo.Servo, inverted : bool = False, offset : int = 0):
        '''A class representing a single rotational joint using a servo motor.
        
        servo: The servo object used in the Joint.
        
        inverted: Inverts the angle on setting if set to True.
        
        offset: Software fix for cheap servo motors that aren't accurate.'''
        self.servo = servo
        self.inv = inverted
        self.offs=offset
        
    def angle(self, angle: int):
        '''Sets the angle of the joint.'''
        if(self.inv):
            self.servo.angle=(180-angle+self.offs)
        else:
            self.servo.angle=(angle+self.offs)
        
class Leg():
    '''A class representing a single leg. Contains 3 joints. The hip joint, the shoulder joint and the middle joint.'''
    def __init__(self, j1: Joint, j2: Joint, j3:Joint, angle:int):
        '''A class representing a single leg. Contains 3 joints. The hip joint, the shoulder joint and the middle joint.
        
        j1: The hip joint.
        
        j2: The shoulder joint.
        
        j3: The middle joint.
        
        angle: Rotational offset the entire leg from the body.'''
        self.j1=j1
        self.j2=j2
        self.j3=j3
        self.angle=angle
        
        
    def moveLeg(self, relX:float, relY:float, relZ:float):
        '''Calculates the angles required to move a leg to a certain coordinate position and moves the joints accordingly.
        
        relX: The relative target X position to the leg.
        
        relY: The relative target Y position to the leg.
        
        relZ: The relative target Z position to the leg.'''
        relY+= Y_rest
        relZ+= Z_rest
        if(self.angle!=0):
            angRad = math.radians(self.angle) 
            tempX = ((relX*math.cos(angRad))-(relY*math.sin(angRad)))
            tempY = ((relY*math.cos(angRad))+(relX*math.sin(angRad)))
            relX=tempX
            relY=tempY
            
        J1= math.degrees(math.atan(relX/relY))
        H= math.sqrt((relY**2)+(relX**2))
        L= math.sqrt((H**2)+(relZ**2))
        J3= math.degrees(math.acos(((J2L * J2L) + (J3L * J3L) - (L * L))   /   (2 * J2L * J3L)))
        B= math.degrees(math.acos(((L * L) + (J2L * J2L) - (J3L * J3L))   /   (2 * L * J2L)))
        A= math.degrees(math.atan(relZ/H))
        J2= (B+A)
        
        
        self.j1.angle(90-int(J1))
        self.j2.angle(90-int(J2))
        if self.j3.inv:
            self.j3.angle(int(J3-J3OffsetAngle))
        else:
            self.j3.angle(int(J3+J3OffsetAngle))
            
class HBMovementState():
    '''The FSM states.'''
    INIT = 1                # unused
    IDLE = 2
    EXIT = 3
    
    STANDINGJUMPSTART=4
    STANDINGJUMP=5
    
    STANDINGTURN=6
    STANDINGTURNSTOP=7
    
    RUNNING=8
    RUNNINGSTOP=9
    
    RUNNINGTURN=10
    RUNNINGTURNSTOP=11
    
    RUNNINGJUMPSTART=12     #
    RUNNINGJUMP=13          # unimplemented: implement if normal jump works
    RUNNINGJUMPLAND=14      #
            
            
class HBMovement:
    '''The class that handles all movement of the HexaBot with a FSM in a seperate thread. You only have to call the start() function.'''
    def __init__(self, userIO:HBUserIO=HBPS4IO()):
        '''The class that handles all movement of the HexaBot with a FSM. You only have to call the start() function.
        
        userIO: You can pass your own userdefined IO interface. Extend the HBUserIO class from this module. The default interface uses a PS4 controller. '''
        sk = ServoKit()
        
        self.__height=0
        self.__globalspeed=1
        
        self.__leg1 = Leg(Joint(sk.servo[4]),Joint(sk.servo[5], True),Joint(sk.servo[6]),5)
        self.__leg2 = Leg(Joint(sk.servo[8]),Joint(sk.servo[9], True),Joint(sk.servo[10]),5)
        self.__leg3 = Leg(Joint(sk.servo[12]),Joint(sk.servo[13], True),Joint(sk.servo[14]),5)
        self.__leg4 = Leg(Joint(sk.servo[16], True),Joint(sk.servo[17]),Joint(sk.servo[18], True),5)
        self.__leg5 = Leg(Joint(sk.servo[20], True),Joint(sk.servo[21]),Joint(sk.servo[22], True),5)
        self.__leg6 = Leg(Joint(sk.servo[24], True),Joint(sk.servo[25]),Joint(sk.servo[31], True),5)
        self.__idle()
        self.__jumpdelay=0
        self.__runCycle=0
        self.__turnCylce=0
        
        self.__state=HBMovementState.IDLE
        
        self.__userIO=userIO
        
        
           
    def start(self):
        '''Starts everything needed for the HexaBot's movement in a new thread and joins it to the calling thread.'''
        self.__userIO.init()
        p = Thread(target=self.__loop)
        p.start()
        p.join
        
    def quit(self):
        '''Stops the HexaBot and closes the thread.'''
        self.__state=HBMovementState.EXIT
            
    def __loop(self):
        '''The main loop responsible for the FSM.'''
        #print("Started main loop!")
        while True:
            if self.__state == HBMovementState.IDLE:
                self.__idle()
                if self.__userIO.increaseHeightCon():
                    self.__height+=1
                elif self.__userIO.decreaseHeightCon():
                    self.__height-=1
                    
                if self.__userIO.increaseSpeedCon():
                    self.__globalspeed+=0.3
                elif self.__userIO.decreaseSpeedCon():
                    self.__globalspeed-=0.3
                    
                if self.__userIO.resetCon():
                    self.__height=0
                    self.__globalspeed=1
                
                if self.__userIO.jumpCon():
                    self.__state=HBMovementState.STANDINGJUMPSTART
                elif self.__userIO.forwardSpeedCon() != -1 or self.__userIO.backwardsSpeedCon() !=-1:
                    self.__state=HBMovementState.RUNNING
                elif self.__userIO.turnCon() != 0:
                    self.__state=HBMovementState.STANDINGTURN
                elif self.__userIO.exitCon():
                    self.__state=HBMovementState.EXIT
            
            elif self.__state==HBMovementState.STANDINGTURN:
                if self.__userIO.turnCon() == 0:
                    self.__state=HBMovementState.STANDINGTURNSTOP
                self.__turn(self.__userIO.turnCon()) 
            
            elif self.__state==HBMovementState.STANDINGTURNSTOP:
                self.__idle()
                self.__turnCylce=0
                self.__state=HBMovementState.IDLE
                    
            elif self.__state ==HBMovementState.STANDINGJUMPSTART:
                self.__jump()
                self.__state = HBMovementState.STANDINGJUMP
                self.__jumpdelay=time.time()
                
            elif self.__state==HBMovementState.STANDINGJUMP:
                self.__idle()
                if time.time()-self.__jumpdelay>1:
                    self.__state=HBMovementState.IDLE
                
            elif self.__state == HBMovementState.RUNNING:
                R = self.__userIO.forwardSpeedCon()
                L = self.__userIO.backwardsSpeedCon()
                if R==-1 and L==-1 or R==1 and L==1:
                    self.__state =HBMovementState.RUNNINGSTOP
                if self.__userIO.turnCon()!=0:
                    self.__state= HBMovementState.RUNNINGTURN
                if R < 0:       
                    R=0
                if L < 0:      
                    L=0
                speed = R-L     
                if(speed!=0):
                    self.__movement(speed)
                    
            elif self.__state == HBMovementState.RUNNINGTURN:
                R = self.__userIO.forwardSpeedCon()
                L = self.__userIO.backwardsSpeedCon()
                if R==-1 and L==-1 or R==1 and L==1:
                    self.__state =HBMovementState.RUNNINGSTOP
                if self.__userIO.turnCon()==0:
                    self.__state= HBMovementState.RUNNING
                if R < 0:       
                    R=0
                if L < 0:       
                    L=0
                speed = R-L     
                if(speed!=0):
                    self.__movement(speed, self.__userIO.turnCon()) 
                    
            elif self.__state == HBMovementState.RUNNINGSTOP:
                self.__idle()
                self.__runCycle=0
                self.__turnCylce=0
                self.__state = HBMovementState.IDLE
                
            elif self.__state == HBMovementState.EXIT:
                self.__userIO.quit()
                break
                
                
    
    
    def __jump(self):
        '''Helper function. Does the required movement for a static jump.'''
        self.__moveAll(0,0,40)
        time.sleep(0.4)
        self.__moveAll(0,0,-20)
        
    def __movement(self, speed:float, turn:float=0):
        '''Helper function. Does the required movement for running and turning while running.'''
        speed*=self.__globalspeed
        x1 = math.sin(self.__runCycle)*(10+(turn*5))
        y1 = math.cos(self.__runCycle)*40
        if(y1<0):
            y1=0
        x2 = math.sin(self.__runCycle+(math.pi))*(10+(turn*5))
        y2 = math.cos(self.__runCycle+(math.pi))*40
        if(y2<0):
            y2=0
            
        y1+=self.__height
        y2+=self.__height
        
        self.__leg1.moveLeg(x1,0,y1)
        self.__leg2.moveLeg(x2,0,y2)
        self.__leg3.moveLeg(x1,0,y1)
        self.__leg4.moveLeg(x2,0,y2)
        self.__leg5.moveLeg(x1,0,y1)
        self.__leg6.moveLeg(x2,0,y2)
        self.__runCycle+=speed  
          
    def __turn(self, turn:float):
        '''Helper function. Does the required movement for a static turn.'''
        turn*=self.__globalspeed
        x1 = math.sin(self.__turnCylce)*10
        y1 = math.cos(self.__turnCylce)*40
        if(y1<0):
            y1=0
        x2 = math.sin(self.__turnCylce+(math.pi))*10
        y2 = math.cos(self.__turnCylce+(math.pi))*40
        if(y2<0):
            y2=0
        
        y1+=self.__height
        y2+=self.__height
        
        self.__leg1.moveLeg(x1,0,y1)
        self.__leg2.moveLeg(x2,0,y2)
        self.__leg3.moveLeg(x1,0,y1)
        self.__leg4.moveLeg(-x2,0,y2)
        self.__leg5.moveLeg(-x1,0,y1)
        self.__leg6.moveLeg(-x2,0,y2)
        self.__turnCylce+=turn
        
    def __moveAll(self,x:float,y:float,z:float):
        '''Helper function. Moves all legs to relative {x,y,z} coordinates.'''
        z+=self.__height
        self.__leg1.moveLeg(x,y,z)
        self.__leg2.moveLeg(x,y,z)
        self.__leg3.moveLeg(x,y,z)
        self.__leg4.moveLeg(x,y,z)
        self.__leg5.moveLeg(x,y,z)
        self.__leg6.moveLeg(x,y,z)
        
        
    def __idle(self):
        '''Helper function. Does the required movement for idling.'''
        self.__moveAll(0,0,0)
        
    
            
    

        
def main():
    bot = HBMovement()
    bot.start()
        
if __name__ == "__main__":
    main()
