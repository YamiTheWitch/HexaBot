from __future__ import annotations
import time
import math
from adafruit_motor import servo
from BetterServoKit import ServoKit2
from PS4Controller import PS4Controller
from threading import Thread

J2L = 76 #length between hip and middlejoint
J3L = 112 #length between middle joint and tip
J3OffsetAngle=-5 #the offset angle between where the tip actually is and where it's supposed to be

Y_rest = 70 #resting Y position
Z_rest = -110 #resting X position

        
class Joint():
    def __init__(self,servo : servo.Servo, inverted : bool = False, offset : int = 0):
        self.servo = servo
        self.inv = inverted
        self.offs=offset
        
    def angle(self, angle: int):
        if(self.inv):
            self.servo.angle=(180-angle+self.offs)
        else:
            self.servo.angle=(angle+self.offs)
        
class Leg():
    def __init__(self, j1: Joint, j2: Joint, j3:Joint, angle:int):
        self.j1=j1
        self.j2=j2
        self.j3=j3
        self.angle=angle
        
        
    def moveLeg(self, relX:float, relY:float, relZ:float):
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
            
class HexaBotState():
    INIT = 1
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
    
    RUNNINGJUMPSTART=12
    RUNNINGJUMP=13
    RUNNINGJUMPLAND=14
            
            
class HexaBot:
    def __init__(self):
        sk = ServoKit2()
        
        self.height=0
        self.globalspeed=1
        
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
        
        self.state=HexaBotState.IDLE
        
        self.onJump=None
        self.onRun=None
        self.onTurn=None
        
        self.__ps4 = PS4Controller()
        self.__ps4.init()
           
    def start(self):
        p = Thread(target=self.__loop)
        p.start()
        p.join
            
    def __loop(self):
        print("Started main loop!")
        while True:
            if self.state == HexaBotState.IDLE:
                self.__idle()
                if self.__ps4.L1_Button:
                    self.height+=1
                elif self.__ps4.R1_Button:
                    self.height-=1
                    
                if self.__ps4.Up_button:
                    self.globalspeed+=0.3
                elif self.__ps4.Down_button:
                    self.globalspeed-=0.3
                    
                if self.__ps4.L3_Button:
                    self.height=0
                    self.globalspeed=1
                
                if self.__ps4.X_button:
                    self.state=HexaBotState.STANDINGJUMPSTART
                elif self.__ps4.R_trigger != -1 or self.__ps4.L_trigger !=-1:
                    self.state=HexaBotState.RUNNING
                elif self.__ps4.Opt_button:
                    self.__ps4.stop()
                    break
                elif self.__ps4.LX_Axis != 0:
                    self.state=HexaBotState.STANDINGTURN
            
            elif self.state==HexaBotState.STANDINGTURN:
                self.__turn(self.__ps4.LX_Axis)
                if self.__ps4.LX_Axis==0:
                    self.state=HexaBotState.STANDINGTURNSTOP
            
            elif self.state==HexaBotState.STANDINGTURNSTOP:
                self.__idle()
                self.__turnCylce=0
                self.state=HexaBotState.IDLE
                    
            elif self.state ==HexaBotState.STANDINGJUMPSTART:
                self.__jump()
                self.state = HexaBotState.STANDINGJUMP
                self.__jumpdelay=time.time()
                
            elif self.state==HexaBotState.STANDINGJUMP:
                self.__idle()
                if time.time()-self.__jumpdelay>1:
                    self.state=HexaBotState.IDLE
                
            elif self.state == HexaBotState.RUNNING:
                R = self.__ps4.R_trigger
                L = self.__ps4.L_trigger
                if R==-1 and L==-1 or R==1 and L==1:
                    self.state =HexaBotState.RUNNINGSTOP
                if self.__ps4.LX_Axis!=0:
                    self.state= HexaBotState.RUNNINGTURN
                if R < 0:
                    R=0
                if L < 0:
                    L=0
                speed = R-L
                if(speed!=0):
                    self.__movement(speed)
                    
            elif self.state == HexaBotState.RUNNINGTURN:
                R = self.__ps4.R_trigger
                L = self.__ps4.L_trigger
                if R==-1 and L==-1 or R==1 and L==1:
                    self.state =HexaBotState.RUNNINGSTOP
                if self.__ps4.LX_Axis==0:
                    self.state= HexaBotState.RUNNING
                if R < 0:
                    R=0
                if L < 0:
                    L=0
                speed = R-L
                if(speed!=0):
                    self.__movement(speed, self.__ps4.LX_Axis)
                    
            elif self.state == HexaBotState.RUNNINGSTOP:
                self.__idle()
                self.__runCycle=0
                self.__turnCylce=0
                self.state = HexaBotState.IDLE
                
                
    
    
    def __jump(self):
        if(self.onJump!=None):
            self.onJump()
        self.__moveAll(0,0,40+self.height)
        time.sleep(0.4)
        self.__moveAll(0,0,-20+self.height)
        
    def __movement(self, speed, turn = 0):
        if(self.onRun!=None):
            self.onRun()
        speed*=self.globalspeed
        x1 = math.sin(self.__runCycle)*(10+(turn*5))
        y1 = math.cos(self.__runCycle)*40
        if(y1<0):
            y1=0
        x2 = math.sin(self.__runCycle+(math.pi))*(10+(turn*5))
        y2 = math.cos(self.__runCycle+(math.pi))*40
        if(y2<0):
            y2=0
            
        y1+=self.height
        y2+=self.height
        
        self.__leg1.moveLeg(x1,0,y1)
        self.__leg2.moveLeg(x2,0,y2)
        self.__leg3.moveLeg(x1,0,y1)
        self.__leg4.moveLeg(x2,0,y2)
        self.__leg5.moveLeg(x1,0,y1)
        self.__leg6.moveLeg(x2,0,y2)
        self.__runCycle+=speed  
          
    def __turn(self, turn):
        if(self.onTurn!=None):
            self.onTurn()
        turn*=self.globalspeed
        x1 = math.sin(self.__turnCylce)*10
        y1 = math.cos(self.__turnCylce)*40
        if(y1<0):
            y1=0
        x2 = math.sin(self.__turnCylce+(math.pi))*10
        y2 = math.cos(self.__turnCylce+(math.pi))*40
        if(y2<0):
            y2=0
        
        y1+=self.height
        y2+=self.height
        
        self.__leg1.moveLeg(x1,0,y1)
        self.__leg2.moveLeg(x2,0,y2)
        self.__leg3.moveLeg(x1,0,y1)
        self.__leg4.moveLeg(-x2,0,y2)
        self.__leg5.moveLeg(-x1,0,y1)
        self.__leg6.moveLeg(-x2,0,y2)
        self.__turnCylce+=turn
        
    def __moveAll(self,x,y,z):
        z+=self.height
        self.__leg1.moveLeg(x,y,z)
        self.__leg2.moveLeg(x,y,z)
        self.__leg3.moveLeg(x,y,z)
        self.__leg4.moveLeg(x,y,z)
        self.__leg5.moveLeg(x,y,z)
        self.__leg6.moveLeg(x,y,z)
        
        
    def __idle(self):
        self.__moveAll(0,0,self.height)
        
    
            
    

        
def main():
    bot = HexaBot()
    bot.start()
        
if __name__ == "__main__":
    main()
