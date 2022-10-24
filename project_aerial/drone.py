from token import RIGHTSHIFT
import numpy as np
import math as m
from sympy.solvers import solve
from sympy import Symbol
import time

# Import cflib and all the necessary
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig

from utilities import *

"""
Class drone:  where we code all the functions used by the drone 
"""


MIN_DISTANCE = 400  # distance use for the ranger to assume there is an obstacle 
VEL_LOCAL = 0.2     # velocity use for local avoidance
LAND_ZONE = 150     # distance in cm from the border  

print('hello')
class quad():

    def __init__(self, cf, id='radio://0/80/2M/E7E7E7E7E7',
                 pos_init=np.array([0, 0, 0]), vel=np.array([None, None, None]),
                 state=0, default_heigth=0.3, default_vel=0.1):
        self.pos_init = pos_init        #pos initial of the drone (map reference)
        self.pos = pos_init             #pos of the drone (in the map reference)
        self.vel = vel                  # velocity of the drone 
        #self.once = True                

        self.yaw = 0                    # value of the yaw
        self.sensors = None             # values in the 4 range sensor 
        self.state = 0                  # state of the drone
        
        self.default_heigth = default_heigth # default height of the drone 
        self.default_vel = default_vel       # default vel of the drone 
        self.id = id
        self.is_flying = False
        self.cf = cf
        self.cb = call_backs(cf)
        self.map = map_utile(pos_init[0], pos_init[1])
        self.pad = find_pad()

        self.multiranger = None
        self.motion_cmder = None
        self.map.update_pad_pos(self.pos_init[0:2]) # put pad position on the map 

        # for searching the pad : 
        self.search_x, self.search_y = self.compute_points_to_search()  # list of point to reach in the landing zone to scan the area
        self.pos_ref = np.array([None,None,None])                       # pos ref of the drone  
        self.idx=0                                                      # index for the next point to reach
       

        # variables to know how to approach the pad when found :
        self.dir=0                  # go in direction x ou y (0 or 1)
        self.vel_pad=1              # positive direction (1) or negative direction (-1)

        # counter to control the yaw : 
        self.counter_yaw=0          # yaw counter to scan in front of the drone to avoid obstacle
        # local : 
        self.move=0                 # variable to check what to do in local (move staight or left_right)
        self.RATE=360.0/8           # Yaw rate velocity
        self.counter=0
      
################################################################################################# START IMPLEMENTATION ############################

    def idle(self):

        return self.state

    def take_off(self, heigth=0.3):
        """
        the drone take off and go in global navigation
        """

        self.motion_cmder.take_off(height=0.3, velocity=0.2)
        self.is_flying = True
        self.state = 2          # go to global nav
        time.sleep(2)
        return self.state

    def global_nav(self):
        """
        The drone update its pose, check obstacle on the side and in front and move in a certain direction depending of it's position
        """
        velocity = 0.3
        #Update pos & sensors
        self.update_pos() # update pos

        self.sensors = np.array([self.cb.front[-1], self.cb.right[-1], 
                                 self.cb.back[-1], self.cb.left[-1]])

        self.check_sides() # check obstacle on the side 
        self.check_local() # check obstacle in front 

        
        if 500 - self.pos[0] < LAND_ZONE : # check if in landing zone
     
            self.zigzag(self.search_x,self.search_y)           # scan the area 
            self.pad_search(first_pad=True)       # check if the pad is found 
            velocity = 0.3
        else :
            self.check_obstacle()   # turn on itstel (change the yaw) to scan the area 
           
            
   
            self.pos_ref = [None,self.cb.var_y_history[-1],None] # update position of reference 
        
        # update velocity of the drone 
        self.update_velocity(pos_ref_x=self.pos_ref[0],pos_ref_y=self.pos_ref[1],vel_x=self.vel[0], vel_y=self.vel[1], velocity = velocity)
        return self.state

    
    def local_nav(self):
        """
        update it's pose, check obstacle and move in x or y to avoid the obstacle. Go back in global nav at the end 
        """
        self.update_pos() # update pose
        self.check_sides() # check obstacles on the side
        self.sensors = np.array([self.cb.front[-1], 
                                 self.cb.back[-1]])

        
        if self.dir==0: # if the drone is in the  x direction
            
            self.move_L_or_R(state=2) # avoid obstacle in left or right direction depending on the obstacle and drone position
            
            
        
        else : 
            if self.vel_pad==1: # if the drone moves on the left 

                if self.move==0:  # avoid first the obstacle by going in the x direction 
                    self.move_straight_landing_region(self.cb.left[-1],state=2)
                else: # move left or right to go around the obstacle 
                    self.move_L_or_R_landing_region()

            
            else: # if the drone moves on the right 
                 
                if self.move==0:  # avoid first the obstacle going in the x direction          
                    self.move_straight_landing_region(self.cb.right[-1],state=2)
                else: # move left or right to go around the obstacle 
                    self.move_L_or_R_landing_region()
                    

        return self.state
  
    def land(self, velocity):     #to land on the right pad
        
        self.motion_cmder.start_linear_motion(0,0,0)
        self.map.update_pad_pos(self.pos[0:2]) #set on the map the right position on the pad 
        self.motion_cmder.land(velocity=velocity)
        self.is_flying = False
        self.state = 7 #to switch to the take_off_back state to return to the departure area (the starting pad)
        time.sleep(0.5)
        return self.state
    
   



    def pad_search(self, first_pad=True,verbose = False): #to look for the landing pad
        
        #Looking for the first border on the pad, as soon as we find it we switch to the pad found state 
        if self.pad.find_pad_out(self.cb.var_z_history,self.cb.var_x_history,self.cb.var_y_history):      
            if first_pad:
                self.state=5
            else:
                self.state=10
            print('find pad')
            self.motion_cmder.start_linear_motion(0,0,0) 
            time.sleep(3)

    #The goal of this function is to detect the 4 borders of the pad doing a cross movement 
    #then compute the middle for each direction to set the center of the pad and land on it

    def pad_found(self, first_pad=True): #Identify the 3 remaining borders to place the drone in the middle of the pad 
        state = 1 #state to check the first border from the 3 remaining ones
        print('state of the first border from the 3 remaining ones',state)

        while state !=4: #Until we don't find the last border of the pad 
            self.update_pos() #To update position of the drone on the map
            if self.pad.states_pad.get(state)=="border1": #detection of the first border from the 3 remaining
                if self.dir==0: #if we arrive from the x direction
                    self.update_velocity(pos_ref_x=None,pos_ref_y=self.pad.pad_bord1[1],
                                        vel_x=self.vel_pad*0.12*m.cos(self.yaw),vel_y=None) 
                else:#Arriving from the y direction               
                    self.update_velocity(pos_ref_x=self.pad.pad_bord1[0],pos_ref_y=None,
                                        vel_x=None,vel_y=self.vel_pad*m.cos(self.yaw)*0.12)
                if self.pad.find_pad_in(self.cb.var_z_history,self.cb.var_x_history,self.cb.var_y_history,var=0): #Looking for the first border from the 3 remaining ones
                    time.sleep(0.5)
                    self.motion_cmder.start_linear_motion(0,0,0) 
                    time.sleep(1)
                    state=2 #switch to the state to check the second border from the 3 remaining ones
                    if self.dir==0: #if we arrive from the x direction
                        self.mid_pad_x=self.pad.pad_bord1[0]+1/2*(self.pad.pad_bord2[0]-self.pad.pad_bord1[0]) #To compute the middle of the pad according to the x axis
                        self.go_to(x_ref=self.mid_pad_x,y_ref=self.pad.pad_bord1[1],z_ref=None) #To go to the middle point on x and stay on it
                    else: #Arriving from the y direction
                        self.mid_pad_y=self.pad.pad_bord1[1]+1/2*(self.pad.pad_bord2[1]-self.pad.pad_bord1[1]) #To compute the middle of the pad according to the y axis
                        self.go_to(x_ref=self.pad.pad_bord1[0],y_ref=self.mid_pad_y,z_ref=None) #To go to the middle point on y and stay on it
                        time.sleep(1)
            if self.pad.states_pad.get(state)=="border2":  #detection of the second border from the 3 remaining
 
                if self.dir==0: #if we arrive from the x direction
                    self.update_velocity(pos_ref_x=self.mid_pad_x,pos_ref_y=None,vel_x=None,vel_y=-0.12)
                else: #Arriving from the y direction
                    self.update_velocity(pos_ref_x=None,pos_ref_y=self.mid_pad_y,vel_x=-0.12,vel_y=None)
                time.sleep(0.2)
                if self.pad.find_pad_in(self.cb.var_z_history,self.cb.var_x_history,self.cb.var_y_history,var=1): #Looking for the second border from the 3 remaining ones
                    time.sleep(0.5)
                    self.motion_cmder.start_linear_motion(0,0,0) 
                    time.sleep(1)
                    if self.dir==0: #if we arrive from the x direction
                        self.go_to(x_ref=self.mid_pad_x,y_ref=self.pad.pad_bord1[1],z_ref=None) #To go back inside of the pad before looking for the last border
                    else: #Arriving from the y direction
                        self.go_to(x_ref=self.pad.pad_bord1[0],y_ref=self.mid_pad_y,z_ref=None) #To go back inside of the pad before looking for the last border
                    state=3 #switch to the state to check the third border from the 3 remaining ones
                    time.sleep(1)

            if self.pad.states_pad.get(state)=="border3": #detection of the third border from the 3 remaining
                if self.dir==0:  #if we arrive from the x direction
                    self.update_velocity(pos_ref_x=self.mid_pad_x,pos_ref_y=None,vel_x=None,vel_y=0.12)
                else: #Arriving from the y direction
                    self.update_velocity(pos_ref_x=None,pos_ref_y=self.mid_pad_y,vel_x=0.12,vel_y=None)
                time.sleep(0.2)
                if self.pad.find_pad_in(self.cb.var_z_history,self.cb.var_x_history,self.cb.var_y_history,var=2):  #Looking for the third border from the 3 remaining ones
                    time.sleep(0.5)
                    self.motion_cmder.start_linear_motion(0,0,0) 
                    time.sleep(1)
                    if self.dir==0:  #if we arrive from the x direction
                        self.mid_pad_y=self.pad.pad_bord3[1]+1/2*(self.pad.pad_bord4[1]-self.pad.pad_bord3[1]) #To compute the middle of the pad according to the y axis
                    else : #Arriving from the y direction
                        self.mid_pad_x=self.pad.pad_bord3[0]+1/2*(self.pad.pad_bord4[0]-self.pad.pad_bord3[0]) #To compute the middle of the pad according to the x axis

                    self.go_to(x_ref=self.mid_pad_x,y_ref=self.mid_pad_y,z_ref=None) #To go to the middle point on y and stay on it
                    time.sleep(1)
                    self.go_to(x_ref=self.mid_pad_x,y_ref=self.mid_pad_y,z_ref=0.2) #To go to the center of the pad and go down to land easily on it
                    state=4 #To quit the while loop when all the borders have been found
        if first_pad: 
            self.state = 6 #switch to the landing state
        else : 
            self.state = 11
        return

    def update_pos(self): #Update the position of the drone on the map
        
        self.pos = np.array([self.cb.var_x_history[-1]*100 + self.pos_init[0],
                            self.cb.var_y_history[-1]*100 + self.pos_init[1],
                            self.cb.var_z_history[-1]*100 + self.pos_init[2]])

    def connect(self, scf, verbose=False):
        self.motion_cmder = MotionCommander(scf,
                                            default_height=self.default_heigth)
        if verbose:
            print('Motion Commander connected and configured')

        self.multiranger = Multiranger(scf)
        if verbose:
            print('multiranger connected and configured')
        return

    def compute_vel_local(self):
        """
        Compute in which direction should the drone go in local avoidance
        """


        disty0 = self.pos[1]
        disty300 = 300 - self.pos[1]

        
        if self.cb.right[-1]<MIN_DISTANCE: #To check if we have an obstacle on the right, so we go to the left
            vel=VEL_LOCAL
            print('case1')
        elif self.cb.left[-1]<MIN_DISTANCE: #To check if we have an obstacle on the left, so we go to the right
            vel=-VEL_LOCAL
            print('case2')
        elif disty0 > disty300 : #Check where we are on the map and then go in the direction where the distance is bigger
            vel = -VEL_LOCAL
            vel = -VEL_LOCAL
            print('case3')
        else :
            vel = VEL_LOCAL
            print('case4')
        
        self.vel_local = vel     #set the velocity depending on the case

    def check_local(self): #check if there is any obstacles around 
    
        if (self.dir==0) or (self.pos[0]<350):  #Check if the drone comes from the x direction and if it is before the landing zone 
            if(self.cb.front[-1] >= MIN_DISTANCE): #Check if there is a front obstacle
                self.vel = np.array([0.3,None,None])
                obstacle = False
            else : #the case where we have a front obstacle 
                self.compute_vel_local() #compute the direction of the avoidance 
                self.vel = np.array([None,None,None])
                self.state = 3  #switch to the local avoidance state 
                if self.pos[0]>350:  #if we are on the landing zone
                    self.idx=self.idx+1 #when we are on the landing zone, we go directly to the next searching point 
                obstacle = True
        elif self.dir==1 : #Check if the drone comes from the y direction
            obstacle = False
            
            if self.vel_pad==1:
                if(self.cb.left[-1] <= MIN_DISTANCE):  #Check if we have a left obstacle 
                    self.vel = np.array([None,None,None])
                    self.state = 3 #switch to the local avoidance state
                    x = self.cb.var_x_history[-1]
                    y = self.cb.var_y_history[-1]
                    distance = m.sqrt(((self.search_x[self.idx]-x)**2)+((self.search_y[self.idx]-y)**2))
                
                    if distance<0.8:
                        self.idx=self.idx+1 #go to the next point if the obstacle is close to the actual point
                    obstacle = True
            else:
                if(self.cb.right[-1] <= MIN_DISTANCE): #Check if we have a left obstacle 
                   
                    self.vel = np.array([None,None,None])
                    self.state = 3  #switch to the local avoidance state
                    x = self.cb.var_x_history[-1]
                    y = self.cb.var_y_history[-1]
                    distance = m.sqrt(((self.search_x[self.idx]-x)**2)+((self.search_y[self.idx]-y)**2))
                
                    if distance<0.8:
                        self.idx=self.idx+1 #go to the next point if the obstacle is close to the actual point
                    obstacle = True

        return obstacle
    
    def check_obstacle(self):  #make a rotation every 3 seconds to check if there is any obstacle around 
        
        if abs(self.cb._time_history[-1]-self.counter_yaw)>3:

            self.check_yaw()
            

    def check_yaw(self): #To check quite often if there is any obstacle around and if there is, move in a direction to avoid it
        obstacle=False
        
        len_before=len(self.cb.front)
        self.motion_cmder.start_linear_motion(0,0,0)
        turn_left=True
        old_yaw=self.yaw
        i=0
        find_pad=False
        while turn_left: #turn a left for a while then turn right two times more then turn left again 
            self.motion_cmder._set_vel_setpoint(0.0, 0.0, 0.0, -self.RATE) #turn left using the yaw coordinate 
            i=i+1  #counter used to turn from a certain angle 
            self.yaw=self.yaw-self.RATE*0.05 #update the yaw value after the turn 
            time.sleep(0.05)
            if i==10:
                turn_left=False
                
        i=0  #reset the counter
       
        while not turn_left: #to turn right 
            self.motion_cmder._set_vel_setpoint(0.0, 0.0, 0.0, self.RATE)
            i=i+1  #counter used to turn from a certain angle 
            self.yaw=self.yaw+self.RATE*0.05 #update the yaw value after the turn 
            time.sleep(0.05)
            
            if i==20: #to turn with the same angle on the right direction 
                turn_left=True
                
        i=0 #reset the counter
        while turn_left: #turn left to go back to the initial yaw (0 degree)
            self.motion_cmder._set_vel_setpoint(0.0, 0.0, 0.0, -self.RATE)
            i=i+1
            self.yaw=self.yaw-self.RATE*0.05
            time.sleep(0.05)
            if i==10:
                turn_left=False
                self.yaw=old_yaw
        self.motion_cmder.stop() #Stop the drone to reset all the velocities to zero 

        #Check if we have an obstacle in front and check for which yaw we have the lowest range value and turn in the opposite direction from the obstacle 
        min_front=min(self.cb.front[len_before:-1])
        idx=np.argmin(self.cb.front[len_before:-1])
        min_left=min(self.cb.left[len_before:-1])
        min_right=min(self.cb.right[len_before:-1])

       
        if min_front<MIN_DISTANCE:
            print(idx<(len(self.cb.front[len_before:-1])/2))
            if (min_left<MIN_DISTANCE) or (idx<(len(self.cb.front[len_before:-1])/2)):
                
                self.go_to(self.cb.var_x_history[-1],self.cb.var_y_history[-1]-0.2,z_ref=None)
                
            else:
                
                self.go_to(self.cb.var_x_history[-1],self.cb.var_y_history[-1]+0.2,z_ref=None)
        
        self.counter_yaw=self.cb._time_history[-1]   
    
    
    def check_sides(self):
        """
        check on the side if we have some obstacles to update them on a map
        """
        if self.cb.right[-1] < MIN_DISTANCE :
            pos_obstacle = [int(self.pos[0]),max(0,int(self.pos[1] - self.cb.right[-1]/10))]
            self.map.update_map_obstacle(pos_obstacle)

        if self.cb.left[-1] < MIN_DISTANCE :
            pos_obstacle = [int(self.pos[0]),min(300,int(self.pos[1] + self.cb.left[-1]/10))]
            self.map.update_map_obstacle(pos_obstacle)

    def go_to(self,x_ref,y_ref,z_ref=None):
        """
        function to go to a reference coordinate x_ref,y_ref,z_ref
        """
        # calculate our distance from the reference coordinate
        x = self.cb.var_x_history[-1]
        y = self.cb.var_y_history[-1]
        distance = m.sqrt(((x_ref-x)**2)+((y_ref-y)**2))
        
        while (distance>0.01): # update the velocity until we get close to the reference coordinate
            self.update_velocity(pos_ref_x=x_ref, pos_ref_y=y_ref, pos_ref_z=z_ref,
                                 vel_x=None, vel_y=None, vel_z=None)
            # calculate the new distance from the reference coordinate 
            x = self.cb.var_x_history[-1]
            y = self.cb.var_y_history[-1]
            distance = m.sqrt(((x_ref-x)**2)+((y_ref-y)**2))

    def update_velocity(self, pos_ref_x=None, pos_ref_y=None, pos_ref_z=None,
                        vel_x=None, vel_y=None, vel_z=None,velocity=0.2):

        """
        Update the velocity in x,y and z depending on the reference point and the reference velocity
        """
        update_vel=[0,0,0]
        coeff=1.5
        
        # update velocity for x
        if pos_ref_x is None: #if we give no reference point and no velocity : return 0
            if vel_x is None: 
                update_vel[0]=0
            else : 
                update_vel[0]=vel_x # if we give only a velocity, it returns the velocity

        elif vel_x is None: # if we give only a reference point, it use a proportional controller with a saturation
            if (pos_ref_x-self.cb.var_x_history[-1])>0:
                update_vel[0]=min(velocity, (pos_ref_x-self.cb.var_x_history[-1])*coeff)
            else:
                update_vel[0]=max(-velocity, (pos_ref_x-self.cb.var_x_history[-1])*coeff)

        # update velocity for y
        if pos_ref_y is None: #if we give no reference point and no velocity : return 0
            if vel_y is None:
                update_vel[1]=0
            else : 
                update_vel[1]=vel_y # if we give only a velocity, it returns the velocity

        elif vel_y is None: # if we give only a reference point, it use a proportional controller with a saturation
            if (pos_ref_y-self.cb.var_y_history[-1])>0:
                update_vel[1]=min(velocity, (pos_ref_y-self.cb.var_y_history[-1])*coeff)
            else:
                update_vel[1]=max(-velocity, (pos_ref_y-self.cb.var_y_history[-1])*coeff)

        # update velocity for z
        if pos_ref_z is None: #if we give no reference point and no velocity : return 0
            if vel_z is None:
                update_vel[2]=0
            else : 
                update_vel[2]=vel_z # if we give only a velocity, it returns the velocity


        elif vel_z is None: # if we give only a reference point, it use a proportional controller with a saturation
            if (pos_ref_z-self.cb.var_z_history[-1])>0:
                update_vel[2]=min(0.15, (pos_ref_z-self.cb.var_z_history[-1])*coeff)
            else:
                update_vel[2]=max(-0.15, (pos_ref_z-self.cb.var_z_history[-1])*coeff)

        # correct velocity in x and y in case the yaw is not 0 :
        new_vel_x=update_vel[0]*m.cos(self.yaw)-update_vel[1]*m.sin(self.yaw)
        new_vel_y=update_vel[1]*m.cos(self.yaw)+update_vel[0]*m.sin(self.yaw)
        
        # give the new velocity to the drone 
        self.motion_cmder.start_linear_motion(new_vel_x,new_vel_y,update_vel[2]) 

        # if we are in global navigation : update the direction (x or y) and the value (positive or negative) of the velocity
        if (self.state==2) or (self.state==8):
            
            if abs(new_vel_x)> abs(new_vel_y):
                self.dir=0
                if new_vel_x>0:
                    self.vel_pad=1
                else:
                    self.vel_pad=-1

            else:
                self.dir=1
                if new_vel_y>0:
                    self.vel_pad=1
                else:
                    self.vel_pad=-1
           
        time.sleep(0.2)


    def zigzag(self,search_x,search_y): #Do some slalom looking for the landing pad
        # assume that we are at the middle 
        self.pos_ref = np.array([search_x[self.idx], search_y[self.idx], None]) #set the positon ref to our searching points
        self.vel = [None,None,None]
        x = self.cb.var_x_history[-1]
        y = self.cb.var_y_history[-1]
        distance = m.sqrt(((search_x[self.idx]-x)**2)+((search_y[self.idx]-y)**2))
        if distance<0.02: #if close to one point go to the next searching point
            self.idx=self.idx+1  
            




    def compute_points_to_search(self): #compute some points to follow during the searching of the landing pad
        search_x = []
        search_y = []
        step = 0.3

        search_x.append(3.6-self.pos_init[0]/100)
        search_y.append(0.3-self.pos_init[1]/100)

        for i in range(0,8,2):
            search_x.append(3.6+i*step-self.pos_init[0]/100)
            search_y.append(2.7-self.pos_init[1]/100)

            search_x.append(3.6+(i+1)*step-self.pos_init[0]/100)
            search_y.append(2.7-self.pos_init[1]/100)

            search_x.append(3.6+(i+1)*step-self.pos_init[0]/100)
            search_y.append(0.3-self.pos_init[1]/100)

            search_x.append(3.6+(i+2)*step-self.pos_init[0]/100)
            search_y.append(0.3-self.pos_init[1]/100)

        return search_x, search_y
        
    def move_L_or_R(self, state): #move left or right depending on the position of the obstacle
        if(self.cb.front[-1] <= MIN_DISTANCE): #if we have a front obstacle 
            self.vel = np.array([0,self.vel_local,0])
            if (self.vel_local>0) & (self.cb.left[-1]<= MIN_DISTANCE) : #if we have a left obstacle
                self.vel_local=-self.vel_local
            if (self.vel_local<0) & (self.cb.right[-1]<= MIN_DISTANCE) : #if we have a right obstacle
                self.vel_local=-self.vel_local
            if state==2: #update obstacles on the map if we are on the starting way
                pos_obstacle = [min(500,int(self.pos[0] + self.sensors[0]/10)),int(self.pos[1])]
                self.map.update_map_obstacle(pos_obstacle)
        else :
            time.sleep(0.5)
            self.vel = np.array([0,0,0])
            if state==8: #if we are on the way back
                self.move=1 #start going straight to go around the obstacle 

            if state==2:  #if we are on the starting way
                self.check_yaw()  #check if there is any obstacles around and switch to the global navigation
                self.state = state   
               
        self.motion_cmder.start_linear_motion(self.vel[0], self.vel[1], self.vel[2])

    def move_straight(self): #go straight to go around the obstacle 
        
        self.vel = np.array([0.2,0,0])
        self.motion_cmder.start_linear_motion(self.vel[0], self.vel[1], self.vel[2])
        not_end_obstacle=True
        begining_obstacle=False
        if self.vel_local>0: #if we avoid the obstacle by the left, we use the right sensor to see if the obstacle stills there until we totally avoid it
            while not_end_obstacle:
                self.counter=self.counter+1
                time.sleep(0.05)
                if (self.cb.right[-1] < MIN_DISTANCE) : 
                    begining_obstacle=True
                    

                if (self.cb.right[-1] > MIN_DISTANCE) & begining_obstacle : #if we completely avoid the obstacle we go back to the main path
                    not_end_obstacle=False
                    self.motion_cmder.start_linear_motion(0 , 0, 0)
                  
                    time.sleep(1)
                    self.state =8 #switch to the global_back state
        else:
            while not_end_obstacle: #if we avoid the obstacle by the right, we use the left sensor to see if the obstacle stills there until we totally avoid it
                self.counter=self.counter+1
                time.sleep(0.05)
                if (self.cb.left[-1] < MIN_DISTANCE) :
                    begining_obstacle=True
                    
                if (self.cb.left[-1] > MIN_DISTANCE) & begining_obstacle :  #if we completely avoid the obstacle we go back to the main path
                    not_end_obstacle=False
                    self.motion_cmder.start_linear_motion(0 , 0, 0)
                    
                    time.sleep(1)
                    self.state =8 #switch to the global_back state
                if self.counter==30:
                    self.counter=0
                    not_end_obstacle=False
                    self.state=8
            self.state=8 
         
        self.move=0 #reset the move variable

    def move_straight_landing_region(self,ranger,state): #go straight to avoid the obstacle when we are moving on the y axis 
        if(ranger <= MIN_DISTANCE): #if there is an obstacle we set it on the map and we start going straight to avoid it 
            self.vel = np.array([0.2,0,0])
            if self.state==2:
                pos_obstacle = [int(self.pos[0]),min(300,int(ranger/10))]
                self.map.update_map_obstacle(pos_obstacle)
        else :    #if we overcome the obstacle
            time.sleep(0.3)
            self.vel = np.array([0,0,0])
            self.move=1 #if we are on the starting way, we start going left or right to overcome the obstacle
            if state==8: 
                self.state=state #if we are on the way back we go to global_back navigation
                self.move=0
            
        self.motion_cmder.start_linear_motion(self.vel[0], self.vel[1], self.vel[2])

    def move_L_or_R_landing_region(self): #go left or right to go around the obstacle 
        if self.vel_pad==1: #select if we go left or right
            self.vel = np.array([0,0.2,0])
        else:
            self.vel = np.array([0,-0.2,0])
        self.motion_cmder.start_linear_motion(self.vel[0], self.vel[1], self.vel[2])
        not_end_obstacle=True
        begining_obstacle=False
        while not_end_obstacle: #use the back sensor to check if we overcome the obstacle
            self.counter=self.counter+1
            time.sleep(0.05)
            if (self.cb.back[-1] < MIN_DISTANCE) :
                pos_obstacle = [int(self.pos[0]-self.cb.back[-1]/10),int(self.pos[1])]
                self.map.update_map_obstacle(pos_obstacle)
                begining_obstacle=True
              
            if (self.cb.back[-1] > MIN_DISTANCE) & begining_obstacle :
                not_end_obstacle=False
                self.motion_cmder.start_linear_motion(0 , 0, 0)
                self.state =2 #switch to the global navigation after the drone overcomes the obstacle
            if self.counter==30:
                self.counter=0
                self.state =2
                not_end_obstacle
        self.move=0


    


################################################################################################# RETURN IMPLEMENTATION ############################
    '''
    The implementation of the functions for the return to the starting point 
    '''
    def take_off_back(self, heigth=0.3):

        """
        the drone take off and go in global_back navigation
        """

        self.motion_cmder.take_off(height=0.3, velocity=0.2)
        self.is_flying = True
        time.sleep(2)
        self.motion_cmder.turn_left(180) #rotate to go in the opposite direction 
        self.yaw=m.pi #update the value of the yaw
        self.state = 8 #switch to the global back state 
        self.go_center=True #boolean to indicate which direction the drone will take on the global navigation
        time.sleep(2)
        return self.state

    def land_back(self, velocity):  #to land on the right pad
        
        self.motion_cmder.start_linear_motion(0,0,0)
        self.motion_cmder.land(velocity=velocity)
        self.is_flying = False
        self.state = 12 #switch to the final state
       
        time.sleep(0.5)
        return self.state
        
    def global_back(self):
        """
        The drone checks obstacles and goes to the starting pad 
        """

        velocity = 0.3


        self.sensors = np.array([self.cb.front[-1], self.cb.right[-1], 
                                 self.cb.back[-1], self.cb.left[-1]])

        
        self.check_local_back() #check if there is any obstacles on the direction where it's moving

        if self.go_center : #move to the mid_pad postion on the y axis before going straight to the starting pad 
            self.pos_ref = [self.cb.var_x_history[-1],-self.mid_pad_y,None]
            self.vel=[None,None,None]
            
            y = self.cb.var_y_history[-1]
            if abs(-self.mid_pad_y-y)<0.05: #set go_center to false when we attend the mid_pad positon on the y axis
                self.go_center=False
                self.search_x_back, self.search_y_back = self.compute_points_to_search_back() #compute some reference points for the zizag function when the drone is close to the starting pad
                self.idx=0
        else: #if the drone is close to the pad we start the slalom and the searching for the starting pad
            if self.cb.var_x_history[-1]+self.mid_pad_x < 0.5: 
                self.zigzag(self.search_x_back,self.search_y_back)
                self.pad_search(first_pad=False)
                velocity = 0.3
            else: #move until the drone is close to the starting pad 

                self.pos_ref = [-self.mid_pad_x,-self.mid_pad_y,None]
                self.vel=[None,None,None]


        
        self.update_velocity(pos_ref_x=self.pos_ref[0],pos_ref_y=self.pos_ref[1],vel_x=self.vel[0], vel_y=self.vel[1], velocity = velocity) #update the velocity 
        return self.state

    

    def check_local_back(self):
        """
        Function to check if there is an obstacle in the direction where the drone is going
        """

        if (self.dir==0) and (self.cb.front[-1] <= MIN_DISTANCE):

                print('obstacle in front')
                self.vel_local=VEL_LOCAL
                self.state=9
        
        elif self.dir==1: 
            if self.vel_pad==1:
                if(self.cb.left[-1] <= MIN_DISTANCE):
                    print("left obstacle")
                    self.state = 9
            else:
                if(self.cb.right[-1] <= MIN_DISTANCE):
                    print("ritght obstacle")
                    self.state = 9

        return self.state

    def local_back(self):
        """
        function to avoid the obstacle : two different method depending on the position of the drone and it's direction
        """
        self.sensors = np.array([self.cb.front[-1], 
                                 self.cb.back[-1]])

        if self.dir==0: #if the drone is going on the x direction
            if self.move==0: #first movement to avoid the obstacle

                self.move_L_or_R(state=8) #move left or right depending on the direction 
            else:

                self.move_straight() #go straight after avoiding the obstacle to go around it 
            
        else : #if the drone is on the y direction 
            if self.vel_pad==1: #check if we are on the left direction  

                if self.move==0: #first movement to avoid the obstacle

                    self.move_straight_landing_region(self.cb.left[-1],state=8)  #go straight to avoid the obstacle        
            else:

                if self.move==0: #first movement to avoid the obstacle
       
                    self.move_straight_landing_region(self.cb.right[-1],state=8) #go straight to avoid the obstacle
        return self.state

    def compute_points_to_search_back(self): #compute some points to follow during the searching of the starting pad 
        search_x = []
        search_y = []
        step = 0.2

        search_x.append(0.4-self.mid_pad_x)
        search_y.append(0.8-self.mid_pad_y)

        for i in range(0,6,2):
            search_x.append(0.4-self.mid_pad_x-i*step)
            search_y.append(-0.8-self.mid_pad_y)

            search_x.append(0.4-self.mid_pad_x-(i+1)*step)
            search_y.append(-0.8-self.mid_pad_y)

            search_x.append(0.4-self.mid_pad_x-(i+1)*step)
            search_y.append(0.8-self.mid_pad_y)

            search_x.append(0.4-self.mid_pad_x-(i+2)*step)
            search_y.append(0.8-self.mid_pad_y)

        return search_x, search_y

################################################################################################################################################